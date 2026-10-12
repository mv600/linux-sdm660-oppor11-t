// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm Technologies, Inc. SDM660 GFX LDO regulator driver
 *
 * The SDM630/SDM660 GPU power rail (VDD_GFX) is served by an on-die
 * LDO/BHS (bypass high side) switch block mapped at 0x506e000.  A Core
 * Power Reduction (CPR4) controller normally manages it through a set of
 * "fuse corner" tables; on boards where the stock DTB configures the gfx
 * corner with qcom,cpr-corner-allow-ldo-mode = 0 and -allow-closed-loop =
 * 0 for every corner, the block is used in open-loop BHS mode only.
 *
 * This is a minimal mainline driver that:
 *   - performs the documented hardware initialization (per-CPU register
 *     configuration table),
 *   - exposes the block as a generic voltage regulator so the GPU driver
 *     can manage the rail via its "vdd" supply,
 *   - supports switching between LDO (regulated) and BHS (bypassed) modes.
 *
 * It intentionally does not implement the CPR4 closed-loop/fuse machinery:
 * that is a separate, much larger piece of work.  The register sequence is
 * derived from the downstream driver drivers/regulator/msm_gfx_ldo.c
 * (SDM660 / VOLTAGE-ops path).
 *
 * Copyright (c) 2015-2017, The Linux Foundation. All rights reserved.
 * Copyright (c) 2026, OPPO R11T mainline bring-up.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>

/* LDO register offsets relative to the block base (0x0506e000) */
#define LDO_ATEST_REG			0x00
#define LDO_CFG0_REG			0x04
#define LDO_CFG1_REG			0x08
#define LDO_CFG2_REG			0x0c
#define LDO_LD_DATA_REG			0x10
#define LDO_VREF_TEST_CFG		0x14
#define LDO_VREF_SET_REG		0x18
#define PWRSWITCH_CTRL_REG		0x1c
#define LDO_STATUS1_REG			0x24
#define PWRSWITCH_STATUS_REG		0x28
#define REF_CURRENT_X1_REG		0x2c
#define REF_CURRENT_X2_REG		0x30
#define ADC_CTL_REG			0x34

/* LDO_VREF_SET_REG */
#define UPDATE_VREF_BIT			BIT(31)
#define SEL_RST_BIT			BIT(16)
#define VREF_VAL_MASK			GENMASK(6, 0)

/* PWRSWITCH_CTRL_REG */
#define LDO_CLAMP_IO_BIT		BIT(31)
#define CPR_BYPASS_IN_LDO_MODE_BIT	BIT(30)
#define EN_LDOAP_CTRL_CPR_BIT		BIT(29)
#define CX_CPR_BYPASS_IN_LDO_MODE_BIT	BIT(10)
#define PWR_SRC_SEL_BIT			BIT(9)
#define ACK_SW_OVR_BIT			BIT(8)
#define LDO_PREON_SW_OVR_BIT		BIT(7)
#define LDO_BYPASS_BIT			BIT(6)
#define LDO_PDN_BIT			BIT(5)
#define LDO_UNDER_SW_CTRL_BIT		BIT(4)
#define BHS_EN_REST_BIT			BIT(2)
#define BHS_EN_FEW_BIT			BIT(1)
#define BHS_UNDER_SW_CTL		BIT(0)

/* PWRSWITCH_STATUS_REG */
#define LDO_VREF_SETTLED_BIT		BIT(4)
#define LDO_READY_BIT			BIT(2)
#define BHS_EN_REST_ACK_BIT		BIT(1)

#define MIN_LDO_VOLTAGE			375000
#define MAX_LDO_VOLTAGE			980000
#define LDO_STEP_VOLTAGE		5000
#define LDO_MODE_SWITCH_TIMEOUT_US	500

#define GET_VREF(uv) DIV_ROUND_UP(((uv) - MIN_LDO_VOLTAGE), LDO_STEP_VOLTAGE)

#define QCOM_GFX_LDO_MAX_OFFSET		0xffff

struct gfx_ldo_config {
	u32 offset;
	u32 value;
};

/*
 * SDM630/SDM660 per-CPU LDO configuration values, taken verbatim from the
 * downstream msm_gfx_ldo.c sdm660_ldo_config[] table.
 */
static const struct gfx_ldo_config sdm660_ldo_config[] = {
	{ LDO_ATEST_REG,		0x00000080 },
	{ LDO_CFG0_REG,			0x0100A600 },
	{ LDO_CFG1_REG,			0x000000A0 },
	{ LDO_CFG2_REG,			0x0000C3FE },
	{ LDO_LD_DATA_REG,		0x00000000 },
	{ LDO_VREF_TEST_CFG,		0x00401100 },
	{ REF_CURRENT_X1_REG,		0x00000230 },
	{ REF_CURRENT_X2_REG,		0x00000048 },
	{ ADC_CTL_REG,			0x00000000 },
	{ QCOM_GFX_LDO_MAX_OFFSET,	QCOM_GFX_LDO_MAX_OFFSET },
};

struct qcom_gfx_ldo {
	void __iomem *base;
	struct device *dev;
	struct regulator_dev *rdev;
	struct regulator_desc desc;

	struct mutex lock;
	bool enabled;
	bool bypass;		/* true when the BHS switch feeds the rail */
	int voltage_uv;		/* last requested LDO target voltage */
};

static void gfx_ldo_configure_voltage(struct qcom_gfx_ldo *ldo, int new_uv)
{
	u32 val, reg;

	val = GET_VREF(new_uv);

	reg = readl(ldo->base + LDO_VREF_SET_REG);
	reg &= ~VREF_VAL_MASK;
	reg |= val & VREF_VAL_MASK;
	writel(reg, ldo->base + LDO_VREF_SET_REG);

	/* Pulse the update bit to latch the new VREF value. */
	reg |= UPDATE_VREF_BIT;
	writel(reg, ldo->base + LDO_VREF_SET_REG);
	mb();

	reg &= ~UPDATE_VREF_BIT;
	writel(reg, ldo->base + LDO_VREF_SET_REG);
	mb();

	ldo->voltage_uv = new_uv;
}

static int gfx_ldo_update_voltage(struct qcom_gfx_ldo *ldo, int new_uv)
{
	u32 reg;
	int timeout_us = LDO_MODE_SWITCH_TIMEOUT_US;

	gfx_ldo_configure_voltage(ldo, new_uv);

	while (timeout_us > 0) {
		reg = readl(ldo->base + PWRSWITCH_STATUS_REG);
		if (reg & (LDO_VREF_SETTLED_BIT | LDO_READY_BIT))
			return 0;
		udelay(10);
		timeout_us -= 10;
	}

	dev_err(ldo->dev, "LDO_VREF_SETTLED timeout, PWRSWITCH_STATUS=0x%x\n",
		reg);

	return -EBUSY;
}

static void gfx_ldo_enable_bhs(struct qcom_gfx_ldo *ldo)
{
	u32 ctl;

	ctl = readl(ldo->base + PWRSWITCH_CTRL_REG);

	/* Put the LDO under SW control and select BHS as the power source. */
	ctl |= LDO_UNDER_SW_CTRL_BIT;
	ctl &= ~PWR_SRC_SEL_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);

	/* Enable the BHS control signals and hand them over to the GDSC. */
	ctl &= ~BHS_EN_FEW_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl &= ~BHS_EN_REST_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl &= ~BHS_UNDER_SW_CTL;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);

	mb();
}

static void gfx_ldo_enable_ldo(struct qcom_gfx_ldo *ldo, int new_uv)
{
	u32 ctl;

	/* Set the VREF target while still powered from BHS. */
	gfx_ldo_configure_voltage(ldo, new_uv);

	ctl = readl(ldo->base + PWRSWITCH_CTRL_REG);

	/* Wake the LDO, keep BHS feeding the rail until the LDO is stable. */
	ctl |= BHS_UNDER_SW_CTL;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl &= ~LDO_UNDER_SW_CTRL_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl |= LDO_PREON_SW_OVR_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl &= ~LDO_BYPASS_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl |= PWR_SRC_SEL_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl &= ~ACK_SW_OVR_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);

	mb();
}

static int gfx_ldo_switch_to_ldo(struct qcom_gfx_ldo *ldo, int new_uv)
{
	u32 ctl, status;
	int timeout_us = LDO_MODE_SWITCH_TIMEOUT_US;

	ctl = readl(ldo->base + PWRSWITCH_CTRL_REG);

	/* Fake-ack the GDSC and select LDO as the power source. */
	ctl |= ACK_SW_OVR_BIT;
	ctl |= PWR_SRC_SEL_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);

	/* Keep BHS powered while the LDO comes up. */
	ctl |= BHS_EN_FEW_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl |= BHS_EN_REST_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl |= BHS_UNDER_SW_CTL;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);

	/* Put the LDO under SW control, de-bypass and power it on. */
	ctl |= LDO_UNDER_SW_CTRL_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl |= LDO_BYPASS_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl &= ~LDO_PDN_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);

	/* Program the target voltage, then power down BHS. */
	gfx_ldo_update_voltage(ldo, new_uv);

	ctl &= ~BHS_EN_FEW_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl &= ~BHS_EN_REST_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	mb();

	while (timeout_us > 0) {
		status = readl(ldo->base + PWRSWITCH_STATUS_REG);
		if (!(status & BHS_EN_REST_ACK_BIT))
			break;
		udelay(10);
		timeout_us -= 10;
	}

	if (timeout_us <= 0)
		dev_warn(ldo->dev, "BHS not off, PWRSWITCH_STATUS=0x%x\n",
			 status);

	/* Remove LDO bypass and expose it to the GDSC. */
	ctl &= ~LDO_BYPASS_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl &= ~ACK_SW_OVR_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl &= ~LDO_UNDER_SW_CTRL_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);

	return 0;
}

static int gfx_ldo_switch_to_bhs(struct qcom_gfx_ldo *ldo)
{
	u32 ctl, status;
	int timeout_us = LDO_MODE_SWITCH_TIMEOUT_US;

	ctl = readl(ldo->base + PWRSWITCH_CTRL_REG);

	/* Fake-ack the GDSC and select BHS as the power source. */
	ctl |= ACK_SW_OVR_BIT;
	ctl &= ~PWR_SRC_SEL_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);

	/* Keep the LDO warmed up while BHS takes over. */
	ctl &= ~LDO_PDN_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl |= LDO_UNDER_SW_CTRL_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);

	/* Power up BHS. */
	ctl |= BHS_EN_FEW_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl |= BHS_EN_REST_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	mb();

	while (timeout_us > 0) {
		status = readl(ldo->base + PWRSWITCH_STATUS_REG);
		if (status & BHS_EN_REST_ACK_BIT)
			break;
		udelay(10);
		timeout_us -= 10;
	}

	if (timeout_us <= 0)
		dev_warn(ldo->dev, "BHS not on, PWRSWITCH_STATUS=0x%x\n",
			 status);

	/* Bypass and pull down the LDO, expose BHS to the GDSC. */
	ctl |= LDO_BYPASS_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl |= LDO_PDN_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl &= ~ACK_SW_OVR_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);
	ctl &= ~BHS_UNDER_SW_CTL;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);

	mb();

	return 0;
}

static int qcom_gfx_ldo_enable(struct regulator_dev *rdev)
{
	struct qcom_gfx_ldo *ldo = rdev_get_drvdata(rdev);
	int ret = 0;

	mutex_lock(&ldo->lock);

	if (!ldo->bypass)
		gfx_ldo_enable_ldo(ldo, ldo->voltage_uv);
	else
		gfx_ldo_enable_bhs(ldo);

	ldo->enabled = true;

	mutex_unlock(&ldo->lock);
	return ret;
}

static int qcom_gfx_ldo_disable(struct regulator_dev *rdev)
{
	struct qcom_gfx_ldo *ldo = rdev_get_drvdata(rdev);

	mutex_lock(&ldo->lock);

	/* BHS/LDO gate control is handled by the GDSC; nothing to write. */
	ldo->enabled = false;

	mutex_unlock(&ldo->lock);
	return 0;
}

static int qcom_gfx_ldo_is_enabled(struct regulator_dev *rdev)
{
	struct qcom_gfx_ldo *ldo = rdev_get_drvdata(rdev);

	return ldo->enabled;
}

static int qcom_gfx_ldo_set_voltage(struct regulator_dev *rdev,
				    int min_uV, int max_uV,
				    unsigned int *selector)
{
	struct qcom_gfx_ldo *ldo = rdev_get_drvdata(rdev);
	int new_uv = clamp_val(min_uV, MIN_LDO_VOLTAGE, MAX_LDO_VOLTAGE), ret;

	mutex_lock(&ldo->lock);

	if (new_uv == ldo->voltage_uv)
		goto out;

	/*
	 * In bypass (BHS) mode the rail is not regulated by the LDO, so a
	 * voltage request is only stored; it is applied on the next switch
	 * to LDO mode.
	 */
	if (!ldo->enabled || ldo->bypass) {
		ldo->voltage_uv = new_uv;
		goto out;
	}

	ret = gfx_ldo_update_voltage(ldo, new_uv);
	if (ret)
		dev_err(ldo->dev, "failed to set voltage %d uV (%d)\n",
			new_uv, ret);
	mutex_unlock(&ldo->lock);
	return ret;

out:
	mutex_unlock(&ldo->lock);
	*selector = 0;
	return 0;
}

static int qcom_gfx_ldo_get_voltage(struct regulator_dev *rdev)
{
	struct qcom_gfx_ldo *ldo = rdev_get_drvdata(rdev);

	return ldo->voltage_uv;
}

static int qcom_gfx_ldo_set_bypass(struct regulator_dev *rdev, bool enable)
{
	struct qcom_gfx_ldo *ldo = rdev_get_drvdata(rdev);
	int ret = 0;

	mutex_lock(&ldo->lock);

	if (ldo->bypass == enable || !ldo->enabled)
		goto out;

	if (enable)
		ret = gfx_ldo_switch_to_bhs(ldo);
	else
		ret = gfx_ldo_switch_to_ldo(ldo, ldo->voltage_uv);

	if (!ret)
		ldo->bypass = enable;

out:
	mutex_unlock(&ldo->lock);
	return ret;
}

static int qcom_gfx_ldo_get_bypass(struct regulator_dev *rdev, bool *enable)
{
	struct qcom_gfx_ldo *ldo = rdev_get_drvdata(rdev);

	mutex_lock(&ldo->lock);
	*enable = ldo->bypass;
	mutex_unlock(&ldo->lock);

	return 0;
}

static const struct regulator_ops qcom_gfx_ldo_voltage_ops = {
	.enable		= qcom_gfx_ldo_enable,
	.disable	= qcom_gfx_ldo_disable,
	.is_enabled	= qcom_gfx_ldo_is_enabled,
	.set_voltage	= qcom_gfx_ldo_set_voltage,
	.get_voltage	= qcom_gfx_ldo_get_voltage,
	.set_bypass	= qcom_gfx_ldo_set_bypass,
	.get_bypass	= qcom_gfx_ldo_get_bypass,
};

static int qcom_gfx_ldo_init(struct qcom_gfx_ldo *ldo)
{
	u32 ctl;
	int i;

	/*
	 * HW initialization.  Clear the IO clamp and enable the
	 * CPR auto-bypass control, then write the per-CPU config table.
	 */
	ctl = readl(ldo->base + PWRSWITCH_CTRL_REG);
	ctl &= ~LDO_CLAMP_IO_BIT;
	ctl |= EN_LDOAP_CTRL_CPR_BIT;
	writel(ctl, ldo->base + PWRSWITCH_CTRL_REG);

	for (i = 0; sdm660_ldo_config[i].offset != QCOM_GFX_LDO_MAX_OFFSET; i++)
		writel(sdm660_ldo_config[i].value,
		       ldo->base + sdm660_ldo_config[i].offset);

	mb();

	return 0;
}

static int qcom_gfx_ldo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_init_data *init_data;
	struct regulator_config reg_config = {};
	struct qcom_gfx_ldo *ldo;
	int ret;

	ldo = devm_kzalloc(dev, sizeof(*ldo), GFP_KERNEL);
	if (!ldo)
		return -ENOMEM;

	ldo->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ldo->base))
		return PTR_ERR(ldo->base);

	ldo->dev = dev;
	ldo->voltage_uv = MIN_LDO_VOLTAGE;
	/* Stock configures the GFX corner in open-loop BHS mode. */
	ldo->bypass = true;
	mutex_init(&ldo->lock);

	ret = qcom_gfx_ldo_init(ldo);
	if (ret)
		return ret;

	init_data = of_get_regulator_init_data(dev, dev->of_node,
					       &ldo->desc);
	if (!init_data)
		return -EINVAL;

	ldo->desc.name = init_data->constraints.name;
	ldo->desc.type = REGULATOR_VOLTAGE;
	ldo->desc.owner = THIS_MODULE;
	ldo->desc.ops = &qcom_gfx_ldo_voltage_ops;

	init_data->constraints.valid_ops_mask |= REGULATOR_CHANGE_VOLTAGE |
						 REGULATOR_CHANGE_STATUS |
						 REGULATOR_CHANGE_BYPASS;

	reg_config.dev = dev;
	reg_config.init_data = init_data;
	reg_config.driver_data = ldo;
	reg_config.of_node = dev->of_node;

	ldo->rdev = devm_regulator_register(dev, &ldo->desc, &reg_config);
	if (IS_ERR(ldo->rdev))
		return dev_err_probe(dev, PTR_ERR(ldo->rdev),
				     "failed to register regulator\n");

	return 0;
}

static const struct of_device_id qcom_gfx_ldo_of_match[] = {
	{ .compatible = "qcom,sdm660-gfx-ldo" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_gfx_ldo_of_match);

static struct platform_driver qcom_gfx_ldo_driver = {
	.probe = qcom_gfx_ldo_probe,
	.driver = {
		.name = "qcom-gfx-ldo",
		.of_match_table = qcom_gfx_ldo_of_match,
	},
};
module_platform_driver(qcom_gfx_ldo_driver);

MODULE_DESCRIPTION("Qualcomm Technologies, Inc. SDM660 GFX LDO regulator driver");
MODULE_LICENSE("GPL v2");