// SPDX-License-Identifier: GPL-2.0
/*
 * OPPO R11T (Qualcomm SDM660) GPU "mem-acc" level register (E3 experiment).
 *
 * WHY THIS EXISTS
 * ---------------
 * The vendor GPU power recipe raises three things per GPU corner
 * (sdm660-regulator.dtsi, gfx_cpr@05061000 / thread@0 -> gfx_vreg_corner):
 *
 *     1. the GX rail voltage     qcom,cpr-voltage-ceiling
 *     2. the VDD_CX system level qcom,system-voltage
 *     3. the "mem-acc" level     qcom,mem-acc-voltage = <1 1 1 2 2 2 2>
 *
 * with qcom,corner-frequencies = <160e6 266e6 370e6 465e6 588e6 647e6 750e6>,
 * so from 465 MHz upwards the GPU memory path is meant to run one "mem-acc"
 * level higher than at the low corners.  This tree implements no per-corner
 * part of that recipe, and in particular nothing ever writes the mem-acc bit,
 * so the high GPU corners run one level below the vendor table.
 *
 * The register, from sdm660-regulator.dtsi:
 *
 *     gfx_mem_acc_vreg: regulator@01fcf004 {
 *             compatible = "qcom,mem-acc-regulator";
 *             reg = <0x01fcf004 0x4>;
 *             reg-names = "acc-sel-l1";
 *             regulator-min-microvolt = <1>;
 *             regulator-max-microvolt = <2>;
 *             qcom,corner-acc-map = <0x1 0x0>;
 *             qcom,acc-sel-l1-bit-pos = <0>;
 *             qcom,acc-sel-l1-bit-size = <1>;
 *     };
 *
 * i.e. one bit at bit 0 of 0x01fcf004: field value 0x1 (the corner-acc-map
 * entry for corner 1) selects mem-acc level 1 and 0x0 selects level 2.
 *
 * This driver is scaffolding for the E3 A/B experiment, not a production
 * model of the vendor regulator: it maps the register and exposes the level
 * through debugfs, nothing else.
 *
 * SAFE BY DEFAULT
 * ---------------
 * Loading the module reads and writes nothing -- the register is touched only
 * when userspace asks for it.  The interface is
 *
 *     cat  /sys/kernel/debug/qcom_r11t_gfx_mem_acc/raw    # read-only
 *     echo 2 > /sys/kernel/debug/qcom_r11t_gfx_mem_acc/level
 *     echo 1 > /sys/kernel/debug/qcom_r11t_gfx_mem_acc/level
 *
 * The first write remembers the value that was in the register, and unload
 * writes that baseline back, so rmmod restores the original state.
 *
 * Register access goes through ioremap(), never through a /dev/mem mapping:
 * an earlier attempt to poke a GFX register via /dev/mem hard-reset this
 * phone with an external abort (see the project log), so all register poking
 * in this project is done from a driver.
 */

#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/seq_file.h>
#include <linux/types.h>

#define DRV_NAME	"qcom_r11t_gfx_mem_acc"

/* Defaults are the SDM660 values; every one of them is a module parameter. */
static unsigned long reg_addr = 0x01fcf004;
module_param(reg_addr, ulong, 0444);
MODULE_PARM_DESC(reg_addr,
		 "acc-sel-l1 register physical address (default 0x01fcf004)");

static unsigned int reg_bit_pos;
module_param(reg_bit_pos, uint, 0444);
MODULE_PARM_DESC(reg_bit_pos, "qcom,acc-sel-l1-bit-pos (default 0)");

static unsigned int reg_bit_size = 1;
module_param(reg_bit_size, uint, 0444);
MODULE_PARM_DESC(reg_bit_size, "qcom,acc-sel-l1-bit-size (default 1)");

static unsigned int level1_val = 0x1;
module_param(level1_val, uint, 0444);
MODULE_PARM_DESC(level1_val,
		 "qcom,corner-acc-map entry for mem-acc level 1 (default 0x1)");

static unsigned int level2_val;
module_param(level2_val, uint, 0444);
MODULE_PARM_DESC(level2_val,
		 "qcom,corner-acc-map entry for mem-acc level 2 (default 0x0)");

static void __iomem *acc_base;
static struct dentry *acc_dir;

/* Register bookkeeping; nothing here is touched until userspace asks. */
static u32 baseline_val;	/* value found before our first write */
static bool baseline_valid;
static bool wrote;		/* we changed the register at least once */

static u32 acc_field_mask(void)
{
	return (reg_bit_size >= 32) ? ~0u : ((1u << reg_bit_size) - 1u);
}

static u32 acc_field(u32 raw)
{
	return (raw >> reg_bit_pos) & acc_field_mask();
}

static int acc_field_to_level(u32 field)
{
	if (field == (level2_val & acc_field_mask()))
		return 2;
	if (field == (level1_val & acc_field_mask()))
		return 1;

	return 0;
}

static int acc_set_level(unsigned int level)
{
	u32 prev, want, mask, raw;

	if (level != 1 && level != 2)
		return -EINVAL;

	prev = ioread32(acc_base);
	if (!baseline_valid) {
		baseline_val = prev;
		baseline_valid = true;
	}

	mask = acc_field_mask();
	want = (level == 1 ? level1_val : level2_val) & mask;
	raw = (prev & ~(mask << reg_bit_pos)) | (want << reg_bit_pos);

	if (raw != prev)
		iowrite32(raw, acc_base);

	wrote = true;

	pr_info(DRV_NAME ": mem-acc level %u (was %d): field 0x%x -> 0x%x, reg 0x%08x -> 0x%08x\n",
		level, acc_field_to_level(acc_field(prev)), acc_field(prev),
		want, prev, raw);

	return 0;
}

static int acc_raw_show(struct seq_file *s, void *unused)
{
	u32 raw = ioread32(acc_base);
	int level = acc_field_to_level(acc_field(raw));

	seq_printf(s, "reg       0x%08lx\n", reg_addr);
	seq_printf(s, "raw       0x%08x\n", raw);
	seq_printf(s, "field     0x%x (pos %u, size %u, mask 0x%x)\n",
		   acc_field(raw), reg_bit_pos, reg_bit_size, acc_field_mask());
	seq_printf(s, "level     %d%s\n", level, level ? "" : " (unrecognised)");
	seq_printf(s, "baseline  %s\n",
		   baseline_valid ? "known" : "not read yet");
	if (baseline_valid)
		seq_printf(s, "          0x%08x\n", baseline_val);
	seq_printf(s, "wrote     %s\n", wrote ? "yes" : "no");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(acc_raw);

static int acc_level_get(void *data, u64 *val)
{
	*val = acc_field_to_level(acc_field(ioread32(acc_base)));

	return 0;
}

static int acc_level_set(void *data, u64 val)
{
	return acc_set_level((unsigned int)val);
}
DEFINE_DEBUGFS_ATTRIBUTE(acc_level_fops, acc_level_get, acc_level_set, "%llu\n");

static int __init acc_init(void)
{
	if (reg_bit_size == 0 || reg_bit_pos + reg_bit_size > 32) {
		pr_err(DRV_NAME ": invalid field: pos %u size %u\n",
		       reg_bit_pos, reg_bit_size);
		return -EINVAL;
	}

	if ((level1_val & acc_field_mask()) ==
	    (level2_val & acc_field_mask())) {
		pr_err(DRV_NAME ": levels 1 and 2 map to the same field value\n");
		return -EINVAL;
	}

	acc_base = ioremap(reg_addr, 4);
	if (!acc_base)
		return -ENOMEM;

	acc_dir = debugfs_create_dir(DRV_NAME, NULL);
	debugfs_create_file("raw", 0444, acc_dir, NULL, &acc_raw_fops);
	debugfs_create_file("level", 0644, acc_dir, NULL, &acc_level_fops);

	pr_info(DRV_NAME ": mapped 0x%08lx (field pos %u size %u, level1=0x%x level2=0x%x); hardware untouched until 'level' is written\n",
		reg_addr, reg_bit_pos, reg_bit_size, level1_val, level2_val);

	return 0;
}

static void __exit acc_exit(void)
{
	if (wrote) {
		iowrite32(baseline_val, acc_base);
		pr_info(DRV_NAME ": baseline 0x%08x written back\n",
			baseline_val);
	}

	debugfs_remove_recursive(acc_dir);
	iounmap(acc_base);
}

module_init(acc_init);
module_exit(acc_exit);

MODULE_DESCRIPTION("OPPO R11T (SDM660) GPU mem-acc level register (E3 experiment)");
MODULE_LICENSE("GPL");
