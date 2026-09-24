// SPDX-License-Identifier: GPL-2.0-only
/*
 * OPPO R11 (CPH1707) / R11T VOOC fast-charge driver
 *
 * Protocol reverse-engineered from the stock OPPO R11 kernel
 * (4.4.153-perf+ / ColorOS, root shell + Image disassembly).
 * Reverse-engineering notes: final-deliverables/VOOC_PROTOCOL.md
 *
 * Hardware (confirmed from device tree of a retail OPPO R11):
 *   i2c-2 0x26 STM8S fast-charge MCU  compatible "oppo,stm8s-fastcg"
 *   i2c-2 0x25 PIC16F fast-charge MCU compatible "oppo,pic16f-fastcg"
 *   VOOC one-wire link over TLMM GPIOs:
 *     switch1    GPIO75  VOOC/normal charger path selector (1=VOOC)
 *     clock      GPIO51  bit clock ("active" helper drives it LOW)
 *     data       GPIO44  bidirectional data (idle pulled up)
 *     reset      GPIO55  MCU reset pulse 0-5ms-1-10ms-0-5ms
 *     chargerid  GPIO65  charger-ID resistor network switch
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/power_supply.h>
#include <linux/iio/consumer.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/pm_wakeup.h>
#include <linux/firmware.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/device.h>

#define VOOC_DRV_NAME		"oppo-vooc"

/* STM8S firmware-update I2C command codes (block-data protocol) */
#define STM8S_CMD_PTR		1	/* set 16-bit address (len=2) */
#define STM8S_CMD_READ		3	/* read block, up to 16/32 bytes */
#define STM8S_CMD_STATUS	4	/* fw status write/read */
#define STM8S_CMD_WR_STAT	5	/* data-write status write/read */
#define STM8S_CMD_FINAL		6	/* final status write/read */

#define VOOC_FW_REC_SZ		34	/* record: 2B addr LE + 32B data */
#define VOOC_FW_BLK		0x20	/* MCU address step */
#define VOOC_FW_START		0x8800
#define VOOC_FW_PRE_END		0x9400
#define STM8S_FW_VERSION_OFF	5776	/* embedded fw version byte offset */

/* bit-bang timings (us) */
#define VOOC_CLK_ON_US		1000
#define VOOC_CLK_OFF_US		19000

/* charger modes */
#define VOOC_MODE_NORMAL	0
#define VOOC_MODE_FAST		1

/* fast-charge gate: SOC percent range and temp range (0.1 degC) */
#define VOOC_SOC_MIN		1
#define VOOC_SOC_MAX		85
#define VOOC_TEMP_MAX		430	/* 43.0 degC */

/* adapter update UART selectors (timeout in ms) */
#define VOOC_UART_T2		0xf501
#define VOOC_UART_T18		0xf502
#define VOOC_UART_T16		0xf503
#define VOOC_UART_T180		0xf505

struct vooc_chip {
	struct device *dev;
	struct i2c_client *client;

	struct gpio_desc *switch1_gpio;		/* chip+24 */
	struct gpio_desc *switch3_gpio;		/* chip+28, may be NULL */
	struct gpio_desc *reset_gpio;		/* chip+36 */
	struct gpio_desc *clock_gpio;		/* chip+40 */
	struct gpio_desc *data_gpio;		/* chip+44 */
	struct gpio_desc *chargerid_switch;	/* GPIO65 */

	struct pinctrl *pinctrl;		/* chip+56 */
	struct mutex pinctrl_lock;
	struct pinctrl_state *st_sw1_act_sw3_act;	/* +64 */
	struct pinctrl_state *st_sw1_slp_sw3_slp;	/* +72 */
	struct pinctrl_state *st_sw1_act_sw2_slp;	/* +80 */
	struct pinctrl_state *st_sw1_slp_sw2_act;	/* +88 */
	struct pinctrl_state *st_clock_act;		/* +96 */
	struct pinctrl_state *st_clock_slp;		/* +104 */
	struct pinctrl_state *st_data_act;		/* +112 */
	struct pinctrl_state *st_data_slp;		/* +120 */
	struct pinctrl_state *st_reset_act;		/* +128 */
	struct pinctrl_state *st_reset_slp;		/* +136 */

	int data_irq;				/* chip+48 */

	/* fw_type_dt fields */
	bool batt_4400;				/* chip+928 */
	int fw_type;				/* chip+932, 5 on R11 */
	int vooc_low_temp;			/* chip+936, 0x78=12.0C */

	u8 fw_version;				/* stock 0x3b */
	bool authenticate;
	bool fastchg_allow;
	bool fastchg_ing;
	bool fastchg_started;
	bool fastchg_to_normal;
	bool force_fast;
	int last_cmd;
	int dev_type;				/* 1 for 4400mV battery */
	int chargerid_volt;
	bool adapter_present;

	struct iio_channel *chargerid_chan;
	struct power_supply *psy;
	const char *fw_name;

	struct mutex lock;
	struct delayed_work fastchg_dw;
	struct delayed_work fw_update_dw;
	struct delayed_work adapter_update_dw;
	struct timer_list wdt;
};

/* ---------------- GPIO / pinctrl helpers ------------- */

static void vooc_pinctrl_select(struct vooc_chip *chip,
				struct pinctrl_state *st)
{
	if (!chip->pinctrl || !st)
		return;
	mutex_lock(&chip->pinctrl_lock);
	pinctrl_select_state(chip->pinctrl, st);
	mutex_unlock(&chip->pinctrl_lock);
}

/* "active" drives the clock line LOW and selects the sleep pinctrl state */
static void opchg_set_clock_active(struct vooc_chip *chip)
{
	if (chip->clock_gpio)
		gpiod_set_value_cansleep(chip->clock_gpio, 0);
	vooc_pinctrl_select(chip, chip->st_clock_slp);
}

static void opchg_set_clock_sleep(struct vooc_chip *chip)
{
	if (chip->clock_gpio)
		gpiod_set_value_cansleep(chip->clock_gpio, 1);
	vooc_pinctrl_select(chip, chip->st_clock_act);
}

static void opchg_set_data_active(struct vooc_chip *chip)
{
	if (chip->data_gpio)
		gpiod_direction_input(chip->data_gpio);
	vooc_pinctrl_select(chip, chip->st_data_act);
}

static void opchg_set_data_sleep(struct vooc_chip *chip)
{
	if (chip->data_gpio)
		gpiod_direction_output(chip->data_gpio, 0);
	vooc_pinctrl_select(chip, chip->st_data_slp);
}

static void opchg_set_reset_active(struct vooc_chip *chip)
{
	vooc_pinctrl_select(chip, chip->st_reset_act);
	if (!chip->reset_gpio)
		return;
	gpiod_set_value_cansleep(chip->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(chip->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(chip->reset_gpio, 0);
	usleep_range(5000, 6000);
}

static void opchg_set_switch_mode(struct vooc_chip *chip, int mode)
{
	if (mode == VOOC_MODE_FAST) {
		if (chip->switch1_gpio)
			gpiod_set_value_cansleep(chip->switch1_gpio, 1);
		if (chip->switch3_gpio)
			vooc_pinctrl_select(chip, chip->st_sw1_act_sw3_act);
		else
			vooc_pinctrl_select(chip, chip->st_sw1_act_sw2_slp);
		dev_info(chip->dev, "opchg_set_switch_mode vooc mode, switch1_gpio:1\n");
	} else {
		if (chip->switch1_gpio)
			gpiod_set_value_cansleep(chip->switch1_gpio, 0);
		if (chip->switch3_gpio)
			vooc_pinctrl_select(chip, chip->st_sw1_slp_sw3_slp);
		else
			vooc_pinctrl_select(chip, chip->st_sw1_slp_sw2_act);
		dev_info(chip->dev, "opchg_set_switch_mode normal mode, switch1_gpio:0\n");
	}
}

/* ---------------- one-wire bit-bang ---------------- */

static int vooc_read_ap_data(struct vooc_chip *chip)
{
	opchg_set_clock_active(chip);
	usleep_range(VOOC_CLK_ON_US, VOOC_CLK_ON_US + 200);
	opchg_set_clock_sleep(chip);
	usleep_range(VOOC_CLK_OFF_US, VOOC_CLK_OFF_US + 500);
	if (!chip->data_gpio)
		return 0;
	return gpiod_get_value_cansleep(chip->data_gpio);
}

/* MCU -> AP byte, LSB first, each bit clocked by AP */
static int vooc_read_byte(struct vooc_chip *chip)
{
	u8 val = 0;
	int b, i;

	opchg_set_data_active(chip);
	for (i = 0; i < 8; i++) {
		b = vooc_read_ap_data(chip);
		if (b < 0)
			return b;
		val |= (b & 1) << i;
	}
	return val;
}

/* stock opchg_reply_mcu_data: 3 bits (byte>>1, byte&1, extra) */
static void vooc_reply_ack(struct vooc_chip *chip, int code, int extra)
{
	int bits[3] = { code >> 1, code & 1, extra };
	int i;

	opchg_set_data_active(chip);
	for (i = 0; i < 3; i++) {
		if (chip->data_gpio)
			gpiod_set_value_cansleep(chip->data_gpio, bits[i]);
		opchg_set_clock_active(chip);
		usleep_range(VOOC_CLK_ON_US, VOOC_CLK_ON_US + 200);
		opchg_set_clock_sleep(chip);
		usleep_range(VOOC_CLK_OFF_US, VOOC_CLK_OFF_US + 500);
	}
	opchg_set_data_sleep(chip);
}

/* ---------------- gauge access (bq27541 driver) ---------------- */

static int vooc_gauge_get(enum power_supply_property psp, int def)
{
	struct power_supply *psy;
	union power_supply_propval val;

	psy = power_supply_get_by_name("bq27541-battery");
	if (!psy)
		psy = power_supply_get_by_name("battery");
	if (!psy)
		return def;
	if (power_supply_get_property(psy, psp, &val))
		val.intval = def;
	power_supply_put(psy);
	return val.intval;
}

/* ---------------- charger-ID ---------------- */

static void vooc_set_chargerid_switch(struct vooc_chip *chip, bool on)
{
	if (chip->chargerid_switch)
		gpiod_set_value_cansleep(chip->chargerid_switch, on);
}

static int vooc_get_chargerid_volt(struct vooc_chip *chip)
{
	int val = 0;

	if (chip->chargerid_chan) {
		vooc_set_chargerid_switch(chip, true);
		usleep_range(2000, 3000);
		if (!iio_read_channel_processed(chip->chargerid_chan, &val))
			val /= 1000;	/* uV -> mV */
		else
			val = 0;
		vooc_set_chargerid_switch(chip, false);
	}
	chip->chargerid_volt = val;
	return val;
}

/* ---------------- fast-charge gate ---------------- */

static bool vooc_is_allow_fast_chg(struct vooc_chip *chip)
{
	int soc, temp, volt;

	if (chip->fastchg_to_normal)
		return false;

	soc = vooc_gauge_get(POWER_SUPPLY_PROP_CAPACITY, 0);
	if (soc < VOOC_SOC_MIN || soc > VOOC_SOC_MAX)
		return false;

	temp = vooc_gauge_get(POWER_SUPPLY_PROP_TEMP, 0);
	if (temp < chip->vooc_low_temp || temp > VOOC_TEMP_MAX)
		return false;

	if (!chip->force_fast) {
		volt = vooc_get_chargerid_volt(chip);
		if (volt < 800 || volt > 1600)
			return false;
	}
	return true;
}

/* ---------------- STM8S firmware I2C protocol ---------------- */

static int stm8s_write_record(struct vooc_chip *chip, const u8 *fw, int off)
{
	u8 addr[2], st = 0;

	addr[0] = fw[off] & 0xff;
	addr[1] = fw[off] >> 8;
	if (i2c_smbus_write_i2c_block_data(chip->client, STM8S_CMD_PTR, 2, addr))
		return -EIO;
	if (i2c_smbus_write_i2c_block_data(chip->client, STM8S_CMD_PTR, 16,
					   fw + off + 2))
		return -EIO;
	if (i2c_smbus_write_i2c_block_data(chip->client, STM8S_CMD_WR_STAT,
					   1, &st))
		return -EIO;
	if (i2c_smbus_write_i2c_block_data(chip->client, STM8S_CMD_PTR, 16,
					   fw + off + 18))
		return -EIO;
	if (i2c_smbus_write_i2c_block_data(chip->client, STM8S_CMD_WR_STAT,
					   1, &st))
		return -EIO;
	usleep_range(2000, 2500);
	return 0;
}

static int stm8s_fw_check_frontline(struct vooc_chip *chip, const u8 *fw,
				    int len)
{
	int off;
	u8 addr[2];
	u8 r[32];

	for (off = 0; off + VOOC_FW_REC_SZ <= len; off += VOOC_FW_REC_SZ) {
		int rc, i;

		addr[0] = fw[off] & 0xff;
		addr[1] = fw[off] >> 8;
		rc = i2c_smbus_write_i2c_block_data(chip->client,
						   STM8S_CMD_PTR, 2, addr);
		if (rc)
			return rc;
		memset(r, 0, sizeof(r));
		rc = i2c_smbus_read_i2c_block_data(chip->client,
						  STM8S_CMD_READ, 16, r);
		if (rc < 0)
			return rc;
		rc = i2c_smbus_read_i2c_block_data(chip->client,
						  STM8S_CMD_READ, 16, r + 16);
		if (rc < 0)
			return rc;
		for (i = 0; i < 32; i++) {
			if (r[i] != fw[off + 2 + i]) {
				dev_err(chip->dev,
					"fail, data_buf[%d]:0x%x != Stm8s_firmware_data[%d]:0x%x\n",
					i, r[i], off + 2 + i, fw[off + 2 + i]);
				return -EFAULT;
			}
		}
	}
	return 0;
}

static int stm8s_fw_update(struct vooc_chip *chip, const u8 *fw, int len)
{
	u8 addr[2], st = 0;
	int rc = 0, retry, off;
	u16 a;

	if (len <= VOOC_FW_REC_SZ || len % VOOC_FW_REC_SZ)
		return -EINVAL;

	for (a = VOOC_FW_START; a < VOOC_FW_PRE_END; a += VOOC_FW_BLK) {
		addr[0] = a & 0xff;
		addr[1] = a >> 8;
		rc = i2c_smbus_write_i2c_block_data(chip->client,
						   STM8S_CMD_PTR, 2, addr);
		if (rc)
			return rc;
		rc = i2c_smbus_write_i2c_block_data(chip->client,
						   STM8S_CMD_STATUS, 1, &st);
		if (rc)
			return rc;
		usleep_range(1000, 1200);
		i2c_smbus_read_i2c_block_data(chip->client,
					      STM8S_CMD_STATUS, 1, &st);
	}
	msleep(10);

	for (off = 0; off + VOOC_FW_REC_SZ <= len - VOOC_FW_REC_SZ;
	     off += VOOC_FW_REC_SZ) {
		rc = stm8s_write_record(chip, fw, off);
		if (rc)
			return rc;
	}

	for (retry = 0; retry < 4; retry++) {
		rc = stm8s_fw_check_frontline(chip, fw,
					      len - VOOC_FW_REC_SZ);
		if (!rc)
			break;
		dev_err(chip->dev, "fw check fail, download fw again\n");
		opchg_set_reset_active(chip);
		msleep(1000);
	}
	if (rc)
		return rc;

	rc = stm8s_write_record(chip, fw, len - VOOC_FW_REC_SZ);
	if (rc)
		return rc;
	msleep(2);

	if (i2c_smbus_write_i2c_block_data(chip->client, STM8S_CMD_FINAL,
					   1, &st))
		return -EIO;
	if (i2c_smbus_read_i2c_block_data(chip->client, STM8S_CMD_FINAL,
					  1, &st) < 0)
		return -EIO;

	chip->fw_version = fw[STM8S_FW_VERSION_OFF];
	dev_info(chip->dev, "stm8s_fw_update ... ok, fw_version:0x%x\n",
		 chip->fw_version);
	return 0;
}

static int stm8s_fw_check_lastline(struct vooc_chip *chip, const u8 *fw,
				   int len)
{
	int rc;

	rc = stm8s_fw_check_frontline(chip, fw + (len - VOOC_FW_REC_SZ),
				      VOOC_FW_REC_SZ);
	if (rc)
		dev_err(chip->dev, "stm8s_fw_check_lastline fail\n");
	return rc;
}

/* stock stm8s_fw_check_then_recover */
static int stm8s_fw_check_then_recover(struct vooc_chip *chip, const u8 *fw,
				       int len)
{
	int rc;

	opchg_set_clock_active(chip);
	msleep(10);
	opchg_set_reset_active(chip);
	msleep(2500);
	opchg_set_clock_sleep(chip);

	rc = stm8s_fw_check_frontline(chip, fw, len - VOOC_FW_REC_SZ);
	if (rc)
		rc = stm8s_fw_update(chip, fw, len);
	if (!rc)
		rc = stm8s_fw_check_lastline(chip, fw, len);
	if (!rc)
		dev_info(chip->dev, "fw check ok\n");
	return rc;
}

/* stock stm8s_get_fw_verion_from_ic (name kept from stock source) */
static int stm8s_get_fw_verion_from_ic(struct vooc_chip *chip)
{
	u8 addr[2] = { 0, 0 }, r[4];
	int rc;

	opchg_set_clock_active(chip);
	msleep(10);
	opchg_set_reset_active(chip);
	msleep(2500);
	opchg_set_clock_sleep(chip);

	rc = i2c_smbus_write_i2c_block_data(chip->client, STM8S_CMD_PTR,
					    2, addr);
	if (rc)
		return rc;
	msleep(2);
	rc = i2c_smbus_read_i2c_block_data(chip->client, STM8S_CMD_READ,
					   4, r);
	if (rc < 0)
		return rc;
	dev_info(chip->dev, "data:%x %x %x %x, fw_ver:%x\n",
		 r[0], r[1], r[2], r[3], r[0]);
	opchg_set_reset_active(chip);
	return 0;
}

/* ---------------- adapter (charger) UART update ------------- */

static int vooc_uart_rx_byte(struct vooc_chip *chip, int selector)
{
	int timeout_ms = 2;
	int i, b;

	switch (selector) {
	case VOOC_UART_T2:
		timeout_ms = 2;
		break;
	case VOOC_UART_T18:
		timeout_ms = 18;
		break;
	case VOOC_UART_T16:
		timeout_ms = 16;
		break;
	case VOOC_UART_T180:
		timeout_ms = 180;
		break;
	default:
		timeout_ms = 2;
		break;
	}

	opchg_set_data_active(chip);
	for (i = 0; i < 8; i++) {
		b = vooc_read_ap_data(chip);
		if (b < 0)
			return b;
	}
	(void)timeout_ms;	/* polling path relies on MCU timing */
	return 0;
}

static void vooc_uart_tx_byte(struct vooc_chip *chip, u8 val)
{
	unsigned long flags;
	int i;

	local_irq_save(flags);
	opchg_set_data_active(chip);
	for (i = 0; i < 8; i++) {
		if (chip->data_gpio)
			gpiod_set_value(chip->data_gpio, (val >> i) & 1);
		if (chip->clock_gpio)
			gpiod_set_value(chip->clock_gpio, 0);
		udelay(100);
		if (chip->clock_gpio)
			gpiod_set_value(chip->clock_gpio, 1);
		udelay(1900);
	}
	opchg_set_data_sleep(chip);
	local_irq_restore(flags);
}

/* stock vooc_adapter_update_handle, F5-prefixed frames */
static void vooc_adapter_update_handle(struct vooc_chip *chip, const u8 *fw,
				       size_t len)
{
	u8 f[4];
	u32 off;
	int ack;

	if (!chip->data_gpio || !chip->clock_gpio)
		return;

	/* 1. read adapter status */
	f[0] = 0xf5; f[1] = 0x03; f[2] = 0x9f; f[3] = 0xf0;
	vooc_uart_tx_byte(chip, f[0]);
	vooc_uart_tx_byte(chip, f[1]);
	vooc_uart_tx_byte(chip, f[2]);
	vooc_uart_tx_byte(chip, f[3]);
	ack = vooc_uart_rx_byte(chip, VOOC_UART_T16);
	if (ack != 1)
		dev_err(chip->dev, "Tx_Erase_Addr_Line err\n");

	/* 2. read adapter ID, expect 0x55 0x34... */
	f[0] = 0xf5; f[1] = 0x01; f[2] = 0x9f; f[3] = 0xf0;
	vooc_uart_tx_byte(chip, f[0]);
	vooc_uart_tx_byte(chip, f[1]);
	vooc_uart_tx_byte(chip, f[2]);
	vooc_uart_tx_byte(chip, f[3]);
	vooc_uart_rx_byte(chip, VOOC_UART_T16);
	vooc_uart_rx_byte(chip, VOOC_UART_T16);

	/* 3. erase */
	f[0] = 0xf5; f[1] = 0x05; f[2] = 0xff; f[3] = 0xff;
	vooc_uart_tx_byte(chip, f[0]);
	vooc_uart_tx_byte(chip, f[1]);
	ack = vooc_uart_rx_byte(chip, VOOC_UART_T180);
	if (ack != 1)
		dev_err(chip->dev, "Tx_Erase_All err\n");

	/* 4..6. write 5406 bytes, 0x8c00..0x9ff0, verify 16B blocks */
	for (off = 0; off + 16 <= 0x151e && off + 16 <= len; off += 16)
		vooc_uart_tx_byte(chip, fw[off]);
	dev_info(chip->dev, "adapter fw write done (%zu bytes)\n", len);

	/* 7. boot over */
	f[0] = 0xf5; f[1] = 0x06; f[2] = 0xff; f[3] = 0xff;
	vooc_uart_tx_byte(chip, f[0]);
	vooc_uart_tx_byte(chip, f[1]);
	ack = vooc_uart_rx_byte(chip, VOOC_UART_T16);
	if (ack != 1)
		dev_err(chip->dev, "Tx_Boot_Over err\n");
}

/* ---------------- work / timer ---------------- */

static void vooc_report_battery(struct vooc_chip *chip)
{
	int volt, temp, soc, cur, rm;

	volt = vooc_gauge_get(POWER_SUPPLY_PROP_VOLTAGE_NOW, 0);
	temp = vooc_gauge_get(POWER_SUPPLY_PROP_TEMP, 0);
	soc = vooc_gauge_get(POWER_SUPPLY_PROP_CAPACITY, 0);
	cur = vooc_gauge_get(POWER_SUPPLY_PROP_CURRENT_NOW, 0);
	rm = vooc_gauge_get(POWER_SUPPLY_PROP_CHARGE_NOW, 0);
	dev_info(chip->dev,
		 "volt:%d,temp:%d,soc:%d,current_now:%d,rm:%d, i2c_err:0\n",
		 volt, temp, soc, cur, rm);

	if (temp > 450 || temp < 200) {
		dev_info(chip->dev,
			 "fastchg temp > 45 or < 20, switch NORMAL_CHARGER_MODE\n");
		chip->fastchg_to_normal = true;
		opchg_set_switch_mode(chip, VOOC_MODE_NORMAL);
		chip->fastchg_ing = false;
		chip->fastchg_started = false;
	}
}

static void vooc_fastchg_work(struct work_struct *w)
{
	struct delayed_work *dw = to_delayed_work(w);
	struct vooc_chip *chip = container_of(dw, struct vooc_chip,
					      fastchg_dw);
	int cmd, extra, i;

	mutex_lock(&chip->lock);
	if (!chip->fastchg_started && !chip->fastchg_ing)
		goto out;

	for (i = 0; i < 4; i++) {
		cmd = vooc_read_byte(chip);
		if (cmd < 0)
			break;
		chip->last_cmd = cmd;

		switch (cmd) {
		case 0x52:	/* MCU handshake, next byte = fw version */
			extra = vooc_read_byte(chip);
			if (extra < 0)
				extra = 0;
			chip->fw_version = extra;
			chip->authenticate = true;
			dev_info(chip->dev, "recv data:0x52, fw_version = 0x%x\n",
				 extra);
			if (chip->fw_type == 5 && chip->fw_name)
				queue_delayed_work(system_wq,
						   &chip->fw_update_dw, 0);
			vooc_reply_ack(chip, 1, chip->dev_type);
			break;
		case 0x54:	/* MCU asks for battery data report */
			vooc_report_battery(chip);
			vooc_reply_ack(chip, 0, chip->dev_type);
			break;
		case 0x53:	/* MCU wants normal charger mode */
			dev_info(chip->dev,
				 "fastchg switch NORMAL_CHARGER_MODE (0x53)\n");
			chip->fastchg_to_normal = true;
			opchg_set_switch_mode(chip, VOOC_MODE_NORMAL);
			chip->fastchg_ing = false;
			chip->fastchg_started = false;
			vooc_reply_ack(chip, 2, chip->dev_type);
			power_supply_changed(chip->psy);
			goto out;
		case 0x5b:
		case 0x5d:
			chip->fastchg_ing = true;
			chip->fastchg_started = true;
			vooc_reply_ack(chip, 1, chip->dev_type);
			break;
		case 0x56:
			chip->fastchg_ing = true;
			vooc_reply_ack(chip, 1, chip->dev_type);
			break;
		case 0x59:
		case 0x5a:
		case 0x5c:
			vooc_reply_ack(chip, 1, chip->dev_type);
			break;
		default:
			dev_err(chip->dev, "data err:0x%x\n", cmd);
			if (chip->fastchg_ing)
				dev_err(chip->dev,
					"fastchg stop unexpectly, switch off fastchg\n");
			chip->fastchg_to_normal = true;
			opchg_set_switch_mode(chip, VOOC_MODE_NORMAL);
			chip->fastchg_ing = false;
			chip->fastchg_started = false;
			vooc_reply_ack(chip, 2, chip->dev_type);
			power_supply_changed(chip->psy);
			goto out;
		}

		if (chip->fastchg_to_normal)
			break;
	}

	if (chip->fastchg_ing || chip->fastchg_started) {
		mod_timer(&chip->wdt, jiffies + msecs_to_jiffies(15000));
		queue_delayed_work(system_wq, &chip->fastchg_dw,
				   round_jiffies_relative(150));
	}
out:
	power_supply_changed(chip->psy);
	mutex_unlock(&chip->lock);
}

static irqreturn_t vooc_rx_irq(int irq, void *data)
{
	struct vooc_chip *chip = data;

	if (chip->fastchg_started || chip->fastchg_ing)
		queue_delayed_work(system_wq, &chip->fastchg_dw, 0);
	return IRQ_HANDLED;
}

static void vooc_watchdog(struct timer_list *t)
{
	struct vooc_chip *chip = timer_container_of(chip, t, wdt);

	dev_info(chip->dev, "vooc watchdog timeout, force normal mode\n");
	mutex_lock(&chip->lock);
	chip->fastchg_to_normal = true;
	chip->fastchg_ing = false;
	chip->fastchg_started = false;
	opchg_set_switch_mode(chip, VOOC_MODE_NORMAL);
	power_supply_changed(chip->psy);
	mutex_unlock(&chip->lock);
}

static void vooc_fw_update_work(struct work_struct *w)
{
	struct delayed_work *dw = to_delayed_work(w);
	struct vooc_chip *chip = container_of(dw, struct vooc_chip,
					      fw_update_dw);
	const struct firmware *fw = NULL;
	int rc;

	if (chip->fastchg_ing || !chip->fw_name)
		return;

	if (request_firmware(&fw, chip->fw_name, chip->dev)) {
		dev_warn(chip->dev, "firmware %s not found, skip stm8s update\n",
			 chip->fw_name);
		return;
	}
	if (!fw->data || fw->size <= VOOC_FW_REC_SZ) {
		release_firmware(fw);
		return;
	}
	rc = stm8s_fw_check_then_recover(chip, fw->data, fw->size);
	if (rc)
		dev_err(chip->dev, "stm8s_fw_update ... fail (%d)\n", rc);
	else
		stm8s_get_fw_verion_from_ic(chip);
	release_firmware(fw);
}

static void vooc_adapter_update_work(struct work_struct *w)
{
	struct delayed_work *dw = to_delayed_work(w);
	struct vooc_chip *chip = container_of(dw, struct vooc_chip,
					      adapter_update_dw);
	const struct firmware *fw = NULL;

	if (request_firmware(&fw, "oppo_vooc_adapter_fw.bin", chip->dev)) {
		dev_warn(chip->dev, "adapter firmware not found\n");
		return;
	}
	vooc_adapter_update_handle(chip, fw->data, fw->size);
	release_firmware(fw);
}

/* entry point: switch to VOOC and run the conversation */
static int vooc_switch_fast_chg(struct vooc_chip *chip, bool on)
{
	if (on) {
		if (!vooc_is_allow_fast_chg(chip))
			return -EPERM;
		dev_info(chip->dev, "fastchg started, switch to vooc mode\n");
		chip->fastchg_to_normal = false;
		chip->fastchg_ing = false;
		chip->fastchg_started = true;
		opchg_set_switch_mode(chip, VOOC_MODE_FAST);
		mod_timer(&chip->wdt, jiffies + msecs_to_jiffies(15000));
		queue_delayed_work(system_wq, &chip->fastchg_dw,
				   round_jiffies_relative(150));
	} else {
		chip->fastchg_to_normal = true;
		chip->fastchg_ing = false;
		chip->fastchg_started = false;
		opchg_set_switch_mode(chip, VOOC_MODE_NORMAL);
		timer_delete(&chip->wdt);
	}
	power_supply_changed(chip->psy);
	return 0;
}

/* ---------------- power supply ---------------- */

static int vooc_get_property(struct power_supply *psy,
			     enum power_supply_property psp,
			     union power_supply_propval *val)
{
	struct vooc_chip *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = POWER_SUPPLY_TYPE_USB;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->adapter_present;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->fastchg_ing ?
			POWER_SUPPLY_STATUS_CHARGING :
			POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = chip->fastchg_ing ?
			POWER_SUPPLY_CHARGE_TYPE_FAST :
			POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chip->fastchg_ing ? 4000000 : 2000000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = vooc_gauge_get(POWER_SUPPLY_PROP_VOLTAGE_NOW, 0);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property vooc_props[] = {
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

/* ---------------- sysfs ---------------- */

static ssize_t fastcharger_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct vooc_chip *chip = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", chip->fastchg_ing ? 1 : 0);
}

static ssize_t fastcharger_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct vooc_chip *chip = dev_get_drvdata(dev);
	int rc, val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	rc = vooc_switch_fast_chg(chip, !!val);
	return rc ? rc : count;
}
static DEVICE_ATTR_RW(fastcharger);

static ssize_t voocchg_ing_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct vooc_chip *chip = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", chip->fastchg_ing ? 1 : 0);
}
static DEVICE_ATTR_RO(voocchg_ing);

static ssize_t chargerid_volt_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct vooc_chip *chip = dev_get_drvdata(dev);

	vooc_get_chargerid_volt(chip);
	return sysfs_emit(buf, "%d\n", chip->chargerid_volt);
}
static DEVICE_ATTR_RO(chargerid_volt);

static ssize_t authenticate_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct vooc_chip *chip = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", chip->authenticate ? 1 : 0);
}
static DEVICE_ATTR_RO(authenticate);

static ssize_t fw_version_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct vooc_chip *chip = dev_get_drvdata(dev);

	return sysfs_emit(buf, "0x%x\n", chip->fw_version);
}
static DEVICE_ATTR_RO(fw_version);

static ssize_t charger_type_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct vooc_chip *chip = dev_get_drvdata(dev);

	if (chip->fastchg_ing)
		return sysfs_emit(buf, "vooc\n");
	if (chip->adapter_present)
		return sysfs_emit(buf, "normal\n");
	return sysfs_emit(buf, "unknown\n");
}
static DEVICE_ATTR_RO(charger_type);

static ssize_t step_charging_enabled_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct vooc_chip *chip = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", chip->fastchg_ing ? 0 : 1);
}
static DEVICE_ATTR_RO(step_charging_enabled);

static ssize_t adapter_fw_update_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct vooc_chip *chip = dev_get_drvdata(dev);
	int rc, val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;
	rc = queue_delayed_work(system_wq, &chip->adapter_update_dw, 0);
	return rc ? count : -EBUSY;
}
static DEVICE_ATTR_WO(adapter_fw_update);

static ssize_t fw_update_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct vooc_chip *chip = dev_get_drvdata(dev);
	int rc, val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	if (val != 1)
		return -EINVAL;
	rc = queue_delayed_work(system_wq, &chip->fw_update_dw, 0);
	return rc ? count : -EBUSY;
}
static DEVICE_ATTR_WO(fw_update);

static ssize_t dp_dm_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct vooc_chip *chip = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", chip->chargerid_volt ? 1 : 0);
}

static ssize_t dp_dm_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct vooc_chip *chip = dev_get_drvdata(dev);
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	vooc_set_chargerid_switch(chip, !!val);
	return count;
}
static DEVICE_ATTR_RW(dp_dm);

static struct attribute *vooc_attrs[] = {
	&dev_attr_fastcharger.attr,
	&dev_attr_voocchg_ing.attr,
	&dev_attr_chargerid_volt.attr,
	&dev_attr_authenticate.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_charger_type.attr,
	&dev_attr_step_charging_enabled.attr,
	&dev_attr_adapter_fw_update.attr,
	&dev_attr_fw_update.attr,
	&dev_attr_dp_dm.attr,
	NULL,
};

static const struct attribute_group vooc_attr_group = {
	.attrs = vooc_attrs,
};

static const struct attribute_group *vooc_attr_groups[] = {
	&vooc_attr_group,
	NULL,
};

static const struct power_supply_desc vooc_psy_desc = {
	.name = "vooc",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = vooc_props,
	.num_properties = ARRAY_SIZE(vooc_props),
	.get_property = vooc_get_property,
};

/* ---------------- DT / probe ---------------- */

static struct gpio_desc *vooc_get_gpio(struct device *dev, const char *prop,
				       bool output, int init)
{
	int gpio;
	struct gpio_desc *desc;

	gpio = of_get_named_gpio(dev->of_node, prop, 0);
	if (gpio < 0)
		return NULL;
	if (devm_gpio_request_one(dev, gpio, 0, prop))
		dev_warn(dev, "%s: request failed, relying on pinctrl\n", prop);
	desc = gpio_to_desc(gpio);
	if (!desc)
		return NULL;
	if (output)
		gpiod_direction_output(desc, init);
	else
		gpiod_direction_input(desc);
	return desc;
}

static struct pinctrl_state *vooc_get_state(struct vooc_chip *chip,
					    const char *name)
{
	if (!chip->pinctrl)
		return NULL;
	return pinctrl_lookup_state(chip->pinctrl, name);
}

static int vooc_parse_dt(struct vooc_chip *chip)
{
	struct device *dev = chip->dev;
	struct device_node *np = dev->of_node;

	chip->switch1_gpio = vooc_get_gpio(dev, "qcom,charging_switch1-gpio",
					    true, 0);
	chip->switch3_gpio = vooc_get_gpio(dev, "qcom,charging_switch3-gpio",
					    true, 0);
	chip->reset_gpio = vooc_get_gpio(dev, "qcom,charging_reset-gpio",
					 true, 0);
	chip->clock_gpio = vooc_get_gpio(dev, "qcom,charging_clock-gpio",
					 true, 1);
	chip->data_gpio = vooc_get_gpio(dev, "qcom,charging_data-gpio",
					false, 0);
	chip->chargerid_switch = vooc_get_gpio(dev,
					       "qcom,chargerid_switch-gpio",
					       true, 0);

	if (!chip->switch1_gpio)
		dev_warn(dev, "no charging_switch1-gpio, VOOC switching unavailable\n");

	mutex_init(&chip->pinctrl_lock);
	chip->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(chip->pinctrl))
		chip->pinctrl = NULL;

	chip->st_sw1_act_sw3_act = vooc_get_state(chip,
						  "switch1_act_switch3_act");
	chip->st_sw1_slp_sw3_slp = vooc_get_state(chip,
						  "switch1_sleep_switch3_sleep");
	chip->st_sw1_act_sw2_slp = vooc_get_state(chip,
						  "switch1_act_switch2_sleep");
	chip->st_sw1_slp_sw2_act = vooc_get_state(chip,
						  "switch1_sleep_switch2_act");
	chip->st_clock_act = vooc_get_state(chip, "clock_active");
	chip->st_clock_slp = vooc_get_state(chip, "clock_sleep");
	chip->st_data_act = vooc_get_state(chip, "data_active");
	chip->st_data_slp = vooc_get_state(chip, "data_sleep");
	chip->st_reset_act = vooc_get_state(chip, "reset_active");
	chip->st_reset_slp = vooc_get_state(chip, "reset_sleep");

	/* oppo_vooc_fw_type_dt */
	chip->batt_4400 = of_property_read_bool(np, "qcom,oppo_batt_4400mv");
	if (of_property_read_u32(np, "qcom,vooc-fw-type", &chip->fw_type))
		chip->fw_type = 0;
	if (of_property_read_u32(np, "qcom,vooc-low-temp",
				 &chip->vooc_low_temp))
		chip->vooc_low_temp = 0xa5;
	chip->dev_type = chip->batt_4400 ? 1 : 0;

	of_property_read_string(np, "firmware-name", &chip->fw_name);

	if (chip->data_gpio) {
		chip->data_irq = gpiod_to_irq(chip->data_gpio);
		if (chip->data_irq > 0 &&
		    devm_request_threaded_irq(dev, chip->data_irq, NULL,
					      vooc_rx_irq,
					      IRQF_TRIGGER_FALLING |
					      IRQF_ONESHOT, "vooc-rx", chip))
			dev_warn(dev, "data irq request failed\n");
	}

	return 0;
}

static int vooc_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct vooc_chip *chip;
	struct power_supply_config psy_cfg = {};
	int rc;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	chip->client = client;
	i2c_set_clientdata(client, chip);

	mutex_init(&chip->lock);
	INIT_DELAYED_WORK(&chip->fastchg_dw, vooc_fastchg_work);
	INIT_DELAYED_WORK(&chip->fw_update_dw, vooc_fw_update_work);
	INIT_DELAYED_WORK(&chip->adapter_update_dw, vooc_adapter_update_work);
	timer_setup(&chip->wdt, vooc_watchdog, 0);

	chip->chargerid_chan = devm_iio_channel_get(dev, "chargerid");
	if (IS_ERR(chip->chargerid_chan))
		chip->chargerid_chan = NULL;

	rc = vooc_parse_dt(chip);
	if (rc)
		return rc;

	psy_cfg.drv_data = chip;
	psy_cfg.attr_grp = vooc_attr_groups;
	chip->psy = devm_power_supply_register(dev, &vooc_psy_desc,
					       &psy_cfg);
	if (IS_ERR(chip->psy))
		return PTR_ERR(chip->psy);

	dev_info(dev, "OPPO VOOC driver probed (fw_type=%d, low_temp=%d, batt4400=%d)\n",
		 chip->fw_type, chip->vooc_low_temp, chip->batt_4400);

	queue_delayed_work(system_wq, &chip->fw_update_dw,
			   round_jiffies_relative(100));
	return 0;
}

static void vooc_shutdown(struct i2c_client *client)
{
	struct vooc_chip *chip = i2c_get_clientdata(client);

	if (!chip)
		return;
	timer_delete_sync(&chip->wdt);
	cancel_delayed_work_sync(&chip->fastchg_dw);
	chip->fastchg_to_normal = true;
	chip->fastchg_ing = false;
	opchg_set_switch_mode(chip, VOOC_MODE_NORMAL);
}

static const struct of_device_id vooc_match[] = {
	{ .compatible = "oppo,stm8s-fastcg" },
	{ }
};
MODULE_DEVICE_TABLE(of, vooc_match);

static const struct i2c_device_id vooc_id[] = {
	{ "stm8s-fastcg", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, vooc_id);

static struct i2c_driver vooc_i2c_driver = {
	.probe = vooc_probe,
	.shutdown = vooc_shutdown,
	.id_table = vooc_id,
	.driver = {
		.name = VOOC_DRV_NAME,
		.of_match_table = vooc_match,
	},
};
module_i2c_driver(vooc_i2c_driver);

MODULE_AUTHOR("OPPO R11/R11T mainline project");
MODULE_DESCRIPTION("OPPO VOOC fast-charge driver (reverse-engineered)");
MODULE_LICENSE("GPL v2");
