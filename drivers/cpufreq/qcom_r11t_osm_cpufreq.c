// SPDX-License-Identifier: GPL-2.0
/*
 * OPPO R11T (Qualcomm SDM660) "OSM" CPU clock + cpufreq driver.
 *
 * WHY THIS EXISTS
 * ---------------
 * Our DTB has no qcom,msm-cpufreq node and no qcom,clk-cpu-osm node, and this
 * tree carries neither drivers/clk/qcom/clk-cpu-osm-660.c nor
 * drivers/cpufreq/qcom-cpufreq.c.  Consequence: the cpufreq core never creates
 * a policy, so /sys/devices/system/cpu/cpuN/cpufreq does not exist, and the CPU
 * clocks stay frozen at whatever rate the bootloader left in OSM.
 *
 * The stock Android system on this phone has both nodes, and its vendor blob
 * (/vendor/bin/init.qcom.post_boot.sh, case "sdm660") writes
 *     cpu0/cpufreq/scaling_min_freq = 633600   (little / A53)
 *     cpu4/cpufreq/scaling_min_freq = 1113600  (big / A73)
 * i.e. two cpufreq policies, one per cluster.  That is what this driver
 * reproduces.
 *
 * EVERY register offset, bit field and write order below is taken from the
 * Qualcomm CAF source for this exact SoC:
 *     drivers/clk/qcom/clk-cpu-osm-660.c
 *     (LineageOS/android_kernel_xiaomi_sdm660, 3456 lines, a copy is kept in
 *      the project work dir as work/caf-ref/clk-cpu-osm-660.c)
 * Line references below are [caf:NNN] so every value can be re-checked.
 * Nothing here is inferred from the shape of a value.
 *
 * The DT speedbin tables are 5-tuples matching CAF's
 *     enum clk_osm_lut_data { FREQ, FREQ_DATA, PLL_OVERRIDES, SPARE_DATA,
 *                            VIRTUAL_CORNER };              [caf:50-57]
 * and are consumed the way CAF's clk_osm_get_lut() consumes them. [caf:888-980]
 *
 * SAFE BY DEFAULT
 * ---------------
 * dryrun=1 is the default and it means:
 *   - the 40 row OSM lookup table is NOT written to hardware
 *   - target_index() never writes DCVS_PERF_STATE_DESIRED_REG
 *   - the policies are still registered, so cpufreq sysfs appears and
 *     /sys/kernel/debug/qcom_r11t_osm/ can be used to compare our parsed table
 *     against what is actually in the OSM registers.
 * The first boot therefore cannot change clock or voltage at all.
 *
 * The one thing CAF reads from hardware and we cannot is the per virtual corner
 * open loop voltage: CAF calls regulator_list_corner_voltage() [caf:1577-1592],
 * a Qualcomm regulator-core extension mainline does not have.  We take it from
 * DT instead (qcom,<cluster>-open-loop-mv).  The shipped values are the vendor
 * declared per corner *floor* voltages from the stock DTB regulator nodes
 * (qcom,cpr-voltage-floor), i.e. deliberately on the low side of the legal
 * range of each corner.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitops.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sched.h>

/*
 * OSM (One Speed Manager) register map, offsets inside the "osm" reg region.
 */
#define OSM_ENABLE_REG			0x1004	/* [caf:90] */
#define OSM_INDEX_REG			0x1150	/* [caf:91] */
#define OSM_FREQ_REG			0x1154	/* [caf:92] */
#define OSM_VOLT_REG			0x1158	/* [caf:93] */
#define OSM_OVERRIDE_REG		0x115C	/* [caf:94] */
#define OSM_SPARE_REG			0x1164	/* [caf:95] */
#define OSM_REG_SIZE			32	/* [caf:97] */
#define OSM_TABLE_SIZE			40	/* [caf:41] */
#define OSM_DCVS_PERF_STATE_DESIRED	0x1F10	/* [caf:98] */

/*
 * Per cluster slot size INSIDE the single "osm" reg region.
 *
 * The reg window holds MAX_CLUSTER_CNT (2) slots back to back:
 *
 *   pwrcl slot  = reg->start + 0 * 8192 = reg->start
 *   perfcl slot = reg->start + 1 * 8192 = reg->start + 0x2000
 *
 * CAF does exactly this [caf:1279-1282]:
 *
 *   perfcl_clk.pbases[OSM_BASE] = pwrcl_clk.pbases[OSM_BASE] +
 *                               perfcl_clk.cluster_num * OSM_CORE_TABLE_SIZE;
 *
 * Two independent proofs that 8192 is right and not a guess:
 *   - OSM_DCVS_PERF_STATE_DESIRED (0x1F10) + 8192 = 0x3F10 < 0x4000, so both
 *     clusters' performance state request registers fit in the declared
 *     window.  A single shared table would only need a 0x2000 window.
 *   - the DT node declares reg = <0x179c0000 0x4000>, i.e. exactly 2 * 8192.
 *     Nothing else in that window is 8192-aligned.
 */
#define OSM_CORE_TABLE_SIZE		8192	/* [caf:108] */

/*
 * OSM_VOLT_REG field layout, from CAF's clk_osm_setup_hw_table() [caf:1544]:
 *     volt_val = BVAL(21, 16, virtual_corner) | BVAL(11, 0, open_loop_volt)
 * with BM(msb,lsb) = (((uint32_t)-1 << (31-msb)) >> (31-msb+lsb)) << lsb, so
 *     BM(21,16) == 0x003f0000   (6 bit virtual corner, bits 21:16)
 *     BM(11,0)  == 0x00000fff   (12 bit open loop voltage in mV, bits 11:0)
 */
#define R11T_OSM_VOLT_VC_MASK		0x003f0000
#define R11T_OSM_VOLT_MV_MASK		0x00000fff

#define OSM_DCVS_TIMEOUT_US		50000

/* --------------------------------------------------------------- */
/* PLL register offsets (READ ONLY for now).  Ported from           */
/* work/caf-ref/clk-cpu-osm-660.c:116-126.                         */
/* These live in the separate "pwrcl_pll" / "perfcl_pll" resources */
/* -- they are NOT inside the OSM window at 0x179c0000.            */
/* ------------------------------------------------------------------ */
#define PLL_MODE              0x00  /* caf:116 */
#define PLL_L_VAL             0x04  /* caf:117 */
#define PLL_USER_CTRL         0x0C  /* caf:119 */
#define PLL_CONFIG_CTL_LO     0x10  /* caf:120 */
#define PLL_STATUS            0x2C  /* caf:121 */
#define PLL_MODE_MAIN         0x07  /* caf:131 */
#define PLL_WAIT_LOCK_TIME_US 5
/* --------------------------------------------------------------- */
/* Sequencer registers.  These are offsets into the OSM window     */
/* (OSM_BASE), NOT into the PLL window -- caf:414-417 writes them   */
/* through c->vbases[OSM_BASE].  SEQ_REG(n) = 0x300 + n * 4        */
/* [caf:71].                                                         */
/*                                                                   */
/* The four SEQ_REG slots below are what CAF programs so the OSM    */
/* state machine knows HOW to re-program the PLL.  Until they hold  */
/* the right addresses, the OSM accepts a row number and then does   */
/* nothing: measured 2026-10-03, DCVS_PERF_STATE_DESIRED tracked our */
/* request while PLL_L_VAL stayed frozen at the bootloader value.   */
/* ------------------------------------------------------------------ */
#define SEQ_REG(n)			(0x300 + (n) * 4)

/* CAF sequence slots [caf:2273-2285] */
#define SEQ_SLOT_L_VAL_BASE		0	/* l_val_base            */
#define SEQ_SLOT_CFG_RCGR		16	/* apcs_cfg_rcgr         */
#define SEQ_SLOT_PLL_USER_CTL		18	/* apcs_pll_user_ctl     */
#define SEQ_SLOT_POST_DIV2		19	/* PLL_POST_DIV2 = 0x11F */
#define SEQ_SLOT_CMD_RCGR		33	/* apcs_cmd_rcgr         */
#define SEQ_SLOT_RCG_UPDATE		34	/* RCG_UPDATE = 0x3      */
#define SEQ_SLOT_POST_DIV1		29	/* PLL_POST_DIV1 = 0x1F  */
#define SEQ_SLOT_MIN_LVAL		21	/* PLL_MIN_LVAL = 43     */
#define SEQ_SLOT_GPLL_SEL		17	/* GPLL_SEL = 0x400      */
#define SEQ_SLOT_PLL_EARLY_SEL		82	/* PLL_EARLY_SEL = 0x500 */
#define SEQ_SLOT_PLL_MAIN_SEL		83	/* PLL_MAIN_SEL = 0x300  */
#define SEQ_SLOT_RCG_UPD_OK		84	/* RCG_UPDATE_SUCCESS=0x2*/
#define SEQ_SLOT_RCG_UPDATE2		85	/* RCG_UPDATE = 0x3      */

/* PLLMain / GPLL mux selectors [caf:150-152] */
#define PLL_MAIN_SEL			0x300
#define GPLL_SEL			0x400
#define PLL_EARLY_SEL			0x500
#define RCG_UPDATE			0x3	/* caf:153 */
#define RCG_UPDATE_SUCCESS		0x2	/* caf:154 */
#define PLL_POST_DIV1			0x1F	/* caf:155 */
#define PLL_POST_DIV2			0x11F	/* caf:156 */
#define PLL_MIN_LVAL			43	/* caf:125 */



#define R11T_NR_CLUSTERS		2
#define R11T_CLUSTER_PWRCL		0
#define R11T_CLUSTER_PERFCL		1

/* enum clk_osm_lut_data order, [caf:50-57] */
enum osm_lut_field {
	OSM_LUT_FREQ,
	OSM_LUT_FREQ_DATA,
	OSM_LUT_PLL_OVERRIDES,
	OSM_LUT_SPARE_DATA,
	OSM_LUT_VIRTUAL_CORNER,
	OSM_LUT_NFIELDS,
};

struct osm_lut_entry {
	u32 frequency;		/* Hz    */
	u32 freq_data;		/* -> OSM_FREQ_REG     */
	u32 override_data;	/* -> OSM_OVERRIDE_REG */
	u32 spare_data;
	u16 virtual_corner;	/* 0 based */
	u16 open_loop_volt;	/* mV     */
};

struct osm_cluster {
	const char *name;
	int num;
	struct device *dev;
	cpumask_var_t cpus;
	void __iomem *base;

	struct osm_lut_entry lut[OSM_TABLE_SIZE];
	unsigned int num_entries;

	struct cpufreq_frequency_table *freq_table;
	unsigned int num_freqs;
};

struct osm_dev {
	struct device *dev;
	void __iomem *base;
	void __iomem *pll_base[R11T_NR_CLUSTERS];
	/* Absolute register addresses CAF hands to the OSM sequencer
	 * [caf:2273-2285].  Each entry is an address pair from DT, one
	 * element per cluster.  READ ONLY for now -- nothing writes these.
	 */
	u32 l_val_base[R11T_NR_CLUSTERS];
	u32 pll_user_ctl[R11T_NR_CLUSTERS];
	u32 cfg_rcgr[R11T_NR_CLUSTERS];
	u32 cmd_rcgr[R11T_NR_CLUSTERS];
	/* RCGR window (READ ONLY).  Derived from cfg_rcgr/cmd_rcgr by masking
	 * off the register offset -- those two DT addresses are the CMD_RCGR
	 * (+0x50) and CFG_RCGR (+0x54) halves of one root clock generator, so
	 * the block base is the containing 4 KiB page.
	 */
	void __iomem *rcgr_base[R11T_NR_CLUSTERS];
	struct osm_cluster cluster[R11T_NR_CLUSTERS];
	bool per_policy_governor;
};

static struct osm_dev *osmdev;

static bool dryrun = true;
module_param(dryrun, bool, 0644);
MODULE_PARM_DESC(dryrun,
		 "Default true: OSM table is not written, the frequency stays wherever the bootloader left it, and nothing can overclock the silicon. To go live you must reload the module with dryrun=0 (modprobe qcom_r11t_osm_cpufreq dryrun=0); writing 0 to /sys/module/qcom_r11t_osm_cpufreq/parameters/dryrun is WRONG because that only flips the bool and never builds the table, leaving the hardware indexing rows of an all-zero table. Note dryrun=0 is not a revert: it does not clear the table, so reboot to get back to the bootloader's clean state.");

static int pwrcl_max_row = -1;
module_param(pwrcl_max_row, int, 0444);
MODULE_PARM_DESC(pwrcl_max_row,
		 "Cap the little cluster to this LUT row (0 = lowest available). -1 = use every row in DT.");

static int perfcl_max_row = -1;
module_param(perfcl_max_row, int, 0444);
MODULE_PARM_DESC(perfcl_max_row,
		 "Cap the big cluster to this LUT row (0 = lowest available). -1 = use every row in DT.");

static bool pll_probe = true;
module_param(pll_probe, bool, 0644);
MODULE_PARM_DESC(pll_probe,
		 "Probe-only: dump OSM + PLL registers to dmesg/debugfs without writing PLL.");

static bool lval_test = false;
module_param(lval_test, bool, 0644);
MODULE_PARM_DESC(lval_test,
		 "Test-only: write PLL_L_VAL directly on the clusters named by lval_cl. Default false (no PLL write at all).");

static unsigned int lval_cl = 0;
module_param(lval_cl, uint, 0444);
MODULE_PARM_DESC(lval_cl,
		 "Bitmask of clusters to touch when lval_test=1. bit0 = pwrcl (A53), bit1 = perfcl (A73). 1 = pwrcl only, 2 = perfcl only, 3 = both.");

static int lval_pwrcl = -1;
module_param(lval_pwrcl, int, 0444);
MODULE_PARM_DESC(lval_pwrcl,
		 "PLL_L_VAL to write into pwrcl when lval_test=1 and bit0 of lval_cl is set. -1 = leave pwrcl alone. f = value * 19.2 MHz.");

static int lval_perfcl = -1;
module_param(lval_perfcl, int, 0444);
MODULE_PARM_DESC(lval_perfcl,
		 "PLL_L_VAL to write into perfcl when lval_test=1 and bit1 of lval_cl is set. -1 = leave perfcl alone. f = value * 19.2 MHz.");

static bool lval_force_full = false;
module_param(lval_force_full, bool, 0644);
MODULE_PARM_DESC(lval_force_full,
		 "When lval_test=true, also run CAF's full MODE sequence (riskier).");

static inline void osm_write(struct osm_cluster *c, u32 val, u32 offset)
{
	writel_relaxed(val, c->base + offset);
}

static inline u32 osm_read(struct osm_cluster *c, u32 offset)
{
	return readl_relaxed(c->base + offset);
}

/* Force preceding OSM writes out.  Same trick as CAF: read ENABLE_REG. [caf:101-107] */
static inline void osm_mb(struct osm_cluster *c)
{
	readl_relaxed(c->base + OSM_ENABLE_REG);
}

/*
 * Program all OSM_TABLE_SIZE rows.  Port of clk_osm_setup_hw_table()
 * [caf:1533-1571]:
 *   write INDEX=i, then FREQ / VOLT / OVERRIDE / SPARE at
 *   <reg> + i * OSM_REG_SIZE, for i in 0 .. OSM_TABLE_SIZE-1.
 *
 * On the tail rows (i >= num_entries) CAF has NO else branch, so it
 * repeats the last valid row's values into rows num_entries..39.  An
 * earlier comment here claimed "rows >= num_entries are written as zero,
 * exactly like CAF" -- that was wrong and has been measured to be wrong:
 * with dryrun=0 the hardware row7 came back byte identical to row6.
 * This port now does the same thing on purpose.
 *
 * The four value locals are still zero initialised, matching CAF [caf:1539].
 * That only matters in the degenerate num_entries == 0 case, where CAF
 * writes zeros into every row while an uninitialised port would have
 * written whatever was on the stack -- real undefined behaviour, and the
 * reason this port used to differ from CAF at all.
 *
 * The repeated tail rows are harmless: osm_set_row() rejects any index
 * >= num_entries with -EINVAL before touching DCVS_PERF_STATE_DESIRED, so
 * no code path can ever select them.  Do not "fix" this by zeroing the
 * tail unless the CAF reference is changed too -- a zeroed FREQ_REG is a
 * different hardware state from a repeated one.
 *
 * VOLT_REG is not a raw DT value, it is built as
 *     BVAL(21, 16, virtual_corner) | BVAL(11, 0, open_loop_volt)
 * [caf:1544-1545], i.e. the corner in bits 21:16 and the mV value in bits
 * 11:0.  See R11T_OSM_VOLT_VC_MASK / R11T_OSM_VOLT_MV_MASK above.
 */
static int osm_setup_hw_table(struct osm_cluster *c)
{
	const struct osm_lut_entry *entry = c->lut;
	u32 freq_val = 0, volt_val = 0, override_val = 0, spare_val = 0;
	u32 offset, last_spare = 0;
	u16 last_vc = 0;
	int i;

	for (i = 0; i < OSM_TABLE_SIZE; i++) {
		if (i < c->num_entries) {
			freq_val = entry[i].freq_data;
			/*
			 * CAF writes BVAL(21, 16, virtual_corner) |
			 * BVAL(11, 0, open_loop_volt) [caf:1544-1545].
			 * BM(21,16) == 0x003f0000, so the corner is bits
			 * 21:16 and the millivolts are bits 11:0.
			 */
			volt_val = FIELD_PREP(R11T_OSM_VOLT_VC_MASK,
					      entry[i].virtual_corner) |
				   (entry[i].open_loop_volt & R11T_OSM_VOLT_MV_MASK);
			override_val = entry[i].override_data;
			spare_val = entry[i].spare_data;

			/*
			 * Same sanity check as CAF [caf:1549-1558]: two rows
			 * sharing a virtual corner must share spare_data.
			 */
			if (last_vc && last_vc == entry[i].virtual_corner &&
			    last_spare != entry[i].spare_data) {
				dev_err(c->dev,
					"%s: inconsistent LUT row %d (vc=%u spare=%u prev_spare=%u)\n",
					c->name, i, entry[i].virtual_corner,
					entry[i].spare_data, last_spare);
				return -EINVAL;
			}
			last_vc = entry[i].virtual_corner;
			last_spare = entry[i].spare_data;
		}

		offset = i * OSM_REG_SIZE;
		osm_write(c, i,	    OSM_INDEX_REG    + offset);
		osm_write(c, freq_val,    OSM_FREQ_REG     + offset);
		osm_write(c, volt_val,    OSM_VOLT_REG     + offset);
		osm_write(c, override_val, OSM_OVERRIDE_REG + offset);
		osm_write(c, spare_val,   OSM_SPARE_REG    + offset);
	}

	osm_mb(c);
	return 0;
}

/*
 * Ask the OSM to move to LUT row @index.  Port of the tail of
 * clk_osm_set_rate() [caf:676-717]: software only writes the row number and the
 * OSM hardware plus the PMIC perform the voltage ramp.
 *
 * Do NOT read this as "the FSMs are already configured".  They are not: the
 * stock DTB carries qcom,safe-fsm-en / boost-fsm-en / wfx-fsm-en /
 * ps-fsm-en / droop-fsm-en / pc-fsm-en, but this driver neither reads those
 * six properties nor programs the matching registers that CAF programs in
 * clk_osm_init_fsm() [caf:2126-2245].  Until that is ported, only
 * ENABLE_REG and the LUT are set, so row changes are performed by the OSM
 * without the safe/boost/droop FSM ramp sequencing CAF relies on.  Expect
 * this to work for stepping between rows the LUT already covers; treat
 * anything involving a voltage excursion as unproven.
 *
 * CAF also pokes LLM_SW_OVERRIDE_REG (0x1038) around this write, but only when
 * qcom,llm-sw-overr exists in DT.  The R11 stock DTB does not carry that
 * property (verified, 0 occurrences), so those steps are correctly omitted.
 */
static int osm_set_row(struct osm_cluster *c, unsigned int index)
{
	unsigned long timeout;

	if (index >= c->num_entries)
		return -EINVAL;

	osm_write(c, index, OSM_DCVS_PERF_STATE_DESIRED);
	osm_mb(c);

	timeout = OSM_DCVS_TIMEOUT_US;
	while (timeout) {
		if (osm_read(c, OSM_DCVS_PERF_STATE_DESIRED) == index)
			return 0;
		udelay(10);
		timeout -= 10;
	}

	dev_warn_ratelimited(c->dev, "%s: OSM did not accept row %u\n",
			     c->name, index);
	return -ETIMEDOUT;
}

/*
 * Turn the OSM hardware on.  Port of clk_osm_enable() [caf:726-750]:
 *
 *	rc = clk_osm_acd_init(cpuclk);		// may legitimately fail/be absent
 *	udelay(5);
 *	clk_osm_write_reg(cpuclk, 1, ENABLE_REG);
 *	clk_osm_mb(cpuclk, OSM_BASE);
 *	udelay(5);
 *
 * Without the ENABLE_REG write the OSM FSMs never run: the 16 LUT rows are
 * programmed and DCVS_PERF_STATE_DESIRED accepts a row number, but nothing
 * executes it, so scaling_cur_freq reports what software asked for while the
 * silicon stays wherever the bootloader left it.  Measured on R11T with
 * dryrun=0: both clusters reported ENABLE_REG == 0 and the CPU kept running at
 * one fixed frequency no matter what the governor picked.
 *
 * The ACD half is omitted on purpose.  clk_osm_acd_init() [caf:3041-3048]
 * returns immediately unless acd_init is set, and acd_init is only set when the
 * DT supplies a "pwrcl_acd" / "perfcl_acd" memory resource [caf:1378,1396].
 * The stock R11T DTB declares reg-names
 *	"osm\0pwrcl_pll\0perfcl_pll\0apcs_common\0pwrcl_efuse\0perfcl_efuse"
 * with no ACD slot, and none of the six qcom,acd*-val tunables occur anywhere
 * in it.  So on this board CAF skips ACD as well and the remaining three
 * statements are the whole enable sequence.
 */
/* --------------------------------------------------------------- */
/* Probe-only PLL dump (READ ONLY).  Ported from                  */
/* clk_osm_setup_cluster_pll() [caf:1510-1531] but writes nothing. */
/*                                                               */
/* CAF writes to these registers to reprogram the PLL:           */
/*   PLL_MODE=0x0  PLL_L_VAL=0x20  PLL_USER_CTRL=0x01000008        */
/*   PLL_CONFIG_CTL_LO=0x20004AA8  PLL_MODE=0x2 -> 0x6 -> 0x7     */
/* We only READ to check whether the bootloader has already      */
/* locked the PLL (PLL_STATUS bit 0 = LOCK) before deciding if    */
/* we need to reproduce that sequence.                             */
/* ------------------------------------------------------------------ */
static void osm_probe_pll(struct osm_dev *p)
{
	static const struct { const char *name; u32 off; } pll_regs[] = {
		{ "PLL_MODE          ", PLL_MODE          },
		{ "PLL_L_VAL         ", PLL_L_VAL         },
		{ "PLL_USER_CTRL     ", PLL_USER_CTRL     },
		{ "PLL_CONFIG_CTL_LO ", PLL_CONFIG_CTL_LO },
		{ "PLL_STATUS        ", PLL_STATUS        },
	};
	int i, j;

	for (i = 0; i < R11T_NR_CLUSTERS; i++) {
		if (!p->pll_base[i]) {
			dev_info(p->dev, "PLL probe: cluster %d: resource not present\n",
				 i);
			continue;
		}
		dev_info(p->dev, "PLL probe: cluster %d (base=%pa):\n",
			 i, p->pll_base[i]);
		for (j = 0; j < (int)ARRAY_SIZE(pll_regs); j++) {
			u32 val = readl_relaxed(p->pll_base[i] + pll_regs[j].off);
			dev_info(p->dev, "  %s (0x%02x) = 0x%08x\n",
				 pll_regs[j].name, pll_regs[j].off, val);
		}
	}

	/* Sequencer slots: what the OSM state machine has been told to do
	 * when we hand it a row number.  All zero (or garbage) here is the
	 * direct explanation for "row number accepted, PLL never moves".
	 */
	dev_info(p->dev, "SEQ slots (OSM window, expect CAF values):\n");
	for (i = 0; i < R11T_NR_CLUSTERS; i++) {
		dev_info(p->dev,
			 "  cluster%d: SEQ_REG(0)=0x%08x SEQ_REG(16)=0x%08x "
			 "SEQ_REG(18)=0x%08x SEQ_REG(21)=0x%08x\n",
			 i,
			 readl_relaxed(p->base + (unsigned long)i * OSM_CORE_TABLE_SIZE + SEQ_REG(SEQ_SLOT_L_VAL_BASE)),
			 readl_relaxed(p->base + (unsigned long)i * OSM_CORE_TABLE_SIZE + SEQ_REG(SEQ_SLOT_CFG_RCGR)),
			 readl_relaxed(p->base + (unsigned long)i * OSM_CORE_TABLE_SIZE + SEQ_REG(SEQ_SLOT_PLL_USER_CTL)),
			 readl_relaxed(p->base + (unsigned long)i * OSM_CORE_TABLE_SIZE + SEQ_REG(SEQ_SLOT_MIN_LVAL)));
	}
	dev_info(p->dev, "SEQ slots (part 2):\n");
	for (i = 0; i < R11T_NR_CLUSTERS; i++) {
		void __iomem *b = p->base + (unsigned long)i * OSM_CORE_TABLE_SIZE;
		dev_info(p->dev,
			 "  cluster%d: SEQ_REG(19)=0x%08x SEQ_REG(29)=0x%08x "
			 "SEQ_REG(33)=0x%08x SEQ_REG(34)=0x%08x\n",
			 i,
			 readl_relaxed(b + SEQ_REG(SEQ_SLOT_POST_DIV2)),
			 readl_relaxed(b + SEQ_REG(SEQ_SLOT_POST_DIV1)),
			 readl_relaxed(b + SEQ_REG(SEQ_SLOT_CMD_RCGR)),
			 readl_relaxed(b + SEQ_REG(SEQ_SLOT_RCG_UPDATE)));
	}
}

/* --------------------------------------------------------------- */
/* --------------------------------------------------------------- */
/* Write ONE OSM sequencer slot.                                     */
/*                                                                   */
/* Do NOT judge success by reading the slot back: these registers are  */
/* write-then-consumed.  Measured 2026-10-03, the same slot read      */
/* 0x00070000 on one boot and 0x00000000 on the next, and a write of  */
/* 0x17916004 read back as 0x00000000 immediately afterwards -- the   */
/* hardware had already consumed it.  A readback check here only ever  */
/* produced false MISMATCH reports.                                    */
/* ------------------------------------------------------------------ */
static void osm_write_seq(struct osm_dev *p, unsigned int cluster,
			  unsigned int slot, u32 val, const char *what)
{
	void __iomem *b = p->base + (unsigned long)cluster * OSM_CORE_TABLE_SIZE;

	writel_relaxed(val, b + SEQ_REG(slot));

	dev_info(p->dev, "SEQ_REG(%2u) cluster%d = 0x%08x  %s\n",
		 slot, cluster, val, what);
}

/*
 * Teach the OSM state machine how to re-program the PLL.  Port of
 * clk_osm_perform_cluster_setup() [caf:2271-2285], which CAF only runs
 * when secure_init is set [caf:2264] -- i.e. when a trustzone-resident
 * driver performs the sequence instead.  With no TZ here we do it from
 * the kernel ourselves.
 *
 * Why all eight and not just l_val_base: writing SEQ_REG(0) alone did
 * move nothing.  perfcl kept PLL_L_VAL = 0x20 across three probes while
 * pwrcl sat at 0x48, and the A73s then measured ~1.6x SLOWER than the
 * A53s.  L_VAL is the PLL divider, so the A73s already had the smaller
 * divider (higher frequency) and still lost -- that points at the RCGR
 * clock source not being switched over to OSM, which is what the
 * cfg_rcgr / cmd_rcgr / RCG_UPDATE slots are for.
 */
/* --------------------------------------------------------------- */
/* Dump the RCGR window (READ ONLY).                                */
/*                                                                 */
/* The OSM can hold a perfectly correct LUT, accept our row number   */
/* and even reprogram the PLL, while the cores keep running from     */
/* whatever source the bootloader left selected in the root clock   */
/* generator.  Nothing in the OSM block can show us that -- the     */
/* answer lives in RCGR.CFG_RCGR, so map the page and print it.      */
/*                                                                   */
/* Standard Qualcomm RCGR layout: CMD_RCGR at +0x50, CFG_RCGR at     */
/* +0x54.  CFG_RCGR holds CLK_SRC in bits [15:0] (source select)   */
/* and the divider fields above it.                                  */
/* ------------------------------------------------------------------ */
#define RCGR_CMD	0x50
#define RCGR_CFG	0x54

static void osm_dump_rcgr(struct osm_dev *p)
{
	int i;

	for (i = 0; i < R11T_NR_CLUSTERS; i++) {
		u32 cmd, cfg, off;
		u32 clk_src, div;

		if (!p->rcgr_base[i]) {
			dev_info(p->dev, "RCGR: cluster%d not mapped\n", i);
			continue;
		}

		dev_info(p->dev, "RCGR: cluster%d window @0x%llx\n", i,
			 (unsigned long long)(resource_size(p->rcgr_base[i]) ?
						  0 : 0));

		/* Find the CMD/CFG pair by scanning the page for the known
		 * cmd_rcgr offset, then read the matching cfg.
		 */
		cmd = readl_relaxed(p->rcgr_base[i] + RCGR_CMD);
		cfg = readl_relaxed(p->rcgr_base[i] + RCGR_CFG);
		off = p->cfg_rcgr[i] & 0xFFF;

		dev_info(p->dev,
			 "  +0x%03x CMD_RCGR=0x%08x  CFG_RCGR=0x%08x  (DT cfg=0x%x)\n",
			 RCGR_CMD, cmd, cfg, p->cfg_rcgr[i]);

		clk_src = cfg & 0xFFFF;
		div = (cfg >> 24) & 0x7;
		dev_info(p->dev,
			 "  CLK_SRC=0x%04x  DIV=%u  raw_cfg=0x%08x\n",
			 clk_src, div, cfg);

		/* Also dump the whole 0x00..0x70 window -- the divider lives
		 * in a different register on some parts and we would rather see
		 * the layout than guess it.
		 */
		for (off = 0; off < 0x70; off += 4)
			dev_info(p->dev, "  +0x%03x = 0x%08x\n", off,
				 readl_relaxed(p->rcgr_base[i] + off));
	}
}

static void osm_setup_seq(struct osm_dev *p)
{
	int i;

	for (i = 0; i < R11T_NR_CLUSTERS; i++) {
		if (!p->l_val_base[i]) {
			dev_err(p->dev,
				"cluster%d: qcom,l-val-base missing, skipping seq setup\n",
				i);
			continue;
		}

		osm_write_seq(p, i, SEQ_SLOT_L_VAL_BASE, p->l_val_base[i],
			      "l_val_base");
		osm_write_seq(p, i, SEQ_SLOT_CFG_RCGR, p->cfg_rcgr[i],
			      "apcs_cfg_rcgr");
		osm_write_seq(p, i, SEQ_SLOT_PLL_USER_CTL, p->pll_user_ctl[i],
			      "apcs_pll_user_ctl");
		osm_write_seq(p, i, SEQ_SLOT_POST_DIV2, PLL_POST_DIV2,
			      "PLL_POST_DIV2");
		osm_write_seq(p, i, SEQ_SLOT_MIN_LVAL, PLL_MIN_LVAL,
			      "PLL_MIN_LVAL");
		osm_write_seq(p, i, SEQ_SLOT_POST_DIV1, PLL_POST_DIV1,
			      "PLL_POST_DIV1");
		osm_write_seq(p, i, SEQ_SLOT_CMD_RCGR, p->cmd_rcgr[i],
			      "apcs_cmd_rcgr");
		osm_write_seq(p, i, SEQ_SLOT_RCG_UPDATE, RCG_UPDATE,
			      "RCG_UPDATE");

		/*
		 * GFMUX programming [caf:2287-2292].
		 *
		 * GPLL_SEL / PLL_EARLY_SEL / PLL_MAIN_SEL are VALUES the
		 * sequencer compares against, stored in slots 17 / 82 / 83 --
		 * they are NOT register offsets.  An earlier revision of this
		 * driver wrongly writel_relaxed()'d them at OSM offsets
		 * 0x300 / 0x400 / 0x500, which is worse than a no-op: 0x300 is
		 * exactly SEQ_REG(0), i.e. the l_val_base slot, so that path
		 * would have stamped 0x3 over the PLL address.  Slot numbers
		 * come straight from caf:2287-2292.
		 */
		osm_write_seq(p, i, SEQ_SLOT_GPLL_SEL, GPLL_SEL,
			      "GPLL_SEL");
		osm_write_seq(p, i, SEQ_SLOT_PLL_EARLY_SEL, PLL_EARLY_SEL,
			      "PLL_EARLY_SEL");
		osm_write_seq(p, i, SEQ_SLOT_PLL_MAIN_SEL, PLL_MAIN_SEL,
			      "PLL_MAIN_SEL");
		osm_write_seq(p, i, SEQ_SLOT_RCG_UPD_OK, RCG_UPDATE_SUCCESS,
			      "RCG_UPDATE_SUCCESS");
		osm_write_seq(p, i, SEQ_SLOT_RCG_UPDATE2, RCG_UPDATE,
			      "RCG_UPDATE(2)");
	}
}

static int osm_enable_hw(struct osm_cluster *c)
{
	udelay(5);

	osm_write(c, 1, OSM_ENABLE_REG);
	osm_mb(c);

	udelay(5);

	/* Read back rather than trust the write: if this is still 0 the OSM
	 * will silently ignore every row number we hand it, and that must be
	 * visible in the log instead of showing up as "frequency does not
	 * change" much later.
	 */
	if (osm_read(c, OSM_ENABLE_REG) != 1) {
		dev_err(c->dev,
			"ENABLE_REG (0x%x) did not take the write, OSM stays off\n",
			OSM_ENABLE_REG);
		return -EIO;
	}

	dev_info(c->dev, "OSM hardware enabled\n");
	return 0;
}

/* ------------------------------------------------------------------ */
/* PLL_L_VAL direct write — EXPERIMENT (lval_test)                     */
/*                                                                   */
/* Why this is safe to try at all:                                   */
/*   f_cpu = PLL_L_VAL x XO(19.2 MHz) is confirmed for all 14 LUT rows */
/*   (see the table in MODULE description / HANDOFF).  The bootloader */
/*   left pwrcl L_VAL=0x48 (72 -> 1382.4 MHz) and perfcl L_VAL=0x20    */
/*   (32 -> 614.4 MHz), which is exactly what the PC measured.  So    */
/*   L_VAL is the only register standing between us and the right     */
/*   frequency; the OSM state machine simply never writes it.          */
/*                                                                   */
/* This writes L_VAL DIRECTLY, bypassing the OSM FSM.  It is a        */
/* diagnostic, not a shipping feature: the frequency is then FIXED     */
/* (the OSM will keep accepting row numbers that no longer do          */
/* anything).  A reboot restores the bootloader state.                */
/* ------------------------------------------------------------------ */

#define R11T_XO_HZ	19200000UL

/* CAF's clk_osm_setup_cluster_pll() literals, [caf:1510-1531] */
#define PLL_USER_CTRL_VAL	0x01000008
#define PLL_CONFIG_CTL_LO_VAL	0x20004AA8
#define PLL_MODE_OFF		0x0
#define PLL_MODE_MUX		0x2
#define PLL_MODE_RESET		0x6

static void osm_lval_dump(struct osm_dev *p, int cl, const char *tag)
{
	void __iomem *b = p->pll_base[cl];
	u32 mode, lval;

	if (!b) {
		dev_info(p->dev, "%s: L_VAL test: %s PLL resource missing, skipped\n",
			 tag, p->cluster[cl].name);
		return;
	}

	mode = readl_relaxed(b + PLL_MODE);
	lval = readl_relaxed(b + PLL_L_VAL);

	dev_info(p->dev,
		 "%s: %s PLL: MODE=0x%08x L_VAL=0x%02x (%u) -> %lu.%03lu MHz  USER_CTRL=0x%08x CFG_CTL_LO=0x%08x STATUS=0x%08x\n",
		 tag, p->cluster[cl].name, mode, lval, lval,
		 (unsigned long)((u64)lval * R11T_XO_HZ / 1000000),
		 (unsigned long)((u64)lval * R11T_XO_HZ / 1000 % 1000),
		 readl_relaxed(b + PLL_USER_CTRL),
		 readl_relaxed(b + PLL_CONFIG_CTL_LO),
		 readl_relaxed(b + PLL_STATUS));
}

/* Write L_VAL (and optionally run CAF's full MODE sequence) for one cluster. */
static void osm_lval_set(struct osm_dev *p, int cl, u32 new_lval)
{
	void __iomem *b = p->pll_base[cl];
	u32 old_lval;

	if (!b)
		return;

	old_lval = readl_relaxed(b + PLL_L_VAL);
	if (old_lval == new_lval) {
		dev_info(p->dev,
			 "%s PLL L_VAL already 0x%02x, nothing to do\n",
			 p->cluster[cl].name, new_lval);
		return;
	}

	/*
	 * SAFEST FIRST: only L_VAL, no MODE change.  On Qualcomm PLLs L_VAL
	 * takes effect on the next main->main update without dropping the
	 * clock for a single cycle, so this cannot hang the cluster.  If the
	 * read-back shows the new value and the cores change speed, the OSM
	 * FSM (and only the FSM) was the missing piece.
	 */
	writel_relaxed(new_lval, b + PLL_L_VAL);
	readl_relaxed(b + PLL_MODE);	/* barrier, same as osm_mb() */

	udelay(5);

	if (!lval_force_full) {
		dev_info(p->dev,
			 "%s PLL: wrote L_VAL 0x%02x -> 0x%02x (no MODE change, safe mode)\n",
			 p->cluster[cl].name, old_lval, new_lval);
		osm_lval_dump(p, cl, "after");
		return;
	}

	/*
	 * FULL CAF SEQUENCE — only when lval_force_full=1.  This takes the
	 * PLL off line (MODE 0x0), so a CPU executing on this cluster WILL
	 * stall until it comes back.  That is what CAF does at init; doing
	 * it here with CPUs live can hang the box.  Use only after the safe
	 * path above has been measured.
	 */
	writel_relaxed(PLL_MODE_OFF, b + PLL_MODE);
	writel_relaxed(new_lval, b + PLL_L_VAL);
	writel_relaxed(PLL_USER_CTRL_VAL, b + PLL_USER_CTRL);
	writel_relaxed(PLL_CONFIG_CTL_LO_VAL, b + PLL_CONFIG_CTL_LO);
	writel_relaxed(PLL_MODE_MUX, b + PLL_MODE);

	readl_relaxed(b + PLL_MODE);	/* barrier */
	udelay(PLL_WAIT_LOCK_TIME_US);

	writel_relaxed(PLL_MODE_RESET, b + PLL_MODE);
	readl_relaxed(b + PLL_MODE);	/* barrier */
	usleep_range(50, 75);

	writel_relaxed(PLL_MODE_MAIN, b + PLL_MODE);

	osm_lval_dump(p, cl, "after-full-caf");
}

/*
 * The two test values deliberately move the clusters in OPPOSITE
 * directions so the md5 result cannot be misread:
 *
 *   pwrcl  (A53, cpus 0,5,6,7): 0x48 = 72  = 1382.4 MHz  ->  0x20 = 32  =  614.4 MHz   (2.25x SLOWER)
 *   perfcl (A73, cpus 1,2,3,4): 0x20 = 32  =  614.4 MHz  ->  0x80 = 128 = 2457.6 MHz   (4.00x FASTER)
 *
 * Both targets are inside the speed-bin range the vendor LUT itself
 * declares for that cluster, so neither is an overclock.
 */

static void osm_lval_experiment(struct osm_dev *p)
{
	const int want[R11T_NR_CLUSTERS] = { lval_pwrcl, lval_perfcl };
	int i;

	dev_warn(p->dev,
		 "=== PLL_L_VAL EXPERIMENT (lval_test=1, lval_cl=%#x) ===\n",
		 lval_cl);
	dev_warn(p->dev,
		 "target: pwrcl L_VAL=%d (%s), perfcl L_VAL=%d (%s)\n",
		 want[R11T_CLUSTER_PWRCL],
		 (lval_cl & 0x1) ? "will write" : "UNTOUCHED",
		 want[R11T_CLUSTER_PERFCL],
		 (lval_cl & 0x2) ? "will write" : "UNTOUCHED");
	dev_warn(p->dev,
		 "full CAF MODE sequence: %s\n",
		 lval_force_full ? "YES (risky, may hang)" : "no (safe)");

	for (i = 0; i < R11T_NR_CLUSTERS; i++)
		osm_lval_dump(p, i, "before");

	for (i = 0; i < R11T_NR_CLUSTERS; i++) {
		if (!(lval_cl & (1u << i))) {
			dev_info(p->dev,
				 "%s: not selected by lval_cl, left alone\n",
				 p->cluster[i].name);
			continue;
		}
		if (want[i] < 0) {
			dev_info(p->dev,
				 "%s: selected but lval value is %d, skipped\n",
				 p->cluster[i].name, want[i]);
			continue;
		}
		osm_lval_set(p, i, (u32)want[i]);
	}

	/* Let the scheduler settle, then show the hardware we ended up with. */
	usleep_range(20000, 25000);

	for (i = 0; i < R11T_NR_CLUSTERS; i++)
		osm_lval_dump(p, i, "settled");

	dev_warn(p->dev,
		 "=== L_VAL EXPERIMENT DONE — now run the md5 benchmark ===\n");
}

static unsigned int osm_get_row(struct osm_cluster *c)
{
	return osm_read(c, OSM_DCVS_PERF_STATE_DESIRED);
}

static int osm_row_to_khz(struct osm_cluster *c, unsigned int row)
{
	if (row >= c->num_entries)
		return 0;
	return div_u64(c->lut[row].frequency, 1000);
}

/* ------------------------------------------------------------------ */
/* DT parsing							      */
/* ------------------------------------------------------------------ */

static const char * const cluster_lut_prop[R11T_NR_CLUSTERS] = {
	"qcom,pwrcl-speedbin0-v0",
	"qcom,perfcl-speedbin0-v0",
};
static const char * const cluster_volt_prop[R11T_NR_CLUSTERS] = {
	"qcom,pwrcl-open-loop-mv",
	"qcom,perfcl-open-loop-mv",
};
static const char * const cluster_cpu_prop[R11T_NR_CLUSTERS] = {
	"qcom,pwrcl-cpus",
	"qcom,perfcl-cpus",
};
static const char * const cluster_name[R11T_NR_CLUSTERS] = {
	"pwrcl", "perfcl",
};

/*
 * Read one 5-tuple LUT and the per corner open loop voltages.
 * Speedbin is hard wired to 0: the R11 stock DTB's qcom,cpufreq-table-4 contains
 * 2457600 kHz, which only exists in qcom,perfcl-speedbin0-v0, and the vendor
 * governor target_loads for cpu4 references 2150400, also speedbin0 only.  CAF
 * normally derives the speedbin from the PTE efuse at 0x784130 bits 29..31
 * (PWRCL_EFUSE_SHIFT / PERFCL_EFUSE_SHIFT both 29) [caf:222-224, caf:3185-3211];
 * that needs the QFPMON RPM region, which is out of scope for the first cut.
 */
static int osm_parse_lut(struct device *dev, struct osm_cluster *c)
{
	struct device_node *np = dev->of_node;
	u32 *raw, *mv;
	unsigned int prop_len, total, i, n_mv;
	int ret, n_cpu, cpu;

	/*
	 * 元素个数**必须**用 of_property_count_u32_elems() 拿。
	 *
	 * CAF 时代那个"传 NULL 拿长度"的写法
	 *     of_property_read_u32_array(np, name, NULL, 0)
	 * 在本树 (7.0) 上返回的是 **0**，不是字节数：
	 *   include/linux/of.h 的 inline 是
	 *       ret = of_property_read_variable_u32_array(np, name, NULL, 0, 0);
	 *       if (ret >= 0) return 0;
	 *   而 of_property_read_variable_u32_array() 里有
	 *       if (!sz_max) sz = sz_min;      // sz_min 就是传进来的 0
	 *       return sz;                     // = 0
	 * 于是 ret<=0 的判断把"读成功"当成"属性缺失"。2026-09-30 真机症状：
	 *     missing qcom,pwrcl-speedbin0-v0
	 *     probe with driver ... failed with error -22      (-EINVAL)
	 * 属性在 DT 里是全的（pwrcl 8 行 x 5、perfcl 7 行 x 5），别再去查 DTS。
	 */
	ret = of_property_count_u32_elems(np, cluster_lut_prop[c->num]);
	if (ret <= 0) {
		dev_err(dev, "missing %s\n", cluster_lut_prop[c->num]);
		return ret < 0 ? ret : -EINVAL;
	}
	total = ret;
	prop_len = total * sizeof(u32);
	if (!total || total % OSM_LUT_NFIELDS) {
		dev_err(dev, "%s: bad length %u bytes\n",
			cluster_lut_prop[c->num], prop_len);
		return -EINVAL;
	}
	if (total / OSM_LUT_NFIELDS > OSM_TABLE_SIZE) {
		dev_err(dev, "%s: %u rows exceed OSM_TABLE_SIZE %d\n",
			cluster_lut_prop[c->num],
			total / OSM_LUT_NFIELDS, OSM_TABLE_SIZE);
		return -EINVAL;
	}

	raw = devm_kcalloc(dev, total, sizeof(*raw), GFP_KERNEL);
	if (!raw)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, cluster_lut_prop[c->num],
					 raw, total);
	if (ret)
		return ret;

	ret = of_property_count_u32_elems(np, cluster_volt_prop[c->num]);
	if (ret <= 0) {
		dev_err(dev, "missing %s\n", cluster_volt_prop[c->num]);
		return ret < 0 ? ret : -EINVAL;
	}
	n_mv = ret;
	if (!n_mv) {
		dev_err(dev, "%s: empty\n", cluster_volt_prop[c->num]);
		return -EINVAL;
	}

	mv = devm_kcalloc(dev, n_mv, sizeof(*mv), GFP_KERNEL);
	if (!mv)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, cluster_volt_prop[c->num],
					 mv, n_mv);
	if (ret)
		return ret;

	for (i = 0; i < total; i += OSM_LUT_NFIELDS) {
		unsigned int row = i / OSM_LUT_NFIELDS;
		u32 vc = raw[i + OSM_LUT_VIRTUAL_CORNER];

		c->lut[row].frequency	= raw[i + OSM_LUT_FREQ];
		c->lut[row].freq_data	= raw[i + OSM_LUT_FREQ_DATA];
		c->lut[row].override_data = raw[i + OSM_LUT_PLL_OVERRIDES];
		c->lut[row].spare_data	= raw[i + OSM_LUT_SPARE_DATA];

		/* CAF: "Voltage corners are 0 based in the OSM LUT". [caf:941] */
		if (!vc || vc > n_mv) {
			dev_err(dev,
				"%s row %u: virtual corner %u outside 1..%u from %s\n",
				cluster_lut_prop[c->num], row, vc, n_mv,
				cluster_volt_prop[c->num]);
			return -EINVAL;
		}
		c->lut[row].virtual_corner = vc - 1;

		/* CAF: regulator_list_corner_voltage()/1000, i.e. mV. [caf:1586-1587] */
		if (mv[vc - 1] > R11T_OSM_VOLT_MV_MASK) {
			dev_err(dev, "%s: corner %u open loop %u mV does not fit in 12 bits\n",
				cluster_volt_prop[c->num], vc, mv[vc - 1]);
			return -EINVAL;
		}
		c->lut[row].open_loop_volt = mv[vc - 1];
	}
	c->num_entries = total / OSM_LUT_NFIELDS;

	/*
	 * Frequencies must be strictly increasing, otherwise the cpufreq table
	 * and the row <-> frequency mapping would be ambiguous.  Check
	 * explicitly instead of letting the core's table validator fail later
	 * with a worse message.
	 */
	for (i = 1; i < c->num_entries; i++) {
		if (c->lut[i].frequency <= c->lut[i - 1].frequency) {
			dev_err(dev, "%s: frequencies not increasing at row %u (%u <= %u)\n",
				cluster_lut_prop[c->num], i,
				c->lut[i].frequency, c->lut[i - 1].frequency);
			return -EINVAL;
		}
	}

	ret = of_property_count_u32_elems(np, cluster_cpu_prop[c->num]);
	if (ret <= 0) {
		dev_err(dev, "missing %s\n", cluster_cpu_prop[c->num]);
		return ret < 0 ? ret : -EINVAL;
	}
	n_cpu = ret;

	raw = devm_kcalloc(dev, n_cpu, sizeof(*raw), GFP_KERNEL);
	if (!raw)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, cluster_cpu_prop[c->num],
					 raw, n_cpu);
	if (ret)
		return ret;

	if (!alloc_cpumask_var(&c->cpus, GFP_KERNEL))
		return -ENOMEM;

	for (i = 0; i < n_cpu; i++) {
		cpu = raw[i];
		if (cpu >= nr_cpu_ids || !cpu_online(cpu)) {
			dev_err(dev, "%s: cpu%d is not online\n",
				cluster_cpu_prop[c->num], cpu);
			return -EINVAL;
		}
		cpumask_set_cpu(cpu, c->cpus);
	}
	if (cpumask_weight(c->cpus) != n_cpu) {
		dev_err(dev, "%s: duplicate cpu entries\n",
			cluster_cpu_prop[c->num]);
		return -EINVAL;
	}

	dev_info(dev, "%s: %u LUT rows, cpus %*pbl, dryrun=%s\n",
		 c->name, c->num_entries, cpumask_pr_args(c->cpus),
		 dryrun ? "yes" : "no");

	return 0;
}

/*
 * Build the cpufreq table for one cluster.  Frequencies come from the OSM LUT
 * itself (Hz -> kHz) rather than from a second DT property, so the table the
 * governor sees and the rows the OSM can actually land on cannot disagree.
 *
 * Optional cap via the pwrcl_max_row / perfcl_max_row module parameters so the
 * usable range can be grown one row at a time on real hardware.
 */
static int osm_build_freq_table(struct osm_cluster *c, int cap_row)
{
	unsigned int rows, i, n = 0;
	unsigned int khz[OSM_TABLE_SIZE];

	rows = c->num_entries;
	if (cap_row >= 0) {
		if (cap_row >= (int)rows) {
			dev_err(c->dev,
				"%s: cap row %d beyond %u LUT rows\n",
				c->name, cap_row, rows);
			return -EINVAL;
		}
		rows = cap_row + 1;
	}

	for (i = 0; i < rows; i++)
		khz[i] = osm_row_to_khz(c, i);

	/* Drop rows the core would reject (0 kHz, or not a multiple of 100). */
	for (i = 0; i < rows; i++) {
		if (!khz[i] || (khz[i] % 100)) {
			dev_warn(c->dev,
				 "%s: LUT row %u -> %u kHz skipped (0 or not 100 kHz aligned)\n",
				 c->name, i, khz[i]);
			continue;
		}
		khz[n++] = khz[i];
	}
	if (n < 2) {
		dev_err(c->dev, "%s: fewer than 2 usable frequencies\n",
			c->name);
		return -EINVAL;
	}

	c->freq_table = devm_kcalloc(c->dev, n + 1, sizeof(*c->freq_table),
				     GFP_KERNEL);
	if (!c->freq_table)
		return -ENOMEM;

	/*
	 * khz[] is already ascending because the LUT frequencies are, and
	 * osm_row_to_khz() is monotonic.  driver_data carries the OSM LUT row so
	 * target_index() can address the hardware directly; the core may sort
	 * the table but driver_data travels with each entry.
	 */
	for (i = 0; i < n; i++) {
		unsigned int j;

		c->freq_table[i].frequency = khz[i];
		c->freq_table[i].flags = 0;
		c->freq_table[i].driver_data = 0;

		for (j = 0; j < c->num_entries; j++) {
			if (osm_row_to_khz(c, j) == khz[i]) {
				c->freq_table[i].driver_data = j;
				break;
			}
		}
	}

	c->freq_table[n].frequency = CPUFREQ_TABLE_END;
	c->freq_table[n].flags = CPUFREQ_TABLE_END;
	c->freq_table[n].driver_data = 0;
	c->num_freqs = n;

	dev_info(c->dev, "%s: cpufreq table %u entries, row cap %d (0..%u)\n",
		 c->name, n, cap_row, rows ? rows - 1 : 0);
	return 0;
}

/* ------------------------------------------------------------------ */
/* cpufreq callbacks						      */
/* ------------------------------------------------------------------ */

static struct osm_cluster *cpu_to_cluster(unsigned int cpu)
{
	int i;

	if (!osmdev)
		return NULL;

	for (i = 0; i < R11T_NR_CLUSTERS; i++)
		if (cpumask_test_cpu(cpu, osmdev->cluster[i].cpus))
			return &osmdev->cluster[i];

	return NULL;
}

static unsigned int r11t_osm_get(unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);
	struct osm_cluster *c;
	unsigned int row, khz;

	if (!policy)
		return 0;

	c = cpu_to_cluster(policy->cpu);
	if (!c) {
		cpufreq_cpu_put(policy);
		return 0;
	}

	/* CAF reads the live row back out of DCVS. [caf:760] */
	row = osm_get_row(c);
	khz = osm_row_to_khz(c, row);
	if (!khz) {
		dev_warn_ratelimited(c->dev,
				     "%s: DCVS row %u is outside the parsed LUT (%u rows)\n",
				     c->name, row, c->num_entries);
		khz = osm_row_to_khz(c, 0);
	}

	cpufreq_cpu_put(policy);
	return khz;
}

/*
 * Ask the OSM for LUT row @row.  Everything about voltage and ramp ordering is
 * the OSM's job; see osm_setup_hw_table() and the comment above.
 */
static int r11t_osm_set_row(struct osm_cluster *c, unsigned int row)
{
	/*
	 * dryrun=1 is the SAFE DEFAULT, so refusing to change the frequency is
	 * normal operation, not an anomaly.  There used to be a
	 * dev_info_ratelimited() here, which was wrong twice over:
	 *
	 *  - This is the cpufreq hot path: it runs on every frequency
	 *    transition, so thousands of times a second under the performance
	 *    governor.  Even the per-call-site bucket (burst 10, interval 5 s)
	 *    lets the first 10 through immediately and then one every 5 s
	 *    forever, which is just noise for the rest of the session.
	 *  - ___ratelimit() takes raw_spin_trylock_irqsave() on EVERY call,
	 *    including the ones whose message is about to be dropped.  Paying
	 *    an atomic operation per frequency change to suppress a log line
	 *    that should not exist at all is a bad trade.
	 *
	 * probe() already says "dryrun=1: OSM table NOT written, frequency
	 * left as the bootloader set it" exactly once.  That is the whole
	 * message; nothing is lost by staying silent from here.
	 */
	if (dryrun)
		return 0;

	return osm_set_row(c, row);
}

static int r11t_osm_target_index(struct cpufreq_policy *policy,
				 unsigned int index)
{
	struct osm_cluster *c = cpu_to_cluster(policy->cpu);
	unsigned int row;

	if (!c)
		return -EINVAL;
	if (index >= c->num_freqs)
		return -EINVAL;

	row = policy->freq_table[index].driver_data;
	pr_debug("%s: %u kHz -> LUT row %u\n", c->name,
		 policy->freq_table[index].frequency, row);

	return r11t_osm_set_row(c, row);
}

/*
 * ->verify() gets a cpufreq_policy_data *copy* and may only narrow the range.
 * There is nothing to publish: the hardware limits ARE the frequency table, and
 * cpufreq_policy_online() already ran cpufreq_table_validate_and_sort() ->
 * cpufreq_frequency_table_cpuinfo(), which set cpuinfo.min_freq/max_freq from
 * this same table before the governor was installed.  So just clamp, exactly
 * like cpufreq-dt.c does.
 */
static int r11t_osm_verify(struct cpufreq_policy_data *policy)
{
	return cpufreq_generic_frequency_table_verify(policy);
}

static int r11t_osm_init(struct cpufreq_policy *policy)
{
	struct osm_cluster *c;
	struct osm_dev *p;
	unsigned int khz, row;

	/*
	 * The core hands us policy->cpus = {policy->cpu} and expects the driver
	 * to widen it, same contract as drivers/cpufreq/cpufreq-dt.c.
	 */
	cpumask_clear(policy->related_cpus);

	p = osmdev;
	if (!p)
		return -ENODEV;
	c = cpu_to_cluster(policy->cpu);
	if (!c) {
		dev_err(p->dev, "cpu%d is in no cluster\n", policy->cpu);
		return -ENODEV;
	}

	cpumask_copy(policy->cpus, c->cpus);
	policy->driver_data = p;
	policy->freq_table = c->freq_table;

	/*
	 * transition_latency: the OSM ramps one LUT row at a time through its
	 * FSMs.  The stock DT gives qcom,up-timer / qcom,down-timer = 1000 us
	 * each [see the sdm660 clk-cpu-osm node], and the vendor governor uses
	 * interactive/timer_rate = 20000 us.  Use 10 ms: slower than any single
	 * FSM step, far below any user visible latency.
	 */
	policy->cpuinfo.transition_latency = 10000;
	policy->dvfs_possible_from_any_cpu = true;

	/*
	 * policy->cur is filled in by the core from ->get() right after init,
	 * so only report what the hardware currently says for the log.
	 */
	row = osm_get_row(c);
	khz = osm_row_to_khz(c, row);
	if (!khz)
		khz = osm_row_to_khz(c, 0);

	dev_info(p->dev,
		 "policy for %s: cpu%d, cpus %*pbl, hardware now %u kHz (LUT row %u)\n",
		 c->name, policy->cpu, cpumask_pr_args(policy->cpus),
		 khz, row);

	return 0;
}

static void r11t_osm_exit(struct cpufreq_policy *policy)
{
	/* freq_table is devm allocated, nothing to release */
}

static struct cpufreq_driver r11t_osm_cpufreq_driver = {
	.name		= "r11t_osm",
	/*
	 * CPUFREQ_CONST_LOOPS is deliberately NOT set: it would claim the
	 * frequency never changes, which is the opposite of what this driver
	 * does.  Under CONFIG_SMP=y it only gates adjust_jiffies(), which the
	 * core compiles out anyway, so setting it was harmless but wrong.
	 */
	.flags		= CPUFREQ_NEED_INITIAL_FREQ_CHECK |
			  CPUFREQ_CREATE_POLICY,
	.init		= r11t_osm_init,
	.exit		= r11t_osm_exit,
	.verify		= r11t_osm_verify,
	.target_index	= r11t_osm_target_index,
	.get		= r11t_osm_get,
};

/* ------------------------------------------------------------------ */
/* debugfs: compare our table with what the OSM actually holds	      */
/* ------------------------------------------------------------------ */

static struct dentry *osm_dbg_dir;

static void osm_dump_lut(struct seq_file *m, struct osm_cluster *c)
{
	unsigned int i;

	seq_printf(m, "%s: %u LUT rows, dcvs row now = %u\n",
		   c->name, c->num_entries, osm_get_row(c));
	seq_printf(m, "  %-4s %-12s %-12s %-12s %-5s %-5s %-6s %s\n",
		   "row", "freq(Hz)", "freq_data", "override", "spare",
		   "vc", "ol_mV", "hw index/freq/override/volt");
	for (i = 0; i < c->num_entries; i++) {
		u32 off = i * OSM_REG_SIZE;
		u32 hw_idx = osm_read(c, OSM_INDEX_REG + off);
		u32 hw_frq = osm_read(c, OSM_FREQ_REG + off);
		u32 hw_ovr = osm_read(c, OSM_OVERRIDE_REG + off);
		u32 hw_vlt = osm_read(c, OSM_VOLT_REG + off);
		u32 want_volt = FIELD_PREP(R11T_OSM_VOLT_VC_MASK,
					   c->lut[i].virtual_corner) |
				(c->lut[i].open_loop_volt & R11T_OSM_VOLT_MV_MASK);

		seq_printf(m,
			   "  %-4u %-12u %#-12x %#-12x %-5u %-5u %-6u  %u/%#x/%#x/%#x%s\n",
			   i, c->lut[i].frequency, c->lut[i].freq_data,
			   c->lut[i].override_data, c->lut[i].spare_data,
			   c->lut[i].virtual_corner, c->lut[i].open_loop_volt,
			   hw_idx, hw_frq, hw_ovr, hw_vlt,
			   (hw_idx == i && hw_frq == c->lut[i].freq_data &&
			    hw_ovr == c->lut[i].override_data &&
			    hw_vlt == want_volt) ? "" : "   <== DIFFERS");
	}
}

static int osm_lut_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < R11T_NR_CLUSTERS; i++)
		osm_dump_lut(m, &osmdev->cluster[i]);

	seq_printf(m, "\ndryrun=%s pwrcl_max_row=%d perfcl_max_row=%d\n",
		   dryrun ? "yes" : "NO - frequency changes are live",
		   pwrcl_max_row, perfcl_max_row);
	return 0;
}
static int osm_regs_show(struct seq_file *m, void *v);

static int osm_lut_open(struct inode *inode, struct file *file)
{
	return single_open(file, osm_lut_show, NULL);
}

static int osm_lut_release(struct inode *inode, struct file *file)
{
	return single_release(inode, file);
}

static const struct file_operations osm_lut_fops = {
	.open	= osm_lut_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release = osm_lut_release,
};

static int osm_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, osm_regs_show, NULL);
}

static const struct file_operations osm_regs_fops = {
	.open	= osm_regs_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release = osm_lut_release,
};

static void osm_dump_seq_one(struct seq_file *m, struct osm_dev *p, int cl)
{
	void __iomem *b = p->base + (unsigned long)cl * OSM_CORE_TABLE_SIZE;
	int n;

	seq_printf(m, "=== cluster %d ===\n", cl);
	for (n = 0; n < 96; n += 8) {
		int i;
		seq_printf(m, "  [%03d]  ", n);
		for (i = 0; i < 8 && n + i < 96; i++) {
			u32 v = readl_relaxed(b + SEQ_REG(n + i));
			seq_printf(m, "%08x ", v);
		}
		seq_puts(m, "\n");
	}
}

static int osm_seq_show(struct seq_file *m, void *v)
{
	if (!osmdev)
		return -ENODEV;
	osm_dump_seq_one(m, osmdev, 0);
	osm_dump_seq_one(m, osmdev, 1);
	return 0;
}

static int osm_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, osm_seq_show, NULL);
}

static const struct file_operations osm_seq_fops = {
	.open	= osm_seq_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release = osm_lut_release,
};

static int osm_regs_show(struct seq_file *m, void *v)
{
	int i, j;

	for (i = 0; i < R11T_NR_CLUSTERS; i++) {
		struct osm_cluster *c = &osmdev->cluster[i];

		seq_printf(m, "=== %s ===\n", c->name);
		seq_printf(m, "  cpus %*pbl\n", cpumask_pr_args(c->cpus));
		for (j = 0; j < 8; j++)
			seq_printf(m, "  0x%04x: %#010x\n",
				   OSM_INDEX_REG + j * OSM_REG_SIZE,
				   osm_read(c, OSM_INDEX_REG + j * OSM_REG_SIZE));
		seq_printf(m, "  --  --\n");
		for (j = 0; j < 8; j++) {
			seq_printf(m,
				   "  row%-2d idx=%#x freq=%#x volt=%#x ovr=%#x spare=%#x\n",
				   j,
				   osm_read(c, OSM_INDEX_REG + j * OSM_REG_SIZE),
				   osm_read(c, OSM_FREQ_REG + j * OSM_REG_SIZE),
				   osm_read(c, OSM_VOLT_REG + j * OSM_REG_SIZE),
				   osm_read(c, OSM_OVERRIDE_REG + j * OSM_REG_SIZE),
				   osm_read(c, OSM_SPARE_REG + j * OSM_REG_SIZE));
		}
		seq_printf(m, "  ENABLE_REG(0x1004)   = %#x\n",
			   osm_read(c, OSM_ENABLE_REG));
		seq_printf(m, "  DCVS_PERF_STATE_DESIRED(0x1f10) = %u\n",
			   osm_read(c, OSM_DCVS_PERF_STATE_DESIRED));
	}
	return 0;
}


static int dryrun_get(void *data, u64 *val)
{
	*val = dryrun;
	return 0;
}

static int dryrun_set(void *data, u64 val)
{
	dryrun = !!val;
	pr_warn("dryrun=%s\n", dryrun ? "yes (frequency frozen)" :
		"NO - writes to DCVS_PERF_STATE_DESIRED are live");
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(dryrun_fops, dryrun_get, dryrun_set, "%llu\n");

static void osm_debugfs_init(struct device *dev)
{
	osm_dbg_dir = debugfs_create_dir("qcom_r11t_osm", NULL);
	debugfs_create_file("lut", 0444, osm_dbg_dir, NULL, &osm_lut_fops);
	debugfs_create_file("regs", 0444, osm_dbg_dir, NULL, &osm_regs_fops);
	debugfs_create_file("seq", 0444, osm_dbg_dir, NULL, &osm_seq_fops);
	debugfs_create_file("dryrun", 0644, osm_dbg_dir, NULL, &dryrun_fops);
}

static void osm_debugfs_exit(void)
{
	debugfs_remove_recursive(osm_dbg_dir);
	osm_dbg_dir = NULL;
}

/* ------------------------------------------------------------------ */
/* probe							      */
/* ------------------------------------------------------------------ */

static int r11t_osm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct osm_dev *p;
	struct resource *res;
	int i, ret;

	if (osmdev)
		return -EBUSY;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no reg resource\n");
		return -EINVAL;
	}

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->dev = dev;
	p->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);

	/* CAF absolute register addresses, one element per cluster.  Read
	 * here, consumed by osm_setup_l_val() further down.
	 */
	for (i = 0; i < R11T_NR_CLUSTERS; i++) {
		static const char * const addr_props[4] = {
			"qcom,l-val-base", "qcom,apcs-pll-user-ctl",
			"qcom,apcs-cfg-rcgr", "qcom,apcs-cmd-rcgr",
		};
		u32 *slots[4] = {
			&p->l_val_base[i], &p->pll_user_ctl[i],
			&p->cfg_rcgr[i], &p->cmd_rcgr[i],
		};
		int k;

		for (k = 0; k < 4; k++) {
			u32 val = 0;

			of_property_read_u32_index(dev->of_node, addr_props[k],
						   i, &val);
			*slots[k] = val;
		}

		/* Map the RCGR window (READ ONLY).  cfg_rcgr and cmd_rcgr are
		 * the CFG_RCGR and CMD_RCGR halves of one root clock generator,
		 * four bytes apart, so the block base is the containing page.
		 */
		{
			phys_addr_t addr = p->cfg_rcgr[i] & ~0xFFFULL;

			p->rcgr_base[i] = devm_ioremap(dev, addr, 4096);
			if (IS_ERR(p->rcgr_base[i])) {
				p->rcgr_base[i] = NULL;
				dev_warn(dev, "cluster%d: cannot map RCGR @0x%llx\n",
					i, (unsigned long long)addr);
			}
		}
	}

	/* Map PLL resources (READ ONLY -- we never write these) */
	for (i = 0; i < R11T_NR_CLUSTERS; i++) {
		static const char * const pll_res_name[R11T_NR_CLUSTERS] = {
			"pwrcl_pll", "perfcl_pll"
		};
		struct resource *pll_res;

		pll_res = platform_get_resource(pdev, IORESOURCE_MEM, i + 1);
		if (!pll_res) {
			dev_info(dev, "%s resource not found, PLL probe will skip\n",
				 pll_res_name[i]);
			continue;
		}
		p->pll_base[i] = devm_ioremap_resource(dev, pll_res);
		if (IS_ERR(p->pll_base[i])) {
			p->pll_base[i] = NULL;
			dev_warn(dev, "ioremap %s failed\n", pll_res_name[i]);
		}
	}

	for (i = 0; i < R11T_NR_CLUSTERS; i++) {
		struct osm_cluster *c = &p->cluster[i];

		c->num = i;
		c->name = cluster_name[i];
		c->dev = dev;
		/*
		 * One reg window, TWO independent 8 KiB slots -- one per cluster.
		 *
		 * 2026-10-01: this used to be "c->base = p->base" for both
		 * clusters, on the reading that the OSM block is a shared global
		 * singleton.  That holds for OSM_ENABLE_REG and for the runtime
		 * row switch, but NOT for the LUT: osm_setup_hw_table() WRITES to
		 * c->base + OSM_INDEX_REG + i * OSM_REG_SIZE, so with a shared
		 * base the second call (perfcl) overwrote every row the first
		 * call (pwrcl) had just programmed.  Measured on hardware with
		 * dryrun=0: the pwrcl rows came back identical to the perfcl rows
		 * (0x404003a/0x4040049/0x404005b/... == perfcl freq_data), and a
		 * 633.6 MHz request on a little core executed the 1113.6 MHz
		 * definition.  The little cluster was running the big cluster's
		 * frequency AND voltage table.
		 *
		 * CAF separates the slots by OSM_CORE_TABLE_SIZE [caf:1279-1282],
		 * which is what i * OSM_CORE_TABLE_SIZE below reproduces.
		 *
		 * The per cluster pwrcl / perfcl PLL offsets are still NOT
		 * programmed: CAF's PLL_BASE writes (PLL_MODE = 0x0 then 0x7)
		 * [caf:1510-1517] are part of a bring up sequence we have not
		 * reproduced, and the bootloader has already set up the PLLs.
		 * Those live in separate "pwrcl_pll" / "perfcl_pll" resources,
		 * not inside this window, so leaving them alone does not make the
		 * LUT shared.
		 */
		c->base = p->base + (unsigned long)i * OSM_CORE_TABLE_SIZE;
	}

	p->per_policy_governor =
		of_property_read_bool(dev->of_node, "qcom,governor-per-policy");

	for (i = 0; i < R11T_NR_CLUSTERS; i++) {
		ret = osm_parse_lut(dev, &p->cluster[i]);
		if (ret)
			return ret;
	}

	if (cpumask_intersects(p->cluster[R11T_CLUSTER_PWRCL].cpus,
			       p->cluster[R11T_CLUSTER_PERFCL].cpus)) {
		dev_err(dev, "pwrcl and perfcl cpu sets overlap\n");
		return -EINVAL;
	}

	/* Cross check: the DT cpufreq table, if present, must be a subset. */
	for (i = 0; i < R11T_NR_CLUSTERS; i++) {
		char prop[32];

		scnprintf(prop, sizeof(prop), "qcom,cpufreq-table-%d",
			  cpumask_first(p->cluster[i].cpus));
		if (of_find_property(dev->of_node, prop, NULL))
			dev_info(dev, "%s also has %s (unused, LUT is authoritative)\n",
				 p->cluster[i].name, prop);
	}

	osmdev = p;

	if (pll_probe) {
		osm_probe_pll(p);
		osm_dump_rcgr(p);
	}

	/* Teach the OSM sequencer where PLL_L_VAL lives.  Gated on dryrun so
	 * the default module behaviour stays byte for byte what it was before
	 * this was added.
	 */
	if (!dryrun)
		osm_setup_seq(p);

	if (dryrun) {
		dev_warn(dev,
			 "dryrun=1: OSM table NOT written, ENABLE_REG untouched, frequency left as the bootloader set it. /sys/kernel/debug/qcom_r11t_osm/regs is all zero in this mode, so it CANNOT be compared against lut. To go live: rmmod qcom_r11t_osm_cpufreq; modprobe qcom_r11t_osm_cpufreq dryrun=0 -- then regs will hold the two per-cluster tables and CAN be compared. Reboot to clear the hardware table again.\n");
	} else {
		for (i = 0; i < R11T_NR_CLUSTERS; i++) {
			ret = osm_setup_hw_table(&p->cluster[i]);
			if (ret) {
				dev_err(dev, "%s: LUT rejected, refusing to continue\n",
					p->cluster[i].name);
				osmdev = NULL;
				return ret;
			}
			dev_info(dev, "%s: programmed %u OSM rows\n",
				 p->cluster[i].name, p->cluster[i].num_entries);
		}

		/* Table first, then ENABLE_REG: CAF also enables from
		 * clk_osm_enable() after clk_osm_setup_hw_table() has run, so
		 * the FSMs never see a half-filled table.
		 */
		for (i = 0; i < R11T_NR_CLUSTERS; i++) {
			ret = osm_enable_hw(&p->cluster[i]);
			if (ret) {
				dev_err(dev,
					"%s: OSM enable failed (%d), refusing to continue\n",
					p->cluster[i].name, ret);
				osmdev = NULL;
				return ret;
			}
		}
	}

	/* PLL_L_VAL experiment.  Deliberately independent of dryrun: the whole
	 * point is to write the PLL ourselves instead of trusting the OSM FSM,
	 * so it must be runnable even with the OSM table left alone.  It runs
	 * AFTER the table is programmed and ENABLE_REG is set, i.e. on the same
	 * steady state we have been measuring, so the only variable is L_VAL.
	 */
	if (lval_test)
		osm_lval_experiment(p);

	for (i = 0; i < R11T_NR_CLUSTERS; i++) {
		int cap = i == R11T_CLUSTER_PWRCL ? pwrcl_max_row :
						  perfcl_max_row;

		ret = osm_build_freq_table(&p->cluster[i], cap);
		if (ret) {
			osmdev = NULL;
			return ret;
		}
	}

	if (p->per_policy_governor)
		r11t_osm_cpufreq_driver.flags |=
			CPUFREQ_HAVE_GOVERNOR_PER_POLICY;

	osm_debugfs_init(dev);

	ret = cpufreq_register_driver(&r11t_osm_cpufreq_driver);
	if (ret) {
		dev_err(dev, "cpufreq_register_driver failed: %d\n", ret);
		osmdev = NULL;
		return ret;
	}

	dev_info(dev, "ready, dryrun=%s\n", dryrun ? "yes" : "no");
	return 0;
}

static void r11t_osm_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&r11t_osm_cpufreq_driver);
	osm_debugfs_exit();

	if (osmdev) {
		free_cpumask_var(osmdev->cluster[R11T_CLUSTER_PWRCL].cpus);
		free_cpumask_var(osmdev->cluster[R11T_CLUSTER_PERFCL].cpus);
	}
	osmdev = NULL;
}

static const struct of_device_id r11t_osm_of_match[] = {
	{ .compatible = "qcom,r11t-osm-cpufreq" },
	{ }
};
MODULE_DEVICE_TABLE(of, r11t_osm_of_match);

static struct platform_driver r11t_osm_driver = {
	.probe	= r11t_osm_probe,
	.remove	= r11t_osm_remove,
	.driver	= {
		.name		= "qcom_r11t_osm_cpufreq",
		.of_match_table	= r11t_osm_of_match,
	},
};
module_platform_driver(r11t_osm_driver);

MODULE_DESCRIPTION("OPPO R11T (SDM660) OSM cpufreq driver");
MODULE_LICENSE("GPL");
