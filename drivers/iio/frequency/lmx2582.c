// SPDX-License-Identifier: GPL-2.0
/*
 * Texas Instruments LMX 2582 High Performance, Wideband
 * PLLatinum RF Synthesizer With Integrated VCO
 *
 * Copyright 2025 PrecisionWave AG
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <asm/div64.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/clk-provider.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/clk/clkscale.h>
#include <linux/rational.h>

#include <dt-bindings/iio/frequency/lmx2582.h>


/* LMX2582_R0 */
#define LMX2582_R0_LD_EN(x)			(((x) & 0x1) << 13)
#define LMX2582_R0_FCAL_HPFD_ADJ(x)		(((x) & 0x3) << 7)
#define LMX2582_R0_FCAL_LPFD_ADJ(x)		(((x) & 0x3) << 5)
#define LMX2582_R0_ACAL_EN(x)			(((x) & 0x1) << 4)
#define LMX2582_R0_FCAL_EN(x)			(((x) & 0x1) << 3)
#define LMX2582_R0_MUXOUT_SEL(x)		(((x) & 0x1) << 2)
#define LMX2582_R0_RESET(x)			(((x) & 0x1) << 1)
#define LMX2582_R0_POWERDOWN(x)			(((x) & 0x1) << 0)

/* LMX2582_R1 */
#define LMX2582_R1_CAL_CLK_DIV(x)		(((x) & 0x7) << 0)

/* LMX2582_R4 */
#define LMX2582_R4_ACAL_CMP_DLY(x)		(((x) & 0xff) << 8)

/* LMX2582_R8 */
#define LMX2582_R8_VCO_IDAC_OVR(x)		(((x) & 0x1) << 13)
#define LMX2582_R8_VCO_CAPCTRL_OVR(x)		(((x) & 0x1) << 10)

/* LMX2582_R9 */
#define LMX2582_R9_OSC_2X(x)			(((x) & 0x1) << 11)
#define LMX2582_R9_REF_EN(x)			(((x) & 0x1) << 9)

/* LMX2582_R10 */
#define LMX2582_R10_MULT(x)			(((x) & 0x1f) << 7)

/* LMX2582_R11 */
#define LMX2582_R11_PLL_R(x)			(((x) & 0xff) << 4)

/* LMX2582_R12 */
#define LMX2582_R12_PLL_R_PRE(x)		(((x) & 0xfff) << 0)

/* LMX2582_R13 */
#define LMX2582_R13_CP_EN(x)			(((x) & 0x1) << 14)
#define LMX2582_R13_PFD_CTL(x)			(((x) & 0x3) << 0)

/* LMX2582_R14 */
#define LMX2582_R14_CP_IDN(x)			(((x) & 0x1f) << 7)
#define LMX2582_R14_CP_IUP(x)			(((x) & 0x1f) << 2)
#define LMX2582_R14_CP_ICOARSE(x)		(((x) & 0x3) << 0)

/* LMX2582_R19 */
#define LMX2582_R19_VCO_IDAC(x)			(((x) & 0x1ff) << 3)

/* LMX2582_R20 */
#define LMX2582_R20_ACAL_VCO_IDAC_STRT(x)	(((x) & 0x1ff) << 0)

/* LMX2582_R22 */
#define LMX2582_R22_VCO_CAPCTRL(x)		(((x) & 0xff) << 0)

/* LMX2582_R23 */
#define LMX2582_R23_FCAL_VCO_SEL_STRT(x)	(((x) & 0x1) << 14)
#define LMX2582_R23_VCO_SEL(x)			(((x) & 0x7) << 11)
#define LMX2582_R23_VCO_SEL_FORCE(x)		(((x) & 0x1) << 10)

/* LMX2582_R30 */
#define LMX2582_R30_MASH_DITHER(x)		(((x) & 0x1) << 10)
#define LMX2582_R30_VTUNE_ADJ(x)		(((x) & 0x3) << 6)

/* LMX2582_R31 */
#define LMX2582_R31_VCO_DISTB_PD(x)		(((x) & 0x1) << 10)
#define LMX2582_R31_VCO_DISTA_PD(x)		(((x) & 0x1) << 9)
#define LMX2582_R31_CHDIV_DIST_PD(x)		(((x) & 0x1) << 7)

/* LMX2582_R34 */
#define LMX2582_R34_CHDIV_EN(x)			(((x) & 0x1) << 5)

/* LMX2582_R35 */
#define LMX2582_R35_CHDIV_SEG2(x)		(((x) & 0xf) << 9)
#define LMX2582_R35_CHDIV_SEG3_EN(x)		(((x) & 0x1) << 8)
#define LMX2582_R35_CHDIV_SEG2_EN(x)		(((x) & 0x1) << 7)
#define LMX2582_R35_CHDIV_SEG1(x)		(((x) & 0x1) << 2)
#define LMX2582_R35_CHDIV_SEG1_EN(x)		(((x) & 0x1) << 1)

/* LMX2582_R36 */
#define LMX2582_R36_CHDIV_DISTB_EN(x)		(((x) & 0x1) << 11)
#define LMX2582_R36_CHDIV_DISTA_EN(x)		(((x) & 0x1) << 10)
#define LMX2582_R36_CHDIV_SEG_SEL(x)		(((x) & 0x7) << 4)
#define LMX2582_R36_CHDIV_SEG3(x)		(((x) & 0xf) << 0)

/* LMX2582_R37 */
#define LMX2582_R37_PLL_N_PRE(x)		(((x) & 0x1) << 12)

/* LMX2582_R38 */
#define LMX2582_R38_PLL_N(x)			(((x) & 0xfff) << 1)

/* LMX2582_R39 */
#define LMX2582_R39_PFD_DLY(x)			(((x) & 0x3f) << 8)

/* LMX2582_R40 */
#define LMX2582_R40_PLL_DEN_31_16(x)		(((x) >> 16) & 0xffff)

/* LMX2582_R41 */
#define LMX2582_R41_PLL_DEN_15_0(x)		((x) & 0xffff)

/* LMX2582_R42 */
#define LMX2582_R42_MASH_SEED_31_16(x)		(((x) >> 16) & 0xffff)

/* LMX2582_R43 */
#define LMX2582_R43_MASH_SEED_15_0(x)		((x) & 0xffff)

/* LMX2582_R44 */
#define LMX2582_R44_PLL_NUM_31_16(x)		(((x) >> 16) & 0xffff)

/* LMX2582_R45 */
#define LMX2582_R45_PLL_NUM_15_0(x)		((x) & 0xffff)

/* LMX2582_R46 */
#define LMX2582_R46_OUTA_POW(x)			(((x) & 0x3f) << 8)
#define LMX2582_R46_OUTB_PD(x)			(((x) & 0x1) << 7)
#define LMX2582_R46_OUTA_PD(x)			(((x) & 0x1) << 6)
#define LMX2582_R46_MASH_ORDER(x)		(((x) & 0x7) << 0)

/* LMX2582_R47 */
#define LMX2582_R47_OUTA_MUX(x)			(((x) & 0x3) << 11)
#define LMX2582_R47_OUTB_POW(x)			(((x) & 0x3f) << 0)

/* LMX2582_R48 */
#define LMX2582_R48_OUTB_MUX(x)			(((x) & 0x3) << 0)

/* LMX2582_R59 */
#define LMX2582_R59_MUXOUT_HDRV(x)		(((x) & 0x1) << 5)

/* LMX2582_R61 */
#define LMX2582_R61_LD_TYPE(x)			(((x) & 0x1) << 0)

/* LMX2582_R64 */
#define LMX2582_R64_ACAL_FAST(x)		(((x) & 0x1) << 9)
#define LMX2582_R64_FCAL_FAST(x)		(((x) & 0x1) << 8)
#define LMX2582_R64_AJUMP_SIZE(x)		(((x) & 0x7) << 5)
#define LMX2582_R64_FJUMP_SIZE(x)		(((x) & 0xf) << 0)

/* LMX2582_R68) (readback) */
#define LMX2582_R68_rb_LD_VTUNE(x)		(((x) >> 9) & 0x3)
#define LMX2582_R68_rb_LVCO_SEL(x)		(((x) >> 5) & 0xf)

/* LMX2582_R69) (readback) */
#define LMX2582_R69_rb_VCO_CAPCTRL(x)		(((x) >> 0) & 0xff)

/* LMX2582_R70) (readback) */
#define LMX2582_R70_rb_VCO_DACISET(x)		(((x) >> 0) & 0x1ff)

/* Specifications */
#define LMX2582_MIN_FREQ_VCO		3550000000ULL /* 3550 MHz */
#define LMX2582_MAX_FREQ_VCO		7100000000ULL /* 7100 MHz */
#define LMX2582_MIN_FREQ_OUT		  20000000ULL /*   20 MHz */
#define LMX2582_MAX_FREQ_OUT		5500000000ULL /* 5500 MHz */
#define LMX2582_MIN_FREQ_REFIN		   5000000ULL /*    5 MHz */
#define LMX2582_MAX_FREQ_REFIN		1400000000ULL /* 1400 MHz */
#define LMX2582_MIN_FREQ_PFD		   5000000ULL /*    5 MHz */
#define LMX2582_MAX_FREQ_PFD		 200000000ULL /*  200 MHz */
#define LMX2582_MAX_FREQ_CAL_CLK	 200000000ULL /*  200 MHz */
#define LMX2582_FVCO_HIGH		6500000000ULL /* 6500 MHz */

#define LMX2582_CHECK_RANGE(freq, range) \
	(((freq) > LMX2582_MAX_ ## range) || ((freq) < LMX2582_MIN_ ## range))
	
#define LMX2582_CLK_COUNT			2
#define LMX2582_PLL_N_MIN			9
#define LMX2582_PLL_N_MAX			4095


/* Tables from Datasheet */
struct lmx2582_frequency_range {
	unsigned long long min;
	unsigned long long max;
};

struct lmx2582_chdiv_register_values {
	u32 seg1;
	u32 seg2;
	u32 seg3;
};

struct lmx2582_channel_divider_min_max {
	struct lmx2582_frequency_range		output_frequency_range;
	struct lmx2582_chdiv_register_values	chdiv_register_values;
	struct lmx2582_frequency_range		vco_frequency_range;
	u32					total_divider;
};

#define MHZ_TO_HZ(_x_) ((_x_) * 1000ULL * 1000ULL)
#define LMX2582_CHDIV_SEG1_DIV(x) LMX2582_CHDIV_SEG1_DIV##x
#define LMX2582_CHDIV_SEG2_DIV1 LMX2582_CHDIV_SEG2_PD
#define LMX2582_CHDIV_SEG2_DIV(x) LMX2582_CHDIV_SEG2_DIV##x
#define LMX2582_CHDIV_SEG3_DIV1 LMX2582_CHDIV_SEG3_PD
#define LMX2582_CHDIV_SEG3_DIV(x) LMX2582_CHDIV_SEG3_DIV##x
#define CLOCK_SPEC_TABLE_ENTRY(fout_min, fout_max,                             \
			       div_seg1, div_seg2, div_seg3, div_tot,          \
			       fvco_min, fvco_max) {                           \
		.output_frequency_range = {                                    \
			.min = MHZ_TO_HZ(fout_min),                            \
			.max = MHZ_TO_HZ(fout_max),                            \
		},                                                             \
		.chdiv_register_values = {                                     \
			.seg1 = LMX2582_CHDIV_SEG1_DIV(div_seg1),              \
			.seg2 = LMX2582_CHDIV_SEG2_DIV(div_seg2),              \
			.seg3 = LMX2582_CHDIV_SEG3_DIV(div_seg3),              \
		},                                                             \
		.vco_frequency_range = {                                       \
			.min = MHZ_TO_HZ(fvco_min),                            \
			.max = MHZ_TO_HZ(fvco_max),                            \
		},                                                             \
		.total_divider = (div_tot)                                     \
}

const struct lmx2582_channel_divider_min_max lmx2582_divided_clock_spec[] = {
	CLOCK_SPEC_TABLE_ENTRY(1775, 3550, 2, 1, 1,   2, 3550, 7100),
	CLOCK_SPEC_TABLE_ENTRY(1184, 2200, 3, 1, 1,   3, 3552, 6600),
	CLOCK_SPEC_TABLE_ENTRY( 888, 1184, 2, 2, 1,   4, 3552, 4736),
	CLOCK_SPEC_TABLE_ENTRY( 592,  888, 3, 2, 1,   6, 3552, 5328),
	CLOCK_SPEC_TABLE_ENTRY( 444,  592, 2, 4, 1,   8, 3552, 4736),
	CLOCK_SPEC_TABLE_ENTRY( 296,  444, 2, 6, 1,  12, 3552, 5328),
	CLOCK_SPEC_TABLE_ENTRY( 222,  296, 2, 8, 1,  16, 3552, 4736),
	CLOCK_SPEC_TABLE_ENTRY( 148,  222, 3, 8, 1,  24, 3552, 5328),
	CLOCK_SPEC_TABLE_ENTRY( 111,  148, 2, 8, 2,  32, 3552, 4736),
	CLOCK_SPEC_TABLE_ENTRY(  99,  111, 3, 6, 2,  36, 3564, 3996),
	CLOCK_SPEC_TABLE_ENTRY(  74,   99, 3, 8, 2,  48, 3552, 4752),
	CLOCK_SPEC_TABLE_ENTRY(  56,   74, 2, 8, 4,  64, 3584, 4736),
	CLOCK_SPEC_TABLE_ENTRY(  37,   56, 2, 8, 6,  96, 3552, 5376),
	CLOCK_SPEC_TABLE_ENTRY(  28,   37, 2, 8, 8, 128, 3584, 4736),
	CLOCK_SPEC_TABLE_ENTRY(  20,   28, 3, 8, 8, 192, 3840, 5376),
	{},
};

struct lmx2582_input_output_range {
	struct lmx2582_frequency_range fin;
	struct lmx2582_frequency_range fout;
};

struct lmx2582_input_path_spec {
	struct lmx2582_frequency_range fosc;
	struct lmx2582_input_output_range oscin_doubler;
	struct lmx2582_input_output_range pre_r_divider;
	struct lmx2582_input_output_range multiplier;
	struct lmx2582_input_output_range post_r_divider;
	struct lmx2582_frequency_range fpfd;
};

static const struct lmx2582_input_path_spec lmx2582_input_path_spec = {
	.fosc = {.min = MHZ_TO_HZ(5), .max = MHZ_TO_HZ(1400) },
	.oscin_doubler = {
		.fin  = {.min = MHZ_TO_HZ(5),  .max = MHZ_TO_HZ(200) },
		.fout = {.min = MHZ_TO_HZ(10), .max = MHZ_TO_HZ(400) },
	},
	.pre_r_divider = {
		.fin  = {.min = MHZ_TO_HZ(10), .max = MHZ_TO_HZ(1400) },
		.fout = {.min = MHZ_TO_HZ(5),  .max = MHZ_TO_HZ(700) },
	},
	.multiplier = {
		.fin  = {.min = MHZ_TO_HZ(40),  .max = MHZ_TO_HZ(70) },
		.fout = {.min = MHZ_TO_HZ(180), .max = MHZ_TO_HZ(250) },
	},
	.post_r_divider = {
		.fin  = {.min = MHZ_TO_HZ(5),    .max = MHZ_TO_HZ(250) },
		.fout = {.min = MHZ_TO_HZ(0.25), .max = MHZ_TO_HZ(125) },
	},
	/* Default architecture with dual-loop PFD can operate between
	   5 and 200 MHz. Extended modes need special attention */
	.fpfd = {.min = MHZ_TO_HZ(5), .max = MHZ_TO_HZ(200) },
};


/* iio attribute indexes */
enum {
	LMX2582_CH_ATTR_FREQ,
	LMX2582_CH_ATTR_PWRDOWN,
	LMX2582_CH_ATTR_CHANNEL_NAME,
	LMX2582_ATTR_FVCO,
	LMX2582_ATTR_FPFD,
	LMX2582_ATTR_FOSC,
	LMX2582_ATTR_CHDIV_TOTAL,
	LMX2582_ATTR_FCHDIV,
	LMX2582_ATTR_PLL_R,
	LMX2582_ATTR_PLL_R_PRE,
	LMX2582_ATTR_PLL_OSC_2X,
	LMX2582_ATTR_MULT,
	LMX2582_ATTR_PLL_N_PRE,
	LMX2582_ATTR_PLL_N,
	LMX2582_ATTR_PLL_NUM,
	LMX2582_ATTR_PLL_DEN,
	LMX2582_ATTR_CHDIV_SEG1,
	LMX2582_ATTR_CHDIV_SEG2,
	LMX2582_ATTR_CHDIV_SEG3,
	LMX2582_ATTR_CHDIV_SEG_SEL,
	LMX2582_ATTR_CP_IUP,
	LMX2582_ATTR_CP_IDN,
	LMX2582_ATTR_CP_ICOARSE,
	LMX2582_ATTR_CP_EN,
};

/* OUTA_MUX and OUTB_MUX valid settings */
enum {
	LMX2582_OUTx_MUX_CHDIV = 0,
	LMX2582_OUTx_MUX_VCO = 1,
};

/* iio channel indexes */
enum {
	LMX2582_CH_RFOUTA,
	LMX2582_CH_RFOUTB,
};

/* iio channel names */
static const char * const lmx2582_ch_names[] = {
	[LMX2582_CH_RFOUTA] = "RFoutA",
	[LMX2582_CH_RFOUTB] = "RFoutB",
};

/* Registers */
enum lmx2582_reg {
	LMX2582_R0 = 0,
	LMX2582_R1 = 1,
	LMX2582_R2 = 2,
	LMX2582_R4 = 4,
	LMX2582_R7 = 7,
	LMX2582_R8 = 8,
	LMX2582_R9 = 9,
	LMX2582_R10 = 10,
	LMX2582_R11 = 11,
	LMX2582_R12 = 12,
	LMX2582_R13 = 13,
	LMX2582_R14 = 14,
	LMX2582_R19 = 19,
	LMX2582_R20 = 20,
	LMX2582_R22 = 22,
	LMX2582_R23 = 23,
	LMX2582_R24 = 24,
	LMX2582_R25 = 25,
	LMX2582_R28 = 28,
	LMX2582_R29 = 29,
	LMX2582_R30 = 30,
	LMX2582_R31 = 31,
	LMX2582_R32 = 32,
	LMX2582_R33 = 33,
	LMX2582_R34 = 34,
	LMX2582_R35 = 35,
	LMX2582_R36 = 36,
	LMX2582_R37 = 37,
	LMX2582_R38 = 38,
	LMX2582_R39 = 39,
	LMX2582_R40 = 40,
	LMX2582_R41 = 41,
	LMX2582_R42 = 42,
	LMX2582_R43 = 43,
	LMX2582_R44 = 44,
	LMX2582_R45 = 45,
	LMX2582_R46 = 46,
	LMX2582_R47 = 47,
	LMX2582_R48 = 48,
	LMX2582_R59 = 59,
	LMX2582_R61 = 61,
	LMX2582_R62 = 62,
	LMX2582_R64 = 64,
	LMX2582_R68 = 68,
	LMX2582_R69 = 69,
	LMX2582_R70 = 70,
	LMX2582_R_NUM,
};

/* Fixed register values */
static const u16 lmx2582_reserved_values[LMX2582_R_NUM] = {
	[LMX2582_R0] = 0x0200,
	[LMX2582_R1] = 0x0808,
	[LMX2582_R2] = 0x0500,
	[LMX2582_R4] = 0x0043,
	[LMX2582_R7] = 0x28b2,
	[LMX2582_R8] = 0x1084,
	[LMX2582_R9] = 0x0102,
	[LMX2582_R10] = 0x1058,
	[LMX2582_R11] = 0x0008,
	[LMX2582_R12] = 0x7000,
	[LMX2582_R13] = 0x0000,
	[LMX2582_R14] = 0x0000,
	[LMX2582_R19] = 0x0005,
	[LMX2582_R20] = 0x0000,
	[LMX2582_R22] = 0x2300,
	[LMX2582_R23] = 0x8042,
	[LMX2582_R24] = 0x0509,
	[LMX2582_R25] = 0x0000,
	[LMX2582_R28] = 0x2924,
	[LMX2582_R29] = 0x0084,
	[LMX2582_R30] = 0x0034,
	[LMX2582_R31] = 0x0001,
	[LMX2582_R32] = 0x210a,
	[LMX2582_R33] = 0x2a0a,
	[LMX2582_R34] = 0xc3ca,
	[LMX2582_R35] = 0x0019,
	[LMX2582_R36] = 0x0000,
	[LMX2582_R37] = 0x4000,
	[LMX2582_R38] = 0x0000,
	[LMX2582_R39] = 0x8004,
	[LMX2582_R40] = 0x0000,
	[LMX2582_R41] = 0x0000,
	[LMX2582_R42] = 0x0000,
	[LMX2582_R43] = 0x0000,
	[LMX2582_R44] = 0x0000,
	[LMX2582_R45] = 0x0000,
	[LMX2582_R46] = 0x0020,
	[LMX2582_R47] = 0x00c0,
	[LMX2582_R48] = 0x03fc,
	[LMX2582_R59] = 0x0000,
	[LMX2582_R61] = 0x0000,
	[LMX2582_R62] = 0x0000,
	[LMX2582_R64] = 0x0010,
};

/* runtime configuration registers values */
struct lmx2582_config {
	/* REG0 */
	bool LD_EN;
	u32 FCAL_HPFD_ADJ;
	u32 FCAL_LPFD_ADJ;
	bool ACAL_EN;
	bool FCAL_EN;
	bool MUXOUT_SEL;
	bool RESET;
	bool POWERDOWN;

	/* REG1 */
	u32 CAL_CLK_DIV;

	/* REG4 */
	u32 ACAL_CMP_DLY;

	/* REG8 */
	bool VCO_IDAC_OVR;
	bool VCO_CAPCTRL_OVR;

	/* REG9 */
	bool OSC_2X;
	bool REF_EN;

	/* REG10 */
	u32 MULT;

	/* REG11 */
	u32 PLL_R;

	/* REG12 */
	u32 PLL_R_PRE;

	/* REG13 */
	bool CP_EN;
	u32 PFD_CTL;

	/* REG14 */
	u32 CP_IDN;
	u32 CP_IUP;
	u32 CP_ICOARSE;

	/* REG19 */
	u32 VCO_IDAC;

	/* REG20 */
	u32 ACAL_VCO_IDAC_STRT;

	/* REG22 */
	u32 VCO_CAPCTRL;

	/* REG23 */
	bool FCAL_VCO_SEL_STRT;
	u32 VCO_SEL;
	bool VCO_SEL_FORCE;

	/* R30 */
	bool MASH_DITHER;
	u32 VTUNE_ADJ;

	/* R31 */
	bool VCO_DISTB_PD;
	bool VCO_DISTA_PD;
	bool CHDIV_DIST_PD;

	/* R34 */
	bool CHDIV_EN;

	/* R35 */
	u32 CHDIV_SEG2;
	bool CHDIV_SEG3_EN;
	bool CHDIV_SEG2_EN;
	u32 CHDIV_SEG1;
	bool CHDIV_SEG1_EN;

	/* R36 */
	bool CHDIV_DISTB_EN;
	bool CHDIV_DISTA_EN;
	u32 CHDIV_SEG_SEL;
	u32 CHDIV_SEG3;

	/* R37 */
	u32 PLL_N_PRE;

	/* R38 */
	u32 PLL_N;

	/* R39 */
	u32 PFD_DLY;

	/* R40/41 */
	u32 PLL_DEN;

	/* R42/43 */
	u32 MASH_SEED;

	/* R44/45 */
	u32 PLL_NUM;

	/* R46 */
	u32 OUTA_POW;
	bool OUTB_PD;
	bool OUTA_PD;
	u32 MASH_ORDER;

	/* R47 */
	u32 OUTA_MUX;
	u32 OUTB_POW;

	/* R48 */
	u32 OUTB_MUX;

	/* R59 */
	bool MUXOUT_HDRV;

	/* R61 */
	bool LD_TYPE;

	/* R64 */
	bool ACAL_FAST;
	bool FCAL_FAST;
	u32 AJUMP_SIZE;
	u32 FJUMP_SIZE;
};
/* Default values */
static const struct lmx2582_config lmx2582_default_values = {
	/* REG0 */
	.LD_EN = true,
	.FCAL_HPFD_ADJ = 0,
	.FCAL_LPFD_ADJ = 0,
	.ACAL_EN = 1,
	.FCAL_EN = 1,
	.MUXOUT_SEL = 1,
	.RESET = 0,
	.POWERDOWN = 0,
	/* REG1 */
	.CAL_CLK_DIV = 3,
	/* REG4 */
	.ACAL_CMP_DLY = 25,
	/* REG8 */
	.VCO_IDAC_OVR = false,
	.VCO_CAPCTRL_OVR = false,
	/* REG9 */
	.OSC_2X = false,
	.REF_EN = true,
	/* REG10 */
	.MULT = 1,
	/* REG11 */
	.PLL_R = 1,
	/* REG12 */
	.PLL_R_PRE = 1,
	/* REG13 */
	.CP_EN = true,
	.PFD_CTL = LMX2582_PFD_CTL_DUAL,
	/* REG14 */
	.CP_IDN = LMX2582_CP_I_0_468MA,
	.CP_IUP = LMX2582_CP_I_0_468MA,
	.CP_ICOARSE = LMX2582_CP_ICOARSE_X1,
	/* REG19 */
	.VCO_IDAC = 300,
	/* REG20 */
	.ACAL_VCO_IDAC_STRT = 300,
	/* REG22 */
	.VCO_CAPCTRL = 0,
	/* REG23 */
	.FCAL_VCO_SEL_STRT = false,
	.VCO_SEL = 1,
	.VCO_SEL_FORCE = false,
	/* R30 */
	.MASH_DITHER = false,
	.VTUNE_ADJ  = 3,
	/* R31 */
	.VCO_DISTB_PD = true,
	.VCO_DISTA_PD = false,
	.CHDIV_DIST_PD = false,
	/* R34 */
	.CHDIV_EN = true,
	/* R35 */
	.CHDIV_SEG2 = LMX2582_CHDIV_SEG2_DIV2,
	.CHDIV_SEG3_EN = false,
	.CHDIV_SEG2_EN = false,
	.CHDIV_SEG1 = LMX2582_CHDIV_SEG1_DIV3,
	.CHDIV_SEG1_EN = false,
	/* R36 */
	.CHDIV_DISTB_EN = false,
	.CHDIV_DISTA_EN = true,
	.CHDIV_SEG_SEL = LMX2582_CHDIV_SEG_SEL_1,
	.CHDIV_SEG3 = LMX2582_CHDIV_SEG3_DIV2,
	/* R37 */
	.PLL_N_PRE = LMX2582_PLL_N_PRE_DIV2,
	/* R38 */
	.PLL_N = 27,
	/* R39 */
	.PFD_DLY = 2,
	/* R40/41 */
	.PLL_DEN = 1000,
	/* R42/43 */
	.MASH_SEED = 0,
	/* R44/45 */
	.PLL_NUM = 0,
	/* R46 */
	.OUTA_POW = 15,
	.OUTB_PD = true,
	.OUTA_PD = false,
	.MASH_ORDER = 3,
	/* R47 */
	.OUTA_MUX = 0,
	.OUTB_POW = 15,
	/* R48 */
	.OUTB_MUX = 0,
	/* R59 */
	.MUXOUT_HDRV = false,
	/* R61 */
	.LD_TYPE = true,
	/* R64 */
	.ACAL_FAST = false,
	.FCAL_FAST = false,
	.AJUMP_SIZE = 3,
	.FJUMP_SIZE = 15,
};

/* output state */
struct lmx2582_output {
	struct clk_hw		hw;
	struct iio_dev		*indio_dev;
	struct lmx2582_state	*st;
	unsigned int		num;
	bool			enabled;
	unsigned int		power;
	unsigned int		out_mux;
	unsigned long long	frequency;
};

#define to_clk_priv(_hw) container_of(_hw, struct lmx2582_output, hw)

/* private device state */
struct lmx2582_state {
	struct spi_device	*spi;
	struct iio_dev		*indio_dev;

	/* debugfs dentry */
	struct dentry		*dent;

	/* device configuration */
	struct lmx2582_config	*conf;

	/* input clock */
	struct clk		*clkin;

	/* output clocks */
	struct clk		*clks[LMX2582_CLK_COUNT];
	struct clk_onecell_data	clk_data;
	struct lmx2582_output	outputs[LMX2582_CLK_COUNT];
	struct clock_scale	scale;

	/* work queue non-sleepable contexts */
	struct work_struct	workq;
	struct wait_queue_head	wq_setup_done;

	/* clock output names for clk subsystem */
	bool			has_clk_out_names;
	const char		*lmx2582_clk_names[LMX2582_CLK_COUNT];

	/* Protect against concurrent accesses to the device */
	struct mutex		lock;

	/* calculated frequency values */
	unsigned long long	fosc;
	unsigned long long	fvco;
	unsigned long long	fpd;
	unsigned long long	fcal_clk;
	unsigned long long	fchdiv;
	u32			chdiv_total;

	/* prepared register values */
	u16			regs[LMX2582_R_NUM];

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	u8			wrval[3] ____cacheline_aligned;
	__be16			rdval ____cacheline_aligned;
};


/* real device access */
static int lmx2582_spi_write(struct lmx2582_state *st, u8 reg, u16 val)
{
	st->wrval[0] = (reg & 0x7f);
	st->wrval[1] = (val >> 8) & 0xff;
	st->wrval[2] = val & 0xff;

	dev_dbg(&st->spi->dev, "write R%d = 0x%.8X\n", reg, val);

	return spi_write(st->spi, &st->wrval[0], 3);
}

static int lmx2582_spi_read(struct lmx2582_state *st, u8 reg, u16* val)
{
	int ret;

	ret = spi_w8r16be(st->spi, reg | 0x80);
	if (ret < 0)
		return ret;
	
	if (val)
		*val = ret;

	dev_dbg(&st->spi->dev, "read R%d -> 0x%.8X\n", reg, ret);

	return 0;
}


/* soft reset for complete reprogram of device */
static int lmx2582_soft_reset(struct lmx2582_state *st)
{
	return lmx2582_spi_write(st, LMX2582_R0,
				st->regs[LMX2582_R0] |
				lmx2582_reserved_values[LMX2582_R0] |
				LMX2582_R0_RESET(1));
}


/* write all registers to device */
static int lmx2582_sync_config(struct lmx2582_state *st)
{
	u16 val;
	int ret, i;

	for (i = LMX2582_R70; i >= LMX2582_R0; i--) {
		val = lmx2582_reserved_values[i];
		val |= st->regs[i];

		switch (i) {
		/* Force update */
		case LMX2582_R0:
		val |= LMX2582_R0_FCAL_EN(1);
			ret = lmx2582_spi_write(st, i, val);
			if (ret < 0)
				return ret;
			break;
		/* Skip non-existent registers */
		case 3:
		case 5:
		case 6:
		case 15:
		case 16:
		case 17:
		case 18:
		case 21:
		case 26:
		case 27:
		case 49:
		case 50:
		case 51:
		case 52:
		case 53:
		case 54:
		case 55:
		case 56:
		case 57:
		case 58:
		case 60:
		case 63:
		case 65:
		case 66:
		case 67:
			break;
		/* Read only registers*/
		case LMX2582_R68:
		case LMX2582_R69:
		case LMX2582_R70:
			break;
		/* "Normal register" */
		default:
			ret = lmx2582_spi_write(st, i, val);
			if (ret < 0)
				return ret;
			break;
		}
	}

	return 0;
}


/* iio debug register access */
static int lmx2582_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int writeval,
			      unsigned int *readval)
{
	struct lmx2582_state *st = iio_priv(indio_dev);
	int ret;

	if (reg >= LMX2582_R_NUM)
		return -EINVAL;

	mutex_lock(&st->lock);
	if (readval == NULL) {
		ret = lmx2582_spi_write(st, reg & 0x7f, writeval & 0xffff);
	} else {
		u16 tmp;
		ret = lmx2582_spi_read(st, reg & 0x7f, &tmp);
		*readval = tmp;
	}
	mutex_unlock(&st->lock);

	return ret;
}


/* calculate total chanel divider from register settings */
static u32 lmx2582_get_chdiv_total(struct lmx2582_config *conf)
{
	u32 divider1;
	u32 divider2;
	u32 divider3;

	/* decode SEG1 */
	if (conf->CHDIV_SEG1 == LMX2582_CHDIV_SEG1_DIV2)
		divider1 = 2;
	else
		divider1 = 3;

	/* decode SEG2 */
	switch (conf->CHDIV_SEG2) {
	case LMX2582_CHDIV_SEG2_DIV2:
		divider2 = 2;
		break;
	case LMX2582_CHDIV_SEG2_DIV4:
		divider2 = 4;
		break;
	case LMX2582_CHDIV_SEG2_DIV6:
		divider2 = 6;
		break;
	case LMX2582_CHDIV_SEG2_DIV8:
		divider2 = 8;
		break;
	case LMX2582_CHDIV_SEG2_PD:
	default:
		divider2 = 0;
		break;
	}

	/* decode SEG3 */
	switch (conf->CHDIV_SEG3) {
	case LMX2582_CHDIV_SEG3_DIV2:
		divider3 = 2;
		break;
	case LMX2582_CHDIV_SEG3_DIV4:
		divider3 = 4;
		break;
	case LMX2582_CHDIV_SEG3_DIV6:
		divider3 = 6;
		break;
	case LMX2582_CHDIV_SEG3_DIV8:
		divider3 = 8;
		break;
	case LMX2582_CHDIV_SEG3_PD:
	default:
		divider3 = 0;
		break;
	}
	
	/* decode mux */
	switch (conf->CHDIV_SEG_SEL) {
	case LMX2582_CHDIV_SEG_SEL_1:
		/* only divider 1 active */
		return divider1;
	case LMX2582_CHDIV_SEG_SEL_12:
		/* divider 1 and 2 active */
		return divider1 * divider2;
	case LMX2582_CHDIV_SEG_SEL_123:
		/* divider 1, 2 and 3 active */
		return divider1 * divider2 * divider3;
	case LMX2582_CHDIV_SEG_SEL_PD:
	default:
		/* power down or invalid */
		break;
	}
	/* power down or invalid setting */
	return 0;
}


/* chip setup */
static int lmx2582_setup(struct lmx2582_state *st, unsigned long parent_rate)
{
	struct lmx2582_config *conf = st->conf;
	int i, ret;

	if (parent_rate)
		st->fosc = parent_rate;
	else
		st->fosc = clk_get_rate(st->clkin);

	/* verify clkin clock range */
	if (LMX2582_CHECK_RANGE(st->fosc, FREQ_REFIN))
		dev_err(&st->spi->dev,
			"OSC frequency (%llu Hz) out of range!",
			st->fosc);

	/* Input path to phase comparator */
	st->fpd = st->fosc;
	st->fpd *= conf->OSC_2X ? 2 : 1;
	st->fpd = div_u64(st->fpd, conf->PLL_R_PRE == 0 ? 1 : conf->PLL_R_PRE);
	st->fpd *= conf->MULT;
	st->fpd = div_u64(st->fpd, conf->PLL_R == 0 ? 1 : conf->PLL_R);

	if (LMX2582_CHECK_RANGE(st->fpd, FREQ_PFD))
		dev_err(&st->spi->dev,
			"PFD frequency (%llu Hz) out of range!",
			st->fpd);

	/* VCO feedback path */
	st->fvco = st->fpd * (conf->PLL_N * conf->PLL_DEN + conf->PLL_NUM);
	st->fvco = div_u64(st->fvco, conf->PLL_DEN == 0 ? 1 : conf->PLL_DEN);
	st->fvco *= conf->PLL_N_PRE ? 4 : 2;

	if (LMX2582_CHECK_RANGE(st->fvco, FREQ_VCO))
		dev_err(&st->spi->dev,
			"VCO frequency (%llu Hz) out of range!",
			st->fvco);

	/* state machine clock */
	for (i = 0; i < 8; i++) {
		conf->CAL_CLK_DIV = i;
		if ((st->fosc >> i) < LMX2582_MAX_FREQ_CAL_CLK)
			break;
	}
	st->fcal_clk = st->fosc >> conf->CAL_CLK_DIV;

	/* output parameters */
	conf->OUTA_MUX = st->outputs[0].out_mux;
	conf->OUTA_POW = st->outputs[0].power;
	conf->OUTB_MUX = st->outputs[1].out_mux;
	conf->OUTB_POW = st->outputs[1].power;

	/* calculate output clocks */
	st->chdiv_total = lmx2582_get_chdiv_total(conf);
	if (st->chdiv_total == 0) 
		st->fchdiv = 0;
	else
		st->fchdiv = div_u64(st->fvco, st->chdiv_total);

	switch(conf->OUTA_MUX & 3) {
	case LMX2582_OUTx_MUX_CHDIV:
		st->outputs[0].frequency =  st->fchdiv;
		break;

	case LMX2582_OUTx_MUX_VCO:
		st->outputs[0].frequency = st->fvco;
		break;
	default: 
		st->outputs[0].frequency = 0;
		break;
	}

	if (st->outputs[0].frequency &&
	    LMX2582_CHECK_RANGE(st->outputs[0].frequency, FREQ_OUT))
		dev_err(&st->spi->dev,
			"OUTA frequency (%llu Hz) out of range!",
			st->outputs[0].frequency);

	switch(conf->OUTB_MUX & 3) {
	case LMX2582_OUTx_MUX_CHDIV:
		st->outputs[1].frequency = st->fchdiv;
		break;
	case LMX2582_OUTx_MUX_VCO:
		st->outputs[1].frequency = st->fvco;
		break;
	default: 
		st->outputs[1].frequency = 0;
		break;
	}
	if (st->outputs[1].frequency &&
	    LMX2582_CHECK_RANGE(st->outputs[1].frequency, FREQ_OUT))
		dev_err(&st->spi->dev,
			"OUTB frequency (%llu Hz) out of range!",
			st->outputs[1].frequency);

	/* VCO Band selection */
	if (st->fvco >= LMX2582_FVCO_HIGH)
		conf->VTUNE_ADJ = 3;
	else
		conf->VTUNE_ADJ = 0;

	/* Determine if channel divider needs power */
	/* Default: All paths disabled */
	conf->CHDIV_DIST_PD = true;
	conf->CHDIV_DISTA_EN = false;
	conf->CHDIV_DISTB_EN = false;
	conf->VCO_DISTA_PD = true;
	conf->VCO_DISTB_PD = true;
	conf->CHDIV_EN = false;

	if (st->outputs[0].enabled) {
		switch (conf->OUTA_MUX & 3) {
		case LMX2582_OUTx_MUX_CHDIV:
			conf->CHDIV_DIST_PD = false;
			conf->CHDIV_DISTA_EN = true;
			conf->VCO_DISTA_PD = true;
			conf->CHDIV_EN = true;
			break;
		case LMX2582_OUTx_MUX_VCO:
			conf->VCO_DISTA_PD = false;
			break;
		}
	}

	if (st->outputs[1].enabled) {
		switch (conf->OUTB_MUX & 3) {
		case LMX2582_OUTx_MUX_CHDIV:
			conf->CHDIV_DIST_PD = false;
			conf->CHDIV_DISTB_EN = true;
			conf->VCO_DISTB_PD = true;
			conf->CHDIV_EN = true;
			break;
		case LMX2582_OUTx_MUX_VCO:
			conf->VCO_DISTB_PD = false;
			break;
		}
	}

	/* Channel divider enable control */
	switch (conf->CHDIV_SEG_SEL) {
	case 1: /* only divider 1 active */
		conf->CHDIV_SEG1_EN = true;
		conf->CHDIV_SEG2_EN = false;
		conf->CHDIV_SEG3_EN = false;
		break;
	case 2: /* divider 1 and 2 active */
		conf->CHDIV_SEG1_EN = true;
		conf->CHDIV_SEG2_EN = true;
		conf->CHDIV_SEG3_EN = false;
		break;
	case 4: /* divider 1, 2 and 3 active */
		conf->CHDIV_SEG1_EN = true;
		conf->CHDIV_SEG2_EN = true;
		conf->CHDIV_SEG3_EN = true;
		break;
	case 0: /* power down */
	default:
		conf->CHDIV_SEG1_EN = false;
		conf->CHDIV_SEG2_EN = false;
		conf->CHDIV_SEG3_EN = false;
		break;
	}

	/* R0 Bit Definitions */
	st->regs[LMX2582_R0] =
		LMX2582_R0_LD_EN(conf->LD_EN) |
		LMX2582_R0_FCAL_HPFD_ADJ(conf->FCAL_HPFD_ADJ) |
		LMX2582_R0_FCAL_LPFD_ADJ(conf->FCAL_LPFD_ADJ) |
		LMX2582_R0_ACAL_EN(conf->ACAL_EN) |
		LMX2582_R0_FCAL_EN(conf->FCAL_EN) |
		LMX2582_R0_MUXOUT_SEL(conf->MUXOUT_SEL) |
		LMX2582_R0_RESET(conf->RESET) |
		LMX2582_R0_POWERDOWN(conf->POWERDOWN);

	/* R1 Bit Definitions */
	st->regs[LMX2582_R1] = LMX2582_R1_CAL_CLK_DIV(conf->CAL_CLK_DIV);

	/* R4 Bit Definitions */
	st->regs[LMX2582_R4] = LMX2582_R4_ACAL_CMP_DLY(conf->ACAL_CMP_DLY);

	/* R8 Bit Definitions */
	st->regs[LMX2582_R8] =
		LMX2582_R8_VCO_IDAC_OVR(conf->VCO_IDAC_OVR) |
		LMX2582_R8_VCO_CAPCTRL_OVR(conf->VCO_CAPCTRL_OVR);

	/* R9 Bit Definitions */
	st->regs[LMX2582_R9] =
		LMX2582_R9_OSC_2X(conf->OSC_2X) |
		LMX2582_R9_REF_EN(conf->REF_EN);

	/* R10 Bit Definitions */
	st->regs[LMX2582_R10] = LMX2582_R10_MULT(conf->MULT);

	/* R11 Bit Definitions */
	st->regs[LMX2582_R11] = LMX2582_R11_PLL_R(conf->PLL_R);

	/* R12 Bit Definitions */
	st->regs[LMX2582_R12] = LMX2582_R12_PLL_R_PRE(conf->PLL_R_PRE);

	/* R13 Bit Definitions */
	st->regs[LMX2582_R13] =
		LMX2582_R13_CP_EN(conf->CP_EN) |
		LMX2582_R13_PFD_CTL(conf->PFD_CTL);

	/* R14 Bit Definitions */
	st->regs[LMX2582_R14] =
		LMX2582_R14_CP_IDN(conf->CP_IDN) |
		LMX2582_R14_CP_IUP(conf->CP_IUP) |
		LMX2582_R14_CP_ICOARSE(conf->CP_ICOARSE);

	/* R19 Bit Definitions */
	st->regs[LMX2582_R19] = LMX2582_R19_VCO_IDAC(conf->VCO_IDAC);

	/* R20 Bit Definitions */
	st->regs[LMX2582_R20] =
		LMX2582_R20_ACAL_VCO_IDAC_STRT(conf->ACAL_VCO_IDAC_STRT);

	/* R22 Bit Definitions */
	st->regs[LMX2582_R22] = LMX2582_R22_VCO_CAPCTRL(conf->VCO_CAPCTRL);

	/* R23 Bit Definitions */
	st->regs[LMX2582_R23] =
		LMX2582_R23_FCAL_VCO_SEL_STRT(conf->FCAL_VCO_SEL_STRT) |
		LMX2582_R23_VCO_SEL(conf->VCO_SEL) |
		LMX2582_R23_VCO_SEL_FORCE(conf->VCO_SEL_FORCE);

	/* R30 Bit Definitions */
	st->regs[LMX2582_R30] =
		LMX2582_R30_MASH_DITHER(conf->MASH_DITHER) |
		LMX2582_R30_VTUNE_ADJ(conf->VTUNE_ADJ);

	/* R31 Bit Definitions */
	st->regs[LMX2582_R31] =
		LMX2582_R31_VCO_DISTB_PD(conf->VCO_DISTB_PD) |
		LMX2582_R31_VCO_DISTA_PD(conf->VCO_DISTA_PD) |
		LMX2582_R31_CHDIV_DIST_PD(conf->CHDIV_DIST_PD);

	/* R34 Bit Definitions */
	st->regs[LMX2582_R34] = LMX2582_R34_CHDIV_EN(conf->CHDIV_EN);

	/* R35 Bit Definitions */
	st->regs[LMX2582_R35] =
		LMX2582_R35_CHDIV_SEG2(conf->CHDIV_SEG2) |
		LMX2582_R35_CHDIV_SEG3_EN(conf->CHDIV_SEG3_EN) |
		LMX2582_R35_CHDIV_SEG2_EN(conf->CHDIV_SEG2_EN) |
		LMX2582_R35_CHDIV_SEG1(conf->CHDIV_SEG1) |
		LMX2582_R35_CHDIV_SEG1_EN(conf->CHDIV_SEG1_EN);

	/* R36 Bit Definitions */
	st->regs[LMX2582_R36] =
		LMX2582_R36_CHDIV_DISTB_EN(conf->CHDIV_DISTB_EN) |
		LMX2582_R36_CHDIV_DISTA_EN(conf->CHDIV_DISTA_EN) |
		LMX2582_R36_CHDIV_SEG_SEL(conf->CHDIV_SEG_SEL) |
		LMX2582_R36_CHDIV_SEG3(conf->CHDIV_SEG3);

	/* R37 Bit Definitions */
	st->regs[LMX2582_R37] = LMX2582_R37_PLL_N_PRE(conf->PLL_N_PRE);

	/* R38 Bit Definitions */
	st->regs[LMX2582_R38] = LMX2582_R38_PLL_N(conf->PLL_N);

	/* R39 Bit Definitions */
	st->regs[LMX2582_R39] = LMX2582_R39_PFD_DLY(conf->PFD_DLY);

	/* R40/41 Bit Definitions */
	st->regs[LMX2582_R40] = LMX2582_R40_PLL_DEN_31_16(conf->PLL_DEN);
	st->regs[LMX2582_R41] = LMX2582_R41_PLL_DEN_15_0(conf->PLL_DEN);
	
	/* R42/43 Bit Definitions */
	st->regs[LMX2582_R42] = LMX2582_R42_MASH_SEED_31_16(conf->MASH_SEED);
	st->regs[LMX2582_R43] = LMX2582_R43_MASH_SEED_15_0(conf->MASH_SEED);

	/* R44/45 Bit Definitions */
	st->regs[LMX2582_R44] = LMX2582_R44_PLL_NUM_31_16(conf->PLL_NUM);
	st->regs[LMX2582_R45] = LMX2582_R45_PLL_NUM_15_0(conf->PLL_NUM);

	/* R46 Bit Definitions */
	st->regs[LMX2582_R46] =
		LMX2582_R46_OUTA_POW(conf->OUTA_POW) |
		LMX2582_R46_OUTB_PD(conf->OUTB_PD) |
		LMX2582_R46_OUTA_PD(conf->OUTA_PD) | 
		LMX2582_R46_MASH_ORDER(conf->MASH_ORDER);

	/* R47 Bit Definitions */
	st->regs[LMX2582_R47] =
		LMX2582_R47_OUTA_MUX(conf->OUTA_MUX) |
		LMX2582_R47_OUTB_POW(conf->OUTB_POW);

	/* R48 Bit Definitions */
	st->regs[LMX2582_R48] = LMX2582_R48_OUTB_MUX(conf->OUTB_MUX);

	/* R59 Bit Definitions */
	st->regs[LMX2582_R59] = LMX2582_R59_MUXOUT_HDRV(conf->MUXOUT_HDRV);

	/* R61 Bit Definitions */
	st->regs[LMX2582_R61] = LMX2582_R61_LD_TYPE(conf->LD_TYPE);

	/* R64 Bit Definitions */
	st->regs[LMX2582_R64] =
		LMX2582_R64_ACAL_FAST(conf->ACAL_FAST) |
		LMX2582_R64_FCAL_FAST(conf->FCAL_FAST) |
		LMX2582_R64_AJUMP_SIZE(conf->AJUMP_SIZE) |
		LMX2582_R64_FJUMP_SIZE(conf->FJUMP_SIZE);

	dev_info(&st->spi->dev,
		 "OSC: %llu Hz, VCO: %llu Hz, FPD: %llu Hz, CAL: %llu Hz, "
		 "chdiv: %u, chdiv freq: %lld Hz\n",
		 st->fosc, st->fvco, st->fpd, st->fcal_clk,
		 st->chdiv_total, st->fchdiv);

	ret = lmx2582_sync_config(st);
	wake_up_interruptible(&st->wq_setup_done);
	return ret;
}


/* iio channel attribute write */
static ssize_t lmx2582_write(struct iio_dev *indio_dev,
			     uintptr_t private,
			     const struct iio_chan_spec *chan,
			     const char *buf,
			     size_t len)
{
	struct lmx2582_state *st = iio_priv(indio_dev);
	long long readin;
	int ret;

	ret = kstrtoll(buf, 10, &readin);
	if (ret)
		return ret;

	switch ((u32)private) {
	case LMX2582_CH_ATTR_FREQ:
		st->outputs[chan->channel].frequency = readin;
		break;
	case LMX2582_CH_ATTR_PWRDOWN:
		st->outputs[chan->channel].enabled = readin == 0;
		break;
	default:
		ret = -EINVAL;
	}

	/* update settings */
	if (ret == 0) {
		mutex_lock(&st->lock);
		ret = lmx2582_setup(st, 0);
		mutex_unlock(&st->lock);
	}

	return ret ? ret : len;
}

/* iio channel attribute read */
static ssize_t lmx2582_read(struct iio_dev *indio_dev,
			    uintptr_t private,
			    const struct iio_chan_spec *chan,
			    char *buf)
{
	struct lmx2582_state *st = iio_priv(indio_dev);
	long long val;
	int ret = 0;

	switch ((u32)private) {
	case LMX2582_CH_ATTR_FREQ:
		val = st->outputs[chan->channel].frequency;
		break;
	case LMX2582_CH_ATTR_PWRDOWN:
		val = st->outputs[chan->channel].enabled ? 0 : 1;
		break;
	case LMX2582_CH_ATTR_CHANNEL_NAME:
		return sprintf(buf, "%s\n", lmx2582_ch_names[chan->channel]);
	default:
		ret = -EINVAL;
		val = 0;
	}

	return ret < 0 ? ret : sprintf(buf, "%lld\n", val);
}


/* iio output multiplexer enum  handling */
static int lmx2582_get_outmux(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan)
{
	struct lmx2582_state *st = iio_priv(indio_dev);

	return st->outputs[chan->channel].out_mux;
}

static int lmx2582_set_outmux(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan,
			      unsigned int mode)
{
	struct lmx2582_state *st = iio_priv(indio_dev);

	st->outputs[chan->channel].out_mux = mode & 0x3;
	return lmx2582_setup(st, 0);
}

static const char * const lmx2582_out_mux_names[] = {
	[0] = "channel-divider",
	[1] = "vco",
};
static const struct iio_enum lmx2582_out_mux_available = {
	.items = lmx2582_out_mux_names,
	.num_items = ARRAY_SIZE(lmx2582_out_mux_names),
	.get = lmx2582_get_outmux,
	.set = lmx2582_set_outmux,
};


/* iio channel attributes */
#define _LMX2582_EXT_INFO(_name, _ident, _shared) { \
	.name = _name, \
	.read = lmx2582_read, \
	.write = lmx2582_write, \
	.private = _ident, \
	.shared = _shared, \
}

static const struct iio_chan_spec_ext_info lmx2582_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_LMX2582_EXT_INFO("frequency", LMX2582_CH_ATTR_FREQ, IIO_SEPARATE),
	_LMX2582_EXT_INFO("powerdown", LMX2582_CH_ATTR_PWRDOWN, IIO_SEPARATE),
	_LMX2582_EXT_INFO("name", LMX2582_CH_ATTR_CHANNEL_NAME, IIO_SEPARATE),
	IIO_ENUM("out_mux", IIO_SEPARATE, &lmx2582_out_mux_available),
	IIO_ENUM_AVAILABLE("out_mux", IIO_SHARED_BY_TYPE, &lmx2582_out_mux_available),
	{ },
};

#define LMX2582_CHANNEL(index) { \
	.type = IIO_ALTVOLTAGE, \
	.output = 1, \
	.channel = index, \
	.ext_info = lmx2582_ext_info, \
	.indexed = 1, \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_PHASE), \
}

static const struct iio_chan_spec lmx2582_chan[LMX2582_CLK_COUNT] = {
	LMX2582_CHANNEL(LMX2582_CH_RFOUTA),
	LMX2582_CHANNEL(LMX2582_CH_RFOUTB),
};


/* iio raw device access */
/* required that iio_attr does not crash in attribute enumeration */
static int lmx2582_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long info)
{
	struct lmx2582_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		*val = st->outputs[chan->channel].enabled;
		return IIO_VAL_INT;
	//case IIO_CHAN_INFO_FREQUENCY:
	case IIO_CHAN_INFO_PHASE:
		*val = 0;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		*val = 0;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int lmx2582_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long info)
{
	struct lmx2582_state *st = iio_priv(indio_dev);
	int ret = 0;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		st->outputs[chan->channel].enabled = val;
		break;
	//case IIO_CHAN_INFO_FREQUENCY:
	case IIO_CHAN_INFO_PHASE:
	default:
		return -EINVAL;
	}

	if (ret == 0) {
		mutex_lock(&st->lock);
		ret = lmx2582_setup(st, 0);
		mutex_unlock(&st->lock);
	}
	
	return ret;
}


/* iio device attributes */
static ssize_t lmx2582_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct lmx2582_state *st = iio_priv(indio_dev);
	int ret = 0;
	u64 val = 0;

	/* unique registers */
	switch ((u32)this_attr->address) {
	case LMX2582_ATTR_FPFD:
		val = st->fpd;
		break;
	case LMX2582_ATTR_FVCO:
		val = st->fvco;
		break;
	case LMX2582_ATTR_FOSC:
		val = st->fosc;
		break;
	case LMX2582_ATTR_CHDIV_TOTAL:
		val = st->chdiv_total;
		break;
	case LMX2582_ATTR_FCHDIV:
		val = st->fchdiv;
		break;
	case LMX2582_ATTR_MULT:
		val = st->conf->MULT;
		break;
	case LMX2582_ATTR_PLL_OSC_2X:
		val = st->conf->OSC_2X;
		break;
	case LMX2582_ATTR_PLL_R_PRE:
		val = st->conf->PLL_R_PRE;
		break;
	case LMX2582_ATTR_PLL_R:
		val = st->conf->PLL_R;
		break;
	case LMX2582_ATTR_PLL_N_PRE:
		val = st->conf->PLL_N_PRE;
		break;
	case LMX2582_ATTR_PLL_N:
		val = st->conf->PLL_N;
		break;
	case LMX2582_ATTR_PLL_NUM:
		val = st->conf->PLL_NUM;
		break;
	case LMX2582_ATTR_PLL_DEN:
		val = st->conf->PLL_DEN;
		break;
	case LMX2582_ATTR_CHDIV_SEG1:
		val = st->conf->CHDIV_SEG1;
		break;
	case LMX2582_ATTR_CHDIV_SEG2:
		val = st->conf->CHDIV_SEG2;
		break;
	case LMX2582_ATTR_CHDIV_SEG3:
		val = st->conf->CHDIV_SEG3;
		break;
	case LMX2582_ATTR_CHDIV_SEG_SEL:
		val = st->conf->CHDIV_SEG_SEL;
		break;
	case LMX2582_ATTR_CP_IUP:
		val = st->conf->CP_IUP;
		break;
	case LMX2582_ATTR_CP_IDN:
		val = st->conf->CP_IDN;
		break;
	case LMX2582_ATTR_CP_ICOARSE:
		val = st->conf->CP_ICOARSE;
		break;
	case LMX2582_ATTR_CP_EN:
		val = st->conf->CP_EN;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	if (ret == 0)
		ret = sprintf(buf, "%lld\n", val);

	return ret;
}

static void lmx2582_force_recalc_rate(struct lmx2582_state *st)
{
	int i;

	dev_info(&st->spi->dev, "lmx2582_force_recalc_rate\n");

	for (i = 0; i < LMX2582_CLK_COUNT; i++) {
		/* HACK: Use reparent to trigger notify */
		clk_hw_reparent(&st->outputs[i].hw, NULL);
		dev_dbg(&st->spi->dev, "recalc rate %d: %lu", i,
			clk_get_rate(st->clks[i]));
	}
}

static ssize_t lmx2582_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct lmx2582_state *st = iio_priv(indio_dev);
	int ret = 0;
	u64 val;

	ret = kstrtoull(buf, 0, &val);
	if(ret)
		return ret;

	/* unique registers */
	switch ((u32)this_attr->address) {
	case LMX2582_ATTR_FPFD:
	case LMX2582_ATTR_FVCO:
	case LMX2582_ATTR_FOSC:
	case LMX2582_ATTR_CHDIV_TOTAL:
	case LMX2582_ATTR_FCHDIV:
		ret = -EINVAL;
		break;
	case LMX2582_ATTR_MULT:
		switch(val) {
		case LMX2582_MULT_BYPASS:
		case LMX2582_MULT_3:
		case LMX2582_MULT_4:
		case LMX2582_MULT_5:
		case LMX2582_MULT_6:
			st->conf->MULT = val;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case LMX2582_ATTR_PLL_OSC_2X:
		st->conf->OSC_2X = val ? true : false;
		break;
	case LMX2582_ATTR_PLL_R_PRE:
		if (val < 4096)
			st->conf->PLL_R_PRE = val;
		else
			ret = -EINVAL;
		break;
	case LMX2582_ATTR_PLL_R:
		if (val < 256)
			st->conf->PLL_R = val;
		else
			ret = -EINVAL;
		break;
	case LMX2582_ATTR_PLL_N_PRE:
		switch(val) {
		case LMX2582_PLL_N_PRE_DIV2:
		case LMX2582_PLL_N_PRE_DIV4:
			st->conf->PLL_N_PRE = val;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	case LMX2582_ATTR_PLL_N:
		if (val < 4096)
			st->conf->PLL_N = val;
		else
			ret = -EINVAL;
		break;
	case LMX2582_ATTR_PLL_NUM:
		if (val <= U32_MAX) 
			st->conf->PLL_NUM = val;
		else
			ret = -EINVAL;
		break;
	case LMX2582_ATTR_PLL_DEN:
		if (val <= U32_MAX) 
			st->conf->PLL_DEN = val;
		else
			ret = -EINVAL;
		break;
	case LMX2582_ATTR_CHDIV_SEG1:
		switch(val) {
		case LMX2582_CHDIV_SEG1_DIV2:
		case LMX2582_CHDIV_SEG1_DIV3:
			st->conf->CHDIV_SEG1 = val;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	case LMX2582_ATTR_CHDIV_SEG2:
		switch(val) {
		case LMX2582_CHDIV_SEG2_PD:
		case LMX2582_CHDIV_SEG2_DIV2:
		case LMX2582_CHDIV_SEG2_DIV4:
		case LMX2582_CHDIV_SEG2_DIV6:
		case LMX2582_CHDIV_SEG2_DIV8:
			st->conf->CHDIV_SEG2 = val;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	case LMX2582_ATTR_CHDIV_SEG3:
		switch(val) {
		case LMX2582_CHDIV_SEG3_PD:
		case LMX2582_CHDIV_SEG3_DIV2:
		case LMX2582_CHDIV_SEG3_DIV4:
		case LMX2582_CHDIV_SEG3_DIV6:
		case LMX2582_CHDIV_SEG3_DIV8:
			st->conf->CHDIV_SEG3 = val;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	case LMX2582_ATTR_CHDIV_SEG_SEL:
		switch(val) {
		case LMX2582_CHDIV_SEG_SEL_PD:
		case LMX2582_CHDIV_SEG_SEL_1:
		case LMX2582_CHDIV_SEG_SEL_12:
		case LMX2582_CHDIV_SEG_SEL_123:
			st->conf->CHDIV_SEG_SEL = val;
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	case LMX2582_ATTR_CP_IUP:
		if(val < 32)
			st->conf->CP_IUP = val;
		else
			ret = -EINVAL;
		break;
	case LMX2582_ATTR_CP_IDN:
		if(val < 32)
			st->conf->CP_IDN = val;
		else
			ret = -EINVAL;
		break;
	case LMX2582_ATTR_CP_ICOARSE:
		switch(val) {
		case LMX2582_CP_ICOARSE_X1:
		case LMX2582_CP_ICOARSE_X2:
		case LMX2582_CP_ICOARSE_X1_5:
		case LMX2582_CP_ICOARSE_X2_5:
			st->conf->CP_ICOARSE = val;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case LMX2582_ATTR_CP_EN:
		st->conf->CP_EN = val ? true : false;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	if (ret == 0) {
		mutex_lock(&st->lock);
		ret = lmx2582_setup(st, 0);
		mutex_unlock(&st->lock);
		lmx2582_force_recalc_rate(st);
	}

	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(fvco, S_IRUGO,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_FVCO);

static IIO_DEVICE_ATTR(fpfd, S_IRUGO,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_FPFD);

static IIO_DEVICE_ATTR(fosc, S_IRUGO,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_FOSC);

static IIO_DEVICE_ATTR(chdiv_total, S_IRUGO,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_CHDIV_TOTAL);

static IIO_DEVICE_ATTR(fchdiv, S_IRUGO,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_FCHDIV);

static IIO_DEVICE_ATTR(pll_r, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_PLL_R);

static IIO_DEVICE_ATTR(pll_r_pre, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_PLL_R_PRE);

static IIO_DEVICE_ATTR(osc_2x, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_PLL_OSC_2X);

static IIO_DEVICE_ATTR(multiplier, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_MULT);

static IIO_DEVICE_ATTR(pll_n_pre, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_PLL_N_PRE);

static IIO_DEVICE_ATTR(pll_n, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_PLL_N);

static IIO_DEVICE_ATTR(pll_num, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_PLL_NUM);

static IIO_DEVICE_ATTR(pll_den, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_PLL_DEN);

static IIO_DEVICE_ATTR(chdiv_seg1, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_CHDIV_SEG1);

static IIO_DEVICE_ATTR(chdiv_seg2, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_CHDIV_SEG2);

static IIO_DEVICE_ATTR(chdiv_seg3, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_CHDIV_SEG3);

static IIO_DEVICE_ATTR(chdiv_seg_sel, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_CHDIV_SEG_SEL);

static IIO_DEVICE_ATTR(cp_iup, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_CP_IUP);

static IIO_DEVICE_ATTR(cp_idn, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_CP_IDN);

static IIO_DEVICE_ATTR(cp_icoarse, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_CP_ICOARSE);

static IIO_DEVICE_ATTR(cp_en, S_IRUGO | S_IWUSR,
	lmx2582_show, lmx2582_store, LMX2582_ATTR_CP_EN);


#define ATTR(_x_) &iio_dev_attr_##_x_.dev_attr.attr
static struct attribute *lmx2582_attributes[] = {
	ATTR(fvco),
	ATTR(fpfd),
	ATTR(fosc),
	ATTR(chdiv_total),
	ATTR(fchdiv),
	ATTR(osc_2x),
	ATTR(pll_r_pre),
	ATTR(multiplier),
	ATTR(pll_r),
	ATTR(pll_n_pre),
	ATTR(pll_n),
	ATTR(pll_num),
	ATTR(pll_den),
	ATTR(chdiv_seg1),
	ATTR(chdiv_seg2),
	ATTR(chdiv_seg3),
	ATTR(chdiv_seg_sel),
	ATTR(cp_iup),
	ATTR(cp_idn),
	ATTR(cp_icoarse),
	ATTR(cp_en),
	NULL,
};

static const struct attribute_group lmx2582_attribute_group = {
	.attrs = lmx2582_attributes,
};

static const struct iio_info lmx2582_info = {
	.debugfs_reg_access = &lmx2582_reg_access,
	.read_raw = &lmx2582_read_raw,
	.write_raw = lmx2582_write_raw,
	.attrs = &lmx2582_attribute_group,
};


/* device tree part */
static void lmx2582_property_u32(struct lmx2582_state *st,
				 const char *name,
				 u32 *val, u32 min, u32 max)
{
	int ret;
	u32 tmp;
	bool out_of_range;

	ret = device_property_read_u32(&st->spi->dev, name, &tmp);
	if (ret == 0)
		out_of_range = tmp > max || tmp < min;
	else
		out_of_range = true;

	if (!out_of_range)
			*val = tmp;

	if (ret < 0 || out_of_range)
		dev_warn(&st->spi->dev,
			 "using default value for %s: %u",
			 name,
			 *val);

	if (st->dent)
		debugfs_create_u32(name, 0644, st->dent, val);
}

static void lmx2582_property_u64(struct lmx2582_state *st,
				 const char *name,
				 u64 *val, u64 min, u64 max)
{
	int ret;
	u64 tmp;
	bool out_of_range;

	ret = device_property_read_u64(&st->spi->dev, name, &tmp);
	if (ret == 0)
		out_of_range = tmp > max || tmp < min;
	else
		out_of_range = true;

	if (!out_of_range)
			*val = tmp;

	if (ret < 0 || out_of_range)
		dev_warn(&st->spi->dev,
			 "using default value for %s: %llu",
			 name,
			 *val);

	if (st->dent)
		debugfs_create_u64(name, 0644, st->dent, val);
}

static void lmx2582_property_bool(struct lmx2582_state *st,
				  const char *name,
				  bool *val)
{
	*val = device_property_read_bool(&st->spi->dev, name);

	if (st->dent)
		debugfs_create_bool(name, 0644, st->dent, val);
}

static struct lmx2582_config *lmx2582_parse_dt(struct lmx2582_state *st)
{
	struct lmx2582_config *conf;
	struct fwnode_handle *child;
	unsigned int channel, tmp;
	bool btemp;
	int ret;

	conf = devm_kzalloc(&st->spi->dev, sizeof(*conf), GFP_KERNEL);
	if (!conf)
		return NULL;
	
	/* copy default values */
	memcpy(conf, &lmx2582_default_values, sizeof(*conf));

	lmx2582_property_bool(st, "lmx,muxout-sel-readback", &btemp);
	conf->MUXOUT_SEL = !btemp;

	lmx2582_property_bool(st, "lmx,osc-2x", &conf->OSC_2X);

	lmx2582_property_u32(st, "lmx,mult", &conf->MULT, 1, 6);
	lmx2582_property_u32(st, "lmx,pll-r", &conf->PLL_R, 1, 128);
	lmx2582_property_u32(st, "lmx,pll-r-pre", &conf->PLL_R_PRE, 1, 128);

	lmx2582_property_u32(st, "lmx,pfd-ctl", &conf->PFD_CTL, 0, 3);

	lmx2582_property_bool(st, "lmx,cp-disable", &btemp);
	conf->CP_EN = !btemp;

	lmx2582_property_u32(st, "lmx,cp-idn", &conf->CP_IDN, 0, 31);
	lmx2582_property_u32(st, "lmx,cp-iup", &conf->CP_IUP, 0, 31);
	lmx2582_property_u32(st, "lmx,cp-icoarse", &conf->CP_ICOARSE, 0, 3);

	lmx2582_property_u32(st, "lmx,pll-n-pre", &conf->PLL_N_PRE, 0, 1);
	lmx2582_property_u32(st, "lmx,pll-n", &conf->PLL_N, 0, 4095);
	lmx2582_property_u32(st, "lmx,pll-den", &conf->PLL_DEN, 1, U32_MAX);
	lmx2582_property_u32(st, "lmx,pll-num", &conf->PLL_NUM, 0, U32_MAX);

	lmx2582_property_bool(st, "lmx,muxout-hdrv", &conf->MUXOUT_HDRV);

	lmx2582_property_bool(st, "lmx,ld-type-calstat", &btemp);
	conf->LD_TYPE = !btemp;

	lmx2582_property_u32(st, "lmx,chdiv-seg1", &conf->CHDIV_SEG1, 0, 1);
	lmx2582_property_u32(st, "lmx,chdiv-seg2", &conf->CHDIV_SEG2, 0, 8);
	lmx2582_property_u32(st, "lmx,chdiv-seg3", &conf->CHDIV_SEG3, 0, 8);
	lmx2582_property_u32(st, "lmx,chdiv-seg-sel", &conf->CHDIV_SEG_SEL, 0, 4);

	ret = device_property_read_string_array(&st->spi->dev,
				"clock-output-names",
				st->lmx2582_clk_names,
				LMX2582_CLK_COUNT);

	if (ret < 0) {
		dev_warn(&st->spi->dev, "Using the default clk names");
		st->has_clk_out_names = false;
	} else {
		st->has_clk_out_names = true;
	}

	ret = of_clk_get_scale(st->spi->dev.of_node, NULL, &st->scale);
	if (ret < 0) {
		st->scale.mult = 1;
		st->scale.div = 10;
	}

	device_for_each_child_node(&st->spi->dev, child) {
		ret = fwnode_property_read_u32(child, "reg", &channel);
		if (ret)
			continue;
		if (channel >= LMX2582_CLK_COUNT)
			continue;

		ret = fwnode_property_present(child, "lmx,output-enable");
		st->outputs[channel].enabled = ret;

		st->outputs[channel].power = 15;
		ret = fwnode_property_read_u32(child, "lmx,output-power", &tmp);
		if (ret == 0)
			st->outputs[channel].power = tmp;

		/* Default to Divider */
		st->outputs[channel].out_mux = LMX2582_OUTx_MUX_CHDIV;
		if (fwnode_property_present(child, "lmx,mux-sel-vco"))
			st->outputs[channel].out_mux = LMX2582_OUTx_MUX_VCO;
	}

	return conf;
}


/* worker to execute sleepable work from non-sleepable contexts */
static void lmx2582_workq_handler(struct work_struct *workq)
{
	struct lmx2582_state *st =
		container_of(workq, struct lmx2582_state, workq);
	mutex_lock(&st->lock);
	lmx2582_setup(st, 0);
	mutex_unlock(&st->lock);
}


/* automatic PLL calculation */
/*
	https://e2e.ti.com/support/clock-timing-group/clock-and-timing/f/clock-timing-forum/1394542/lmx2582evm-formula-for-calculations
	After studying the Python Code, we were able to make generalized formula.
	This formula can be used in generalised form with other controller where:

	freq is FoutA_FREQ
	PLL_DEN is fixed to 1000
	F_OSC is 100Mhz
	OSC_2X calculated from Register 9
	DIVIDER value is from CHDIV_SEG1,CHDIV_SEG2,CHDIV_SEG3
	
	Fvco	= (freq * DIVIDER);
	PLL_N_PRE = VCO_2X;
	Fin	= (Fvco * (VCO_2X + 1));
	PreN	= 2 * (PLL_N_PRE + 1);
	Fpd	= (MULT * (F_OSC / R) * (1 + OSC_2X));
	
	FracN	= (Fin / (Fpd * PreN));
	N	= floor(FracN);
	PLL_NUM	= round(PLL_DEN * (FracN - N));
*/

static void scale_ull(unsigned long long in_num, unsigned long long in_den,
		      unsigned long* out_num, unsigned long* out_den)
{
#if ULLONG_MAX == ULONG_MAX
	*out_num = in_num;
	*out_den = in_den;
#else
	unsigned long long num = in_num;
	unsigned long long den = in_den;

	while((num > (ULONG_MAX * 1ULL)) || (den > (ULONG_MAX * 1ULL))) {
		num >>= 1;
		den >>= 1;
	}

	*out_num = num;
	*out_den = den;
#endif
}

static int find_pll_n_divider(struct lmx2582_state* st,
			      unsigned long long freq,
			      bool allow_fractional,
			      u32* pll_n, u32* pll_n_num, u32* pll_n_den,
			      unsigned long long* fout)
{
	const struct lmx2582_channel_divider_min_max *entry;
	unsigned long long fvco;
	unsigned long long n;
	unsigned long long error, fpd_n_pre;
	unsigned long error_ul, fpd_n_pre_ul;
	unsigned long num, den;
	int i;
	
	fpd_n_pre = st->fpd * (st->conf->PLL_N_PRE ? 4 : 2);

	for (i = 0; i < ARRAY_SIZE(lmx2582_divided_clock_spec); i++) {
		entry = &lmx2582_divided_clock_spec[i];

		fvco = freq * entry->total_divider;

		/* check VCO range */
		if(fvco < entry->vco_frequency_range.min)
			continue;
		if(fvco > entry->vco_frequency_range.max)
			continue;

		n = div64_u64(fvco, fpd_n_pre);

		/* check N range */
		if (n < LMX2582_PLL_N_MIN)
			continue;
		if (n > LMX2582_PLL_N_MAX)
			continue;

		/* found candidate */
		error = fvco - n * fpd_n_pre;

		/* integer-only has no error */
		if (!allow_fractional && error != 0)
			continue;

		dev_dbg(&st->spi->dev, "find_pll_n_divider: freq: %lld"
			", n: %lld, vco=%lld, total div: %u, error: %llu\n",
			freq, n, fvco, entry->total_divider, error);

		/* optimize fraction */
		scale_ull(error, fpd_n_pre, &error_ul, &fpd_n_pre_ul);

		rational_best_approximation(error_ul, fpd_n_pre_ul,
					    U32_MAX, U32_MAX,
					    &num, &den);

		dev_dbg(&st->spi->dev,
			"find_pll_n_divider: num: %lu, den: %lu\n",
			num, den);

		/* output values */
		*fout = div_u64(fvco, entry->total_divider);
		*pll_n_num = num;
		*pll_n_den = den;
		*pll_n = n;
		return i;
	}

	/* no entry found */
	return -1;
}

static void lmx2582_apply_settings(struct lmx2582_state* st,
				   int clock_spec_index,
				   u32 pll_n, u32 pll_num, u32 pll_den)
{
	const struct lmx2582_channel_divider_min_max *entry;
	entry = &lmx2582_divided_clock_spec[clock_spec_index];

	st->conf->PLL_N = pll_n;
	st->conf->PLL_NUM = pll_num;
	st->conf->PLL_DEN = pll_den;

	st->conf->CHDIV_SEG1 = entry->chdiv_register_values.seg1;
	st->conf->CHDIV_SEG2 = entry->chdiv_register_values.seg2;
	st->conf->CHDIV_SEG3 = entry->chdiv_register_values.seg3;

	if (entry->chdiv_register_values.seg3 != LMX2582_CHDIV_SEG3_PD)
		st->conf->CHDIV_SEG_SEL = LMX2582_CHDIV_SEG_SEL_123;
	else if (entry->chdiv_register_values.seg2 != LMX2582_CHDIV_SEG2_PD)
		st->conf->CHDIV_SEG_SEL = LMX2582_CHDIV_SEG_SEL_12;
	else
		st->conf->CHDIV_SEG_SEL = LMX2582_CHDIV_SEG_SEL_1;
}

/* clock system integration */
static unsigned long lmx2582_clk_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct lmx2582_state *st = to_clk_priv(hw)->st;
	unsigned long long rate;

	rate = to_clk_priv(hw)->frequency;

	dev_info(&st->spi->dev,
		"lmx2582_clk_recalc_rate rate=%llu, parent_rate=%lu\n",
		rate, parent_rate);

	return to_ccf_scaled(rate, &st->scale);
}

static long lmx2582_clk_round_rate(struct clk_hw *hw,
				   unsigned long rate,
				   unsigned long *parent_rate)
{
	struct lmx2582_state *st = to_clk_priv(hw)->st;
	unsigned long long scaled_rate;
	unsigned long long fout;
	int ret;
	u32 pll_n, pll_num, pll_den;

	scaled_rate = from_ccf_scaled(rate, &st->scale);

	dev_info(&st->spi->dev,
		"lmx2582_clk_round_rate scaled_rate=%llu, parent_rate=%lu\n",
		scaled_rate, parent_rate ? *parent_rate : 0);

	if (!rate)
		return 0;
 
	/* try integer only first */
	ret = find_pll_n_divider(st, scaled_rate, false,
				 &pll_n, &pll_num, &pll_den, &fout);
	if (ret >= 0)
		dev_info(&st->spi->dev, "found integer-N only configuration\n");
	if (ret < 0) {
		dev_warn(&st->spi->dev,
			 "Did not find valid integer pll N divider");

		/* integer only failed. try fractional */
		ret = find_pll_n_divider(st, scaled_rate, true,
					 &pll_n, &pll_num, &pll_den, &fout);

		if (ret < 0) {
			dev_err(&st->spi->dev,
				"Did not find valid fractional pll N divider");
			return -EINVAL;
		}

		dev_info(&st->spi->dev, "found fractional-N configuration\n");
	}

	dev_info(&st->spi->dev, "lmx2582_clk_round_rate: "
		"pll_n=%u, pll_num=%u, pll_den=%u, fout=%llu\n",
		pll_n, pll_num, pll_den, fout);

	return to_ccf_scaled(fout, &st->scale);
}

static int lmx2582_clk_set_rate(struct clk_hw *hw,
				unsigned long rate,
				unsigned long parent_rate)
{
	struct lmx2582_state *st = to_clk_priv(hw)->st;
	unsigned long long scaled_rate;
	unsigned long long fout;
	int ret;
	u32 pll_n, pll_num, pll_den;

	scaled_rate = from_ccf_scaled(rate, &st->scale);

	dev_info(&st->spi->dev,
		"lmx2582_clk_set_rate for rate=%lu and parent_rate=%lu\n",
		rate, parent_rate);

	/* try integer only first */
	ret = find_pll_n_divider(st, scaled_rate, false,
				 &pll_n, &pll_num, &pll_den, &fout);
	if (ret >= 0)
		dev_info(&st->spi->dev, "Found integer-N only configuration\n");
	if (ret < 0) {
		dev_warn(&st->spi->dev,
			 "Did not find valid integer-N only configuration\n");

		/* integer only failed. try fractional */
		ret = find_pll_n_divider(st, scaled_rate, true,
					 &pll_n, &pll_num, &pll_den, &fout);

		if (ret < 0) {
			dev_err(&st->spi->dev, "Did not find valid fractional-N"
				" configuration\n");
			return -EINVAL;
		}

		dev_info(&st->spi->dev, "Found fractional-N configuration\n");
	}

	dev_info(&st->spi->dev, "PLL: N=%u, NUM=%u, DEN=%u, fout=%llu Hz\n",
		 pll_n, pll_num, pll_den, fout);
	
	lmx2582_apply_settings(st, ret, pll_n, pll_num, pll_den);

	return lmx2582_setup(st, parent_rate);
}

static int lmx2582_clk_enable(struct clk_hw *hw)
{
	to_clk_priv(hw)->enabled = true;
	schedule_work(&to_clk_priv(hw)->st->workq);
	return 0;
}

static void lmx2582_clk_disable(struct clk_hw *hw)
{
	to_clk_priv(hw)->enabled = false;
	schedule_work(&to_clk_priv(hw)->st->workq);
}

static int lmx2582_clk_is_enabled(struct clk_hw *hw)
{
	return to_clk_priv(hw)->enabled;
}

static const struct clk_ops lmx2582_clock_ops = {
	.recalc_rate = lmx2582_clk_recalc_rate,
	.round_rate = lmx2582_clk_round_rate,
	.set_rate = lmx2582_clk_set_rate,
	.enable = lmx2582_clk_enable,
	.disable = lmx2582_clk_disable,
	.is_enabled = lmx2582_clk_is_enabled,
};


/* clock subsystem cleanup */
static void lmx2582_of_clk_del_provider(void *data)
{
	struct lmx2582_state *st = data;

	of_clk_del_provider(st->spi->dev.of_node);
}

static void lmx2582_action_clk_disable(void *data)
{
	struct clk *clk = data;

	clk_disable_unprepare(clk);
}

static void lmx2582_powerdown(void *data)
{
	struct lmx2582_state *st = data;

	mutex_lock(&st->lock);
	st->regs[LMX2582_R0] |= LMX2582_R0_POWERDOWN(1);
	lmx2582_sync_config(st);
	mutex_unlock(&st->lock);
}

/* register output clocks */
static int lmx2582_clk_register(struct iio_dev *indio_dev,
				unsigned int channel,
				const char *parent_name)
{
	struct lmx2582_state *st = iio_priv(indio_dev);
	struct clk_init_data init;
	struct clk *clk_out;
	char name[128];

	if (!st->has_clk_out_names) {
		snprintf(name, sizeof(name),
			 "%s_RFout%c", indio_dev->name,
			 'A' + channel);
		st->lmx2582_clk_names[channel] = name;
	}
	init.name = st->lmx2582_clk_names[channel];
	init.ops = &lmx2582_clock_ops;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);
	init.flags = CLK_GET_RATE_NOCACHE;

	st->outputs[channel].hw.init = &init;
	st->outputs[channel].indio_dev = indio_dev;
	st->outputs[channel].st = st;
	st->outputs[channel].num = channel;

	clk_out = devm_clk_register(&st->spi->dev, &st->outputs[channel].hw);
	if (IS_ERR(clk_out))
		return PTR_ERR(clk_out);

	st->clks[channel] = clk_out;

	return 0;
}

static int lmx2582_clks_register(struct iio_dev *indio_dev)
{
	struct lmx2582_state *st = iio_priv(indio_dev);
	int i, ret;

	st->clk_data.clks = devm_kcalloc(&st->spi->dev,
					 LMX2582_CLK_COUNT,
					 sizeof(struct clk *), GFP_KERNEL);
	if (!st->clk_data.clks)
		return -ENOMEM;

	for (i = 0; i < LMX2582_CLK_COUNT; i++) {
		ret = lmx2582_clk_register(indio_dev, i,
					   __clk_get_name(st->clkin));
		if (ret < 0) {
			dev_err(&st->spi->dev,
				"Clock provider register failed\n");
			return ret;
		}
	}

	st->clk_data.clks = st->clks;
	st->clk_data.clk_num = LMX2582_CLK_COUNT;
	ret = of_clk_add_provider(st->spi->dev.of_node,
				  of_clk_src_onecell_get, &st->clk_data);
	if (ret < 0)
		return ret;

	return devm_add_action_or_reset(&st->spi->dev,
					lmx2582_of_clk_del_provider, st);
}


/* device probe */
static int lmx2582_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct iio_dev *indio_dev;
	struct lmx2582_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->clkin = devm_clk_get(&spi->dev, "clkin");
	if (IS_ERR(st->clkin))
		return -EPROBE_DEFER;

	ret = clk_prepare_enable(st->clkin);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev,
		lmx2582_action_clk_disable, st->clkin);
	if (ret)
		return ret;

	spi_set_drvdata(spi, indio_dev);
	st->indio_dev = indio_dev;
	st->spi = spi;
	init_waitqueue_head(&st->wq_setup_done);
	mutex_init(&st->lock);
	INIT_WORK(&st->workq, lmx2582_workq_handler);

	if (spi->dev.of_node)
		indio_dev->name = spi->dev.of_node->name;
	else
		indio_dev->name = id->name;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &lmx2582_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = &lmx2582_chan[0];
	indio_dev->num_channels = LMX2582_CLK_COUNT;

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret)
		return ret;

	st->dent = iio_get_debugfs_dentry(indio_dev);

	st->conf = lmx2582_parse_dt(st);
	if (!st->conf)
		return -ENOMEM;

	ret = lmx2582_soft_reset(st);
	if (ret)
		return ret;
	
	ret = lmx2582_setup(st, 0);
	if (ret)
		return ret;

	ret = lmx2582_clks_register(indio_dev);
	if (ret)
		return ret;

	return devm_add_action_or_reset(&spi->dev, lmx2582_powerdown, st);
}


/* driver registration */
static const struct spi_device_id lmx2582_id[] = {
	{"lmx2582", 0},
	{}
};

static struct spi_driver lmx2582_driver = {
	.driver = {
		.name	= "lmx2582",
	},
	.probe		= lmx2582_probe,
	.id_table	= lmx2582_id,
};
module_spi_driver(lmx2582_driver);

MODULE_AUTHOR("Philipp Diethelm");
MODULE_DESCRIPTION("Texas Instruments LMX2582 SPI PLL");
MODULE_LICENSE("GPL v2");
