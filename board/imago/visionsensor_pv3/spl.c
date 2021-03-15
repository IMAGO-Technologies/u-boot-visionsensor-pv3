/*
 * spl.c
 *
 * Copyright (C) IMAGO Technologies GmbH - <https://www.imago-technologies.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <spl.h>
#include <asm/io.h>
#include <errno.h>
#include <bloblist.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/sys_proto.h>
#include <power/pmic.h>
#include <power/bd71837.h>
#include <asm/arch/clock.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <asm/arch/ddr.h>
#include "vspv3_board.h"

DECLARE_GLOBAL_DATA_PTR;

#define GPIO1_7 IMX_GPIO_NR(1, 7)
#define GPIO1_8 IMX_GPIO_NR(1, 8)
#define GPIO1_9 IMX_GPIO_NR(1, 9)

static int get_board_type(void)
{
	int result;
	unsigned char type = 0;
	
	gpio_request(GPIO1_7, "gpio1 7");
	gpio_request(GPIO1_8, "gpio1 8");
	gpio_request(GPIO1_9, "gpio1 9");
	gpio_direction_input(GPIO1_7);
	gpio_direction_input(GPIO1_8);
	gpio_direction_input(GPIO1_9);

	result = gpio_get_value(GPIO1_7);
	if (result < 0) {
		printf("%s: error reading GPIO1_7\n", __func__);
		return -1;
	}
	type |= result;
	
	result = gpio_get_value(GPIO1_8);
	if (result < 0) {
		printf("%s: error reading GPIO1_8\n", __func__);
		return -1;
	}
	type |= (result << 1);

	result = gpio_get_value(GPIO1_9);
	if (result < 0) {
		printf("%s: error reading GPIO1_9\n", __func__);
		return -1;
	}
	type |= (result << 2);
	
	return type;
}

static int get_board_config(void)
{
	const unsigned char chip_addr = 0x43;
	unsigned char buf = 0;

	i2c_set_bus_num(3);
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
	if (i2c_probe(chip_addr)) {
		return -1;
	}

	// read device id
	if (i2c_read(chip_addr, 0x01, 1, &buf, 1)) {
		return -1;
	}
	if ((buf & 0x40) != 0x40) {
		return -1;
	}

	// select Pull-Up
	buf = 0xff;
	if (i2c_write(chip_addr, 0x0d, 1, &buf, 1)) {
		return -1;
	}

	// enable Pull-Up
	buf = 0xff;
	if (i2c_write(chip_addr, 0x0b, 1, &buf, 1)) {
		return -1;
	}

	// wait for stable signal
	udelay(100);

	// read intput state
	if (i2c_read(chip_addr, 0x0f, 1, &buf, 1)) {
		return -1;
	}

	return buf;
}

extern struct dram_timing_info dram_timing_2gb;

void spl_dram_init(void)
{
	int has2gb = 0;
	phys_size_t ram_size = 0x40000000;
	int board_type = get_board_type();
	struct BloblistInfo *pBloblistInfo;
	int board_cfg = 0;

	switch (board_type) {
		case BOARD_TYPE_VSPV3_revA:
			printf("Board: LK1193 rev.A\n");
			// GPIO expander doesn't work for rev. A, just return with known value:
			board_cfg = 0x21;
			break;
		case BOARD_TYPE_VSPV3:
			printf("Board: LK1193\n");
			board_cfg = get_board_config();
			break;
		case BOARD_TYPE_VSPV3_5MP:
			printf("Board: LK1203\n");
			board_cfg = get_board_config();
			break;
		default:
			printf("Board: unknown (%d)\n", board_type);
			board_cfg = 0x21;
			break;
	}

	/* read RAM size from GPIO expander */
	if (board_cfg < 0)
		printf("Error reading GPIO expander, assuming 1 GB RAM configuration.\n");
	else
		has2gb = (board_cfg & 0x01) == 0;

	/* RAM initialization */
	if (has2gb) {
		ram_size = 0x80000000;
		ddr_init(&dram_timing_2gb);
	} else {
		ddr_init(&dram_timing);
	}

	/* store memory size in bloblist for regular U-Boot */
	pBloblistInfo = bloblist_add(4711, sizeof(*pBloblistInfo));
	if (pBloblistInfo == NULL) {
		printf("Error adding bloblist.\n");
	}
	else {
		pBloblistInfo->ram_size = ram_size;
		pBloblistInfo->board_type = board_type;
		pBloblistInfo->board_cfg = board_cfg;
	}
}

#define I2C_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PE)
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C1 */
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = IMX8MM_PAD_I2C1_SCL_I2C1_SCL | PC,
		.gpio_mode = IMX8MM_PAD_I2C1_SCL_GPIO5_IO14 | PC,
		.gp = IMX_GPIO_NR(5, 14),
	},
	.sda = {
		.i2c_mode = IMX8MM_PAD_I2C1_SDA_I2C1_SDA | PC,
		.gpio_mode = IMX8MM_PAD_I2C1_SDA_GPIO5_IO15 | PC,
		.gp = IMX_GPIO_NR(5, 15),
	},
};

/* I2C4 */
struct i2c_pads_info i2c_pad_info4 = {
	.scl = {
		.i2c_mode = IMX8MM_PAD_I2C4_SCL_I2C4_SCL | PC | ((iomux_v3_cfg_t)(IOMUX_CONFIG_SION) << MUX_MODE_SHIFT),
		.gpio_mode = IMX8MM_PAD_I2C4_SCL_GPIO5_IO20 | PC,
		.gp = IMX_GPIO_NR(5, 20),
	},
	.sda = {
		.i2c_mode = IMX8MM_PAD_I2C4_SDA_I2C4_SDA | PC | ((iomux_v3_cfg_t)(IOMUX_CONFIG_SION) << MUX_MODE_SHIFT),
		.gpio_mode = IMX8MM_PAD_I2C4_SDA_GPIO5_IO21 | PC,
		.gp = IMX_GPIO_NR(5, 21),
	},
};

#define USDHC2_CD_GPIO	IMX_GPIO_NR(2, 18)
#define USDHC2_PWR_GPIO IMX_GPIO_NR(2, 19)

#define USDHC_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE |PAD_CTL_PE | \
			 PAD_CTL_FSEL2)
#define USDHC_GPIO_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_DSE1)

static iomux_v3_cfg_t const usdhc3_pads[] = {
	IMX8MM_PAD_NAND_WE_B_USDHC3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_WP_B_USDHC3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA04_USDHC3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA05_USDHC3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA06_USDHC3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_DATA07_USDHC3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_RE_B_USDHC3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_CE2_B_USDHC3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_CE3_B_USDHC3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_NAND_CLE_USDHC3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	IMX8MM_PAD_SD2_CLK_USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_CMD_USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA0_USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA1_USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA2_USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_DATA3_USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MM_PAD_SD2_RESET_B_GPIO2_IO19 | MUX_PAD_CTRL(USDHC_GPIO_PAD_CTRL),
};

/*
 * The evk board uses DAT3 to detect CD card plugin,
 * in u-boot we mux the pin to GPIO when doing board_mmc_getcd.
 */
static iomux_v3_cfg_t const usdhc2_cd_pad =
	IMX8MM_PAD_SD2_DATA3_GPIO2_IO18 | MUX_PAD_CTRL(USDHC_GPIO_PAD_CTRL);

static iomux_v3_cfg_t const usdhc2_dat3_pad =
	IMX8MM_PAD_SD2_DATA3_USDHC2_DATA3 |
	MUX_PAD_CTRL(USDHC_PAD_CTRL);


static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC2_BASE_ADDR, 0, 1},
	{USDHC3_BASE_ADDR, 0, 1},
};

 static iomux_v3_cfg_t const gpio_pads[] = {
	IMX8MM_PAD_GPIO1_IO07_GPIO1_IO7 | MUX_PAD_CTRL(PAD_CTL_PUE | PAD_CTL_PE),
	IMX8MM_PAD_GPIO1_IO08_GPIO1_IO8 | MUX_PAD_CTRL(PAD_CTL_PE),
	IMX8MM_PAD_GPIO1_IO09_GPIO1_IO9 | MUX_PAD_CTRL(PAD_CTL_PE),
};

int board_mmc_init(bd_t *bis)
{
	int i, ret;
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    USDHC1
	 * mmc1                    USDHC2
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			init_clk_usdhc(1);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			gpio_request(USDHC2_PWR_GPIO, "usdhc2_reset");
			gpio_direction_output(USDHC2_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC2_PWR_GPIO, 1);
			break;
		case 1:
			init_clk_usdhc(2);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC3_BASE_ADDR:
		ret = 1;
		break;
	case USDHC2_BASE_ADDR:
		imx_iomux_v3_setup_pad(usdhc2_cd_pad);
		gpio_request(USDHC2_CD_GPIO, "usdhc2 cd");
		gpio_direction_input(USDHC2_CD_GPIO);

		/*
		 * Since it is the DAT3 pin, this pin is pulled to
		 * low voltage if no card
		 */
		ret = gpio_get_value(USDHC2_CD_GPIO);

		imx_iomux_v3_setup_pad(usdhc2_dat3_pad);
		return ret;
	}

	return 1;
}

#ifdef CONFIG_POWER
#define I2C_PMIC	0
int power_init_board(void)
{
	struct pmic *p;
	int ret;

	ret = power_bd71837_init(I2C_PMIC);
	if (ret)
		printf("power init failed");

	p = pmic_get("BD71837");
	pmic_probe(p);


	/* decrease RESET key long push time from the default 10s to 10ms */
	pmic_reg_write(p, BD71837_PWRONCONFIG1, 0x0);

	/* unlock the PMIC regs */
	pmic_reg_write(p, BD71837_REGLOCK, 0x1);

	/* increase VDD_SOC to typical value 0.82V (no PCIe, else 0.85V) before first DRAM access */
	pmic_reg_write(p, BD71837_BUCK1_VOLT_RUN, 0x0c);

	/* increase VDD_DRAM to 0.975v for 3Ghz DDR */
	pmic_reg_write(p, BD71837_BUCK5_VOLT, 0x83);

#ifndef CONFIG_IMX8M_LPDDR4
	/* increase NVCC_DRAM_1V2 to 1.2v for DDR4 */
	pmic_reg_write(p, BD71837_BUCK8_VOLT, 0x28);
#endif

	/* lock the PMIC regs */
	pmic_reg_write(p, BD71837_REGLOCK, 0x11);

	return 0;
}
#endif

void spl_board_init(void)
{
#ifndef CONFIG_SPL_USB_SDP_SUPPORT
	/* Serial download mode */
	if (is_usb_boot()) {
		puts("Back to ROM, SDP\n");
		restore_boot_params();
	}
#endif
	puts("Normal Boot\n");
}

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	/* Just empty function now - can't decide what to choose */
	debug("%s: %s\n", __func__, name);

	return 0;
}
#endif

void board_init_f(ulong dummy)
{
	int ret;

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	arch_cpu_init();

	board_early_init_f();

	timer_init();

	preloader_console_init();

	ret = spl_init();
	if (ret) {
		debug("spl_init() failed: %d\n", ret);
		hang();
	}

	imx_iomux_v3_setup_multiple_pads(gpio_pads, ARRAY_SIZE(gpio_pads));

	// TrustZone
	enable_tzc380();

	/* Adjust pmic voltage to 1.0V for 800M */
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(3, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info4);

	i2c_set_bus_num(0);

	power_init_board();

	/* DDR initialization */
	spl_dram_init();

	board_init_r(NULL, 0);
}
