/*
 * board.c
 *
 * Copyright (C) IMAGO Technologies GmbH - <https://www.imago-technologies.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <bloblist.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <environment.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <asm/arch/imx8mm_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <spl.h>
#include <asm/mach-imx/dma.h>
#include <power/pmic.h>
#include <usb.h>
#include <sec_mipi_dsim.h>
#include <imx_mipi_dsi_bridge.h>
#include <mipi_dsi_panel.h>
#include <asm/mach-imx/video.h>
#include "vspv3_board.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_ODE | PAD_CTL_PUE | PAD_CTL_PE)

static iomux_v3_cfg_t const uart_pads[] = {
	IMX8MM_PAD_UART2_RXD_UART2_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MM_PAD_UART2_TXD_UART2_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const wdog_pads[] = {
	IMX8MM_PAD_GPIO1_IO02_WDOG1_WDOG_B  | MUX_PAD_CTRL(WDOG_PAD_CTRL),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));

	init_uart_clk(1);

	return 0;
}

int dram_init(void)
{
	struct BloblistInfo *pBloblistInfo;

	/* get RAM size from bloblist stored by SPL */
	pBloblistInfo = bloblist_find(4711, sizeof(*pBloblistInfo));
	if (pBloblistInfo == NULL) {
		printf("Error reading bloblist info from SPL.\n");
		gd->ram_size = 0x40000000;
	} else {
		gd->ram_size = pBloblistInfo->ram_size;
	}

	/* rom_pointer[1] contains the size of TEE occupies */
	gd->ram_size -= rom_pointer[1];

	return 0;
}

int dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM;
	gd->bd->bi_dram[0].size = gd->ram_size;

	return 0;
}

static int tca7408_init(void)
{
	const unsigned char chip_addr = 0x44;
	unsigned char buf = 0;
	struct udevice *dev;
	int ret;

	ret = i2c_get_chip_for_busnum(3, chip_addr, 1, &dev);
	if (ret) {
		printf("%s: Cannot find udev for a bus 3\n", __func__);
		return -1;
	}

	// read device id
	if (dm_i2c_read(dev, 0x01, &buf, 1)) {
		printf("%s: error reading device id\n", __func__);
		return -1;
	}
	if ((buf & 0x40) != 0x40) {
		printf("%s: invalid device id\n", __func__);
		return -1;
	}

	// Pull-Up/-Down select
	buf = 0x80;
	if (dm_i2c_write(dev, 0x0d, &buf, 1)) {
		printf("%s: error setting output register\n", __func__);
		return -1;
	}

	// output high-impedance
	buf = 0x00;
	if (dm_i2c_write(dev, 0x07, &buf, 1)) {
		printf("%s: error setting output high-impedance register\n", __func__);
		return -1;
	}

	// output register
	buf = 0x00;
	if (dm_i2c_write(dev, 0x05, &buf, 1)) {
		printf("%s: error setting output register\n", __func__);
		return -1;
	}

	// I/O direction
	buf = 0x01;
	if (dm_i2c_write(dev, 0x03, &buf, 1)) {
		printf("%s: error setting I/O direction\n", __func__);
		return -1;
	}
	
	return 0;
}

static int tca7408_get_input(void)
{
	const unsigned char chip_addr = 0x44;
	unsigned char buf = 0;
	struct udevice *dev;
	int ret;

	ret = i2c_get_chip_for_busnum(3, chip_addr, 1, &dev);
	if (ret) {
		printf("%s: Cannot find udev for a bus 3\n", __func__);
		return -1;
	}

	// read intput state
	if (dm_i2c_read(dev, 0x0f, &buf, 1)) {
		printf("%s: error reading input state\n", __func__);
		return -1;
	}
	
	return buf;
}

static int tca7408_set_output(unsigned char value)
{
	const unsigned char chip_addr = 0x44;
	struct udevice *dev;
	int ret;

	ret = i2c_get_chip_for_busnum(3, chip_addr, 1, &dev);
	if (ret) {
		printf("%s: Cannot find udev for a bus 3\n", __func__);
		return -1;
	}

	// output register
	if (dm_i2c_write(dev, 0x05, &value, 1)) {
		printf("%s: error setting output register\n", __func__);
		return -1;
	}
	
	return 0;
}

#define PCIE_RST_PAD IMX_GPIO_NR(4, 21)

static iomux_v3_cfg_t const pcie_rst_pads[] = {
	IMX8MM_PAD_SAI2_RXFS_GPIO4_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static int setup_pcie(void *fdt)
{
	const char *path;
	int offs;
	int ret = tca7408_init();
	const char *fdt_val = "disabled";
	
	if (ret < 0)
		return ret;

	ret = tca7408_get_input();
	if (ret < 0)
		return ret;
	if ((ret & 0x80) == 0) {
		// pull PCIe reset low
		imx_iomux_v3_setup_multiple_pads(pcie_rst_pads,
						 ARRAY_SIZE(pcie_rst_pads));
		gpio_request(PCIE_RST_PAD, "pcie_rst");
		gpio_direction_output(PCIE_RST_PAD, 0);

		// enable supply
		tca7408_set_output(0x01);
		printf("   PCIe enabled.\n");
		fdt_val = "okay";
	}
	else
		tca7408_set_output(0x00);

	path = "/hsio/pcie@33800000";
	offs = fdt_path_offset(fdt, path);
	if (offs < 0) {
		printf("%s(): node %s not found.\n", __func__, path);
		return offs;
	}

	ret = fdt_setprop_string(fdt, offs, "status", fdt_val);
	if (ret < 0) {
		printf("%s(): fdt_setprop_string(): %s\n", __func__, fdt_strerror(ret));
		return ret;
	}
	
	return 0;
}

static int disable_fpga(void *fdt)
{
	const char *path;
	int offs, ret;

	path = "/ecspi@30830000/imago-fpga@0";
	offs = fdt_path_offset(fdt, path);
	if (offs < 0) {
		printf("%s(): node %s not found.\n", __func__, path);
		return offs;
	}

	ret = fdt_setprop_string(fdt, offs, "status", "disabled");
	if (ret < 0) {
		printf("%s(): fdt_setprop_string(): %s\n", __func__, fdt_strerror(ret));
		return ret;
	}

	printf("   FPGA disabled (SPI).\n");

	return 0;
}

static int enable_usb(void *fdt)
{
	const char *path;
	int offs, ret;

	path = "/usb@32e40000";
	offs = fdt_path_offset(fdt, path);
	if (offs < 0) {
		printf("%s(): node %s not found.\n", __func__, path);
		return offs;
	}

	ret = fdt_setprop_string(fdt, offs, "status", "okay");
	if (ret < 0) {
		printf("%s(): fdt_setprop_string(): %s\n", __func__, fdt_strerror(ret));
		return ret;
	}

	printf("   USB enabled.\n");

	return 0;
}

static int setup_mipi_csi(void *fdt, unsigned int lanes, unsigned int clk_hs_settle)
{
	const char *path;
	int offs, ret;

	path = "/mipi_csi@32e30000/port/endpoint@1";
	offs = fdt_path_offset(fdt, path);
	if (offs < 0) {
		printf("%s(): node %s not found.\n", __func__, path);
		return offs;
	}

	ret = fdt_setprop_u32(fdt, offs, "data-lanes", lanes);
	if (ret < 0) {
		printf("%s(): fdt_setprop_u32(): %s\n", __func__, fdt_strerror(ret));
		return ret;
	}
	ret = fdt_setprop_u32(fdt, offs, "csis-hs-settle", clk_hs_settle);
	if (ret < 0) {
		printf("%s(): fdt_setprop_u32(): %s\n", __func__, fdt_strerror(ret));
		return ret;
	}

	printf("   MIPI CSI: %u lanes, clk_hs_settle = %u.\n", lanes, clk_hs_settle);

	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ToggleTRST(int toggle);
int ft_board_setup(void *fdt, bd_t *bd)
{
	struct BloblistInfo *pBloblistInfo;

	/* get bloblist stored by SPL */
	pBloblistInfo = bloblist_find(4711, sizeof(*pBloblistInfo));
	if (pBloblistInfo == NULL) {
		printf("Error reading bloblist info from SPL.\n");
		return 0;
	}

	if (pBloblistInfo->board_cfg & BOARD_CFG_PCIE_M2)
		setup_pcie(fdt);

	if (pBloblistInfo->board_type == BOARD_TYPE_VSPV3_JMS)
	{
		const char *path;
		int offs, ret;

		disable_fpga(fdt);
		enable_usb(fdt);
		setup_mipi_csi(fdt, 4, 8);

		path = "/csi1_bridge@32e20000";
		offs = fdt_path_offset(fdt, path);
		if (offs < 0) {
			printf("%s(): node %s not found.\n", __func__, path);
		}
		else {
			ret = fdt_setprop_empty(fdt, offs, "imago,event-mode");
			if (ret < 0) {
				printf("%s(): fdt_setprop_empty(): %s\n", __func__, fdt_strerror(ret));
			}
		}
	}

	return 0;
}
#endif

#ifdef CONFIG_FEC_MXC
#define FEC_RST_PAD IMX_GPIO_NR(4, 22)
static iomux_v3_cfg_t const fec1_rst_pads[] = {
	IMX8MM_PAD_SAI2_RXC_GPIO4_IO22 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_fec(void)
{
	imx_iomux_v3_setup_multiple_pads(fec1_rst_pads,
					 ARRAY_SIZE(fec1_rst_pads));

	gpio_request(FEC_RST_PAD, "fec1_rst");
	gpio_direction_output(FEC_RST_PAD, 0);
	udelay(500);
	gpio_direction_output(FEC_RST_PAD, 1);
}

static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

	setup_iomux_fec();

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&gpr->gpr[1],
			IOMUXC_GPR_GPR1_GPR_ENET1_TX_CLK_SEL_MASK, 0);
	return set_clk_enet(ENET_125MHZ);
}

int board_phy_config(struct phy_device *phydev)
{
	/* enable rgmii rxc skew and phy mode select to RGMII copper */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	// rgmii rx clock delay enable
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x00);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x82ee);
	
	// rgmii tx clock delay enable
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	// LED_ACT: active
	phy_write(phydev, MDIO_DEVAD_NONE, 0x19, 0x2000);

	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif


int board_init(void)
{

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

	return 0;
}

static void board_set_ethaddr(void)
{
	uint8_t mac_addr[6];

	eeprom_init(CONFIG_SYS_I2C_EEPROM_BUS);

	if (eeprom_read(0x50, 68, mac_addr, 6))
	{
		printf("Error: Could not read EEPROM content!\n");
		return;
	}

	if (is_valid_ethaddr(mac_addr))
	{
		char buf[7];
		sprintf(buf, "%pM", mac_addr);
		env_set("ethaddr", buf);
	}
}

int board_late_init(void)
{
	struct BloblistInfo *pBloblistInfo;
	board_set_ethaddr();

	/* get bloblist stored by SPL */
	pBloblistInfo = bloblist_find(4711, sizeof(*pBloblistInfo));
	if (pBloblistInfo == NULL) {
		printf("Error reading bloblist info from SPL.\n");
		return 0;
	}

	if (pBloblistInfo->board_type == BOARD_TYPE_VSPV3_JMS)
	{
		printf("Loading sensor FPGA...\n");
		fpga_init("/jms_sensor.iea", "/jms_sensor.ied");
	}

	return 0;
}

phys_size_t get_effective_memsize(void)
{
	return gd->ram_size;
}
