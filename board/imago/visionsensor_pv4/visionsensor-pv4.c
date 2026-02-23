// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2025-2026 IMAGO Technologies GmbH
 */

#include <env_internal.h>
#include <env.h>
#include <init.h>
#include <fdt_support.h>
#include <asm/arch/clock.h>
#include <usb.h>
#include <dwc3-uboot.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <power/regulator.h>
#include <scmi_agent.h>
#include "../dts/upstream/src/arm64/freescale/imx95-power.h"
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <dm/uclass.h>
#include <dm/uclass-internal.h>

extern int board_fix_fdt_fuse(void *fdt);

int board_early_init_f(void)
{
	init_uart_clk(1);

	return 0;
}

static int imx9_scmi_power_domain_enable(u32 domain, bool enable)
{
	struct udevice *dev;
	int ret;

	ret = uclass_get_device_by_name(UCLASS_CLK, "protocol@14", &dev);
	if (ret)
		return ret;

	return scmi_pwd_state_set(dev, 0, domain, enable ? 0 : BIT(30));
}

enum env_location env_get_location(enum env_operation op, int prio)
{
	if (prio == 0)
		return ENVL_FAT;
	else
		return ENVL_UNKNOWN;
}

int board_init(void)
{
	imx9_scmi_power_domain_enable(IMX95_PD_CAMERA, false);

/*	imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, true);
	pci_init(); */

/*	power_on_m7("mx95evkrpmsg");*/

	return 0;
}

int board_late_init(void)
{
	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP

static int fdt_find_and_setprop_u32(void *fdt, const char *node, const char *prop, uint32_t val, int create)
{
	fdt32_t tmp = cpu_to_fdt32(val);
	int ret = fdt_find_and_setprop(fdt, node, prop, &tmp, sizeof(tmp), create);

	if (ret < 0)
		printf("   dtb: error setting property %s/%s = \"%u\"\n", node, prop, val);
	else
		printf("   dtb: setting property %s/%s = \"%u\"\n", node, prop, val);

	return ret;
}

static int fdt_find_and_setprop_string(void *fdt, const char *node, const char *prop, const char *val)
{
	int ret = fdt_find_and_setprop(fdt, node, prop, val, strlen(val)+1, 0);

	if (ret < 0)
		printf("   dtb: error setting property %s/%s = \"%s\"\n", node, prop, val);
	else
		printf("   dtb: setting property %s/%s = \"%s\"\n", node, prop, val);

	return ret;
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	u32 rev_major = (get_cpu_rev() & 0x000F0) >> 4;

	// update NEO ISP node for rev. A1
	if (rev_major < 2)
	{
		fdt_find_and_setprop_string(blob, "/soc/isp@4ae00000", "compatible", "nxp,imx95-a0-neoisp");
	}

	return 0;
}
#endif

#if 0
void board_quiesce_devices(void)
{
	int ret;

	ret = imx9_scmi_power_domain_enable(IMX95_PD_HSIO_TOP, false);
	if (ret) {
		printf("%s: Failed for HSIO MIX: %d\n", __func__, ret);
		return;
	}
}
#endif

#if IS_ENABLED(CONFIG_OF_BOARD_FIXUP)

int board_fix_fdt(void *fdt)
{
	/* Remove nodes based on fuses. */
	board_fix_fdt_fuse(fdt);

	return 0;
}

#endif

int scmi_misc_ddrinfo(u32 ddrc_id, struct scmi_ddr_info_out *out);

int board_phys_sdram_size(phys_size_t *size)
{
	struct scmi_ddr_info_out ddr_info;
	int ret;
	u32 ddrc_id = 0, ddrc_num = 1;
	phys_size_t start, end;
	u32 rev_major = (get_cpu_rev() & 0x000F0) >> 4;

	if (!size)
		return -EINVAL;

	*size = 0;
	
	// Rev. A1: the old System Manager doesn't support reading DDR info,
	// but we know we have 8GB installed:
	if (rev_major < 2)
	{
		// ~2GB + 6GB, PHYS_SDRAM_2_SIZE is not set to leave the default at 2GB:
		*size = PHYS_SDRAM_SIZE + 0x180000000UL;
		return 0;
	}

	do {
		ret = scmi_misc_ddrinfo(ddrc_id++, &ddr_info);
		if (ret) {
			/* if get DDR info failed, fall to default config */
			*size = PHYS_SDRAM_SIZE;
#ifdef PHYS_SDRAM_2_SIZE
			*size += PHYS_SDRAM_2_SIZE;
#endif
			return 0;
		} else {
			ddrc_num = ((ddr_info.attributes >> 16) & 0x3);
			start = ddr_info.starthigh;
			start <<= 32;
			start += ddr_info.startlow;

			end = ddr_info.endhigh;
			end <<= 32;
			end += ddr_info.endlow;

			*size += end + 1 - start;

			debug("ddr info attr 0x%x, start 0x%x 0x%x, end 0x%x 0x%x, mts %u\n",
				ddr_info.attributes, ddr_info.starthigh, ddr_info.startlow,
				ddr_info.endhigh, ddr_info.endlow, ddr_info.mts);
		}
	} while (ddrc_id < ddrc_num);

	/* SM reports total DDR size, need remove secure memory */
	*size -= PHYS_SDRAM - 0x80000000;

	return 0;
}
