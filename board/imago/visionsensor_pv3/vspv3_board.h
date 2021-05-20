/*
 * visionsensor_pv3.h
 *
 * Copyright (C) IMAGO Technologies GmbH - <https://www.imago-technologies.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __VSPV3_BOARD_H__
#define __VSPV3_BOARD_H__

enum BOARD_TYPE {
	BOARD_TYPE_VSPV3_revA = 0,
	BOARD_TYPE_VSPV3 = 1,
	BOARD_TYPE_VSPV3_5MP = 2,
	BOARD_TYPE_VSPV3_JMS = 3,
};

struct BloblistInfo {
	phys_size_t ram_size;
	int board_type;
	int board_cfg;
};

#define BOARD_CFG_2GB     0x0
#define BOARD_CFG_1GB     0x1
#define BOARD_CFG_PCIE_M2 0x2

int fpga_init(const char *algo_file, const char *data_file);

#endif
