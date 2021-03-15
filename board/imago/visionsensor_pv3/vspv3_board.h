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
};

struct BloblistInfo {
	phys_size_t ram_size;
	int board_type;
	int board_cfg;
};

#endif
