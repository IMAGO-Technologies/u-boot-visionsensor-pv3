#
# Copyright (C) IMAGO Technologies GmbH - <https://www.imago-technologies.com>
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y += board.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-y += lpddr4_timing.o
obj-y += lpddr4_timing_2gb.o
else
obj-y += hardware.o
obj-y += i2c_core.o
obj-y += i2c_main.o
endif
