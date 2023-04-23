// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPI driver for UWB SR1xx
 * Copyright (C) 2018-2022 NXP.
 *
 * Author: Manjunatha Venkatesh <manjunatha.venkatesh@nxp.com>
 */
#define SR1XX_MAGIC 0xEA
#define SR1XX_SET_PWR _IOW(SR1XX_MAGIC, 0x01, uint32_t)
#define SR1XX_SET_FWD _IOW(SR1XX_MAGIC, 0x02, uint32_t)
#define SR1XX_ESE_RESET _IOW(SR1XX_MAGIC, 0x03, uint32_t)
#define SR1XX_GET_THROUGHPUT _IOW(SR1XX_MAGIC, 0x04, uint32_t)

enum {
	PWR_DISABLE = 0,
	PWR_ENABLE,
	ABORT_READ_PENDING
};
