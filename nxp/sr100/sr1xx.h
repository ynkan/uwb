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

#define NORMAL_MODE_HEADER_LEN 4
#define HBCI_MODE_HEADER_LEN 4
#define NORMAL_MODE_LEN_OFFSET 3
#define UCI_NORMAL_PKT_SIZE 0

#define EXTND_LEN_INDICATOR_OFFSET 1
#define EXTND_LEN_INDICATOR_OFFSET_MASK 0x80
#define EXTENDED_LENGTH_OFFSET 2

struct sr1xx_spi_platform_data {
	unsigned int irq_gpio;
	unsigned int ce_gpio;
	unsigned int spi_handshake_gpio;
	unsigned int rtc_sync_gpio;
	struct regulator *regulator_1v8_dig;
	struct regulator *regulator_1v8_rf;
	unsigned int vdd_1v8_gpio; /* to control VDD for super interposer board */
	unsigned int vdd_1v8_rf_gpio;
	unsigned int vbat_3v6_gpio;
};
enum {
	PWR_DISABLE = 0,
	PWR_ENABLE,
	ABORT_READ_PENDING
};
