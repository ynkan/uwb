// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPI driver for UWB SR1xx
 * Copyright (C) 2018-2022 NXP.
 *
 * Author: Manjunatha Venkatesh <manjunatha.venkatesh@nxp.com>
 */
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include "sr1xx.h"

#define UCI_HEADER_LEN 4
#define HBCI_HEADER_LEN 4
#define UCI_PAYLOAD_LEN_OFFSET 3

#define UCI_EXT_PAYLOAD_LEN_IND_OFFSET 1
#define UCI_EXT_PAYLOAD_LEN_IND_OFFSET_MASK 0x80
#define UCI_EXT_PAYLOAD_LEN_OFFSET 2
#define UCI_MT_MASK 0xE0

#define SR1XX_TXBUF_SIZE 4200
#define SR1XX_RXBUF_SIZE 4200
#define SR1XX_MAX_TX_BUF_SIZE 4200

#define MAX_RETRY_COUNT_FOR_IRQ_CHECK 100
#define MAX_RETRY_COUNT_FOR_HANDSHAKE 1000

/* Macro to define SPI clock frequency */
#define SR1XX_SPI_CLOCK 16000000L
#define WAKEUP_SRC_TIMEOUT (2000)

/* Maximum UCI packet size supported from the driver */
#define MAX_UCI_PKT_SIZE 4200

#define ESE_COLD_RESET 0

#if ESE_COLD_RESET
#include "../nfc/common_ese.h"
/*Invoke cold reset if no response from eSE*/
extern int perform_ese_cold_reset(unsigned long source);
#endif

struct sr1xx_spi_platform_data {
	unsigned int irq_gpio;
	unsigned int ce_gpio;
	unsigned int spi_handshake_gpio;

	struct regulator *regulator_1v8_dig;
	struct regulator *regulator_1v8_rf;
};

#define USB_HSPHY_1P8_VOL_MIN			1704000 /* uV */
#define USB_HSPHY_1P8_VOL_MAX			1800000 /* uV */

/* TODO: remove dynamic load changes, this was only for Eos development */
#define USB_HSPHY_1P8_HPM_LOAD			19000	/* uA */
#define USB_HSPHY_1P8_LOWPOWER_LOAD		1000	/* uA */

/* Device specific macro and structure */
struct sr1xx_dev {
	wait_queue_head_t read_wq;          /* wait queue for read interrupt */
	struct spi_device *spi;             /* spi device structure */
	struct miscdevice sr1xx_device;     /* char device as misc driver */
	unsigned int ce_gpio;               /* SW reset gpio */
	unsigned int irq_gpio;              /* SR1XX will interrupt host for any ntf */
	unsigned int spi_handshake_gpio;    /* host ready to read data */

	unsigned char *tx_buffer;           /* transmit buffer */
	unsigned char *rx_buffer;           /* receive buffer */
	unsigned int write_count;           /* holds numbers of byte written */
	unsigned int read_count;            /* hold numbers of byte read */

	struct mutex sr1xx_access_lock;     /* lock used to synchronize read and write */
	size_t total_bytes_to_read;         /* total bytes read from the device */

	bool is_extended_len_bit_set;       /* variable to check ext payload Len */
	bool is_fw_dwnld_enabled;           /* used to indicate fw download mode */
	int mode;                           /* indicate write or read mode */
	long timeout_in_ms;                 /* wait event interrupt timeout in ms */

	atomic_t read_abort_requested;      /* used to indicate read abort request */
	atomic_t irq_received;              /* flag to indicate that irq is received */
	bool pwr_enabled;                   /* flag to indicate pwr */

	struct regulator	*regulator_1v8_dig;
	struct regulator	*regulator_1v8_rf;
};

enum spi_status_codes {
	TRANSCEIVE_SUCCESS,
	TRANSCEIVE_FAIL,
	IRQ_WAIT_REQUEST,
	IRQ_WAIT_TIMEOUT
};

/* Spi write/read operation mode */
enum spi_operation_modes { SR1XX_WRITE_MODE, SR1XX_READ_MODE };

static int sr1xx_dev_open(struct inode *inode, struct file *filp)
{
	struct sr1xx_dev *sr1xx_dev = container_of(filp->private_data, struct sr1xx_dev, sr1xx_device);

	filp->private_data = sr1xx_dev;
	return 0;
}

static irqreturn_t sr1xx_dev_irq_handler(int irq, void *dev_id)
{
	struct sr1xx_dev *sr1xx_dev = dev_id;

	atomic_set(&sr1xx_dev->irq_received, 1);

	/* Wake up waiting readers */
	wake_up(&sr1xx_dev->read_wq);

	return IRQ_HANDLED;
}

static int sr1xx_power_ctl(struct sr1xx_dev *sr1xx_dev, bool on)
{
	int ret = 0;

	dev_info(&sr1xx_dev->spi->dev, "requested chip %s.\n", on ? "enabled" : "disabled");

	if (on == sr1xx_dev->pwr_enabled) {
		dev_warn(&sr1xx_dev->spi->dev, "chip already %s.\n", on ? "enabled" : "disabled");
		return 0;
	}

	/* TODO: remove dynamic load changes, this was only for Eos development */
	if (on) {
		ret = regulator_set_load(sr1xx_dev->regulator_1v8_dig, USB_HSPHY_1P8_HPM_LOAD);
		if (ret) {
			dev_err(&sr1xx_dev->spi->dev, "failed to set dig regulator load\n");
			return ret;
		}
		ret = regulator_set_load(sr1xx_dev->regulator_1v8_rf, USB_HSPHY_1P8_HPM_LOAD);
		if (ret) {
			dev_err(&sr1xx_dev->spi->dev, "failed to set rf regulator load\n");
			return ret;
		}
		usleep_range(50, 100);

		sr1xx_dev->pwr_enabled = true;

		/* ignore the irq asserted from the last power cycle */
		atomic_set(&sr1xx_dev->irq_received, 0);

		gpio_set_value(sr1xx_dev->ce_gpio, 1);
		msleep(10);
	} else if (!on) {
		gpio_set_value(sr1xx_dev->ce_gpio, 0);
		msleep(10);

		ret = regulator_set_load(sr1xx_dev->regulator_1v8_dig, USB_HSPHY_1P8_LOWPOWER_LOAD);
		if (ret) {
			dev_err(&sr1xx_dev->spi->dev, "failed to set dig regulator load\n");
			return ret;
		}
		ret = regulator_set_load(sr1xx_dev->regulator_1v8_rf, USB_HSPHY_1P8_LOWPOWER_LOAD);
		if (ret) {
			dev_err(&sr1xx_dev->spi->dev, "failed to set rf regulator load\n");
			return ret;
		}

		sr1xx_dev->pwr_enabled = false;
	}

	return ret;
}

static long sr1xx_dev_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	int ret = 0;
	struct sr1xx_dev *sr1xx_dev = NULL;

	sr1xx_dev = filp->private_data;
	if (sr1xx_dev == NULL) {
		ret = -EINVAL;
		pr_err("%s sr1xx_dev is NULL\n", __func__);
		return ret;
	}
	switch (cmd) {
	case SR1XX_SET_PWR:
		if (arg == PWR_ENABLE) {
			ret = sr1xx_power_ctl(sr1xx_dev, true);
		} else if (arg == PWR_DISABLE) {
			ret = sr1xx_power_ctl(sr1xx_dev, false);
		} else if (arg == ABORT_READ_PENDING) {
			atomic_set(&sr1xx_dev->read_abort_requested, 1);
			/* Wake up waiting readers */
			wake_up(&sr1xx_dev->read_wq);
		}
		break;
	case SR1XX_SET_FWD:
		if (arg == 1) {
			sr1xx_dev->is_fw_dwnld_enabled = true;
			atomic_set(&sr1xx_dev->read_abort_requested, 0);
		}
		else if(arg == 0) {
			sr1xx_dev->is_fw_dwnld_enabled = false;
		}
		break;

	case SR1XX_GET_THROUGHPUT:
		dev_warn(&sr1xx_dev->spi->dev, "SR1XX_GET_THROUGHPUT not supported!\n");
		break;
#if ESE_COLD_RESET
	case SR1XX_ESE_RESET:
		dev_info(&sr1xx_dev->spi->dev, "%s SR1XX_ESE_RESET_ Enter\n", __func__);
		ret = perform_ese_cold_reset(ESE_CLD_RST_OTHER);
		break;
#endif
	default:
		dev_err(&sr1xx_dev->spi->dev, " Error case");
		ret = -EINVAL;
	}
	return ret;
}

#if defined(CONFIG_COMPAT)
static long sr1xx_dev_ioctl_compat(struct file* filp, unsigned int cmd, unsigned long arg)
{
	return sr1xx_dev_ioctl(filp, cmd, arg);
}
#else
#define sr1xx_dev_ioctl_compat NULL
#endif

static int sr1xx_wait_irq(struct sr1xx_dev* sr1xx_dev, long timeout)
{
	int ret;

	if (timeout) {
		ret = wait_event_interruptible_timeout(sr1xx_dev->read_wq,
				atomic_read(&sr1xx_dev->irq_received), timeout);
		if (ret > 0)
			ret = 0;
		else if (ret == 0)
			ret = -ETIMEDOUT;
	} else {
		ret = wait_event_interruptible(sr1xx_dev->read_wq, atomic_read(&sr1xx_dev->irq_received));
	}

	if (!ret)
		atomic_set(&sr1xx_dev->irq_received, 0);

	return ret;
}

/**
 * sr1xx_wait_for_irq_gpio_low
 *
 * Function to wait till irq gpio goes low state
 *
 */
static void sr1xx_wait_for_irq_gpio_low(struct sr1xx_dev *sr1xx_dev)
{
	unsigned long timeout;
	timeout = jiffies + msecs_to_jiffies(10);
	do {
		usleep_range(10, 15);
		if (time_after(jiffies, timeout)) {
			dev_info(&sr1xx_dev->spi->dev,
				 "UWBS not released the IRQ even after 10ms");
			break;
		}
	} while (gpio_get_value(sr1xx_dev->irq_gpio));
}

/**
 * sr1xx_dev_transceive
 * @op_mode indicates write/read operation
 *
 * Write and Read logic implemented under same api with
 * mutex lock protection so write and read synchronized
 *
 * During Uwb ranging sequence(read) need to block write sequence
 * in order to avoid some race condition scenarios.
 *
 * Returns     : Number of bytes write/read if read is success else
 *               indicate each error code
 */
static int sr1xx_dev_transceive(struct sr1xx_dev *sr1xx_dev, int op_mode,
				int count)
{
	int ret, retry_count;
	mutex_lock(&sr1xx_dev->sr1xx_access_lock);
	sr1xx_dev->mode = op_mode;
	sr1xx_dev->total_bytes_to_read = 0;
	sr1xx_dev->is_extended_len_bit_set = 0;
	ret = -EIO;
	retry_count = 0;

	switch (sr1xx_dev->mode) {
	case SR1XX_WRITE_MODE:
		sr1xx_dev->write_count = 0;
		/* UCI Header write */
		ret = spi_write(sr1xx_dev->spi, sr1xx_dev->tx_buffer,
				UCI_HEADER_LEN);
		if (ret < 0) {
			ret = -EIO;
			dev_err(&sr1xx_dev->spi->dev,
				"spi_write header : Failed.\n");
			goto transceive_end;
		} else {
			count -= UCI_HEADER_LEN;
		}
		if (count > 0) {
			/* In between header write and payload write UWBS needs some time */
			usleep_range(30, 50);
			/* UCI Payload write */
			ret = spi_write(sr1xx_dev->spi,
					sr1xx_dev->tx_buffer +
					UCI_HEADER_LEN, count);
			if (ret < 0) {
				ret = -EIO;
				dev_err(&sr1xx_dev->spi->dev,
					"spi_write payload : Failed.\n");
				goto transceive_end;
			}
		}
		sr1xx_dev->write_count = count + UCI_HEADER_LEN;
		ret = TRANSCEIVE_SUCCESS;
		break;
	case SR1XX_READ_MODE:
		if (!sr1xx_dev->is_fw_dwnld_enabled && !gpio_get_value(sr1xx_dev->irq_gpio)) {
			dev_err(&sr1xx_dev->spi->dev,
				"IRQ might have gone low due to write ");
			ret = IRQ_WAIT_REQUEST;
			goto transceive_end;
		}
		gpio_set_value(sr1xx_dev->spi_handshake_gpio, 1);
		/*
		 * Goog: skip gpio polling on irq line
		 * waiting for the 2nd irq assertion after RX_SYNC
		 */
		ret = sr1xx_wait_irq(sr1xx_dev, sr1xx_dev->timeout_in_ms);
		if (ret < 0) {
			ret = IRQ_WAIT_TIMEOUT;
			goto transceive_end;
		}
		sr1xx_dev->read_count = 0;
		ret = spi_read(sr1xx_dev->spi, (void *)sr1xx_dev->rx_buffer, UCI_HEADER_LEN);
		if (ret < 0) {
			dev_err(&sr1xx_dev->spi->dev, "sr1xx_dev_read: spi read error %d\n ", ret);
			goto transceive_end;
		}
		if ((sr1xx_dev->rx_buffer[0] & UCI_MT_MASK) == 0) {
			sr1xx_dev->total_bytes_to_read = sr1xx_dev->rx_buffer[UCI_PAYLOAD_LEN_OFFSET];
			sr1xx_dev->total_bytes_to_read =
				((sr1xx_dev->total_bytes_to_read << 8) |
				sr1xx_dev->rx_buffer[UCI_EXT_PAYLOAD_LEN_OFFSET]);
		} else {
			sr1xx_dev->is_extended_len_bit_set =
			    (sr1xx_dev->rx_buffer[UCI_EXT_PAYLOAD_LEN_IND_OFFSET] & UCI_EXT_PAYLOAD_LEN_IND_OFFSET_MASK);
			sr1xx_dev->total_bytes_to_read = sr1xx_dev->rx_buffer[UCI_PAYLOAD_LEN_OFFSET];
			if (sr1xx_dev->is_extended_len_bit_set) {
				sr1xx_dev->total_bytes_to_read =
				    ((sr1xx_dev->total_bytes_to_read << 8) |
				     sr1xx_dev->rx_buffer[UCI_EXT_PAYLOAD_LEN_OFFSET]);
			}
		}
		if (sr1xx_dev->total_bytes_to_read > (MAX_UCI_PKT_SIZE - UCI_HEADER_LEN)) {
			dev_err(&sr1xx_dev->spi->dev, "Length %d  exceeds the max limit %d....",
				(int)sr1xx_dev->total_bytes_to_read,
				(int)MAX_UCI_PKT_SIZE);
			ret = -ENOBUFS;
			goto transceive_end;
		}
		if (sr1xx_dev->total_bytes_to_read > 0) {
			ret = spi_read(sr1xx_dev->spi,
				      (void *)(sr1xx_dev->rx_buffer + UCI_HEADER_LEN),
				      sr1xx_dev->total_bytes_to_read);
			if (ret < 0) {
				dev_err(&sr1xx_dev->spi->dev,
					"sr1xx_dev_read: spi read error.. %d\n ",
					ret);
				goto transceive_end;
			}
		}
		sr1xx_dev->read_count = (unsigned int)(sr1xx_dev->total_bytes_to_read + UCI_HEADER_LEN);
		sr1xx_wait_for_irq_gpio_low(sr1xx_dev);
		ret = TRANSCEIVE_SUCCESS;
		gpio_set_value(sr1xx_dev->spi_handshake_gpio, 0);
		break;
	default:
		dev_err(&sr1xx_dev->spi->dev, "invalid operation .....");
		break;
	}
transceive_end:
	if (sr1xx_dev->mode == SR1XX_READ_MODE)
		gpio_set_value(sr1xx_dev->spi_handshake_gpio, 0);

	mutex_unlock(&sr1xx_dev->sr1xx_access_lock);
	return ret;
}

/**
 * sr1xx_hbci_write
 *
 * Used to write hbci(SR1xx BootROM Command Interface) packets
 * during firmware download sequence.
 *
 * Returns: TRANSCEIVE_SUCCESS on success or error code on fail
 */
static int sr1xx_hbci_write(struct sr1xx_dev *sr1xx_dev, int count)
{
	int ret;

	sr1xx_dev->write_count = 0;
	/* HBCI write */
	ret = spi_write(sr1xx_dev->spi, sr1xx_dev->tx_buffer, count);
	if (ret < 0) {
		ret = -EIO;
		dev_err(&sr1xx_dev->spi->dev, "spi_write fw download : Failed.\n");
		goto hbci_write_fail;
	}
	sr1xx_dev->write_count = count;
	ret = TRANSCEIVE_SUCCESS;
	return ret;
hbci_write_fail:
	dev_err(&sr1xx_dev->spi->dev, "%s failed...%d", __func__, ret);
	return ret;
}

static ssize_t sr1xx_dev_write(struct file *filp, const char *buf, size_t count, loff_t * offset)
{
	int ret;
	struct sr1xx_dev *sr1xx_dev;

	sr1xx_dev = filp->private_data;
	if (count > SR1XX_MAX_TX_BUF_SIZE || count > SR1XX_TXBUF_SIZE) {
		dev_err(&sr1xx_dev->spi->dev, "%s : Write Size Exceeds\n", __func__);
		ret = -ENOBUFS;
		goto write_end;
	}
	if (copy_from_user(sr1xx_dev->tx_buffer, buf, count)) {
		dev_err(&sr1xx_dev->spi->dev, "%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}
	if (sr1xx_dev->is_fw_dwnld_enabled)
		ret = sr1xx_hbci_write(sr1xx_dev, count);
	else
		ret = sr1xx_dev_transceive(sr1xx_dev, SR1XX_WRITE_MODE, count);
	if (ret == TRANSCEIVE_SUCCESS)
		ret = sr1xx_dev->write_count;
	else
		dev_err(&sr1xx_dev->spi->dev, "write failed......");
write_end:
	return ret;
}

/**
 * sr1xx_hbci_read
 *
 * Function used to read data from sr1xx on SPI line
 * as part of firmware download sequence.
 *
 * Returns: Number of bytes read if read is success else (-EIO
 *               otherwise indicate each error code
 */
static ssize_t sr1xx_hbci_read(struct sr1xx_dev *sr1xx_dev, char *buf, size_t count)
{
	int ret = -EIO;

	if (count > SR1XX_RXBUF_SIZE) {
		dev_err(&sr1xx_dev->spi->dev, "count(%zu) out of range(0-%d)\n",
			count, SR1XX_RXBUF_SIZE);
		ret = -EINVAL;
		goto hbci_fail;
	}
	/* Wait for inetrrupt up to 500ms */
	ret = sr1xx_wait_irq(sr1xx_dev, sr1xx_dev->timeout_in_ms);
	if (ret < 0) {
		dev_err(&sr1xx_dev->spi->dev,
			"hbci wait_event_interruptible timeout() : Failed.\n");
		ret = -EIO;
		goto hbci_fail;
	}
	if (atomic_cmpxchg(&sr1xx_dev->read_abort_requested, 1, 0)) {
		dev_err(&sr1xx_dev->spi->dev, "HBCI Abort Read pending......\n");
		return ret;
	}
	ret = spi_read(sr1xx_dev->spi, (void *)sr1xx_dev->rx_buffer, count);
	if (ret < 0) {
		dev_err(&sr1xx_dev->spi->dev,
			"sr1xx_dev_read: spi read error %d\n ", ret);
		goto hbci_fail;
	}
	ret = count;
	if (copy_to_user(buf, sr1xx_dev->rx_buffer, count)) {
		dev_err(&sr1xx_dev->spi->dev, "sr1xx_dev_read: copy to user failed\n");
		ret = -EFAULT;
	}
	return ret;
hbci_fail:
	dev_err(&sr1xx_dev->spi->dev, "Error sr1xx_fw_download ret %d Exit\n", ret);
	return ret;
}

static ssize_t sr1xx_dev_read(struct file *filp, char *buf, size_t count, loff_t * offset)
{
	struct sr1xx_dev *sr1xx_dev = filp->private_data;
	int ret = -EIO;

	if (!sr1xx_dev->pwr_enabled)
		return -EIO;

	memset(sr1xx_dev->rx_buffer, 0x00, SR1XX_RXBUF_SIZE);

	/* HBCI packet read */
	if (sr1xx_dev->is_fw_dwnld_enabled) {
		ret = sr1xx_hbci_read(sr1xx_dev, buf, count);
		goto read_end;
	}
	/* UCI packet read */
	if (!atomic_read(&sr1xx_dev->irq_received) && (filp->f_flags & O_NONBLOCK)) {
		ret = -EAGAIN;
		goto read_end;
	}

	do {
		if (!atomic_read(&sr1xx_dev->read_abort_requested)) {
			ret = sr1xx_wait_irq(sr1xx_dev, 0);
			if (ret && !atomic_read(&sr1xx_dev->read_abort_requested)) {
				dev_err(&sr1xx_dev->spi->dev, "wait_event_interruptible() : Failed.\n");
				goto read_end;
			}
		}
		if (atomic_cmpxchg(&sr1xx_dev->read_abort_requested, 1, 0)) {
			dev_err(&sr1xx_dev->spi->dev, "Abort Read pending......\n");
			return -EAGAIN;
		}
		ret = sr1xx_dev_transceive(sr1xx_dev, SR1XX_READ_MODE, count);
		if (ret == IRQ_WAIT_REQUEST) {
			dev_err(&sr1xx_dev->spi->dev,
				"Irq is low due to write hence irq is requested again...\n");
		}
	} while (ret == IRQ_WAIT_REQUEST);
	if (ret == TRANSCEIVE_SUCCESS) {
		if (copy_to_user(buf, sr1xx_dev->rx_buffer, sr1xx_dev->read_count)) {
			dev_err(&sr1xx_dev->spi->dev, "%s: copy to user failed\n", __func__);
			ret = -EFAULT;
			goto read_end;
		}
		ret = sr1xx_dev->read_count;
	} else if (ret == IRQ_WAIT_TIMEOUT) {
		dev_err(&sr1xx_dev->spi->dev, "Second irq is not received..Time out...");
		ret = -ETIME;
	} else {
		dev_err(&sr1xx_dev->spi->dev, "spi read failed...%d", ret);
		ret = -EIO;
	}
read_end:
	return ret;
}

static int sr1xx_hw_setup(struct device *dev,
			  struct sr1xx_spi_platform_data *platform_data)
{
	int ret;

	ret = devm_gpio_request(dev, platform_data->irq_gpio, "sr1xx irq");
	if (ret < 0) {
		dev_err(dev, "gpio request failed gpio = 0x%x\n", platform_data->irq_gpio);
		goto fail_gpio;
	}

	ret = gpio_direction_input(platform_data->irq_gpio);
	if (ret < 0) {
		dev_err(dev, "gpio request failed gpio = 0x%x\n", platform_data->irq_gpio);
		goto fail_gpio;
	}

	ret = devm_gpio_request(dev, platform_data->ce_gpio, "sr1xx ce");
	if (ret < 0) {
		dev_err(dev, "sr1xx - Failed setting ce gpio - %d\n", platform_data->ce_gpio);
		goto fail_gpio;
	}

	ret = gpio_direction_output(platform_data->ce_gpio, 1);
	if (ret < 0) {
		dev_err(dev, "sr1xx - Failed requesting ri gpio - %d\n", platform_data->spi_handshake_gpio);
		goto fail_gpio;
	}

	ret = devm_gpio_request(dev, platform_data->spi_handshake_gpio, "sr1xx ri");
	if (ret < 0) {
		dev_err(dev, "sr1xx - Failed requesting ri gpio - %d\n", platform_data->spi_handshake_gpio);
		goto fail_gpio;
	}

	ret = gpio_direction_output(platform_data->spi_handshake_gpio, 0);
	if (ret < 0) {
		dev_err(dev, "sr1xx - Failed setting spi handeshake gpio - %d\n", platform_data->spi_handshake_gpio);
		goto fail_gpio;
	}
	/* TODO: start with low power mode only for Eos */
	ret = regulator_set_load(platform_data->regulator_1v8_dig, USB_HSPHY_1P8_LOWPOWER_LOAD);
	if (ret < 0) {
		dev_err(dev, "Unable to set HPM of regulator_1v8_dig:%d\n", ret);
		goto fail_gpio;
	}
	ret = regulator_set_voltage(platform_data->regulator_1v8_dig, USB_HSPHY_1P8_VOL_MIN, USB_HSPHY_1P8_VOL_MAX);
	if (ret) {
		dev_err(dev, "Unable to set voltage for regulator_1v8_dig:%d\n", ret);
		goto fail_gpio;
	}

	ret = regulator_set_load(platform_data->regulator_1v8_rf, USB_HSPHY_1P8_LOWPOWER_LOAD);
	if (ret < 0) {
		dev_err(dev, "Unable to set HPM of regulator_1v8_rf:%d\n", ret);
		goto fail_gpio;
	}
	ret = regulator_set_voltage(platform_data->regulator_1v8_rf, USB_HSPHY_1P8_VOL_MIN, USB_HSPHY_1P8_VOL_MAX);
	if (ret) {
		dev_err(dev, "Unable to set voltage for regulator_1v8_rf:%d\n", ret);
		goto fail_gpio;
	}
	dev_info(dev," PMIC Power configure : %s \n", __func__);
	ret = 0;
	return ret;

fail_gpio:
	dev_err(dev, "sr1xx_hw_setup failed\n");
	return ret;
}

static inline void sr1xx_set_data(struct spi_device *spi, void *data)
{
	dev_set_drvdata(&spi->dev, data);
}

static inline void *sr1xx_get_data(const struct spi_device *spi)
{
	return dev_get_drvdata(&spi->dev);
}

/* Possible fops on the sr1xx device */
static const struct file_operations sr1xx_dev_fops = {
	.owner = THIS_MODULE,
	.read = sr1xx_dev_read,
	.write = sr1xx_dev_write,
	.open = sr1xx_dev_open,
	.unlocked_ioctl = sr1xx_dev_ioctl,
	.compat_ioctl = sr1xx_dev_ioctl_compat,
};

static int sr1xx_parse_dt(struct device *dev, struct sr1xx_spi_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->irq_gpio = of_get_named_gpio(np, "nxp,sr1xx-irq", 0);
	if (!gpio_is_valid(pdata->irq_gpio)) {
		return -EINVAL;
	}
	pdata->ce_gpio = of_get_named_gpio(np, "nxp,sr1xx-ce", 0);
	if (!gpio_is_valid(pdata->ce_gpio)) {
		return -EINVAL;
	}
	pdata->spi_handshake_gpio = of_get_named_gpio(np, "nxp,sr1xx-ri", 0);
	if (!gpio_is_valid(pdata->spi_handshake_gpio)) {
		return -EINVAL;
	}
	pdata->regulator_1v8_dig = devm_regulator_get(dev, "nxp,sr1xx-dig");
	if (IS_ERR(pdata->regulator_1v8_dig)) {
		dev_err(dev, "unable to get sr1xx-dig supply\n");
		return -EINVAL;
	}

	pdata->regulator_1v8_rf = devm_regulator_get(dev, "nxp,sr1xx-rf");
	if (IS_ERR(pdata->regulator_1v8_rf)) {
		pdata->regulator_1v8_rf = NULL;
		dev_err(dev, "unable to get sr1xx-rf supply\n");
	}

	return 0;
}

static int sr1xx_probe(struct spi_device *spi)
{
	int ret;
	struct sr1xx_spi_platform_data platform_data;
	struct sr1xx_dev *sr1xx_dev = NULL;
	unsigned int irq_flags;

	dev_info(&spi->dev, "%s chip select : %d , bus number = %d\n", __func__,
		 spi->chip_select, spi->master->bus_num);

	ret = sr1xx_parse_dt(&spi->dev, &platform_data);
	if (ret) {
		dev_err(&spi->dev, "%s - Failed to parse DT\n", __func__);
		return ret;
	}

	sr1xx_dev = kzalloc(sizeof(*sr1xx_dev), GFP_KERNEL);
	if (!sr1xx_dev) {
		return -ENOMEM;
	}
	ret = sr1xx_hw_setup(&spi->dev, &platform_data);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed hw_setup\n");
		goto err_alloc;
	}

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	spi->max_speed_hz = SR1XX_SPI_CLOCK;
	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "failed to do spi_setup()\n");
		goto err_alloc;
	}

	sr1xx_dev->spi = spi;
	sr1xx_dev->sr1xx_device.minor = MISC_DYNAMIC_MINOR;
	sr1xx_dev->sr1xx_device.name = "srxxx";
	sr1xx_dev->sr1xx_device.fops = &sr1xx_dev_fops;
	sr1xx_dev->sr1xx_device.parent = &spi->dev;

	sr1xx_dev->irq_gpio = platform_data.irq_gpio;
	sr1xx_dev->ce_gpio = platform_data.ce_gpio;
	sr1xx_dev->spi_handshake_gpio = platform_data.spi_handshake_gpio;

	sr1xx_dev->regulator_1v8_dig = platform_data.regulator_1v8_dig;
	sr1xx_dev->regulator_1v8_rf = platform_data.regulator_1v8_rf;

	dev_set_drvdata(&spi->dev, sr1xx_dev);

	/* Goog: start with hard power down mode */
	gpio_set_value(sr1xx_dev->ce_gpio, 0);

	/* init mutex and queues */
	init_waitqueue_head(&sr1xx_dev->read_wq);
	mutex_init(&sr1xx_dev->sr1xx_access_lock);

	ret = misc_register(&sr1xx_dev->sr1xx_device);
	if (ret < 0) {
		dev_err(&spi->dev, "misc_register failed! %d\n", ret);
		goto err_mutex;
	}

	sr1xx_dev->tx_buffer = kzalloc(SR1XX_TXBUF_SIZE, GFP_KERNEL);
	sr1xx_dev->rx_buffer = kzalloc(SR1XX_RXBUF_SIZE, GFP_KERNEL);
	if (!sr1xx_dev->tx_buffer || !sr1xx_dev->rx_buffer) {
		ret = -ENOMEM;
		goto err_misc;
	}

	sr1xx_dev->spi->irq = gpio_to_irq(sr1xx_dev->irq_gpio);
	if (sr1xx_dev->spi->irq < 0) {
		dev_err(&spi->dev, "gpio_to_irq request failed gpio = 0x%x\n", sr1xx_dev->irq_gpio);
		goto err_misc;
	}
	/* request irq. The irq is set whenever the chip has data available
	 * for reading. It is cleared when all data has been read.
	 */
	irq_flags = IRQF_TRIGGER_RISING;
	atomic_set(&sr1xx_dev->irq_received, 0);
	atomic_set(&sr1xx_dev->read_abort_requested, 0);
	sr1xx_dev->timeout_in_ms = 500;

	ret = request_irq(sr1xx_dev->spi->irq, sr1xx_dev_irq_handler, irq_flags,
			  sr1xx_dev->sr1xx_device.name, sr1xx_dev);
	if (ret) {
		dev_err(&spi->dev, "request_irq failed\n");
		goto err_misc;
	}

	ret = regulator_enable(sr1xx_dev->regulator_1v8_dig);
	if (ret) {
		dev_err(&spi->dev, "Unable to enable dig regulator: %d\n", ret);
		goto err_misc;
	}
	ret = regulator_enable(sr1xx_dev->regulator_1v8_rf);
	if (ret) {
		dev_err(&spi->dev, "Unable to enable rf regulator: %d\n", ret);
		goto err_regulator;
	}

	return 0;
err_regulator:
	regulator_disable(sr1xx_dev->regulator_1v8_dig);
err_misc:
	if (sr1xx_dev->tx_buffer)
		kfree(sr1xx_dev->tx_buffer);
	if (sr1xx_dev->rx_buffer)
		kfree(sr1xx_dev->rx_buffer);
	misc_deregister(&sr1xx_dev->sr1xx_device);
err_mutex:
	mutex_destroy(&sr1xx_dev->sr1xx_access_lock);
err_alloc:
	kfree(sr1xx_dev);
	dev_err(&spi->dev, "ERROR: Exit : %s ret %d\n", __func__, ret);
	return ret;
}

static int sr1xx_remove(struct spi_device *spi)
{
	struct sr1xx_dev *sr1xx_dev = sr1xx_get_data(spi);

	if (!sr1xx_dev) {
		dev_err(&spi->dev, "sr1xx_dev is NULL \n");
		return -EINVAL;
	}
	mutex_destroy(&sr1xx_dev->sr1xx_access_lock);
	free_irq(sr1xx_dev->spi->irq, sr1xx_dev);

	regulator_disable(sr1xx_dev->regulator_1v8_rf);
	regulator_disable(sr1xx_dev->regulator_1v8_dig);

	misc_deregister(&sr1xx_dev->sr1xx_device);
	if (sr1xx_dev) {
		kfree(sr1xx_dev->tx_buffer);
		kfree(sr1xx_dev->rx_buffer);
		kfree(sr1xx_dev);
	}
	return 0;
}

/**
 * sr1xx_dev_suspend
 *
 * Executed before putting the system into a sleep state
 *
 */
int sr1xx_dev_suspend(struct device *dev)
{
	struct sr1xx_dev *sr1xx_dev = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(sr1xx_dev->spi->irq);
	return 0;
}

/**
 * sr1xx_dev_resume
 *
 * Executed after waking the system up from a sleep state
 *
 */
int sr1xx_dev_resume(struct device *dev)
{
	struct sr1xx_dev *sr1xx_dev = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(sr1xx_dev->spi->irq);

	return 0;
}

static const struct of_device_id sr1xx_dt_match[] = {
	{
		.compatible = "nxp,srxxx",
	},
	{ }
};

static const struct dev_pm_ops sr1xx_dev_pm_ops =
    { SET_SYSTEM_SLEEP_PM_OPS(sr1xx_dev_suspend, sr1xx_dev_resume)
};

static struct spi_driver sr1xx_driver = {
	.driver = {
		.name = "srxxx",
		.pm = &sr1xx_dev_pm_ops,
		.of_match_table = sr1xx_dt_match,
	},
	.probe = sr1xx_probe,
	.remove = (sr1xx_remove),
};

module_spi_driver(sr1xx_driver);

MODULE_AUTHOR("Manjunatha Venkatesh <manjunatha.venkatesh@nxp.com>");
MODULE_DESCRIPTION("NXP SR1XX SPI driver");
MODULE_LICENSE("GPL");
