// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPI driver for UWB SR1xx
 * Copyright (C) 2018-2022 NXP.
 * Copyright (C) 2024 Google, Inc.
 *
 * Author: Manjunatha Venkatesh <manjunatha.venkatesh@nxp.com>
 * Author: Ikjoon Jang <ikjn@google.com>
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pm_qos.h>
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

/* Macro to define SPI clock frequency */
#define SR1XX_SPI_CLOCK 16000000L

/* Maximum UCI packet size supported from the driver */
#define MAX_UCI_PKT_SIZE 4200

/*
 * TX delay between header and payload
 * XXX: Do we really need this?
 */
#define WRITE_DELAY 		100
#define WRITE_DELAY_RANGE_DIFF	50

/* TX fallback when RX is pending */
#define WRITE_FALLBACK_DELAY_MS		5
#define WRITE_FALLBACK_MAX_RETRIES	10

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

enum sr1xx_suspend_mode {
	SR1XX_SUSPEND_NONE = 0, /* Do nothing on suspend */
	SR1XX_SUSPEND_RF_LPM,   /* Low power mode for RF regulator */
	SR1XX_SUSPEND_ALL_LPM,  /* Low power mode for RF & DIG regulator */
	SR1XX_SUSPEND_OFF,      /* Power down */
};

#define NR_RX_BUFFERS		64
#define NR_DROPS_ON_FULL	4	/* drop oldest packets when rxq is full */
#define FALLBACK_ON_FULL_MS	100	/* wait before dropping packets */
#define MAX_RETRIES_ON_FULL	3

struct rx_buffer {
	void	*buff;
	size_t	len;
};

struct rx_queue {
	int head;
	int tail;
	spinlock_t lock;
	struct rx_buffer buffers[NR_RX_BUFFERS];
	wait_queue_head_t wq;
};

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

	struct mutex sr1xx_access_lock;     /* lock used to synchronize read and write */

	struct task_struct *rx_thread;
	struct rx_queue rx_queue;

	long timeout_in_ms;                 /* wait event interrupt timeout in ms */

#define FLAGS_FW_DOWNLOAD	0
#define FLAGS_READ_ABORTED	1
#define FLAGS_IRQ_RECEIVED	2
#define FLAGS_PWR_ENABLED	3
#define FLAGS_SUSPENDED		4
#define FLAGS_RX_ONGOING	5
#define FLAGS_QOS_APPLIED	6
	volatile unsigned long flags;

	enum sr1xx_suspend_mode suspend_mode;

	struct pm_qos_request qos_latency;

	struct regulator	*regulator_1v8_dig;
	struct regulator	*regulator_1v8_rf;
};

enum spi_status_codes {
	TRANSCEIVE_SUCCESS,
	TRANSCEIVE_FAIL,
	IRQ_WAIT_REQUEST,
	IRQ_WAIT_TIMEOUT
};

static int start_rx_thread(struct sr1xx_dev *sr1xx_dev);
static int stop_rx_thread(struct sr1xx_dev *sr1xx_dev);

static bool srflags_test(struct sr1xx_dev *sr1xx_dev, unsigned long flags)
{
	return test_bit(flags, &sr1xx_dev->flags);
}

static void srflags_set(struct sr1xx_dev *sr1xx_dev, unsigned long flags)
{
	smp_mb__before_atomic();
	set_bit(flags, &sr1xx_dev->flags);
	smp_mb__after_atomic();
}

static void srflags_clear(struct sr1xx_dev *sr1xx_dev, unsigned long flags)
{
	smp_mb__before_atomic();
	clear_bit(flags, &sr1xx_dev->flags);
	smp_mb__after_atomic();
}

static bool srflags_test_and_set(struct sr1xx_dev *sr1xx_dev, unsigned long flags)
{
	bool ret;

	smp_mb__before_atomic();
	ret = test_and_set_bit(flags, &sr1xx_dev->flags);
	smp_mb__after_atomic();
	return ret;
}

static bool srflags_test_and_clear(struct sr1xx_dev *sr1xx_dev, unsigned long flags)
{
	bool ret;

	smp_mb__before_atomic();
	ret = test_and_clear_bit(flags, &sr1xx_dev->flags);
	smp_mb__after_atomic();
	return ret;
}

static inline struct sr1xx_dev *get_dev_from_rxq(struct rx_queue *rxq)
{
	return container_of(rxq, struct sr1xx_dev, rx_queue);
}

static void rxq_init(struct rx_queue *rxq)
{
	memset(rxq, 0, sizeof(*rxq));
	spin_lock_init(&rxq->lock);
	init_waitqueue_head(&rxq->wq);
}

static void rxq_clear(struct rx_queue *rxq)
{
	int i;

	spin_lock(&rxq->lock);
	for (i = 0; i < NR_RX_BUFFERS; i++) {
		struct rx_buffer *rxbuf = &rxq->buffers[i];
		if (rxbuf->buff) {
			kfree(rxbuf->buff);
			rxbuf->buff = NULL;
			rxbuf->len = 0;
		}
	}
	rxq->head = 0;
	rxq->tail = 0;
	spin_unlock(&rxq->lock);
}

static void free_one_buffer(struct rx_buffer *rx_buff)
{
	if (rx_buff->buff) {
		kfree(rx_buff->buff);
		rx_buff->buff = NULL;
	}
	rx_buff->len = 0;
}

static size_t rxq_len(struct rx_queue *rxq)
{
	if (rxq->head >= rxq->tail)
		return rxq->head - rxq->tail;
	else
		return NR_RX_BUFFERS - rxq->tail + rxq->head;
}

static size_t rxq_is_empty(struct rx_queue *rxq)
{
	return rxq_len(rxq) == 0;
}

static int rxq_pop(struct rx_queue *rxq, struct rx_buffer *rx_buff)
{
	int ret;
	struct sr1xx_dev *sr1xx_dev = get_dev_from_rxq(rxq);

	for (;;) {
		spin_lock(&rxq->lock);
		if (rxq->head == rxq->tail) {
			/* Empty */
			spin_unlock(&rxq->lock);
			dev_dbg(&sr1xx_dev->spi->dev,
				"rx ring empty, wait for rx event.\n");
			ret = wait_event_interruptible(rxq->wq, rxq->head != rxq->tail ||
						       srflags_test(sr1xx_dev, FLAGS_READ_ABORTED));
			if (ret)
				return ret;
			if (srflags_test(sr1xx_dev, FLAGS_READ_ABORTED)) {
				return -ESHUTDOWN;
			}
		} else {
			dev_dbg(&sr1xx_dev->spi->dev,
				"Dequeue rx ring slot %d, %zu bytes\n",
				rxq->tail, rxq->buffers[rxq->tail].len);
			memcpy(rx_buff, &rxq->buffers[rxq->tail], sizeof(*rx_buff));
			rxq->buffers[rxq->tail].buff = NULL;
			rxq->buffers[rxq->tail].len = 0;
			rxq->tail = (rxq->tail + 1) % NR_RX_BUFFERS;
			spin_unlock(&rxq->lock);
			wake_up_all(&rxq->wq);
			break;
		}
	}
	return 0;
}

static int rxq_push(struct rx_queue *rxq, void *buff, size_t len)
{
	struct sr1xx_dev *sr1xx_dev = get_dev_from_rxq(rxq);
	int i, ret, next, retries;
	void *copied_buff;

	copied_buff = kmalloc(len, GFP_KERNEL);
	if (!copied_buff) {
		dev_err(&sr1xx_dev->spi->dev, "Failed to allocate rx buffer %zu bytes.\n", len);
		return -ENOMEM;
	}
	memcpy(copied_buff, buff, len);

	retries = MAX_RETRIES_ON_FULL;
	while(retries--) {
		if (srflags_test(sr1xx_dev, FLAGS_READ_ABORTED)) {
			dev_warn(&sr1xx_dev->spi->dev,
				 "packet received while aborted\n");
			kfree(copied_buff);
			return -ESHUTDOWN;
		}

		spin_lock(&rxq->lock);
		next = (rxq->head + 1) % NR_RX_BUFFERS;
		if (next == rxq->tail) {
			/* Full */
			spin_unlock(&rxq->lock);
			dev_warn(&sr1xx_dev->spi->dev, "RX queue full\n");
			ret = wait_event_interruptible_timeout(rxq->wq, next != rxq->tail ||
						srflags_test(sr1xx_dev, FLAGS_READ_ABORTED),
						FALLBACK_ON_FULL_MS);
			if (ret <= 0) {
				dev_err(&sr1xx_dev->spi->dev, "Drop %u packets.\n",
					NR_DROPS_ON_FULL);

				spin_lock(&rxq->lock);
				for (i = 0; i < NR_DROPS_ON_FULL; i++) {
					int tail = rxq->tail;
					free_one_buffer(&rxq->buffers[tail]);
					rxq->tail = (tail + 1) % NR_RX_BUFFERS;
				}
				spin_unlock(&rxq->lock);
			}
		} else {
			struct rx_buffer *rx_buff = &rxq->buffers[rxq->head];

			rx_buff->buff = copied_buff;
			rx_buff->len = len;
			rxq->head = next;
			dev_dbg(&sr1xx_dev->spi->dev,
				"Queued a packet to slot %d (%zu bytes), q_len=%zu\n",
				rxq->head, len, rxq_len(rxq));
			spin_unlock(&rxq->lock);
			wake_up_all(&rxq->wq);
			return 0;
		}
	}

	kfree(copied_buff);
	return -ENOMEM;
}

static void sr1xx_fw_latency_qos(struct sr1xx_dev *sr1xx_dev, bool on)
{
	if (on) {
		/*
		 * FW download involves many spi transactions.
		 * turn off C-states during FW download (around 500ms)
		 * to shorten the time.
		 */
		if (!srflags_test_and_set(sr1xx_dev, FLAGS_QOS_APPLIED)) {
			memset(&sr1xx_dev->qos_latency, 0, sizeof(sr1xx_dev->qos_latency));
			cpu_latency_qos_add_request(&sr1xx_dev->qos_latency, 0);
			pr_info("cpu latency pm qos applied.");
		}
	} else {
		if (srflags_test_and_clear(sr1xx_dev, FLAGS_QOS_APPLIED)) {
			pr_info("cpu latency pm qos removed.");
			cpu_latency_qos_remove_request(&sr1xx_dev->qos_latency);
		}
	}
}

/* Spi write/read operation mode */
enum spi_operation_modes { SR1XX_WRITE_MODE, SR1XX_READ_MODE };

static int sr1xx_dev_open(struct inode *inode, struct file *filp)
{
	struct sr1xx_dev *sr1xx_dev = container_of(filp->private_data, struct sr1xx_dev, sr1xx_device);

	filp->private_data = sr1xx_dev;
	return 0;
}

static int sr1xx_dev_release(struct inode *inode, struct file *filp)
{
	struct sr1xx_dev *sr1xx_dev = filp->private_data;

	stop_rx_thread(sr1xx_dev);

	sr1xx_fw_latency_qos(sr1xx_dev, false);

	return 0;
}

static irqreturn_t sr1xx_dev_irq_handler(int irq, void *dev_id)
{
	struct sr1xx_dev *sr1xx_dev = dev_id;

	pm_wakeup_dev_event(&sr1xx_dev->spi->dev, 500, true);

	srflags_set(sr1xx_dev, FLAGS_IRQ_RECEIVED);

	/* Wake up waiting readers */
	wake_up(&sr1xx_dev->read_wq);

	return IRQ_HANDLED;
}

static int sr1xx_regulator_ctl(struct sr1xx_dev *sr1xx_dev, bool hpm_dig, bool hpm_rf)
{
	int ua_dig = hpm_dig ? USB_HSPHY_1P8_HPM_LOAD : USB_HSPHY_1P8_LOWPOWER_LOAD;
	int ua_rf = hpm_rf ? USB_HSPHY_1P8_HPM_LOAD : USB_HSPHY_1P8_LOWPOWER_LOAD;
	int ua_dig_org = hpm_dig ? USB_HSPHY_1P8_LOWPOWER_LOAD : USB_HSPHY_1P8_HPM_LOAD;
	int ret;

	/*
	 * In case of failed operation of power/regulator control,
	 * caller of ioctl should close the device.
	 */
	ret = regulator_set_load(sr1xx_dev->regulator_1v8_dig, ua_dig);
	if (ret) {
		dev_err(&sr1xx_dev->spi->dev, "failed to set dig regulator load %duA\n", ua_dig);
		return ret;
	}
	ret = regulator_set_load(sr1xx_dev->regulator_1v8_rf, ua_rf);
	if (ret) {
		regulator_set_load(sr1xx_dev->regulator_1v8_dig, ua_dig_org);
		dev_err(&sr1xx_dev->spi->dev, "failed to set rf regulator load\n", ua_rf);
		return ret;
	}
	usleep_range(50, 100);
	return 0;
}

static int sr1xx_power_ctl(struct sr1xx_dev *sr1xx_dev, bool on)
{
	int ret;

	if (on) {
		srflags_clear(sr1xx_dev, FLAGS_SUSPENDED);
		ret = sr1xx_regulator_ctl(sr1xx_dev, true, true);
		if (ret) {
			return ret;
		}
		if (!srflags_test_and_set(sr1xx_dev, FLAGS_PWR_ENABLED)) {
			dev_info(&sr1xx_dev->spi->dev, "requested chip enabled.\n");

			gpio_set_value(sr1xx_dev->ce_gpio, 1);
			/* chip will be turned on within 950us */
			msleep(2);
			/* ignore the irq asserted from the last power cycle */
			srflags_clear(sr1xx_dev, FLAGS_IRQ_RECEIVED);
		}
	} else {
		if (srflags_test_and_clear(sr1xx_dev, FLAGS_PWR_ENABLED)) {
			dev_info(&sr1xx_dev->spi->dev, "requested chip disabled.\n");
			gpio_set_value(sr1xx_dev->ce_gpio, 0);
		}
		/* CE low > 80usec --> HPD */
		usleep_range(160, 240);
		ret = sr1xx_regulator_ctl(sr1xx_dev, false, false);
	}
	return ret;
}

static int sr1xx_suspend_ctl(struct sr1xx_dev *sr1xx_dev, bool suspend)
{
	int ret = 0;

	dev_info(&sr1xx_dev->spi->dev, "suspend control: %s, mode=%d", suspend ? "SUSPEND" : "RESUME",
			sr1xx_dev->suspend_mode);

	if (srflags_test(sr1xx_dev, FLAGS_SUSPENDED) == suspend) {
		dev_warn(&sr1xx_dev->spi->dev, "suspend control: already in %s, mode=%d",
			suspend ? "SUSPEND" : "RESUME", sr1xx_dev->suspend_mode);
		return 0;
	}

	if (!suspend) {
		/* resume: power_ctl(true) also makes regulators NPM */
		ret = sr1xx_power_ctl(sr1xx_dev, true);
	} else {
		/* suspend */
		switch (sr1xx_dev->suspend_mode) {
		case SR1XX_SUSPEND_RF_LPM:
			ret = sr1xx_regulator_ctl(sr1xx_dev, true, false);
			break;
		case SR1XX_SUSPEND_ALL_LPM:
			ret = sr1xx_regulator_ctl(sr1xx_dev, false, false);
			break;
		case SR1XX_SUSPEND_OFF:
			ret = sr1xx_power_ctl(sr1xx_dev, false);
			break;
		case SR1XX_SUSPEND_NONE:
			/* fall through */
		default:
			break;
		}
	}
	if (!ret) {
		if (suspend)
			srflags_set(sr1xx_dev, FLAGS_SUSPENDED);
		else
			srflags_clear(sr1xx_dev, FLAGS_SUSPENDED);
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
			stop_rx_thread(sr1xx_dev);
			ret = sr1xx_power_ctl(sr1xx_dev, true);
		} else if (arg == PWR_DISABLE) {
			stop_rx_thread(sr1xx_dev);
			ret = sr1xx_power_ctl(sr1xx_dev, false);
		} else if (arg == PWR_SUSPEND) {
			sr1xx_suspend_ctl(sr1xx_dev, true);
		} else if (arg == PWR_RESUME) {
			sr1xx_suspend_ctl(sr1xx_dev, false);
		} else if (arg == ABORT_READ_PENDING) {
			dev_info(&sr1xx_dev->spi->dev, "Abort read requested\n");
			/* This will set FLAGS_READ_ABORTED */
			stop_rx_thread(sr1xx_dev);
		}
		break;
	case SR1XX_SET_FWD:
		if (arg == 1) {
			if (!srflags_test_and_set(sr1xx_dev, FLAGS_FW_DOWNLOAD)) {
				srflags_clear(sr1xx_dev, FLAGS_READ_ABORTED);
				sr1xx_fw_latency_qos(sr1xx_dev, true);
			} else {
				dev_warn(&sr1xx_dev->spi->dev,
					 "FW download mode already set.\n");
			}
		}
		else if(arg == 0) {
			if (srflags_test_and_clear(sr1xx_dev, FLAGS_FW_DOWNLOAD)) {
				srflags_clear(sr1xx_dev, FLAGS_READ_ABORTED);
				sr1xx_fw_latency_qos(sr1xx_dev, false);
				ret = start_rx_thread(sr1xx_dev);
			} else {
				dev_warn(&sr1xx_dev->spi->dev,
					 "FW download mode already cleared.\n");
			}
		}
		break;

	case SR1XX_GET_THROUGHPUT:
		dev_warn(&sr1xx_dev->spi->dev, "SR1XX_GET_THROUGHPUT not supported!\n");
		break;
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

static int sr1xx_wait_irq(struct sr1xx_dev* sr1xx_dev, long timeout_ms)
{
	int ret;

	if (timeout_ms) {
		long timeout = msecs_to_jiffies(timeout_ms);
		ret = wait_event_interruptible_timeout(sr1xx_dev->read_wq,
				srflags_test(sr1xx_dev, FLAGS_IRQ_RECEIVED), timeout);
		if (ret > 0)
			ret = 0;
		else if (ret == 0)
			ret = -ETIMEDOUT;
	} else {
		ret = wait_event_interruptible(sr1xx_dev->read_wq,
				srflags_test(sr1xx_dev, FLAGS_IRQ_RECEIVED) ||
				srflags_test(sr1xx_dev, FLAGS_READ_ABORTED));
	}

	if (!ret)
		srflags_clear(sr1xx_dev, FLAGS_IRQ_RECEIVED);

	return ret;
}

/*
 * RX sync signal + wait 2nd IRQ
 * This should be called after 1st IRQ.
 */
static int sr1xx_rx_handshake(struct sr1xx_dev* sr1xx_dev)
{
	/*
	 * Rarely, IRQ could be asserted twice before RXSYNC:
	 *
	 *  SPI    __________||||______
	 *                   _________
	 * RXSYNC  _________|
	 *            _    _      ______
	 * INT     __| |__| |____|
	 *
	 *
	 * In case of that, SPI RX transaction should be delayed after
	 * 3rd INT signal, otherwise host will see garbages of zeros
	 * from MISO lines. So here, clear IRQ_RECEIVED flags before RX_SYNC.
	 */
	int ret;

	if (srflags_test_and_clear(sr1xx_dev, FLAGS_IRQ_RECEIVED)) {
		dev_warn(&sr1xx_dev->spi->dev, "RX irq asserted more than once before rx sync\n");
	}

	gpio_set_value(sr1xx_dev->spi_handshake_gpio, 1);
	ret = sr1xx_wait_irq(sr1xx_dev, sr1xx_dev->timeout_in_ms);
	if (ret < 0) {
		dev_err(&sr1xx_dev->spi->dev, "2nd RX IRQ might not asserted.\n");
	}
	return ret;
}

/**
 * sr1xx_wait_for_irq_gpio_low
 *
 * Function to wait till irq gpio goes low state
 *
 */
static int sr1xx_wait_for_irq_gpio_low(struct sr1xx_dev *sr1xx_dev)
{
	unsigned long timeout;
	timeout = jiffies + msecs_to_jiffies(10);

	for (;;) {
		if (!gpio_get_value(sr1xx_dev->irq_gpio))
			return 0;

		usleep_range(10, 15);
		if (time_after(jiffies, timeout)) {
			break;
		}
	}
	return -ETIMEDOUT;
}

/*
 * thread should be started when FW download mode gets off
 */
static int sr1xx_rx_thread(void *data)
{
	int ret;
	struct sr1xx_dev *sr1xx_dev = data;

	dev_info(&sr1xx_dev->spi->dev, "RX thread started.\n");

	while (1) {
		size_t payload_len;
		uint8_t *buff = sr1xx_dev->rx_buffer;

		gpio_set_value(sr1xx_dev->spi_handshake_gpio, 0);

		if (srflags_test(sr1xx_dev, FLAGS_FW_DOWNLOAD)) {
			dev_err(&sr1xx_dev->spi->dev, "RX thread is running while HBCI mode.\n");
			break;
		}

		/* For RX/TX contention check */
		srflags_clear(sr1xx_dev, FLAGS_RX_ONGOING);
		ret = sr1xx_wait_irq(sr1xx_dev, 0);
		if (ret) {
			if (ret != -ERESTARTSYS) {
				dev_err(&sr1xx_dev->spi->dev, "Failed to wait irq ret=%d\n", ret);
			}
			continue;
		}
		srflags_set(sr1xx_dev, FLAGS_RX_ONGOING);

		if (srflags_test(sr1xx_dev, FLAGS_READ_ABORTED)) {
			dev_info(&sr1xx_dev->spi->dev, "Read aborted.\n");
			break;
		}

		if (!srflags_test(sr1xx_dev, FLAGS_PWR_ENABLED) ||
		    srflags_test(sr1xx_dev, FLAGS_SUSPENDED)) {
			dev_err(&sr1xx_dev->spi->dev, "RX IRQ while power down.\n");
			break;
		}

		/* TX/RX lock */
		if (!mutex_trylock(&sr1xx_dev->sr1xx_access_lock)) {
			dev_dbg(&sr1xx_dev->spi->dev, "TX/RX contention - wait TX.\n");
			mutex_lock(&sr1xx_dev->sr1xx_access_lock);
			dev_dbg(&sr1xx_dev->spi->dev, "TX/RX contention - continue RX.\n");
		}

		if (!gpio_get_value(sr1xx_dev->irq_gpio)) {
			dev_err(&sr1xx_dev->spi->dev, "IRQ might have gone low due to write.\n");
			mutex_unlock(&sr1xx_dev->sr1xx_access_lock);
			continue;
		}

		/* RX handshake + wait 2nd IRQ */
		ret = sr1xx_rx_handshake(sr1xx_dev);
		if (ret < 0) {
			mutex_unlock(&sr1xx_dev->sr1xx_access_lock);
			continue;
		}

		/* UCI header */
		ret = spi_read(sr1xx_dev->spi, buff, UCI_HEADER_LEN);
		if (ret < 0) {
			dev_err(&sr1xx_dev->spi->dev, "UCI header read error %d\n ", ret);
			mutex_unlock(&sr1xx_dev->sr1xx_access_lock);
			continue;
		}

		if ((sr1xx_dev->rx_buffer[0] & UCI_MT_MASK) == 0) {
			/* Data packet */
			payload_len = (buff[UCI_PAYLOAD_LEN_OFFSET] << 8) | buff[UCI_EXT_PAYLOAD_LEN_OFFSET];
		} else {
			/* UCI packet */
			bool is_ext = buff[UCI_EXT_PAYLOAD_LEN_IND_OFFSET] & UCI_EXT_PAYLOAD_LEN_IND_OFFSET_MASK;

			if (is_ext)
				payload_len = (buff[UCI_PAYLOAD_LEN_OFFSET] << 8) | buff[UCI_EXT_PAYLOAD_LEN_OFFSET];
			else
				payload_len = buff[UCI_PAYLOAD_LEN_OFFSET];

			if ((payload_len + UCI_HEADER_LEN) > MAX_UCI_PKT_SIZE) {
				dev_err(&sr1xx_dev->spi->dev, "UCI too big to receive (payload %zu bytes).\n", payload_len);
				mutex_unlock(&sr1xx_dev->sr1xx_access_lock);
				continue;
			}

			ret = spi_read(sr1xx_dev->spi, buff + UCI_HEADER_LEN, payload_len);
			if (ret < 0) {
				dev_err(&sr1xx_dev->spi->dev, "UCI payload read error %d (payload=%zu).\n ",
					ret, payload_len);
				mutex_unlock(&sr1xx_dev->sr1xx_access_lock);
				continue;
			}

			ret = sr1xx_wait_for_irq_gpio_low(sr1xx_dev);
			if (ret) {
				dev_err(&sr1xx_dev->spi->dev, "UWBS not released the IRQ after RX.\n");
			}

			rxq_push(&sr1xx_dev->rx_queue, buff, UCI_HEADER_LEN + payload_len);
		}
		mutex_unlock(&sr1xx_dev->sr1xx_access_lock);
	} /* end of loop */

	gpio_set_value(sr1xx_dev->spi_handshake_gpio, 0);

	rxq_clear(&sr1xx_dev->rx_queue);

	srflags_clear(sr1xx_dev, FLAGS_RX_ONGOING);

	/* kthread_stop() should be called */
	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
	}
	set_current_state(TASK_RUNNING);

	dev_info(&sr1xx_dev->spi->dev, "RX thread stopped.\n");

	return 0;
}

static int start_rx_thread(struct sr1xx_dev *sr1xx_dev)
{
	struct task_struct *thread;

	if (sr1xx_dev->rx_thread) {
		dev_err(&sr1xx_dev->spi->dev, "Cannot init RX thread, it's alive.\n");
		return -EIO;
	}

	if (!rxq_is_empty(&sr1xx_dev->rx_queue)) {
		dev_warn(&sr1xx_dev->spi->dev, "RX queue was not empty, clear it.\n");
		rxq_clear(&sr1xx_dev->rx_queue);
	}

	thread = kthread_run(sr1xx_rx_thread, sr1xx_dev, "sr1xx-rx-thread");
	if (IS_ERR_OR_NULL(thread)) {
		dev_err(&sr1xx_dev->spi->dev, "Failed to create rx thread.\n");
		return PTR_ERR(thread);
	}

	sr1xx_dev->rx_thread = thread;

	return 0;
}

static int stop_rx_thread(struct sr1xx_dev *sr1xx_dev)
{
	if (!sr1xx_dev->rx_thread) {
		dev_info(&sr1xx_dev->spi->dev, "RX thread has already stopped.\n");
		return -ENOENT;
	}

	dev_info(&sr1xx_dev->spi->dev, "Stopping RX thread.\n");

	srflags_set(sr1xx_dev, FLAGS_READ_ABORTED);

	/* wake up from sr1xx_wait_irq() */
	wake_up(&sr1xx_dev->read_wq);

	/* wake up rxq users */
	wake_up_all(&sr1xx_dev->rx_queue.wq);

	kthread_stop(sr1xx_dev->rx_thread);

	sr1xx_dev->rx_thread = NULL;

	return 0;
}

static ssize_t sr1xx_dev_write(struct file *filp, const char *buf, size_t count, loff_t * offset)
{
	int ret;
	struct sr1xx_dev *sr1xx_dev;

	sr1xx_dev = filp->private_data;

	if (!srflags_test(sr1xx_dev, FLAGS_PWR_ENABLED) || srflags_test(sr1xx_dev, FLAGS_SUSPENDED)) {
		dev_err(&sr1xx_dev->spi->dev, "write called while in low power mode %s,%s\n",
				srflags_test(sr1xx_dev, FLAGS_PWR_ENABLED) ? "ACTIVE" : "HPD",
				srflags_test(sr1xx_dev, FLAGS_SUSPENDED) ? "LPM" : "NPM");
	}

	if (count < UCI_HEADER_LEN) {
		dev_err(&sr1xx_dev->spi->dev, "Write Size too small\n");
		return -ENOBUFS;
	}

	/* CMD/RSP turnaround max = 800ms */
	pm_wakeup_dev_event(&sr1xx_dev->spi->dev, 1 * MSEC_PER_SEC, true);

	if (count > SR1XX_MAX_TX_BUF_SIZE || count > SR1XX_TXBUF_SIZE) {
		dev_err(&sr1xx_dev->spi->dev, "Write Size Exceeds\n");
		return -ENOBUFS;
	}

	if (copy_from_user(sr1xx_dev->tx_buffer, buf, count)) {
		dev_err(&sr1xx_dev->spi->dev, "%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	if (srflags_test(sr1xx_dev, FLAGS_FW_DOWNLOAD)) {
		ret = spi_write(sr1xx_dev->spi, sr1xx_dev->tx_buffer, count);
		if (ret < 0) {
			dev_err(&sr1xx_dev->spi->dev, "Failed to write HBCI packet.\n");
			return ret;
		}
	} else {
		int retries = WRITE_FALLBACK_MAX_RETRIES;
		while (retries--) {
			mutex_lock(&sr1xx_dev->sr1xx_access_lock);

			/*
			 * To have lower chances of RX/TX race - TX fallback on collision
			 */
			if (!srflags_test(sr1xx_dev, FLAGS_IRQ_RECEIVED) &&
			    !srflags_test(sr1xx_dev, FLAGS_RX_ONGOING)) {
				break;
			}
			dev_dbg(&sr1xx_dev->spi->dev, "TX/RX contention - TX fallback.\n");
			mutex_unlock(&sr1xx_dev->sr1xx_access_lock);
			msleep(WRITE_FALLBACK_DELAY_MS);
		}

		if (retries < 0) {
			dev_warn(&sr1xx_dev->spi->dev, "TX/RX contention detected from TX side.\n");
		}

		/* Header */
		ret = spi_write(sr1xx_dev->spi, sr1xx_dev->tx_buffer, UCI_HEADER_LEN);
		if (ret < 0) {
			mutex_unlock(&sr1xx_dev->sr1xx_access_lock);
			dev_err(&sr1xx_dev->spi->dev, "Failed to write UCI header.\n");
			return ret;
		}

		/*
		 * XXX: can't we just use one transfer?
		 * In between header write and payload write UWBS needs some time
		 */
		usleep_range(WRITE_DELAY, WRITE_DELAY + WRITE_DELAY_RANGE_DIFF);

		/* Payload */
		ret = spi_write(sr1xx_dev->spi, sr1xx_dev->tx_buffer + UCI_HEADER_LEN, count - UCI_HEADER_LEN);
		if (ret < 0) {
			dev_err(&sr1xx_dev->spi->dev, "Failed to write UCI header.\n");
		}

		if (srflags_test(sr1xx_dev, FLAGS_IRQ_RECEIVED) ||
		    srflags_test(sr1xx_dev, FLAGS_RX_ONGOING)) {
			dev_dbg(&sr1xx_dev->spi->dev, "TX/RX contention detected on TX side.\n");
		}
		mutex_unlock(&sr1xx_dev->sr1xx_access_lock);
	}
	return count;
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
	if (srflags_test_and_clear(sr1xx_dev, FLAGS_READ_ABORTED)) {
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

static ssize_t sr1xx_dev_read(struct file *filp, char *buf, size_t count, loff_t *offset)
{
	struct sr1xx_dev *sr1xx_dev = filp->private_data;
	int ret;
	struct rx_buffer rx_buff;
	size_t copy_len;

	if (!srflags_test(sr1xx_dev, FLAGS_PWR_ENABLED))
		return -EIO;

	/*
	 * HBCI packet read
	 * RX thread should not be running
	 */
	if (srflags_test(sr1xx_dev, FLAGS_FW_DOWNLOAD)) {
		return sr1xx_hbci_read(sr1xx_dev, buf, count);
	}

	/* UCI packet read */
	if ((filp->f_flags & O_NONBLOCK) && rxq_is_empty(&sr1xx_dev->rx_queue)) {
		return -EAGAIN;
	}

	ret = rxq_pop(&sr1xx_dev->rx_queue, &rx_buff);
	if (ret < 0) {
		return ret;
	}

	if (count < rx_buff.len) {
		dev_err(&sr1xx_dev->spi->dev, "User rx buffer size(%zu) is not enough (packet=%zu), trim it.\n",
				count, rx_buff.len);
		copy_len = count;
	} else {
		copy_len = rx_buff.len;
	}

	if (copy_to_user(buf, rx_buff.buff, copy_len)) {
		dev_err(&sr1xx_dev->spi->dev, "failed to copy_to_user.\n");
		copy_len = 0;
	}

	kfree(rx_buff.buff);
	return copy_len;
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
	/* start with low power mode */
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
	.release = sr1xx_dev_release,
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

static ssize_t suspend_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sr1xx_dev *sr1xx_dev = dev_get_drvdata(dev);
	return sysfs_emit(buf, "%d\n", sr1xx_dev->suspend_mode);
}

static ssize_t suspend_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct sr1xx_dev *sr1xx_dev = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	ret = kstrtoul(buf, 0, &value);
	if (ret < 0 || ret > SR1XX_SUSPEND_OFF)
		return -EINVAL;
	sr1xx_dev->suspend_mode = value;
	return count;
}
static DEVICE_ATTR_RW(suspend_mode);

static struct attribute *sr1xx_attrs[] = {
	&dev_attr_suspend_mode.attr,
	NULL,
};
ATTRIBUTE_GROUPS(sr1xx);

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

	/* Default suspend mode = ALL LDOs LPM */
	sr1xx_dev->suspend_mode = SR1XX_SUSPEND_ALL_LPM;

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

	rxq_init(&sr1xx_dev->rx_queue);

	sr1xx_dev->spi->irq = gpio_to_irq(sr1xx_dev->irq_gpio);
	if (sr1xx_dev->spi->irq < 0) {
		dev_err(&spi->dev, "gpio_to_irq request failed gpio = 0x%x\n", sr1xx_dev->irq_gpio);
		goto err_misc;
	}
	/* request irq. The irq is set whenever the chip has data available
	 * for reading. It is cleared when all data has been read.
	 */
	irq_flags = IRQF_TRIGGER_RISING;
	sr1xx_dev->timeout_in_ms = 500;

	ret = devm_request_irq(&spi->dev, sr1xx_dev->spi->irq, sr1xx_dev_irq_handler, irq_flags,
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

	device_init_wakeup(&spi->dev, true);

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

	stop_rx_thread(sr1xx_dev);

	mutex_destroy(&sr1xx_dev->sr1xx_access_lock);

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

	if (device_may_wakeup(dev)) {
		enable_irq_wake(sr1xx_dev->spi->irq);
		disable_irq(sr1xx_dev->spi->irq);
	}
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

	if (device_may_wakeup(dev)) {
		disable_irq_wake(sr1xx_dev->spi->irq);
		enable_irq(sr1xx_dev->spi->irq);
	}

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
		.dev_groups = sr1xx_groups,
	},
	.probe = sr1xx_probe,
	.remove = (sr1xx_remove),
};

module_spi_driver(sr1xx_driver);

MODULE_AUTHOR("Manjunatha Venkatesh <manjunatha.venkatesh@nxp.com>");
MODULE_DESCRIPTION("NXP SR1XX SPI driver");
MODULE_LICENSE("GPL");
