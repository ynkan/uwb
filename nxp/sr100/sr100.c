/*====================================================================================*/
/*                                                                                    */
/*                        Copyright 2018-2019 NXP                                     */
/*                                                                                    */
/* This program is free software; you can redistribute it and/or modify               */
/* it under the terms of the GNU General Public License as published by               */
/* the Free Software Foundation; either version 2 of the License, or                  */
/* (at your option) any later version.                                                */
/*                                                                                    */
/* This program is distributed in the hope that it will be useful,                    */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of                     */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                      */
/* GNU General Public License for more details.                                       */
/*                                                                                    */
/* You should have received a copy of the GNU General Public License                  */
/* along with this program; if not, write to the Free Software                        */
/* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA          */
/*                                                                                    */
/*====================================================================================*/

/**
* \addtogroup spi_driver
*
* @{ */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <linux/linkage.h>

#include "sr100.h"

/* To control VDD gpios in Hikey for HVH board */
#define HVH_VDD_ENABLE 0
/* To control VDD Pmic for EOS */
#define PMIC_VDD_ENABLE 1
/* Cold reset Feature in case of Secure Element tx timeout */
#define ESE_COLD_RESET 0

#if ESE_COLD_RESET
#include "../pn8xT/pn553-i2c/cold_reset.h"
/*Invoke cold reset if no response from eSE*/
extern int ese_cold_reset(ese_cold_reset_origin_t src);
#endif

#define SR100_TXBUF_SIZE 4200
#define SR100_RXBUF_SIZE 4200
#define SR100_MAX_TX_BUF_SIZE 4200
#define MAX_READ_RETRY_COUNT 10
/* Macro to define SPI clock frequency */

#define SR100_SPI_CLOCK 16000000L;
#define ENABLE_THROUGHPUT_MEASUREMENT 1

/* Maximum UCI packet size supported from the driver */
#define MAX_UCI_PKT_SIZE 4200

#define USB_HSPHY_1P8_VOL_MIN			1704000 /* uV */
#define USB_HSPHY_1P8_VOL_MAX			1800000 /* uV */
#define USB_HSPHY_1P8_HPM_LOAD			19000	/* uA */

/* Different driver debug lever */
enum SR100_DEBUG_LEVEL { SR100_DEBUG_OFF, SR100_FULL_DEBUG, SR100_KERN_ALERT};
enum spi_status_codes {
	spi_transcive_success,
	spi_transcive_fail,
	spi_irq_wait_request,
	spi_irq_wait_timeout
};
enum spi_operation_modes {SR100_WRITE_MODE, SR100_READ_MODE};


/* Variable to store current debug level request by ioctl */
static unsigned char debug_level;

#define SR100_DBG_MSG(msg...)                                              \
  switch (debug_level) {                                                   \
    case SR100_DEBUG_OFF:                                                  \
      break;                                                               \
    case SR100_FULL_DEBUG:                                                 \
      printk(KERN_INFO "[NXP-SR100] :  " msg);                             \
      break;                                                               \
    case SR100_KERN_ALERT:                                                 \
      printk(KERN_ALERT "[NXP-SR100] :  " msg);                            \
      break;                                                               \
    default:                                                               \
      printk(KERN_ERR "[NXP-SR100] :  Wrong debug level %d", debug_level); \
      break;                                                               \
  }

#define SR100_ERR_MSG(msg...) printk(KERN_ERR "[NXP-SR100] : " msg);

/* Device specific macro and structure */
struct sr100_dev {
	wait_queue_head_t read_wq;      /* wait queue for read interrupt */
	struct spi_device* spi;         /* spi device structure */
	struct miscdevice sr100_device; /* char device as misc driver */
	unsigned int ce_gpio;           /* SW Reset gpio */
	unsigned int irq_gpio;          /* SR100 will interrupt DH for any ntf */
	unsigned int spi_handshake_gpio;     /* host ready to read data */

	atomic_t irq_received;          /* flag to indicate that irq is received */
	atomic_t read_abort_requested;
	bool pwr_enabled;               /* flag to indicate pwr */
	bool is_fw_dwnld_enabled;

	unsigned char* tx_buffer;       /* transmit buffer */
	unsigned char* rx_buffer;       /* receive buffer buffer */
	unsigned int write_count;       /* Holds nubers of  byte writen*/
	unsigned int read_count;        /* Hold nubers of  byte read */
	struct mutex  sr100_access_lock;/* Hold mutex lock to between read and write */
	size_t totalBtyesToRead;
	size_t IsExtndLenIndication;
	int mode;
	long timeOutInMs;
#if HVH_VDD_ENABLE
	unsigned int        vdd_1v8_gpio;
	unsigned int        vdd_1v8_rf_gpio;
	unsigned int        vbat_3v6_gpio;
#elif PMIC_VDD_ENABLE
	struct regulator	*regulator_1v8_dig;
	struct regulator	*regulator_1v8_rf;
#endif
};
#if (ENABLE_THROUGHPUT_MEASUREMENT == 1)
#define READ_THROUGH_PUT 0x01
#define WRITE_THROUGH_PUT 0x02
struct sr100_through_put {
	struct timespec64 rstart_tv;
	struct timespec64 wstart_tv;
	struct timespec64 rstop_tv;
	struct timespec64 wstop_tv;
	unsigned long total_through_put_wbytes;
	unsigned long total_through_put_rbytes;
	unsigned long total_through_put_rtime;
	unsigned long total_through_put_wtime;
};
static struct sr100_through_put sr100_through_put_t;
static void sr100_start_throughput_measurement(unsigned int type);
static void sr100_stop_throughput_measurement(unsigned int type, int no_of_bytes);

/******************************************************************************
 * Function    : sr100_start_throughput_measurement
 *
 * Description : Start this api to measaure the spi performance
 *
 * Parameters  : type  :  sr100 device Write/Read
 *
 * Returns     : Returns void
 ****************************************************************************/
static void sr100_start_throughput_measurement(unsigned int type)
{
	if (type == READ_THROUGH_PUT) {
		memset(&sr100_through_put_t.rstart_tv, 0x00, sizeof(struct timespec64));
		ktime_get_real_ts64(&sr100_through_put_t.rstart_tv);
	}
	else if (type == WRITE_THROUGH_PUT) {
		memset(&sr100_through_put_t.wstart_tv, 0x00, sizeof(struct timespec64));
		ktime_get_real_ts64(&sr100_through_put_t.wstart_tv);
	}
	else {
		printk(KERN_ALERT " sr100_start_throughput_measurement: wrong type = %d", type);
	}
}
/******************************************************************************
 * Function    : sr100_stop_throughput_measurement
 *
 * Description : Stop this api to end the measaure of the spi performance
 *
 * Parameters  : type  :  sr100 device Write/Read
 *
 * Returns     : Returns void
 ****************************************************************************/
static void sr100_stop_throughput_measurement(unsigned int type, int no_of_bytes)
{
	if (type == READ_THROUGH_PUT) {
		memset(&sr100_through_put_t.rstop_tv, 0x00, sizeof(struct timespec64));
		ktime_get_real_ts64(&sr100_through_put_t.rstop_tv);
		sr100_through_put_t.total_through_put_rbytes += no_of_bytes;
		sr100_through_put_t.total_through_put_rtime += (sr100_through_put_t.rstop_tv.tv_nsec - sr100_through_put_t.rstart_tv.tv_nsec) + ((sr100_through_put_t.rstop_tv.tv_sec - sr100_through_put_t.rstart_tv.tv_sec) * 1000000000);
	}
	else if (type == WRITE_THROUGH_PUT) {
		memset(&sr100_through_put_t.wstop_tv, 0x00, sizeof(struct timespec64));
		ktime_get_real_ts64(&sr100_through_put_t.wstop_tv);
		sr100_through_put_t.total_through_put_wbytes += no_of_bytes;
		sr100_through_put_t.total_through_put_wtime += (sr100_through_put_t.wstop_tv.tv_nsec - sr100_through_put_t.wstart_tv.tv_nsec) + ((sr100_through_put_t.wstop_tv.tv_sec - sr100_through_put_t.wstart_tv.tv_sec) * 1000000000);
	}
	else {
		printk(KERN_ALERT " sr100_stop_throughput_measurement: wrong type = %d", type);
	}
}
#endif

/******************************************************************************
 * Function    : sr100_dev_open
 *
 * Description : Open sr100 device node and returns instance to the user space
 *
 * Parameters  : inode  :  sr100 device node path
 *               filep  :  File pointer to structure of sr100 device
 *
 * Returns     : Returns file descriptor for sr100 device
 *               otherwise indicate each error code
 ****************************************************************************/
static int sr100_dev_open(struct inode* inode, struct file* filp)
{
	struct sr100_dev* sr100_dev = container_of(filp->private_data, struct sr100_dev, sr100_device);
	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);

	filp->private_data = sr100_dev;
	SR100_DBG_MSG("%s : Major No: %d, Minor No: %d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

/******************************************************************************
 * Function    : sr100_dev_irq_handler
 *
 * Description : Will get called when interrupt line asserted from SR100
 *
 * Parameters  : irq    :  IRQ Number
 *               dev_id :  sr100 device Id
 *
 * Returns     : Returns IRQ Handler
 ****************************************************************************/
static irqreturn_t sr100_dev_irq_handler(int irq, void* dev_id)
{
	struct sr100_dev* sr100_dev = dev_id;

	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);

	atomic_set(&sr100_dev->irq_received, 1);

	/* Wake up waiting readers */
	wake_up(&sr100_dev->read_wq);

	SR100_DBG_MSG("Exit : %s\n", __FUNCTION__);
	return IRQ_HANDLED;
}

static void sr100_power_ctl(struct sr100_dev *sr100_dev, bool on)
{
	if (on && !sr100_dev->pwr_enabled) {
		printk(KERN_ALERT "UWB SR100 chip enable");

		sr100_dev->pwr_enabled = true;

		/* ignore the irq asserted from the last power cycle */
		atomic_set(&sr100_dev->irq_received, 0);

		gpio_set_value(sr100_dev->ce_gpio, 1);
		msleep(10);
	} else if (!on && sr100_dev->pwr_enabled) {
		printk(KERN_ALERT "UWB SR100 chip disable");

		sr100_dev->pwr_enabled = false;
		gpio_set_value(sr100_dev->ce_gpio, 0);
		msleep(10);
	}
}

static int sr100_wait_irq(struct sr100_dev* sr100_dev, long timeout)
{
	int ret;

	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);

	if (timeout) {
		ret = wait_event_interruptible_timeout(sr100_dev->read_wq,
				atomic_read(&sr100_dev->irq_received), timeout);
		if (ret > 0)
			ret = 0;
		else if (ret == 0)
			ret = -ETIMEDOUT;
	} else {
		ret = wait_event_interruptible(sr100_dev->read_wq, atomic_read(&sr100_dev->irq_received));
	}

	if (!ret)
		atomic_set(&sr100_dev->irq_received, 0);

	SR100_DBG_MSG("-%s(%d)\n", __FUNCTION__, ret);

	return ret;
}

/******************************************************************************
 * Function    : sr100_dev_iotcl
 *
 * Description : Input/OutPut control from user space to perform required
 *               operation on sr100 device.
 *
 * Parameters  : cmd    :  Indicates what operation needs to be done sr100
 *               arg    :  Value to be passed to sr100 to do the required
 *                         opeation
 *
 * Returns     : 0 on success and (-1) on error
 ****************************************************************************/
static long sr100_dev_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct sr100_dev* sr100_dev = NULL;
	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);
	sr100_dev = filp->private_data;
	switch (cmd) {
		case SR100_SET_PWR:
			if (arg == PWR_ENABLE) {
				sr100_power_ctl(sr100_dev, true);
			}
			else if (arg == PWR_DISABLE) {
				sr100_power_ctl(sr100_dev, false);
			}
			else if (arg == ABORT_READ_PENDING) {
				pr_info("%s Abort Read Pending\n", __func__);
				atomic_set(&sr100_dev->read_abort_requested, 1);
				/* Wake up waiting readers */
				wake_up(&sr100_dev->read_wq);
			}
			break;
		case SR100_SET_FWD:
			if (arg == 1) {
				sr100_dev->is_fw_dwnld_enabled = true;
				atomic_set(&sr100_dev->read_abort_requested, 0);
				pr_info("%s FW download enabled.\n", __func__);
			}
			else if(arg == 0) {
				sr100_dev->is_fw_dwnld_enabled = false;
				pr_info("%s FW download disabled.\n", __func__);
			}
			break;

		case SR100_GET_THROUGHPUT:
			if (arg == 0) {
#if (ENABLE_THROUGHPUT_MEASUREMENT == 1)
				printk(KERN_ALERT  " **************** Write-Read Throughput: **************");
				printk(KERN_ALERT " No of Write Bytes = %ld", sr100_through_put_t.total_through_put_wbytes);
				printk(KERN_ALERT " No of Read Bytes = %ld", sr100_through_put_t.total_through_put_rbytes);
				printk(KERN_ALERT " Total Write Time (sec) = %ld.%09ld", sr100_through_put_t.total_through_put_wtime / 1000000000, sr100_through_put_t.total_through_put_wtime % 1000000000);
				printk(KERN_ALERT " Total Read  Time (sec) = %ld.%09ld", sr100_through_put_t.total_through_put_rtime / 1000000000, sr100_through_put_t.total_through_put_rtime % 1000000000);
				printk(KERN_ALERT " Total Write-Read Time (sec) = %ld.%09ld",
                                                                  (sr100_through_put_t.total_through_put_wtime + sr100_through_put_t.total_through_put_rtime) / 1000000000,
                                                                  (sr100_through_put_t.total_through_put_wtime + sr100_through_put_t.total_through_put_rtime) % 1000000000);
				sr100_through_put_t.total_through_put_wbytes = 0;
				sr100_through_put_t.total_through_put_rbytes = 0;
				sr100_through_put_t.total_through_put_wtime = 0;
				sr100_through_put_t.total_through_put_rtime = 0;
				printk(KERN_ALERT " **************** Write-Read Throughput: **************");
#endif
			}
			break;
#if ESE_COLD_RESET
		case SR100_ESE_RESET:
			pr_info("%s SR100_ESE_RESET Enter\n", __func__);
			ret = ese_cold_reset(ESE_COLD_RESET_SOURCE_UWB);
			break;
#endif
		default:
			printk(KERN_ALERT " Error case");
			ret = -EINVAL;  // ToDo: After adding proper switch cases we have to
			// return with error statusi here
	}

	return ret;
}
/******************************************************************************
* Function    : sr100_dev_transceive
*
* Description : Used to Write/read data from SR100
*
* Parameters  : sr100_dev :sr100  device structure pointer
*               op_mode   :Indicates write/read mode
*               count  :  Number of bytes to be write/read
* Returns     : Number of bytes write/read if read is success else (-1)
*               otherwise indicate each error code
****************************************************************************/

static int sr100_dev_transceive(struct sr100_dev* sr100_dev, int op_mode, int count)
{
	int ret, retry_count;
	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);
	mutex_lock(&sr100_dev->sr100_access_lock);
	sr100_dev->mode = op_mode;
	sr100_dev->totalBtyesToRead = 0;
	sr100_dev->IsExtndLenIndication = 0;
	ret = -1;
	retry_count = 0;

	switch(sr100_dev->mode) {
	case SR100_WRITE_MODE:
		sr100_dev->write_count = 0;
		/* UCI Header write */
		ret = spi_write(sr100_dev->spi, sr100_dev->tx_buffer, NORMAL_MODE_HEADER_LEN);
		if (ret < 0) {
			ret = -EIO;
			printk("spi_write header : Failed.\n");
			goto transcive_end;
		} else {
			count -= NORMAL_MODE_HEADER_LEN;
		}
		if(count > 0) {
			usleep_range(30, 50);
			/* UCI Payload write */
			ret = spi_write(sr100_dev->spi, sr100_dev->tx_buffer + NORMAL_MODE_HEADER_LEN, count);
			if (ret < 0) {
				ret = -EIO;
				printk("spi_write payload : Failed.\n");
				goto transcive_end;
			}
		}
		sr100_dev->write_count = count + NORMAL_MODE_HEADER_LEN;
		ret = spi_transcive_success;
		break;
	case SR100_READ_MODE:
		if (!sr100_dev->is_fw_dwnld_enabled && !gpio_get_value(sr100_dev->irq_gpio)) {
			SR100_DBG_MSG("IRQ might have gone before RX_SYNC.\n");
			ret = spi_irq_wait_request;
			goto transcive_end;
		}

		sr100_dev->read_count = 0;

		SR100_DBG_MSG("RX_SYNC on\n");
		gpio_set_value(sr100_dev->spi_handshake_gpio, 1);

		/* waiting for the 2nd irq assertion after RX_SYNC */
		ret = sr100_wait_irq(sr100_dev, sr100_dev->timeOutInMs);
		if (ret < 0) {
			SR100_DBG_MSG("Second IRQ is Low\n");
			ret = spi_irq_wait_timeout;
			goto transcive_end;
		}

		ret = spi_read(sr100_dev->spi, (void*)sr100_dev->rx_buffer, NORMAL_MODE_HEADER_LEN);
		if (ret < 0) {
			pr_info("sr100_dev_read: spi read error %d\n ", ret);
			ret = spi_transcive_fail;
			goto transcive_end;
		}

		sr100_dev->IsExtndLenIndication = (sr100_dev->rx_buffer[EXTND_LEN_INDICATOR_OFFSET] & EXTND_LEN_INDICATOR_OFFSET_MASK);
		sr100_dev->totalBtyesToRead = sr100_dev->rx_buffer[NORMAL_MODE_LEN_OFFSET];
		if(sr100_dev->IsExtndLenIndication) {
			sr100_dev->totalBtyesToRead = ((sr100_dev->totalBtyesToRead << 8) | sr100_dev->rx_buffer[EXTENDED_LENGTH_OFFSET]);
		}
		if(sr100_dev->totalBtyesToRead > (MAX_UCI_PKT_SIZE - NORMAL_MODE_HEADER_LEN)) {
			printk("Length %d  exceeds the max limit %d....", (int)sr100_dev->totalBtyesToRead, (int)MAX_UCI_PKT_SIZE);
			ret = -1;
			goto transcive_end;
		}
		if(sr100_dev->totalBtyesToRead > 0) {
			ret = spi_read(sr100_dev->spi, (void*)(sr100_dev->rx_buffer + NORMAL_MODE_HEADER_LEN), sr100_dev->totalBtyesToRead);
			if (ret < 0) {
				printk("sr100_dev_read: spi read error.. %d\n ", ret);
				goto transcive_end;
			}
		}
		sr100_dev->read_count = (unsigned int)(sr100_dev->totalBtyesToRead + NORMAL_MODE_HEADER_LEN);

		/* check the irq line state */
		retry_count = 0;
		do {
			usleep_range(10, 15);
			retry_count++;
			if(retry_count == 1000) {
				printk("Slave not released the IRQ even after 10ms");
				break;
			}
		} while(gpio_get_value(sr100_dev->irq_gpio));

		ret = spi_transcive_success;

		SR100_DBG_MSG("RX_SYNC off\n");
		gpio_set_value(sr100_dev->spi_handshake_gpio, 0);
		break;
	default:
		printk("invalid operation .....");
		break;
	}
transcive_end:
	if(sr100_dev->mode == SR100_READ_MODE) {
		gpio_set_value(sr100_dev->spi_handshake_gpio, 0);
	}
	mutex_unlock(&sr100_dev->sr100_access_lock);
	SR100_DBG_MSG("Exit : %s\n", __FUNCTION__);
	return ret;
}

/******************************************************************************
* Function    : sr100_hbci_write
*
* Description : Used to write hbci packets
*
* Parameters  : sr100_dev :sr100  device structure pointer
*               count  :  Number of bytes to be write
* Returns     : return  success(spi_transcive_success)or fail (-1)
****************************************************************************/

static int sr100_hbci_write(struct sr100_dev* sr100_dev, int count)
{
	int ret;
	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);
	sr100_dev->write_count = 0;

	/* HBCI write */
	ret = spi_write(sr100_dev->spi, sr100_dev->tx_buffer, count);
	if (ret < 0) {
		ret = -EIO;
		printk("spi_write fw download : Failed.\n");
		goto hbci_write_fail;
	}
	sr100_dev->write_count = count;
	ret = spi_transcive_success;
	return ret;
hbci_write_fail:
	printk("sr100_hbci_write failed...%d", ret);
	SR100_DBG_MSG("Exit : %s\n", __FUNCTION__);
	return ret;
}

/******************************************************************************
 * Function    : sr100_dev_write
 *
 * Description : Write Data to sr100 on SPI line
 *
 * Parameters  : filp   :  Device Node  File Pointer
 *               buf    :  Buffer which contains data to be sent to sr100
 *               count  :  Number of bytes to be send
 *               offset :  Pointer to a object that indicates file position
 *                         user is accessing.
 * Returns     : Number of bytes writen if write is success else (-1)
 *               otherwise indicate each error code
 ****************************************************************************/
static ssize_t sr100_dev_write(struct file* filp, const char* buf, size_t count, loff_t* offset)
{
	int ret;
	struct sr100_dev* sr100_dev;
	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);
	sr100_dev = filp->private_data;

	if (!sr100_dev->pwr_enabled)
		return -EIO;

	if (count > SR100_MAX_TX_BUF_SIZE || count > SR100_TXBUF_SIZE) {
		SR100_ERR_MSG("%s : Write Size Exceeds\n", __func__);
		ret = -ENOBUFS;
		goto write_end;
	}
	if (copy_from_user(sr100_dev->tx_buffer, buf, count)) {
		SR100_ERR_MSG("%s : failed to copy from user space \n", __func__);
		ret = -EFAULT;
		goto write_end;
	}
#if (ENABLE_THROUGHPUT_MEASUREMENT == 1)
	sr100_start_throughput_measurement(WRITE_THROUGH_PUT);
#endif
	if(sr100_dev->is_fw_dwnld_enabled) {
		ret = sr100_hbci_write(sr100_dev, count);
	}
	else {
		ret = sr100_dev_transceive(sr100_dev, SR100_WRITE_MODE, count);
	}
	if(ret == spi_transcive_success) {
		ret =  sr100_dev->write_count;
	}
	else {
		printk("write failed......");
	}
#if (ENABLE_THROUGHPUT_MEASUREMENT == 1)
	sr100_stop_throughput_measurement(WRITE_THROUGH_PUT, ret);
#endif
write_end:
	SR100_DBG_MSG("Exit : %s\n", __FUNCTION__);
	return ret;
}

/******************************************************************************
 * Function    : sr100_hbci_read
 *
 * Description : Read Data From sr100 on SPI line
 *
 * Parameters  : sr100_dev : sr100 device structure
 *               buf    :  Buffer which contains data to be read from sr100
 *               count  :  Number of bytes to be read
 *
 * Returns     : Number of bytes read if read is success else (-1)
 *               otherwise indicate each error code
 ****************************************************************************/
static ssize_t sr100_hbci_read(struct sr100_dev *sr100_dev, char* buf, size_t count)
{
	int ret = -EIO;
	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);
	if(count > SR100_RXBUF_SIZE) {
		SR100_ERR_MSG("count(%d) out of range(0-%d)\n", count, SR100_RXBUF_SIZE);
		ret = -EINVAL;
		goto hbci_fail;
	}
	/* wait for inetrrupt upto 500ms after that timeout will happen and returns read fail */
	ret = sr100_wait_irq(sr100_dev, sr100_dev->timeOutInMs);
	if (ret < 0) {
		printk("hbci wait_event_interruptible timeout() : Failed.\n");
		ret = -1;
		goto hbci_fail;
	}

	if (atomic_cmpxchg(&sr100_dev->read_abort_requested, 1, 0)) {
		printk("HBCI Abort Read pending......");
		SR100_DBG_MSG("Exit : %s\n", __FUNCTION__);
		return ret;
	}

#if (ENABLE_THROUGHPUT_MEASUREMENT == 1)
	sr100_start_throughput_measurement(READ_THROUGH_PUT);
#endif
	ret = spi_read(sr100_dev->spi, (void*)sr100_dev->rx_buffer, count);
	if (ret < 0) {
		pr_info("sr100_dev_read: spi read error %d\n ", ret);
		goto hbci_fail;
	}
	ret = count;
#if (ENABLE_THROUGHPUT_MEASUREMENT == 1)
	sr100_stop_throughput_measurement(READ_THROUGH_PUT, count);
#endif
	if (copy_to_user(buf, sr100_dev->rx_buffer, count)) {
		pr_info("sr100_dev_read: copy to user failed\n");
		ret = -EFAULT;
	}
	SR100_DBG_MSG("Exit : %s\n", __FUNCTION__);
	return ret;
hbci_fail:
	printk("Error sr100_fw_download ret %d Exit\n", ret);
	SR100_DBG_MSG("Exit : %s\n", __FUNCTION__);
	return ret;
}
/******************************************************************************
 * Function    : sr100_dev_read
 *
 * Description : Used to read data from SR100
 *
 * Parameters  : filp   :  Device Node  File Pointer
 *               buf    :  Buffer which contains data to be read from sr100
 *               count  :  Number of bytes to be read
 *               offset :  Pointer to a object that indicates file position
 *                         user is accessing.
 * Returns     : Number of bytes read if read is success else (-1)
 *               otherwise indicate each error code
 ****************************************************************************/
static ssize_t sr100_dev_read(struct file* filp, char* buf, size_t count, loff_t* offset)
{
	struct sr100_dev* sr100_dev = filp->private_data;
	int ret = -EIO;
	int retry_count = 0;

	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);

	if (!sr100_dev->pwr_enabled)
		return -EIO;

	memset(sr100_dev->rx_buffer, 0x00, SR100_RXBUF_SIZE);

	/*HBCI packet read*/
	if(sr100_dev->is_fw_dwnld_enabled) {
		ret = sr100_hbci_read(sr100_dev, buf, count);
		goto read_end;
	}

	/*UCI packet read*/
	if (!atomic_read(&sr100_dev->irq_received) && (filp->f_flags & O_NONBLOCK)) {
		ret = -EAGAIN;
		goto read_end;
	}

first_irq_wait:
	retry_count++;

	if (!atomic_read(&sr100_dev->read_abort_requested)) {
		ret = sr100_wait_irq(sr100_dev, 0);
		if (ret && !atomic_read(&sr100_dev->read_abort_requested)) {
			printk("wait_event_interruptible() : Failed.\n");
			goto read_end;
		}
	}
	if (atomic_cmpxchg(&sr100_dev->read_abort_requested, 1, 0)) {
		printk("Abort Read pending......");
		ret = -EINTR;
		goto read_end;
	}

	ret = sr100_dev_transceive(sr100_dev, SR100_READ_MODE, count);
	if(ret == spi_transcive_success) {
		if (copy_to_user(buf, sr100_dev->rx_buffer, sr100_dev->read_count)) {
			pr_info("sr100_dev_read: copy to user failed\n");
			SR100_DBG_MSG("sr100_dev_read: copy to user failed\n");
			ret = -EFAULT;
			goto read_end;
		}
		ret = sr100_dev->read_count;
	}
	else if(ret == spi_irq_wait_request) {
		printk(" irq is low due to write hence irq is requested again...");
		if (retry_count >= 3) {
			ret = -ETIMEDOUT;
			goto read_end;
		} else {
			goto first_irq_wait;
		}
	}
	else if(ret == spi_irq_wait_timeout) {
		printk("second irq is not received..Time out...");
		ret = -1;
	}
	else {
		SR100_DBG_MSG("spi read failed...%d\n", ret);
		ret = -1;
	}
read_end:
	SR100_DBG_MSG("Exit : %s\n", __FUNCTION__);
	return ret;
}

/******************************************************************************
 * Function    : sr100_hw_setup
 *
 * Description : Used to read data from SR100
 *
 * Parameters  : platform_data :  struct sr100_spi_platform_data *
 *
 * Returns     : retval 0 if ok else -1 on error
 ****************************************************************************/
static int sr100_hw_setup(struct device *dev, struct sr100_spi_platform_data* platform_data)
{
	int ret;
	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);
	ret = devm_gpio_request(dev, platform_data->irq_gpio, "sr100 irq");
	if (ret < 0) {
		SR100_ERR_MSG("gpio request failed gpio = 0x%x\n", platform_data->irq_gpio);
		goto fail;
	}

	ret = gpio_direction_input(platform_data->irq_gpio);
	if (ret < 0) {
		SR100_ERR_MSG("gpio request failed gpio = 0x%x\n", platform_data->irq_gpio);
		goto fail_irq;
	}

	ret = devm_gpio_request(dev, platform_data->ce_gpio, "sr100 ce");
	if (ret < 0) {
		SR100_ERR_MSG("gpio request failed gpio = 0x%x\n", platform_data->ce_gpio);
		goto fail;
	}

	ret = gpio_direction_output(platform_data->ce_gpio, 1);
	if (ret < 0) {
		pr_info("sr100 - Failed setting ce gpio - %d\n", platform_data->ce_gpio);
		goto fail_gpio;
	}

	ret = devm_gpio_request(dev, platform_data->spi_handshake_gpio, "sr100 ri");
	if (ret < 0) {
		pr_info("sr100 - Failed requesting ri gpio - %d\n", platform_data->spi_handshake_gpio);
		goto fail_gpio;
	}

	ret = gpio_direction_output(platform_data->spi_handshake_gpio, 0);
	if (ret < 0) {
		pr_info("sr100 - Failed setting spi handeshake gpio - %d\n", platform_data->spi_handshake_gpio);
		goto fail_gpio;
	}
#if HVH_VDD_ENABLE
	ret = devm_gpio_request(dev, platform_data->vdd_1v8_gpio, "sup_vdd_1v8");
	if (ret) {
		pr_info("%s:  sr100 vdd_1v8_gpio failed\n", __func__);
		goto fail_gpio;
	}
	ret = gpio_direction_output(platform_data->vdd_1v8_gpio, 0);
	if (ret < 0) {
		pr_err("%s : not able to set vdd_1v8_gpio as output\n", __func__);
		goto fail_gpio;
	}
	ret = devm_gpio_request(dev, platform_data->vdd_1v8_rf_gpio, "sup_vdd_rf");
	if (ret) {
		pr_info("%s:  sr100 vdd_1v8_rf_gpio failed\n", __func__);
		goto fail_gpio;
	}
	ret = gpio_direction_output(platform_data->vdd_1v8_rf_gpio, 0);
	if (ret < 0) {
		pr_err("%s : not able to set vdd_1v8_rf_gpio as output\n", __func__);
		goto fail_gpio;
	}
	ret = devm_gpio_request(dev, platform_data->vbat_3v6_gpio, "sup_vbat_3v6");
	if (ret) {
		pr_info("%s:  sr100 sup_vbat_3v6 failed\n", __func__);
		goto fail_gpio;
	}
	ret = gpio_direction_output(platform_data->vbat_3v6_gpio, 0);
	if (ret < 0) {
		pr_err("%s : not able to set vbat_3v6_gpio as output\n", __func__);
		goto fail_gpio;
	}

	pr_info(" HVH Power enable: %s \n", __func__);
#elif PMIC_VDD_ENABLE
	ret = regulator_set_load(platform_data->regulator_1v8_dig, USB_HSPHY_1P8_HPM_LOAD);
	if (ret < 0) {
		pr_err("Unable to set HPM of regulator_1v8_dig:%d\n", ret);
		goto fail_gpio;
	}
	ret = regulator_set_voltage(platform_data->regulator_1v8_dig, USB_HSPHY_1P8_VOL_MIN, USB_HSPHY_1P8_VOL_MAX);
	if (ret) {
		pr_err("Unable to set voltage for regulator_1v8_dig:%d\n", ret);
		goto fail_gpio;
	}

	ret = regulator_set_load(platform_data->regulator_1v8_rf, USB_HSPHY_1P8_HPM_LOAD);
	if (ret < 0) {
		pr_err("Unable to set HPM of regulator_1v8_rf:%d\n", ret);
		goto fail_gpio;
	}
	ret = regulator_set_voltage(platform_data->regulator_1v8_rf, USB_HSPHY_1P8_VOL_MIN, USB_HSPHY_1P8_VOL_MAX);
	if (ret) {
		pr_err("Unable to set voltage for regulator_1v8_rf:%d\n", ret);
		goto fail_gpio;
	}
	pr_info(" PMIC Power configure : %s \n", __func__);
#endif
	ret = 0;
	SR100_DBG_MSG("Exit : %s\n", __FUNCTION__);
	return ret;

fail_gpio:
fail_irq:
fail:
	SR100_ERR_MSG("sr100_hw_setup failed\n");
	return ret;
}
/******************************************************************************
 * Function    : sr100_set_data
 *
 * Description : Set the SR100 device specific context for future use
 *
 * Parameters  : spi :  struct spi_device *
 *               data:  void*
 *
 * Returns     : retval 0 if ok else -1 on error
 ****************************************************************************/
static inline void sr100_set_data(struct spi_device* spi, void* data)
{
	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);
	dev_set_drvdata(&spi->dev, data);
}

/******************************************************************************
 * Function    : sr100_get_data
 *
 * Description : Get the SR100 device specific context
 *
 * Parameters  : spi :  struct spi_device *
 *
 * Returns     : retval 0 if ok else -1 on error
 ****************************************************************************/
static inline void* sr100_get_data(const struct spi_device* spi)
{
	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);
	return dev_get_drvdata(&spi->dev);
}

/* possible fops on the sr100 device */
static const struct file_operations sr100_dev_fops = {
	.owner = THIS_MODULE,
	.read = sr100_dev_read,
	.write = sr100_dev_write,
	.open = sr100_dev_open,
	.unlocked_ioctl = sr100_dev_ioctl,
};
/******************************************************************************
 * Function    : sr100_parse_dt
 *
 * Description : Parse the dtsi configartion
 *
 * Parameters  : dev :  struct spi_device *
 *               pdata: Ponter to platform data
 *
 * Returns     : retval 0 if ok else -1 on error
 ****************************************************************************/
static int sr100_parse_dt(struct device* dev, struct sr100_spi_platform_data* pdata)
{
	struct device_node* np = dev->of_node;
	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);

	pdata->irq_gpio = of_get_named_gpio(np, "nxp,sr100-irq", 0);
	if (!gpio_is_valid(pdata->irq_gpio)) {
		return -EINVAL;
	}
	pdata->ce_gpio = of_get_named_gpio(np, "nxp,sr100-ce", 0);
	if (!gpio_is_valid(pdata->ce_gpio)) {
		return -EINVAL;
	}
	pdata->spi_handshake_gpio = of_get_named_gpio(np, "nxp,sr100-ri", 0);
	if (!gpio_is_valid(pdata->spi_handshake_gpio)) {
		return -EINVAL;
	}
#if HVH_VDD_ENABLE
	pdata->vdd_1v8_gpio = of_get_named_gpio(np, "nxp,sr100-vdd", 0);
	if ((!gpio_is_valid(pdata->vdd_1v8_gpio)))
		return -EINVAL;
	pdata->vdd_1v8_rf_gpio = of_get_named_gpio(np, "nxp,sr100-dig", 0);
	if ((!gpio_is_valid(pdata->vdd_1v8_rf_gpio)))
		return -EINVAL;
	pdata->vbat_3v6_gpio = of_get_named_gpio(np, "nxp,sr100-vbat", 0);
	if ((!gpio_is_valid(pdata->vbat_3v6_gpio)))
		return -EINVAL;
	pr_info("sr100 : vdd_1v8_gpio = %d, vdd_1v8_rf_gpio = %d, vbat_3v6_gpio = %d \n",
	        pdata->vdd_1v8_gpio, pdata->vdd_1v8_rf_gpio, pdata->vbat_3v6_gpio);
#elif PMIC_VDD_ENABLE
	pdata->regulator_1v8_dig = devm_regulator_get(dev, "nxp,sr100-dig");
	if (IS_ERR(pdata->regulator_1v8_dig)) {
		SR100_DBG_MSG("unable to get sr100-dig supply\n");
		return -EINVAL;
	}

	pdata->regulator_1v8_rf = devm_regulator_get(dev, "nxp,sr100-rf");
	if (IS_ERR(pdata->regulator_1v8_rf)) {
		pdata->regulator_1v8_rf = NULL;
		SR100_DBG_MSG("unable to get sr100-rf supply\n");
	}
#endif
	pr_info("sr100 : irq_gpio = %d, ce_gpio = %d, spi_handshake_gpio = %d \n",
	        pdata->irq_gpio, pdata->ce_gpio, pdata->spi_handshake_gpio);
	return 0;
}
/******************************************************************************
 * Function    : sr100_probe
 *
 * Description : To probe for SR100 SPI interface. If found initialize the SPI
 *               clock,bit rate & SPI mode. It will create the dev entry
 *               (SR100) for user space.
 * Parameters  : spi :  struct spi_device *
 *
 * Returns     : retval 0 if ok else -1 on error
 ****************************************************************************/
static int sr100_probe(struct spi_device* spi)
{
	int ret;
	struct sr100_spi_platform_data* platform_data = NULL;
	struct sr100_spi_platform_data platform_data1;
	struct sr100_dev* sr100_dev = NULL;
	unsigned int irq_flags;
	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);

	SR100_DBG_MSG("chip select : %d , bus number = %d \n", spi->chip_select, spi->master->bus_num);

	ret = sr100_parse_dt(&spi->dev, &platform_data1);
	if (ret) {
		pr_err("%s - Failed to parse DT\n", __func__);
		goto err_exit;
	}
	platform_data = &platform_data1;

	sr100_dev = kzalloc(sizeof(*sr100_dev), GFP_KERNEL);
	if (sr100_dev == NULL) {
		SR100_ERR_MSG("failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}
	ret = sr100_hw_setup(&spi->dev, platform_data);
	if (ret < 0) {
		SR100_ERR_MSG("Failed to sr100_enable_SR100_IRQ_ENABLE\n");
		goto err_exit0;
	}

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	spi->max_speed_hz = SR100_SPI_CLOCK;
	ret = spi_setup(spi);
	if (ret < 0) {
		SR100_ERR_MSG("failed to do spi_setup()\n");
		goto err_exit0;
	}

	sr100_dev->spi = spi;
	sr100_dev->sr100_device.minor = MISC_DYNAMIC_MINOR;
	sr100_dev->sr100_device.name = "sr100";
	sr100_dev->sr100_device.fops = &sr100_dev_fops;
	sr100_dev->sr100_device.parent = &spi->dev;
	sr100_dev->irq_gpio = platform_data->irq_gpio;
	sr100_dev->ce_gpio = platform_data->ce_gpio;
	sr100_dev->spi_handshake_gpio = platform_data->spi_handshake_gpio;

#if HVH_VDD_ENABLE
	sr100_dev->vdd_1v8_gpio = platform_data->vdd_1v8_gpio;
	sr100_dev->vdd_1v8_rf_gpio = platform_data->vdd_1v8_rf_gpio;
	sr100_dev->vbat_3v6_gpio = platform_data->vbat_3v6_gpio;
#elif PMIC_VDD_ENABLE
	sr100_dev->regulator_1v8_dig = platform_data->regulator_1v8_dig;
	sr100_dev->regulator_1v8_rf = platform_data->regulator_1v8_rf;
#endif
	sr100_dev->tx_buffer = kzalloc(SR100_TXBUF_SIZE, GFP_KERNEL);
	sr100_dev->rx_buffer = kzalloc(SR100_RXBUF_SIZE, GFP_KERNEL);
	if (sr100_dev->tx_buffer == NULL) {
		ret = -ENOMEM;
		goto exit_free_dev;
	}
	if (sr100_dev->rx_buffer == NULL) {
		ret = -ENOMEM;
		goto exit_free_dev;
	}

	dev_set_drvdata(&spi->dev, sr100_dev);

	/* init mutex and queues */
	init_waitqueue_head(&sr100_dev->read_wq);
	mutex_init(&sr100_dev->sr100_access_lock);

	ret = misc_register(&sr100_dev->sr100_device);
	if (ret < 0) {
		SR100_ERR_MSG("misc_register failed! %d\n", ret);
		goto err_exit0;
	}

	sr100_dev->spi->irq = gpio_to_irq(platform_data->irq_gpio);

	if (sr100_dev->spi->irq < 0) {
		SR100_ERR_MSG("gpio_to_irq request failed gpio = 0x%x\n",
		              platform_data->irq_gpio);
		goto err_exit1;
	}
	/* request irq.  the irq is set whenever the chip has data available
	     * for reading.  it is cleared when all data has been read.
	     */
	irq_flags = IRQF_TRIGGER_RISING;
	//irq_flags = IRQ_TYPE_LEVEL_HIGH;
	atomic_set(&sr100_dev->irq_received, 0);
	atomic_set(&sr100_dev->read_abort_requested, 0);
	sr100_dev->timeOutInMs = 500;

	ret = request_irq(sr100_dev->spi->irq, sr100_dev_irq_handler, irq_flags,
	                  sr100_dev->sr100_device.name, sr100_dev);
	if (ret) {
		SR100_ERR_MSG("request_irq failed\n");
		goto err_exit1;
	}
	//sr100_disable_irq(sr100_dev);
	gpio_set_value(sr100_dev->ce_gpio, 0);
#if HVH_VDD_ENABLE
	gpio_set_value(sr100_dev->vdd_1v8_gpio, 1);
	gpio_set_value(sr100_dev->vdd_1v8_rf_gpio, 1);
	gpio_set_value(sr100_dev->vbat_3v6_gpio, 1);
	pr_info(" VDD Req for HVH: %s\n", __func__);
#elif PMIC_VDD_ENABLE
	ret = regulator_enable(sr100_dev->regulator_1v8_dig);
	if (ret) {
		SR100_DBG_MSG("Unable to enable dig regulator: %d\n", ret);
		goto err_exit1;
	}
	ret = regulator_enable(sr100_dev->regulator_1v8_rf);
	if (ret) {
		SR100_DBG_MSG("Unable to enable rf regulator: %d\n", ret);
		goto exit_regulator;
	}
#endif

#if 0
	sr100_dev->pwr_enabled = true;
	gpio_set_value(sr100_dev->ce_gpio, 1);
#endif

	SR100_DBG_MSG("Exit : %s\n", __FUNCTION__);
	return ret;

#if PMIC_VDD_ENABLE
exit_regulator:
	regulator_disable(sr100_dev->regulator_1v8_dig);
#endif
err_exit1:
exit_free_dev:
	if (sr100_dev != NULL) {
		if (sr100_dev->tx_buffer) {
			kfree(sr100_dev->tx_buffer);
		}
		if (sr100_dev->rx_buffer) {
			kfree(sr100_dev->rx_buffer);
		}
		misc_deregister(&sr100_dev->sr100_device);
	}
err_exit0:
	if (sr100_dev != NULL) {
		mutex_destroy(&sr100_dev->sr100_access_lock);
	}
err_exit:
	if (sr100_dev != NULL) kfree(sr100_dev);
	SR100_DBG_MSG("ERROR: Exit : %s ret %d\n", __FUNCTION__, ret);
	return ret;
}

/******************************************************************************
 * Function    : sr100_remove
 *
 * Description : Will get called when the device is removed to release the
 *                 resources.
 *
 * Parameters  : spi :  struct spi_device *
 *
 * Returns     : retval 0 if ok else -1 on error
 ****************************************************************************/
static int sr100_remove(struct spi_device* spi)
{
	struct sr100_dev* sr100_dev = sr100_get_data(spi);
	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);
	mutex_destroy(&sr100_dev->sr100_access_lock);
	free_irq(sr100_dev->spi->irq, sr100_dev);
#if PMIC_VDD_ENABLE
	regulator_disable(sr100_dev->regulator_1v8_rf);
	regulator_disable(sr100_dev->regulator_1v8_dig);
#endif
	misc_deregister(&sr100_dev->sr100_device);
	if (sr100_dev != NULL) {
		if (sr100_dev->tx_buffer != NULL) kfree(sr100_dev->tx_buffer);
		if (sr100_dev->rx_buffer != NULL) kfree(sr100_dev->rx_buffer);
		kfree(sr100_dev);
	}
	SR100_DBG_MSG("Exit : %s\n", __FUNCTION__);
	return 0;
}
static struct of_device_id sr100_dt_match[] = {{
		.compatible = "nxp,sr100",
	},
	{}
};
static struct spi_driver sr100_driver = {
	.driver =
	{
		.name = "sr100",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		.of_match_table = sr100_dt_match,
	},
	.probe = sr100_probe,
	.remove = (sr100_remove),
};

/******************************************************************************
 * Function    : sr100_dev_init
 *
 * Description : Module init interface
 *
 * Parameters  :void
 *
 * Returns     : returns handle
 ****************************************************************************/
static int __init sr100_dev_init(void)
{
	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);
	return spi_register_driver(&sr100_driver);
}
module_init(sr100_dev_init);

module_param(debug_level, byte, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug_level, "Debug level (0=default,off, 1=INFO, 2=ALERT)");

/******************************************************************************
 * Function    : sr100_dev_exit
 *
 * Description : Module Exit interface
 *
 * Parameters  :void
 *
 * Returns     : returns void
 ****************************************************************************/
static void __exit sr100_dev_exit(void)
{
	SR100_DBG_MSG("Entry : %s\n", __FUNCTION__);
	spi_unregister_driver(&sr100_driver);
	SR100_DBG_MSG("Exit : %s\n", __FUNCTION__);
}
module_exit(sr100_dev_exit);

MODULE_AUTHOR("Manjunatha Venkatesh");
MODULE_DESCRIPTION("NXP SR100 SPI driver");
MODULE_LICENSE("GPL");

/** @} */
