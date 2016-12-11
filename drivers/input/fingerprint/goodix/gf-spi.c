#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/delay.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/err.h>
//#include "../pinctrl/core.h"


#include <linux/usb.h>
//#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/power_supply.h>

#include <linux/gpio.h>
//#include <mach/gpio.h>
//#include <plat/gpio-cfg.h>
#include <linux/kthread.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/completion.h>
#include "gf-spi.h"

/*spi device name*/
#define SPI_DEV_NAME   "spidev"
/*device name after register in charater*/
#define DEV_NAME "goodix_fp"

#define GF_PID "GFx18M"
#define GF_PID_LEN 6
#define GF_INPUT_MENU_KEY   KEY_MENU
#define GF_INPUT_BACK_KEY   KEY_BACK
#define GF_INPUT_HOME_KEY   KEY_HOME
#define GF_FF_KEY           KEY_POWER

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define	CHRD_DRIVER_NAME		"goodix"
#define	CLASS_NAME			    "goodix-spi"
#define SPIDEV_MAJOR			154	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */
static DECLARE_BITMAP(minors, N_SPI_MINORS);
#define FW_UPDATE               0
#define ESD_PROTECT             0
#define CFG_UPDATE              0

/**************************debug******************************/
#define SPI_ASYNC   1

#define DEFAULT_DEBUG   (0x1<<0)
#define SUSPEND_DEBUG   (0x1<<1)
#define SPI_DEBUG       (0x1<<2)
#define TIME_DEBUG      (0x1<<3)
#define FLOW_DEBUG      (0x1<<4)
#define gf_debug(level, fmt, args...) do{ \
    if(g_debug & level) {\
		pr_info("gf" fmt, ##args); \
    } \
}while(0)

#define FUNC_ENTRY()  gf_debug(FLOW_DEBUG, "gf:%s, entry\n", __func__)
#define FUNC_EXIT()  gf_debug(FLOW_DEBUG,"gf:%s, exit\n", __func__)

struct config_buf {
    unsigned int date;
    unsigned char buffer[249];
};

struct gf_config {
    unsigned char type; //hardware type
    unsigned char pid; //productor ID
    unsigned char config_num; //how many configs this productor has.
    struct config_buf *config;
};
#if FW_UPDATE
static struct config_buf config_buf_list[] = {
    {
		.date = 0x7df051c,
		.buffer = {
	    	0x41,0x3c,0x3c,0xe4,0x0c,0x30,0x3f,0x02,0x00,0x50,0x40,0x50,0x50,0xe4,0x0c,0x30,
	    	0x2f,0x03,0x00,0x03,0x11,0xa0,0x0d,0x00,0x14,0x03,0x0f,0x0f,0x0f,0xb2,0x3f,0xb3,
	    	0x33,0x03,0x90,0x01,0x40,0x05,0x0e,0x80,0x20,0x0f,0x22,0x00,0x08,0x07,0x08,0x06,
	    	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	    	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	    	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	    	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	    	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x00,0x25,0x04,0xca,0xa4,0x26,0x66,0x00,
	    	0x00,0x00,0x01,0x00,0x01,0x0f,0x96,0x00,0x01,0x02,0x85,0x00,0x03,0x20,0x20,0x50,
	    	0x3e,0x11,0x01,0x00,0x00,0x00,0x00,0x03,0x09,0x00,0x31,0x00,0x07,0x14,0x41,0x00,
	    	0x50,0x00,0x00,0x00,0x20,0x00,0x04,0x00,0x32,0x01,0xa0,0x00,0x00,0x79,0xc8,0x00,
	    	0x00,0x00,0x28,0x00,0x05,0x04,0x30,0x00,0x08,0x00,0x07,0x00,0x20,0x00,0x18,0x00,
	    	0x3d,0x00,0x48,0x00,0x22,0x00,0x00,0x00,0x03,0x07,0x80,0x00,0x20,0x00,0x20,0x00,
	    	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	    	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	    	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2c,0x01
		},
    },

};
static struct gf_config config_list[] = {
    {
		.type = 0,
		.pid = 1,
		.config_num = 1,
		.config = &config_buf_list[0],
    }
};
#endif
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
/*************************data stream***********************
 *	FRAME NO  | RING_H  | RING_L  |  DATA STREAM | CHECKSUM |
 *     1B      |   1B    |  1B     |    2048B     |  2B      |
 ************************************************************/
static unsigned bufsiz = 8 * (2048+5);
unsigned long g_debug = DEFAULT_DEBUG;
static void gf_irq_cfg(struct gf_dev* gf_dev)
{
	/*Config IRQ pin, referring to platform.*/
	struct pinctrl *pinctrl1;

	pinctrl1 = devm_pinctrl_get_select(&gf_dev->spi->dev, "goodix_irq_gpio");

	if (IS_ERR(pinctrl1))
		dev_err(&gf_dev->spi->dev, "failed to set external interrupt");

#if 0
 int ret;

    /* Get pinctrl if target uses pinctrl */
    gf_dev->gf_pinctrl = devm_pinctrl_get(&(gf_dev->spi->dev));
    if (IS_ERR_OR_NULL(gf_dev->gf_pinctrl)) {
        ret = PTR_ERR(gf_dev->gf_pinctrl);
        dev_err(&gf_dev->spi->dev,
            "Target does not use pinctrl %d\n", ret);
        goto err_pinctrl_get;
    }


    gf_dev->pinctrl_state_interrupt
        = pinctrl_lookup_state(gf_dev->fp_pinctrl,
            PINCTRL_STATE_INTERRUPT);
    if (IS_ERR_OR_NULL(gf_dev->pinctrl_state_interrupt)) {
        ret = PTR_ERR(gf_dev->pinctrl_state_interrupt);
        dev_err(&gf_dev->spi->dev,
            "Can not lookup %s pinstate %d\n",
            PINCTRL_STATE_INTERRUPT, ret);
        goto err_pinctrl_lookup;
    }


    printk("%s --\n", __func__);
    return 0;

err_pinctrl_lookup:
    devm_pinctrl_put(gf_dev->gf_pinctrl);
err_pinctrl_get:
    gf_dev->gf_pinctrl = NULL;
    return ret;

#endif

}

static unsigned char GF_FW[]=
{
#include "gf_fw.i"
};
#define FW_LENGTH (42*1024)
unsigned char* fw = GF_FW;

module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

void print_16hex(u8 *config, u8 len)
{
    u8 i,j = 0;
    printk("dump hex \n");
    for(i = 0 ; i< len ; i++) {
	printk("0x%x " , config[i]);
		if(j++ == 15) {
	    	j = 0;
	    	printk("\n");
		}
    }
    printk("\n");
}


/* -------------------------------------------------------------------- */
/* devfs                                */
/* -------------------------------------------------------------------- */
static ssize_t gf_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    printk("Show.\n");
    return 0;
}
static ssize_t gf_debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int debug_level = 0;
    sscanf(buf, "%d", &debug_level);
    printk("Store. debug_level = %d\n", debug_level);
    return strnlen(buf, count);
}
static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR, gf_debug_show, gf_debug_store);

static struct attribute *gf_debug_attrs[] = {
    &dev_attr_debug.attr,
    NULL
};

static const struct attribute_group gf_debug_attr_group = {
    .attrs = gf_debug_attrs,
    .name = "debug"
};

#ifdef SPI_ASYNC
static void gf_spi_complete(void *arg)
{
    complete(arg);
}
#endif //SPI_ASYNC

void gf_spi_setup(struct gf_dev *gf_dev, int max_speed_hz)
{
    gf_dev->spi->mode = SPI_MODE_0; //CPOL=CPHA=0
    gf_dev->spi->max_speed_hz = max_speed_hz;
    gf_dev->spi->irq = gpio_to_irq(gf_dev->irq_gpio);
    gf_dev->spi->bits_per_word = 8;
    spi_setup(gf_dev->spi);
}

/**********************************************************
 *Message format:
 *	write cmd   |  ADDR_H |ADDR_L  |  data stream  |
 *    1B         |   1B    |  1B    |  length       |
 *
 * read buffer length should be 1 + 1 + 1 + data_length
 ***********************************************************/
int gf_spi_write_bytes(struct gf_dev *gf_dev,
	u16 addr, u32 data_len, u8 *tx_buf)
{
#ifdef SPI_ASYNC
    DECLARE_COMPLETION_ONSTACK(read_done);
#endif
    struct spi_message msg;
    struct spi_transfer *xfer;
    int ret = 0;

    xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
    if( xfer == NULL){
		pr_warn("No memory for command.\n");
		return -ENOMEM;
    }

    /*send gf command to device.*/
    spi_message_init(&msg);
    tx_buf[0] = GF_W;
    tx_buf[1] = (u8)((addr >> 8)&0xFF);
    tx_buf[2] = (u8)(addr & 0xFF);
    xfer[0].tx_buf = tx_buf;
    xfer[0].len = data_len + 3;
    xfer[0].delay_usecs = 5;
    spi_message_add_tail(xfer, &msg);
#ifdef SPI_ASYNC
    msg.complete = gf_spi_complete;
    msg.context = &read_done;

    spin_lock_irq(&gf_dev->spi_lock);
    ret = spi_async(gf_dev->spi, &msg);
    spin_unlock_irq(&gf_dev->spi_lock);
    if(ret == 0) {
		wait_for_completion(&read_done);
		if(msg.status == 0)
	    	//ret = msg.actual_length - GF_WDATA_OFFSET;
	    	ret = data_len;
    }
#else
    ret = spi_sync(gf_dev->spi, &msg);
    if(ret == 0) {
		//ret = msg.actual_length - GF_WDATA_OFFSET;
		ret = data_len;
    }
#endif
    gf_debug(SPI_DEBUG, "ret = %d, actual_length = %d\n", ret, msg.actual_length);
    kfree(xfer);
    if(xfer != NULL)
	xfer = NULL;

    return ret;
}

/*************************************************************
 *First message:
 *	write cmd   |  ADDR_H |ADDR_L  |
 *    1B         |   1B    |  1B    |
 *Second message:
 *	read cmd   |  data stream  |
 *    1B        |   length    |
 *
 * read buffer length should be 1 + 1 + 1 + 1 + data_length
 **************************************************************/
int gf_spi_read_bytes(struct gf_dev *gf_dev,
	u16 addr, u32 data_len, u8 *rx_buf)
{
#ifdef SPI_ASYNC
    DECLARE_COMPLETION_ONSTACK(write_done);
#endif //SPI_ASYNC
    struct spi_message msg;
    struct spi_transfer *xfer;
    int ret = 0;

    xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
    if( xfer == NULL){
		pr_warn("No memory for command.\n");
		return -ENOMEM;
    }

    /*send gf command to device.*/
    spi_message_init(&msg);
    rx_buf[0] = GF_W;
    rx_buf[1] = (u8)((addr >> 8)&0xFF);
    rx_buf[2] = (u8)(addr & 0xFF);
    xfer[0].tx_buf = rx_buf;
    xfer[0].len = 3;
    xfer[0].delay_usecs = 5;
    spi_message_add_tail(&xfer[0], &msg);

    /*if wanted to read data from gf.
     *Should write Read command to device
     *before read any data from device.
     */
    //memset(rx_buf, 0xff, data_len);
    spi_sync(gf_dev->spi, &msg);
    spi_message_init(&msg);
    rx_buf[4] = GF_R;
    xfer[1].tx_buf = &rx_buf[4];

    xfer[1].rx_buf = &rx_buf[4];
    xfer[1].len = data_len + 1;
    xfer[1].delay_usecs = 5;

    spi_message_add_tail(&xfer[1], &msg);

#ifdef SPI_ASYNC
    msg.complete = gf_spi_complete;
    msg.context = &write_done;

    spin_lock_irq(&gf_dev->spi_lock);
    ret = spi_async(gf_dev->spi, &msg);
    spin_unlock_irq(&gf_dev->spi_lock);
    if(ret == 0) {
		wait_for_completion(&write_done);
		if(msg.status == 0){
	    	//ret = msg.actual_length - 1;//GF_RDATA_OFFSET;
	    	ret = data_len;
			}
		ret = data_len;
    }
#else
    ret = spi_sync(gf_dev->spi, &msg);
    if(ret == 0){
		//ret = msg.actual_length - 1;//GF_RDATA_OFFSET;
		ret = data_len;
    }
#endif
    gf_debug(SPI_DEBUG, "ret = %d, actual_length = %d\n", ret, msg.actual_length);
    kfree(xfer);
    if(xfer != NULL)
	xfer = NULL;
    return ret;
}

static int gf_spi_read_byte(struct gf_dev *gf_dev, u16 addr, u8 *value)
{
    int status = 0;
    mutex_lock(&gf_dev->buf_lock);

    status = gf_spi_read_bytes(gf_dev, addr, 1, gf_dev->buffer);
    *value = gf_dev->buffer[GF_RDATA_OFFSET];
    gf_debug(SPI_DEBUG, "value = 0x%x, buffer[3] = 0x%x\n", *value, gf_dev->buffer[3]);
    mutex_unlock(&gf_dev->buf_lock);
    return status;
}
static int gf_spi_write_byte(struct gf_dev *gf_dev, u16 addr, u8 value)
{
    int status = 0;
    mutex_lock(&gf_dev->buf_lock);
    gf_dev->buffer[GF_WDATA_OFFSET] = value;
    status = gf_spi_write_bytes(gf_dev, addr, 1, gf_dev->buffer);
    mutex_unlock(&gf_dev->buf_lock);
    return status;
}

/*-------------------------------------------------------------------------*/
/* Read-only message with current device setup */
static ssize_t gf_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct gf_dev *gf_dev = filp->private_data;
    ssize_t	status = 0;
    //long int t1, t2;

    FUNC_ENTRY();
    if ((count > bufsiz)||(count == 0)) {
		pr_warn("Max size for write buffer is %d. wanted length is %ld\n", bufsiz, count);
		FUNC_EXIT();
		return -EMSGSIZE;
    }

    gf_dev = filp->private_data;

    mutex_lock(&gf_dev->buf_lock);
	printk("[bernard]gf %ld\n", count);

    gf_dev->spi->max_speed_hz=1*1000*1000;
    //t1 = ktime_to_us(ktime_get());
    spi_setup(gf_dev->spi);
    status = gf_spi_read_bytes(gf_dev, GF_BUFFER_DATA, count, gf_dev->buffer);
    if(status > 0) {
		unsigned long missing = 0;
		missing = copy_to_user(buf, gf_dev->buffer + GF_RDATA_OFFSET, status);
		if(missing == status)
	    	status = -EFAULT;
		} else {
			printk("[bernard]gf Failed to read data from SPI device.\n");
		status = -EFAULT;
    }
		printk("[bernard]gf %s exit\n", __func__);

    // t2 = ktime_to_us(ktime_get());
    //pr_info("read time use: %ld\n", t2-t1);
    mutex_unlock(&gf_dev->buf_lock);

    return status;
}

/* Write-only message with current device setup */
static ssize_t gf_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos)
{
    struct gf_dev *gf_dev = filp->private_data;
    ssize_t			status = 0;
    FUNC_ENTRY();
    if(count > bufsiz) {
	pr_warn("Max size for write buffer is %d\n", bufsiz);
	return -EMSGSIZE;
    }

    mutex_lock(&gf_dev->buf_lock);
    status = copy_from_user(gf_dev->buffer + GF_WDATA_OFFSET, buf, count);
    if(status == 0) {
	gf_dev->spi->max_speed_hz=2*1000*1000;
	spi_setup(gf_dev->spi);
	status = gf_spi_write_bytes(gf_dev, GF_BUFFER_DATA, count, gf_dev->buffer);
    } else {
	pr_err("Failed to xfer data through SPI bus.\n");
	status = -EFAULT;
    }
    mutex_unlock(&gf_dev->buf_lock);
    FUNC_EXIT();
    return status;
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct gf_dev *gf_dev = NULL;
    struct gf_ioc_transfer *ioc = NULL;
    int			err = 0;
    u32			tmp = 0;
    int 		retval = 0;
	u8 *temp_buf;
    FUNC_ENTRY();
    if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
	return -ENOTTY;

    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
	err = !access_ok(VERIFY_WRITE,
		(void __user *)arg, _IOC_SIZE(cmd));
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
	err = !access_ok(VERIFY_READ,
		(void __user *)arg, _IOC_SIZE(cmd));
    if (err)
	return -EFAULT;

    gf_dev = (struct gf_dev *)filp->private_data;

    switch(cmd) {
	case GF_IOC_CMD:
	    ioc = kzalloc(sizeof(*ioc), GFP_KERNEL);
	    /*copy command data from user to kernel.*/
	    if(copy_from_user(ioc, (struct gf_ioc_transfer*)arg, sizeof(*ioc))){
		pr_err("Failed to copy command from user to kernel.\n");
		retval = -EFAULT;
		break;
	    }

	    if((ioc->len > bufsiz)||(ioc->len == 0)) {
		pr_warn("The request length[%d] is longer than supported maximum buffer length[%d].\n",
			ioc->len, bufsiz);
		retval = -EMSGSIZE;
		break;
	    }

	    mutex_lock(&gf_dev->buf_lock);
	    gf_dev->spi->max_speed_hz=1*1000*1000;
	    spi_setup(gf_dev->spi);
	    if(ioc->cmd == GF_R) {
#if PROCESSOR_64_BIT //begin lzd:for 64 bit processor
			/*if want to read data from hardware.*/
			temp_buf=(void __user*)(unsigned long)ioc->buf;
			pr_info("Read data from 0x%x, len = 0x%x buf = 0x%p\n", (int)ioc->addr, (int)ioc->len, (void __user*)((unsigned long)ioc->buf));
			gf_debug(DEFAULT_DEBUG,"(lzd##64bit-ioctl##)Read data from 0x%x, len = 0x%x buf = 0x%p\n",(int)ioc->addr, (int)ioc->len, (void __user*)((unsigned long)ioc->buf));
			gf_spi_read_bytes(gf_dev, ioc->addr, ioc->len, gf_dev->buffer);
			if(copy_to_user((void __user*)(unsigned long)ioc->buf, gf_dev->buffer + GF_RDATA_OFFSET, ioc->len)) {
		    	pr_err("Failed to copy data from kernel to user.\n");
		    	retval = -EFAULT;
		    	mutex_unlock(&gf_dev->buf_lock);
		    	break;
			}
			//print_16hex(ioc->buf, ioc->len);
		} else if (ioc->cmd == GF_W) {
			/*if want to read data from hardware.*/
			//print_16hex(ioc->buf, ioc->len);
			pr_info("Write data from 0x%x, len = 0x%x\n", ioc->addr, ioc->len);
			gf_debug(DEFAULT_DEBUG,"(lzd##64bit-ioctl##)Write data from 0x%x, len = 0x%x\n", ioc->addr, ioc->len);
			if(ioc->addr == 0x8043){
				temp_buf=(void __user*)(unsigned long)ioc->buf;
		    	gf_dev->mode = temp_buf[0];
		    	pr_info("set mode 0x%x \n", gf_dev->mode);
				gf_debug(DEFAULT_DEBUG,"(lzd##64bit-ioctl##)set mode 0x%x \n", gf_dev->mode);
			}

			if(copy_from_user(gf_dev->buffer + GF_WDATA_OFFSET, (void __user*)(unsigned long)ioc->buf, ioc->len)){
		    	pr_err("Failed to copy data from user to kernel.\n");
		    	retval = -EFAULT;
		    	mutex_unlock(&gf_dev->buf_lock);
		    	break;
			}
#else //end lzd:64 bit processor and begin lzd:32 bit processor
			pr_info("Read data from 0x%x, len = 0x%x buf = 0x%p\n", ioc->addr, ioc->len, ioc->buf);
			gf_debug(DEFAULT_DEBUG,"(lzd##32bit-ioctl##)Read data from 0x%x, len = 0x%x buf = 0x%p\n", ioc->addr, ioc->len, ioc->buf);
			gf_spi_read_bytes(gf_dev, ioc->addr, ioc->len, gf_dev->buffer);
			if(copy_to_user(ioc->buf, gf_dev->buffer + GF_RDATA_OFFSET, ioc->len)) {
		    	pr_err("Failed to copy data from kernel to user.\n");
		    	retval = -EFAULT;
		    	mutex_unlock(&gf_dev->buf_lock);
		    	break;
			}
			//print_16hex(ioc->buf, ioc->len);
	    } else if (ioc->cmd == GF_W) {
			/*if want to read data from hardware.*/
			//print_16hex(ioc->buf, ioc->len);
			pr_info("Write data from 0x%x, len = 0x%x\n", ioc->addr, ioc->len);

			if(ioc->addr == 0x8043){
		    	gf_dev->mode = ioc->buf[0];
		    	pr_info("set mode 0x%x \n", gf_dev->mode);
			}

			if(copy_from_user(gf_dev->buffer + GF_WDATA_OFFSET, ioc->buf, ioc->len)){
		    	pr_err("Failed to copy data from user to kernel.\n");
		    	retval = -EFAULT;
		    	mutex_unlock(&gf_dev->buf_lock);
		    	break;
			}
#endif //end lzd:32 bit processor
			gf_spi_write_bytes(gf_dev, ioc->addr, ioc->len, gf_dev->buffer);
	    } else {
			pr_warn("Error command for gf.\n");
	    }
	    if(ioc != NULL) {
			kfree(ioc);
			ioc = NULL;
	    }
	    mutex_unlock(&gf_dev->buf_lock);
	    break;
	case GF_IOC_REINIT:
	    disable_irq(gf_dev->spi->irq);
	    gf_hw_reset(gf_dev,50);
	    enable_irq(gf_dev->spi->irq);

	    gf_debug(FLOW_DEBUG,"wake-up gf\n");
	    break;
	case GF_IOC_SETSPEED:
	    retval = __get_user(tmp, (u32 __user*)arg);
	    if(tmp > 8*1000*1000) {
			pr_warn("The maximum SPI speed is 8MHz.\n");
			retval = -EMSGSIZE;
			break;
	    }
	    if(retval == 0) {
			//gf_dev->spi->max_speed_hz=tmp;
			//spi_setup(gf_dev->spi);
			//gf_debug(DEFAULT_DEBUG, "spi speed changed to %d\n", tmp);
	    }
	    break;
	default:
	    pr_warn("gf doesn't support this command(%d)\n", cmd);
	    break;
    }
    FUNC_EXIT();
    return retval;
}

static unsigned int gf_poll(struct file *filp, struct poll_table_struct *wait)
{
    struct gf_dev *gf_dev = filp->private_data;
    gf_spi_read_byte(gf_dev, GF_BUFFER_STATUS, &gf_dev->buf_status);
    if((gf_dev->buf_status & GF_BUF_STA_MASK) == GF_BUF_STA_READY) {
		return (POLLIN|POLLRDNORM);
    } else {
		gf_debug(DEFAULT_DEBUG, "Poll no data.\n");
    }
    return 0;
}

#if FW_UPDATE
static bool hw_config(struct gf_dev *gf_dev)
{
    mutex_lock(&gf_dev->buf_lock);
    memcpy(gf_dev->buffer + GF_WDATA_OFFSET, config_list[0].config[0].buffer, GF_CFG_LEN);
    gf_spi_write_bytes(gf_dev, GF_CFG_ADDR, GF_CFG_LEN, gf_dev->buffer);
    mutex_unlock(&gf_dev->buf_lock);

    return true;
}
static int isUpdate(struct gf_dev *gf_dev)
{
    unsigned char version[16];
    unsigned int ver_fw = 0;
    unsigned int ver_file = 0;
    unsigned char* fw = GF_FW;
    unsigned char fw_running = 0;
    const unsigned int OFFSET = 7;

    msleep(300);
    gf_spi_read_byte(gf_dev, 0x41e4, &fw_running);
    pr_info("%s: 0x41e4 = 0x%x\n", __func__, fw_running);
    if(fw_running == 0xbe) {
	/*firmware running*/
	ver_file = (int)(fw[12] & 0xF0) <<12;
	ver_file |= (int)(fw[12] & 0x0F)<<8;
	ver_file |= fw[13];	//get the fw version in the i file;

	/*In case we want to upgrade to a special firmware. Such as debug firmware.*/
	if(ver_file != 0x5a5a) {
	    mutex_lock(&gf_dev->buf_lock);
	    gf_spi_read_bytes(gf_dev,0x8000,16,gf_dev->buffer);
	    memcpy(version, gf_dev->buffer + GF_RDATA_OFFSET, 16);
	    mutex_unlock(&gf_dev->buf_lock);
	    if(memcmp(version, GF_PID, GF_PID_LEN)) {
			pr_info("version: 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n", version[0], version[1],
			version[2], version[3], version[4], version[5]);
			return 1;
	    }
	    if((version[OFFSET]>9) || ((version[OFFSET + 1])>9)) {
			pr_info("version: 8-0x%x; 9-0x%x\n", version[OFFSET], version[OFFSET + 1]);
			return 1;
	    }

	    //get the current fw version
	    ver_fw  = (unsigned int)version[OFFSET] << 16;
	    ver_fw |= (unsigned int)version[OFFSET + 1] << 8;
	    ver_fw |= (unsigned int)version[OFFSET + 2];
	    pr_info("ver_fw: 0x%06x; ver_file:0x%06x\n", ver_fw, ver_file);

	    if(ver_fw == ver_file){
			/*If the running firmware is or ahead of the file's firmware. No need to do upgrade.*/
			return 0;
	    }
	}
		pr_info("Current Ver: 0x%x, Upgrade to Ver: 0x%x\n", ver_fw, ver_file);
    }else {
	/*no firmware.*/
		pr_info("No running firmware. Value = 0x%x\n", fw_running);
    }
    return 1;
}

#endif
#if FW_UPDATE

static u8 is_9p_ready_ok(struct gf_dev *gf_dev)
{
    u8 tmpBuf[16] = {0};
    u8 *ptr =NULL;
    u16 time_out = 0;
    gf_spi_read_bytes(gf_dev, 0x4220, 4, tmpBuf);

    ptr = &tmpBuf[GF_RDATA_OFFSET];

    while(ptr[0] !=0x02 || ptr[1] !=0x08 || ptr[2] !=0x90 || ptr[3] !=0x00)
    {
		time_out++;
		if (time_out > 200){
	    	return 0;
		}

		gf_spi_read_bytes(gf_dev, 0x4220, 4, tmpBuf);
		ptr = &tmpBuf[GF_RDATA_OFFSET];
    }
    printk("%s , timeout = %d\n",__func__,  time_out);
    return 1;
}

static int gf_fw_update_init(struct gf_dev *gf_dev)
{
    u8 retry_cnt = 5;
    u8 value;

    while(retry_cnt--){
		//set spi miso input pull up
		gf_miso_pullup();

		//reset and delay 5ms
		//gpio_set_value(GF_RST_PIN, 0);
		//mdelay(5);
		//gpio_set_value(GF_RST_PIN, 1);
		gf_hw_reset(gf_dev,50);
		mdelay(1);

		//recover miso back spi
		gf_miso_backnal();
		gf_spi_setup(gf_dev, 1000*1000);
		if(!is_9p_ready_ok(gf_dev)){
	    	pr_err("check 9p ver fail \n");
	    	retry_cnt = 0xFF;
	    	break;
		}

		mdelay(10);
		gf_spi_write_byte(gf_dev, 0x5081, 0x00);

		gf_spi_write_byte(gf_dev, 0x4180, 0x0C);
		gf_spi_read_byte(gf_dev, 0x4180, &value);
		if (value == 0x0C){
	    	pr_info("########hold SS51 and DSP successfully!\n");
	    	break;
		}
    }

    if(retry_cnt == 0xFF) {
		pr_info("Faile to hold SS51 and DSP.\n");
		return 0;
    } else {
		pr_info("Hold retry_cnt=%d\n",retry_cnt);
		gf_spi_write_byte(gf_dev, 0x4010, 0);
		return 1;
    }
}
#endif
#if ESD_PROTECT
static void gf_timer_work(struct work_struct *work)
{
    unsigned char value[4];
#if FW_UPDATE
    unsigned char* p_fw = GF_FW;
#endif
    struct gf_dev *gf_dev;
    int ret = 0;
    u8 mode = 0xFF;
    FUNC_ENTRY();
    if(work == NULL){
		pr_info("[info] %s wrong work\n",__func__);
		return;
    }
    gf_dev = container_of(work, struct gf_dev, spi_work);

    if(gf_dev->mode == GF_FF_MODE)
		goto exit;

    ret = power_supply_is_system_supplied();
    //pr_info("BEN: power_supply ret = %d\n", ret);

    mutex_lock(&gf_dev->buf_lock);
    gf_dev->spi->max_speed_hz= 1000*1000;//SPI_SPEED_MIN;
    spi_setup(gf_dev->spi);
    mutex_unlock(&gf_dev->buf_lock);

    gf_spi_read_byte(gf_dev, 0x8040, &value[0]);
    gf_spi_read_byte(gf_dev, 0x8000, &value[1]);
    gf_spi_read_byte(gf_dev, 0x8043, &value[2]); //&& value[1] == 0x47 && value[2] == 0x56
    //pr_info("timer_work: value[0] = 0x%x, 1:0x%x, 2:0x%x\n", value[0], value[1], value[2]);
    if(value[0] == 0xC6 && value[1] == 0x47){
		gf_spi_write_byte(gf_dev, 0x8040, 0xAA);
		mdelay(1);
    }else{
		gf_spi_read_byte(gf_dev, 0x8040, &value[0]);
		gf_spi_read_byte(gf_dev, 0x8000, &value[1]);
		gf_spi_read_byte(gf_dev, 0x8043, &value[2]); //&& value[1] == 0x47 && value[2] == 0x56
		if(value[0] == 0xC6 && value[1] == 0x47){
	    	gf_spi_write_byte(gf_dev, 0x8040, 0xAA);
	    	mdelay(1);
		}else{
	    	pr_info("##Jason hardware works abnormal,do reset! 0x8040=0x%x 0x8000=0x%x 0x8046=0x%x\n"
				,value[0],value[1],value[2]);
	    	disable_irq(gf_dev->spi->irq);
	    	gf_hw_reset(gf_dev,50);

	    	gf_spi_read_byte(gf_dev, 0x41e4, &value[0]);
	    	gf_spi_read_byte(gf_dev, 0x8000, &value[1]);
	    	pr_info("[info] %s read 0x41e4 finish value = 0x%x, 0x8000=0x%x\n", __func__,value[0], value[1]);
#if FW_UPDATE
	    	if(value[0] != 0xbe) {
				gf_spi_read_byte(gf_dev, 0x41e4, &value[0]);
				if(value[0] != 0xbe) {
		    		/***********************************firmware update*********************************/
		    		pr_info("[info] %s firmware update start\n", __func__);
		    		del_timer_sync(&gf_dev->gf_timer);
		    		if(gf_fw_update_init(gf_dev)) {
					gf_fw_update(gf_dev, p_fw, FW_LENGTH);
					gf_hw_reset(gf_dev,50);
		    	}
		    	gf_dev->gf_timer.expires = jiffies + 2 * HZ;
		    	add_timer(&gf_dev->gf_timer);
			}
	    }
	    /***************************************update config********************************/
	    //pr_info("[info] %s write 0xaa \n", __func__);
	    ret = gf_spi_write_byte(gf_dev, 0x8040, 0xAA);
	    if(!ret)
			pr_info("[info] %s write 0x8040 fail\n", __func__);

	    if(!hw_config(gf_dev))
			pr_info("[info] %s write config fail\n", __func__);
#endif
	    enable_irq(gf_dev->spi->irq);
	}
    }
	/*if mode was changed by reset, we should set the mode  back to the primary mode*/
    gf_spi_read_byte(gf_dev, GF_MODE_STATUS,&mode);
    if(mode != gf_dev->mode) {
		pr_info("[info] %s set mode back\n", __func__);
		gf_spi_write_byte(gf_dev, GF_MODE_STATUS, gf_dev->mode);
		gf_spi_read_byte(gf_dev, GF_MODE_STATUS, &mode);
		pr_info("[info] %s mode444 = %d\n", __func__, mode);
    }

exit:
    mod_timer(&gf_dev->gf_timer, jiffies + 2*HZ);//??whether 2s is ok
    FUNC_EXIT();
}

static void gf_timer_func(unsigned long arg)
{
    struct gf_dev* gf_dev = (struct gf_dev*)arg;
    schedule_work(&gf_dev->spi_work);
}
#endif

static irqreturn_t gf_irq(int irq, void* handle)
{
    struct gf_dev *gf_dev = (struct gf_dev *)handle;
    u8 mode = 0x80;
    u8	status;

    gf_spi_read_byte(gf_dev, GF_BUFFER_STATUS, &status);
    gf_debug(DEFAULT_DEBUG, "IRQ status = 0x%x\n", status);
    if(!(status & GF_BUF_STA_MASK)) {
		gf_debug(DEFAULT_DEBUG, "Invalid IRQ = 0x%x\n", status);
		//pr_info("gfx1xm:Invalid IRQ = 0x%x\n", status);
		return IRQ_HANDLED;
    }
	if(!(status & (GF_IMAGE_MASK | GF_KEY_MASK))) {
		gf_spi_write_byte(gf_dev, GF_BUFFER_STATUS, (status & 0x7F));
		//pr_info("gfx1xm:Invalid IRQ = 0x%x\n", status);
		return IRQ_HANDLED;
	}
    gf_spi_read_byte(gf_dev, GF_MODE_STATUS, &mode);
    gf_debug(SUSPEND_DEBUG, "status = 0x%x, mode = %d\n", status, mode);
    pr_info("[info] %s irq happend,status = 0x%x, mode = %d\n", __func__, status, mode);

    switch(mode)
    {
	case GF_FF_MODE:
	    if((status & GF_HOME_KEY_MASK) && (status & GF_HOME_KEY_STA)){
			pr_info("gf: wake device.\n");
			//gf_spi_write_byte(gf_dev, GF_MODE_STATUS, 0x00);
			input_report_key(gf_dev->input, GF_FF_KEY, 1);
			input_sync(gf_dev->input);
			input_report_key(gf_dev->input, GF_FF_KEY, 0);
			input_sync(gf_dev->input);
	    } else {
			break;
	    }
	case GF_IMAGE_MODE:
#ifdef GF_FASYNC
	    if(gf_dev->async) {
			kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
	    }
#endif
	    break;
	case GF_KEY_MODE:
	    printk("gf:Key mode: status = 0x%x\n", status);
	    if  ((status & GF_KEY_MASK) && (status & GF_BUF_STA_MASK)) {
			if (status & GF_HOME_KEY_MASK) {
		    	input_report_key(gf_dev->input, GF_INPUT_HOME_KEY, (status & GF_HOME_KEY_STA)>>4);
		    	input_sync(gf_dev->input);
			}
			else if (status & GF_MENU_KEY_MASK){
		    	input_report_key(gf_dev->input, GF_INPUT_MENU_KEY, (status & GF_MENU_KEY_STA)>>2);
		    	input_sync(gf_dev->input);

			}else if (status & GF_BACK_KEY_MASK){
		    	input_report_key(gf_dev->input, GF_INPUT_BACK_KEY, (status & GF_BACK_KEY_STA));
		    	input_sync(gf_dev->input);
			}
	    }
	    //gf_spi_write_byte(gf_dev, GF_BUFFER_STATUS, (status & 0x7F));
#ifdef GF_FASYNC
				if(gf_dev->async) {
					kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
				}
#endif
	    break;
	case GF_SLEEP_MODE:
	    pr_warn("gf:Should not happen in sleep mode.\n");
	    break;
	case GF_DEBUG_MODE:
#ifdef GF_FASYNC
	    if(gf_dev->async) {
			kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
	    }
#endif
	    break;
	default:
	    pr_warn("gf:Unknown mode. mode = 0x%x\n", mode);
	    break;

    }

    return IRQ_HANDLED;
}

static int gf_open(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev;
    int			status = -ENXIO;

    FUNC_ENTRY();
    mutex_lock(&device_list_lock);

    list_for_each_entry(gf_dev, &device_list, device_entry) {
		if(gf_dev->devt == inode->i_rdev) {
	    	gf_debug(DEFAULT_DEBUG, "Found\n");
	    	status = 0;
	    	break;
		}
    }

    if(status == 0){
		mutex_lock(&gf_dev->buf_lock);
		if( gf_dev->buffer == NULL) {
	    	gf_dev->buffer = kzalloc(bufsiz + GF_RDATA_OFFSET, GFP_KERNEL);
	    	if(gf_dev->buffer == NULL) {
				dev_dbg(&gf_dev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
	    	}
		}
		mutex_unlock(&gf_dev->buf_lock);

		if(status == 0) {
	    	gf_dev->users++;
	    	filp->private_data = gf_dev;
	    	nonseekable_open(inode, filp);
	    	gf_debug(DEFAULT_DEBUG, "Succeed to open device. irq = %d\n", gf_dev->spi->irq);
	    	enable_irq(gf_dev->spi->irq);
		}
    } else {
		pr_err("No device for minor %d\n", iminor(inode));
    }
    mutex_unlock(&device_list_lock);
    FUNC_EXIT();
    return status;
}

#ifdef GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
    struct gf_dev *gf_dev = filp->private_data;
    int ret;

    FUNC_ENTRY();
    ret = fasync_helper(fd, filp, mode, &gf_dev->async);
    FUNC_EXIT();
    return ret;
}
#endif

static int gf_release(struct inode *inode, struct file *filp)
{
    struct gf_dev *gf_dev;
    int    status = 0;
    FUNC_ENTRY();
    mutex_lock(&device_list_lock);
    gf_dev = filp->private_data;
    filp->private_data = NULL;

    /*last close??*/
    gf_dev->users --;
    if(!gf_dev->users) {
		gf_debug(DEFAULT_DEBUG, "disble_irq. irq = %d\n", gf_dev->spi->irq);
		disable_irq(gf_dev->spi->irq);
    }
    mutex_unlock(&device_list_lock);
    FUNC_EXIT();
    return status;
}

static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}

static const struct file_operations gf_fops = {
    .owner =	THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    .write =	gf_write,
    .read =		gf_read,
    .unlocked_ioctl = gf_ioctl,
    .compat_ioctl	= gf_compat_ioctl,
    .open =		gf_open,
    .release =	gf_release,
    .poll   = gf_poll,
#ifdef GF_FASYNC
    .fasync = gf_fasync,
#endif
};

/*GPIO pins reference.*/
int gf_parse_dts(struct gf_dev* gf_dev)
{
    int rc = 0;

    /*get pwr resource*/
/*	gf_dev->pwr_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"goodix,gpio_pwr",0);
    if(!gpio_is_valid(gf_dev->pwr_gpio)) {
        dev_err(&gf_dev->spi->dev, "PWR GPIO is invalid.\n");
        return -1;
    }
    rc = gpio_request(gf_dev->pwr_gpio, "goodix_pwr");
    if(rc) {
        dev_err(&gf_dev->spi->dev, "Failed to request PWR GPIO. rc = %d\n", rc);
        return -1;
    }
*/

	/*get cs_gpio resource*/
/*    gf_dev->cs_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"goodix,gpio_cs",0);
    if(!gpio_is_valid(gf_dev->cs_gpio)) {
        dev_err(&gf_dev->spi->dev, "CS GPIO is invalid.\n");
        return -1;
    }
    rc = gpio_request(gf_dev->cs_gpio, "goodix_cs");
    if(rc) {
        dev_err(&gf_dev->spi->dev, "Failed to request CS GPIO. rc = %d\n", rc);
        return -1;
    }
    gpio_direction_output(gf_dev->cs_gpio, 1);
*/
    /*get reset resource*/
    gf_dev->reset_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"goodix,gpio_reset",0);
    if(!gpio_is_valid(gf_dev->reset_gpio)) {
        dev_err(&gf_dev->spi->dev, "RESET GPIO is invalid.\n");
        return -1;
    }
    rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
    if(rc) {
        dev_err(&gf_dev->spi->dev, "Failed to request RESET GPIO. rc = %d\n", rc);
        return -1;
    }
    gpio_direction_output(gf_dev->reset_gpio, 1);

    /*get irq resourece*/
    gf_dev->irq_gpio = of_get_named_gpio(gf_dev->spi->dev.of_node,"goodix,gpio_irq",0);
    if(!gpio_is_valid(gf_dev->irq_gpio)) {
        dev_err(&gf_dev->spi->dev, "IRQ GPIO is invalid.\n");
        return -1;
    }
    rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
    if(rc) {
        dev_err(&gf_dev->spi->dev, "Failed to request IRQ GPIO. rc = %d\n", rc);
        return -1;
    }
    gpio_direction_input(gf_dev->irq_gpio);

    return 0;
}

void gf_cleanup(struct gf_dev* gf_dev)
{
/*	if (gpio_is_valid(gf_dev->cs_gpio)){
        gpio_free(gf_dev->cs_gpio);
        pr_info("gfremove cs_gpio success\n");
    }*/

    if (gpio_is_valid(gf_dev->irq_gpio)){
        gpio_free(gf_dev->irq_gpio);
        pr_info("gfremove irq_gpio success\n");
    }
    if (gpio_is_valid(gf_dev->reset_gpio)){
        gpio_free(gf_dev->reset_gpio);
        pr_info("gf:remove reset_gpio success\n");
    }
/*
    if (gpio_is_valid(gf_dev->pwr_gpio)){
        gpio_free(gf_dev->pwr_gpio);
        pr_info("gf:remove pwr_gpio success\n");
    }
*/
}

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *gf_spi_class;
/*-------------------------------------------------------------------------*/
static int  gf_probe(struct spi_device *spi)
{
    struct gf_dev	*gf_dev;
	struct regulator *vreg;
    int	status = 0;
    int ret = 0;
    unsigned long minor;
    int err = 0;
    unsigned char version[16] = {0};
    FUNC_ENTRY();
    printk("zb test:start %s\n",__func__);
    /* Allocate driver data */
    gf_dev = kzalloc(sizeof(*gf_dev), GFP_KERNEL);
    if (!gf_dev){
		dev_err(&spi->dev,"Failed to alloc memory for gf device.\n");
		FUNC_EXIT();
		return -ENOMEM;
    }

    /* Initialize the driver data */
    gf_dev->spi = spi;
    spin_lock_init(&gf_dev->spi_lock);
    mutex_init(&gf_dev->buf_lock);
    INIT_LIST_HEAD(&gf_dev->device_entry);
	gf_dev->cs_gpio    = -EINVAL;
	gf_dev->irq_gpio   = -EINVAL;
	gf_dev->reset_gpio = -EINVAL;

	if (gf_parse_dts(gf_dev))
		goto err;

	vreg = regulator_get(&spi->dev,"vdd_ana");
	if (!vreg) {
		dev_err(&spi->dev, "gf:Unable to get vdd_ana\n");
		goto err;
	}
	if (regulator_count_voltages(vreg) > 0) {
		ret = regulator_set_voltage(vreg, 2800000,2800000);
			if (ret){
				dev_err(&spi->dev,"gf:Unable to set voltage on vdd_ana");
				goto err;
			}
	}
	ret = regulator_enable(vreg);
	if (ret) {
		dev_err(&spi->dev, "gf:error enabling vdd_ana %d\n",ret);
		regulator_put(vreg);
		vreg = NULL;
		goto err;
	}
	dev_dbg(&spi->dev,"gf:Set voltage on vdd_ana for goodix fingerprint");
    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&device_list_lock);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS) {
		struct device *dev;

		status = sysfs_create_group(&spi->dev.kobj,&gf_debug_attr_group);
		if(status){
	    	dev_err(&spi->dev,"gf:Failed to create sysfs file.\n");
	    	goto err;
		}

		gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf_spi_class, &spi->dev, gf_dev->devt,
			gf_dev, DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
    } else {
		dev_err(&spi->dev, "gf:no minor number available!\n");
		status = -ENODEV;
    }
    if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf_dev->device_entry, &device_list);
    }
    mutex_unlock(&device_list_lock);
    if (status == 0){
		gf_dev->buffer = kzalloc(bufsiz + GF_RDATA_OFFSET, GFP_KERNEL);
		if(gf_dev->buffer == NULL) {
	    	kfree(gf_dev);
	    	status = -ENOMEM;
	    	goto err;
		}
		spi_set_drvdata(spi, gf_dev);

		/*register device within input system.*/
		gf_dev->input = input_allocate_device();
		if(gf_dev->input == NULL) {
	    	dev_err(&spi->dev,"Failed to allocate input device.\n");
	    	status = -ENOMEM;
	    	kfree(gf_dev->buffer);
	    	kfree(gf_dev);
	    	goto err;
		}

		__set_bit(EV_KEY, gf_dev->input->evbit);
		__set_bit(GF_INPUT_HOME_KEY, gf_dev->input->keybit);
		__set_bit(GF_INPUT_MENU_KEY, gf_dev->input->keybit);
		__set_bit(GF_INPUT_BACK_KEY, gf_dev->input->keybit);
		__set_bit(GF_FF_KEY, gf_dev->input->keybit);

		gf_dev->input->name = "tiny4412-key";
		if(input_register_device(gf_dev->input)) {
	    	dev_err(&spi->dev,"Failed to register input device.\n");
		}

		/*setup gf configurations.*/
		dev_dbg(&spi->dev, "Setting gf device configuration.\n");
		/*SPI parameters.*/
		gf_spi_setup(gf_dev, 1000*1000);

		gf_irq_cfg(gf_dev);
		dev_info(&spi->dev, "gf interrupt NO. = %d\n", gf_dev->spi->irq);

#if FW_UPDATE
		if(isUpdate(gf_dev)) {
	    	unsigned char* fw = GF_FW;
	    	/*Do upgrade action.*/
	    	if(gf_fw_update_init(gf_dev)) {
				gf_fw_update(gf_dev, fw, FW_LENGTH);
				gf_hw_reset(gf_dev,50);
	    	}
		}
		/*write config*/
		if(!hw_config(gf_dev))
	    	dev_err(&spi->dev, "[info] %s write config fail\n", __func__);
#endif

#if 1
		err = request_threaded_irq(spi->irq, NULL, gf_irq,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			dev_name(&spi->dev), gf_dev);
#else
		err = request_irq(gf_dev->spi->irq, gf_irq,
			IRQ_TYPE_EDGE_RISING,//IRQ_TYPE_LEVEL_HIGH,
			dev_name(&gf_dev->spi->dev), gf_dev);
#endif
		if(!err) {
	    	disable_irq(gf_dev->spi->irq);
		}

		ret = gf_spi_read_bytes(gf_dev,0x8000,16,gf_dev->buffer);
		memcpy(version, gf_dev->buffer + GF_RDATA_OFFSET, 16);
		for(ret = 0; ret <16; ret++)
	    	dev_info(&spi->dev, "version[%d] = %x \n", ret,version[ret]);

#if ESD_PROTECT
		INIT_WORK(&gf_dev->spi_work, gf_timer_work);
		init_timer(&gf_dev->gf_timer);
		gf_dev->gf_timer.function = gf_timer_func;
		gf_dev->gf_timer.expires = jiffies + 3*HZ;
		gf_dev->gf_timer.data = gf_dev;
		add_timer(&gf_dev->gf_timer);
#endif // ESD_PROTECT

		dev_info(&spi->dev, "gf:%s:GF installed.\n", __func__);
    }else
	    goto err;
	return 0;

err:
    FUNC_EXIT();
	gf_cleanup(gf_dev);
	kfree(gf_dev);
    return status;
}

static int  gf_remove(struct spi_device *spi)
{
    struct gf_dev	*gf_dev = spi_get_drvdata(spi);
    FUNC_ENTRY();

    /* make sure ops on existing fds can abort cleanly */
    if(gf_dev->spi->irq) {
		free_irq(gf_dev->spi->irq, gf_dev);
    }

#if ESD_PROTECT
    del_timer_sync(&gf_dev->gf_timer);
    cancel_work_sync(&gf_dev->spi_work);
#endif

    spin_lock_irq(&gf_dev->spi_lock);
    gf_dev->spi = NULL;
    spi_set_drvdata(spi, NULL);
    spin_unlock_irq(&gf_dev->spi_lock);
/*
	if(gf_dev->spi_wq != NULL) {
		flush_workqueue(gf_dev->spi_wq);
		destroy_workqueue(gf_dev->spi_wq);
	}
*/
    /* prevent new opens */
    mutex_lock(&device_list_lock);
    sysfs_remove_group(&spi->dev.kobj, &gf_debug_attr_group);
    list_del(&gf_dev->device_entry);
    device_destroy(gf_spi_class, gf_dev->devt);
    clear_bit(MINOR(gf_dev->devt), minors);
    if (gf_dev->users == 0) {
		if(gf_dev->input != NULL)
	    input_unregister_device(gf_dev->input);

		if(gf_dev->buffer != NULL)
	    	kfree(gf_dev->buffer);
		kfree(gf_dev);
    }
    mutex_unlock(&device_list_lock);

    FUNC_EXIT();
    return 0;
}

static int gf_suspend_test(struct device *dev)
{
    printk(KERN_ERR"gf_suspend_test.\n");
    g_debug |= SUSPEND_DEBUG;
    return 0;
}

static int gf_resume_test(struct device *dev)
{
    printk(KERN_ERR"gf_resume_test.\n");
    g_debug &= ~SUSPEND_DEBUG;
    return 0;
}
static const struct dev_pm_ops gf_pm = {
    .suspend = gf_suspend_test,
    .resume = gf_resume_test
};

static struct of_device_id gf_of_match_table[] = {
	{
		.compatible = "goodix,fingerprint",
	},
	{},
};

static struct spi_driver gf_spi_driver = {
    .driver = {
		.name =		SPI_DEV_NAME,
		.owner =	THIS_MODULE,
		.pm = &gf_pm,
		.of_match_table = gf_of_match_table,
    },
    .probe =	gf_probe,
    .remove =	gf_remove,
    //.suspend = gf_suspend_test,
    //.resume = gf_resume_test,

    /* NOTE:  suspend/resume methods are not necessary here.
     * We don't do anything except pass the requests to/from
     * the underlying controller.  The refrigerator handles
     * most issues; the controller driver handles the rest.
     */
};

/*-------------------------------------------------------------------------*/

static int __init gf_init(void)
{
    int status;

    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
     */
     printk("zb test:enter %s\n",__func__);
    BUILD_BUG_ON(N_SPI_MINORS > 256);
    status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
    if (status < 0){
		pr_warn("Failed to register char device!\n");
		FUNC_EXIT();
		return status;
    }
    gf_spi_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(gf_spi_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
		pr_warn("Failed to create class.\n");
		FUNC_EXIT();
		return PTR_ERR(gf_spi_class);
    }
	printk("zb test:enter 1 %s\n",__func__);
    status = spi_register_driver(&gf_spi_driver);
	printk("zb test:enter 2 %s\n",__func__);
    if (status < 0) {
		class_destroy(gf_spi_class);
		unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
		pr_warn("Failed to register SPI driver.\n");
    }
    return status;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
    spi_unregister_driver(&gf_spi_driver);
    class_destroy(gf_spi_class);
    unregister_chrdev(SPIDEV_MAJOR, gf_spi_driver.driver.name);
}
module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf-spi");


