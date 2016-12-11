#ifndef __GF_SPI_H
#define __GF_SPI_H

#include <linux/types.h>
#include <linux/err.h>
#include <linux/pinctrl/consumer.h>
/********************GF Mapping**********************/
#define GF_BASE             (0x8000)
#define GF_OFFSET(x)        (GF_BASE + x)

#define GF_VERSION	        GF_OFFSET(0)
#define GF_CONFIG_DATA      GF_OFFSET(0x40)
#define GF_CFG_ADDR	        GF_OFFSET(0x47)
#define GF_MODE_STATUS      GF_OFFSET(0x043)
//#define GF_MIXER_DATA     GF_OFFSET(0x140)
#define GF_BUFFER_STATUS	GF_OFFSET(0x140)
#define GF_KEY_DATA         GF_OFFSET(0x142)
#define GF_NOISE_DATA       GF_OFFSET(0x144)
#define GF_LONG_PRESS_STDP  GF_OFFSET(0x146)
#define GF_BUFFER_DATA      GF_OFFSET(0x141)

#define GF_BUF_STA_MASK     (0x1<<7)
#define	GF_BUF_STA_READY	(0x1<<7)
#define	GF_BUF_STA_BUSY     (0x0<<7)

#define	GF_IMAGE_MASK       (0x1<<6)
#define	GF_IMAGE_ENABLE     (0x1)
#define	GF_IMAGE_DISABLE	(0x0)

#define	GF_KEY_MASK	        (GF_HOME_KEY_MASK | \
							GF_MENU_KEY_MASK | \
                            GF_BACK_KEY_MASK )

//home key
#define	GF_HOME_KEY_MASK	(0x1<<5)
#define	GF_HOME_KEY_ENABL   (0x1)
#define	GF_HOME_KEY_DISABLE (0x0)
#define	GF_HOME_KEY_STA     (0x1<<4)

//menu key
#define	GF_MENU_KEY_MASK    (0x1<<3)
#define	GF_MENU_KEY_ENABLE	(0x1)
#define	GF_MENU_KEY_DISABLE	(0x0)
#define	GF_MENU_KEY_STA	(0x1<<2)

//back key
#define	GF_BACK_KEY_MASK    (0x1<<1)
#define	GF_BACK_KEY_ENABLE  (0x1)
#define	GF_BACK_KEY_DISABLE (0x0)
#define	GF_BACK_KEY_STA     (0x1<<0)

#define	GF_IMAGE_MODE       (0x00)
#define	GF_KEY_MODE	        (0x01)
#define GF_SLEEP_MODE       (0x02)
#define GF_FF_MODE		    (0x03)
#define GF_DEBUG_MODE       (0x56)

/**********************GF ops****************************/
#define GF_W                0xF0
#define GF_R                0xF1
#define GF_WDATA_OFFSET     (0x3)
#define GF_RDATA_OFFSET     (0x5)
#define GF_CFG_LEN          (249)   /*config data length*/

/**********************************************************/
//#undef GF_FASYNC
#define GF_FASYNC 		1//If support fasync mechanism.

#define PROCESSOR_64_BIT 1
#define PINCTRL_STATE_INTERRUPT "goodix_irq_active"

/*************************************************************/
struct gf_dev {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct input_dev        *input;

	struct workqueue_struct *spi_wq;
	struct work_struct     spi_work;
	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned		users;
	u8			*buffer;
	u8  		buf_status;
	u8 mode;
	u32			reset_gpio;
	u32			irq_gpio;
	u32 		cs_gpio;
//    struct pinctrl *gf_pinctrl;
 //   struct pinctrl_state *pinctrl_state_interrupt;
	struct timer_list   gf_timer;
#ifdef GF_FASYNC
	struct  fasync_struct *async;
#endif
};

/**********************IO Magic**********************/
#define  GF_IOC_MAGIC    'g'  //define magic number
struct gf_ioc_transfer {
	u8	cmd;
	u8 reserve;
	u16	addr;
	u32 len;
#if PROCESSOR_64_BIT
	u32  buf;
#else
	u8 *buf;
#endif
};
//define commands
/*read/write GF registers*/
#define  GF_IOC_CMD	_IOWR(GF_IOC_MAGIC, 1, struct gf_ioc_transfer)
#define  GF_IOC_REINIT	_IO(GF_IOC_MAGIC, 0)
#define  GF_IOC_SETSPEED	_IOW(GF_IOC_MAGIC, 2, u32)
//#define  GF_IOC_DISABLE    _IO(GF_IOC_MAGIC, 3)
//#define  GF_IOC_ENABLE     _IO(GF_IOC_MAGIC, 4)

#define  GF_IOC_MAXNR    3

/*******************Refering to hardware platform*****************************/
#if 0
/*Confure the IRQ pin for GF irq if necessary*/
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
#endif
inline static void gf_miso_pullup(void)
{
	/*Config MISO pin, referring to platform.*/

}

inline static void gf_miso_backnal(void)
{
	/*Config IRQ pin, referring to platform.*/

}

/********************************************************************
*CPU output low level in RST pin to reset GF. This is the MUST action for GF.
*Take care of this function. IO Pin driver strength / glitch and so on.
********************************************************************/
inline static void gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if(gf_dev == NULL) {
        pr_err("gf:%s:Input buff is NULL.\n",__func__);
  //      return NULL;
    }
    gpio_direction_output(gf_dev->reset_gpio, 1);
    gpio_set_value(gf_dev->reset_gpio, 0);
    mdelay(3);
    gpio_set_value(gf_dev->reset_gpio, 1);
    mdelay(delay_ms);
//    return NULL;
}

int gf_spi_read_bytes(struct gf_dev *gf_dev, u16 addr, u32 data_len, u8 *rx_buf);
int gf_spi_write_bytes(struct gf_dev *gf_dev, u16 addr, u32 data_len, u8 *tx_buf);
int gf_fw_update(struct gf_dev* gf_dev, unsigned char *buf, unsigned short len);

#endif
