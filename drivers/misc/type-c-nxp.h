/*
 * type-c-nxp.h  --  NXP USB Type-C Configuration Channel Logic and Port Control driver
 *
 * Copyright 2015 XI'AN YEP TELECOM TECHNOLOGY CO.,LTD.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __TUSB5150_H__
#define __TUSB5150_H__
#include <linux/kernel.h>
#include <linux/wakelock.h>

	 /*****************************************************************************
	  *  CallState
	  ****************************************************************************/
#define CALL_IDLE (0)
#define CALL_ACTIVE (1)

#define K_ERR	(1<<4)
#define K_INFO	(1<<1)

#define t_printk(level, fmt, args...) do { \
			 if (0xff & level) { \
				 pr_err("[ptn5150]" fmt, ## args); \
			 } \
		 } while (0)

#define NXP_TYPE_C_I2C_NAME	"ptn5150"

#define USB_1P8_VOL_MAX	1800000 /* uV */
#define USB_2P95_VOL_MAX	2950000 /* uV */
#define NXP_STS_ID_REG 0x01
#define NXP_STS_A_REG 0x02
#define NXP_STS_INT_REG 0x19
#define NXP_ATTACH_TO_NONE	0x0
#define NXP_ATTACH_TO_DFP	0x1
#define NXP_ATTACH_TO_UFP	0x2
#define NXP_MODE_UFP_MASK	(0x00)   /* UFP mode */
#define NXP_MODE_DFP_MASK	(0x02)   /* DFP mode */
#define NXP_MODE_DRP_MASK	(0x04)   /* DRP mode */
#define NXP_MODE_DEFAULT_MASK	(0x00)   /* default mode */
#define NXP_MODE_MASK	(0xF9)   /* DFP mode */
enum {
    NXP_MANU_CURR_MODE_DFP= 0,
    NXP_MANU_CURR_MODE_UFP = 1,
    NXP_MANU_CURR_MODE_DRP = 2,
    NXP_MANU_CURR_MODE_DEFAULT = 3,
};
struct ptn5150_usb_type_c {
	struct i2c_client	*client;
	struct power_supply	*usb_psy;
	u8			attach_state;
	u8			dir_state;
	u8			status_2_reg;
	u8			manu_curr_mode;
	bool        info_notif_ready;
    struct notifier_block typec_reboot_notif;
	bool		at_bootup;
	bool		irq_set_work;
	bool		manu_set_work;
	bool		manu_set_mode;
	bool		manu_set_DRP_mode;
	bool		manu_set_DFP_mode;
	bool		manu_set_UFP_mode;
	struct delayed_work nxpusb_set_mode_dwork;
    struct notifier_block typec_set_notif;
	int irq_gpio;
	int audio_swch_gpio;
	int hph_lr_swch_gpio;
	int vconn1_swch_gpio;
	int vconn2_swch_gpio;
	int board_id1_gpio;
	int board_id2_gpio;
	int board_id3_gpio;
	int uart_tx_gpio;
	int uart_rx_gpio;
	bool active_low;
	u8 status_4_reg;
	struct delayed_work  eint_work;
	struct delayed_work  trysnk_work1;
	struct delayed_work  trysnk_work2;
	struct regulator	*i2c_1p8;
	struct regulator	*i2c_2p95;
	spinlock_t irq_enabled_lock;
	struct wake_lock    ptn5150_wake_lock;
	struct mutex ptn5150_lock;
	bool typec_audio_mode;
};

struct ptn5150_usb_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *audio_switch_act;
	struct pinctrl_state *audio_switch_sus;
	struct pinctrl_state *vconn1_swch_act;
	struct pinctrl_state *vconn1_swch_sus;
	struct pinctrl_state *vconn2_swch_act;
	struct pinctrl_state *vconn2_swch_sus;
	struct pinctrl_state *board_id_act;
	struct pinctrl_state *board_id_sus;
	struct pinctrl_state *usb_irq_act;
	struct pinctrl_state *hph_lr_switch_act;
	struct pinctrl_state *hph_lr_switch_sus;
	struct pinctrl_state *uart_console_sus;
};

extern struct ptn5150_usb_type_c *ptn5150_usb;

int usb_audio_switch_gpio_set(struct ptn5150_usb_type_c *pdata, bool value);
void ptn5150_usb_audio_mode_set(bool mode);

#endif /* _TUSB320_H_ */

