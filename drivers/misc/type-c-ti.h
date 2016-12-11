/*
 * tusb320.h  --  TI USB Type-C Configuration Channel Logic and Port Control driver
 *
 * Copyright 2015 XI'AN YEP TELECOM TECHNOLOGY CO.,LTD.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _TISUB_H_
#define _TIUSB_H_
#include <linux/wakelock.h>

#define TYPE_C_I2C_NAME	"usb-type-c-ti"
#define TI_I2C_DELAY_MS	100

#define CCD_DEFAULT		0x0
#define CCD_MEDIUM		0x1
#define CCD_HIGH		0x3
#define CCD_MASK		(0x30)	      /* charging current status */

#define MAX_CURRENT_BC1P2	500
#define MAX_CURRENT_MEDIUM     1500
#define MAX_CURRENT_HIGH       3000

#define TIUSB_1P8_VOL_MAX	1800000 /* uV */
#define TIUSB_2P95_VOL_MAX	2950000 /* uV */

#define TI_INTS_ATTACH_MASK	(0xC0)   /* current attach state interrupt */
#define TI_ATTACH_TO_DFP	0x2	 /* configured as UFP attaches to DFP */
#define TI_MODE_DFP_MASK	(0x20)   /* DFP mode */
#define TI_MODE_UFP_MASK	(0x10)   /* UFP mode */
#define TI_MODE_DRP_MASK	(0x30)   /* DRP mode */
#define TI_MODE_DEFAULT_MASK	(0x00)   /* default mode */
#define TI_MODE_MASK	(0xCF)   /* DFP mode */
#define TI_ATTACH_TO_UFP	0x1	 /* configured as DFP attaches to UFP */
#define TI_ATTACH_TO_NONE	0x0	 /* configured as DRP attaches None */

#define TI_CURRENT_MODE_DEFAULT_MASK    0x00 /* 500mA/900mA */
#define TI_CURRENT_MODE_MID_MASK    0x40 /* 1.5A*/
#define TI_CURRENT_MODE_MASK    0x3F /* Reg08 MASK */
enum {
    MANU_CURR_MODE_DFP= 0,
    MANU_CURR_MODE_UFP = 1,
    MANU_CURR_MODE_DRP = 2,
    MANU_CURR_MODE_DEFAULT = 3,
};
#define TI_STS_0_REG		0x0
#define TI_STS_8_REG		0x8
#define TI_STS_9_REG		0x9
#define TI_STS_A_REG		0xa
#define TI_INTS_STATUS		BIT(4)

struct ti_usb_type_c {
	struct i2c_client	*client;
	struct power_supply	*usb_psy;
	int			max_current;
	u8			attach_state;
	u8			dir_state;
	bool              info_notif_ready;
    struct notifier_block typec_set_notif;
    struct notifier_block typec_reboot_notif;
	bool			at_bootup;
	bool		       manu_set_DRP_mode;
	bool		       manu_set_DFP_mode;
	bool		       manu_set_UFP_mode;
	bool		       manu_set_mode;
	bool		       manu_set_work;
	bool		       irq_set_work;
	u8			manu_curr_mode;
	u8			status_10_reg;
	struct delayed_work tiusb_set_mode_dwork;
	u8			status_8_reg;
	u8			status_9_reg;
	int			enb_gpio;
	int			enb_gpio_polarity;
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
	struct delayed_work tiusb_read_regdata_dwork;
	struct regulator	*i2c_1p8;
	struct regulator	*i2c_2p95;
	spinlock_t irq_enabled_lock;
	struct wake_lock    ti_usb_wake_lock;
	struct mutex ti_usb_lock;
	bool typec_audio_mode;
};

struct tiusb_pinctrl_info {
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

extern struct ti_usb_type_c *ti_usb;

int tiusb_audio_switch_gpio_set(struct ti_usb_type_c *pdata, bool value);
void tiusb_audio_mode_set(bool mode);

#endif /* _TUSB320_H_ */

