/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/type-c_notify.h>
#include "type-c-ti.h"
#include "type-c-info.h"
#include "../../../sound/soc/codecs/wcd-mbhc-v2.h"
#ifdef CONFIG_GET_HARDWARE_INFO
#include <asm/hardware_info.h>
#endif
#include <linux/reboot.h>
#include <linux/usb.h>
static struct tiusb_pinctrl_info pinctrl_info;
struct ti_usb_type_c *ti_usb;
static bool disable_on_suspend;
static bool ti_otg_have_disconnected = false;
extern bool is_ti_type_c_register;
extern bool is_attached_ufp;
extern int usb_audio_digital_swap_analog;
extern void msm_otg_vbus_power_set(bool enable);

module_param(disable_on_suspend , bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(disable_on_suspend,
	"Whether to disable chip on suspend if state is not attached");

#ifdef CONFIG_TYPE_C_INFO
#define TIUSB_TYPE_C_DIR_MASK 0x20
static int typec_set_notifier(struct notifier_block *self,unsigned long action, void *data)
{
    int *mode;
    u8 curr_mode;
    struct type_c_event_data *evdata = data;

    if(ti_usb->at_bootup)//do not set DRP or other modes manually at bootup.
        return 0;

    if (evdata && evdata->data && action == TYPE_C_SET_MODE)
    {
        mode=evdata->data;
        pr_err("%s: mode = %d\n",__func__, *mode);

/*mode = 0: set DFP mode from UI.*/
/*mode = 1: set UFP mode from UI.*/
/*mode = 2: set DRP mode from UI.*/
/*mode = 3 or other value: set default(DRP) mode from UI.*/

    if (0 == (*mode))
        curr_mode = MANU_CURR_MODE_DFP;
        else if (1 == (*mode))
        curr_mode = MANU_CURR_MODE_UFP;
        else if (2 == (*mode))
        curr_mode = MANU_CURR_MODE_DRP;
    else
        curr_mode = MANU_CURR_MODE_DEFAULT;

    pr_err("%s: ti_usb->manu_curr_mode = %d\n",__func__, ti_usb->manu_curr_mode);

    /*so do nothing when as the same as last time.*/
    if(ti_usb->manu_curr_mode == curr_mode){
        pr_err("so do nothing as same as current mode,current_mode=%d\n",ti_usb->manu_curr_mode);
        return 0;
    }
    /*do nothing when any usb device is connected*/
    /*if (TI_ATTACH_TO_NONE != ti_usb->attach_state){
        pr_err("usb device is connected,do not set mode manually,ti_usb->attach_state=%d\n",ti_usb->attach_state);
        return 0;
    }*/

    ti_usb->manu_curr_mode = curr_mode;
    if (ti_usb->manu_curr_mode == MANU_CURR_MODE_DFP){
        ti_usb->manu_set_DFP_mode = true;
        ti_usb->manu_set_DRP_mode = false;
        ti_usb->manu_set_UFP_mode = false;
    }
    if (ti_usb->manu_curr_mode == MANU_CURR_MODE_UFP){
        ti_usb->manu_set_DFP_mode = false;
        ti_usb->manu_set_DRP_mode = false;
        ti_usb->manu_set_UFP_mode = true;
    }
    if (ti_usb->manu_curr_mode == MANU_CURR_MODE_DRP){
        ti_usb->manu_set_DRP_mode = true;
        ti_usb->manu_set_DFP_mode = false;
        ti_usb->manu_set_UFP_mode = false;
    }

    if (ti_usb->manu_curr_mode == MANU_CURR_MODE_DEFAULT){
        ti_usb->manu_set_DRP_mode = true;
        ti_usb->manu_set_DFP_mode = false;
        ti_usb->manu_set_UFP_mode = false;
    }
    ti_usb->manu_set_mode = true;
    schedule_delayed_work(&ti_usb->tiusb_set_mode_dwork, msecs_to_jiffies(10));
    }
    /*indicate that notifier is ready*/
    if (evdata && evdata->data && action == TYPE_C_INFO_NOTIFY_READY)
    {
        ti_usb->info_notif_ready = true;
    }
    return NOTIFY_OK;
}
#endif
static int typec_reboot_notifier(struct notifier_block *self,unsigned long code, void *data)
{
    int rc;
    if ((code == SYS_HALT) || (code == SYS_POWER_OFF)){
        ti_usb->status_10_reg = ((ti_usb->status_10_reg) & ( TI_MODE_MASK)) | (TI_MODE_UFP_MASK);
        dev_err(&ti_usb->client->dev, "set UFP mode during poweroff process\n");
        rc = i2c_smbus_write_byte_data(ti_usb->client, TI_STS_A_REG, ti_usb->status_10_reg);
        if (rc < 0)
            dev_err(&ti_usb->client->dev, "set UFP mode failed during poweroff process\n");
    }
    mdelay(900);
    return NOTIFY_DONE;
}

static int tiusb_read_regdata(struct i2c_client *i2c)
{
	int rc;
	uint16_t saddr = i2c->addr;
	u8 attach_state, mask = TI_INTS_ATTACH_MASK;

	rc = i2c_smbus_read_byte_data(i2c, TI_STS_8_REG);
	if (rc < 0)
		return -EIO;
	ti_usb->status_8_reg = rc;

	rc = i2c_smbus_read_byte_data(i2c, TI_STS_9_REG);
	if (rc < 0)
		return -EIO;
	ti_usb->status_9_reg = rc;

	/* Clear interrupt */
	rc = i2c_smbus_write_byte_data(i2c, TI_STS_9_REG, ti_usb->status_9_reg);
	if (rc < 0)
		return -EIO;

	dev_err(&i2c->dev, "i2c read from 0x%x-[%x %x]\n", saddr,
				ti_usb->status_8_reg, ti_usb->status_9_reg);
	if (!(ti_usb->status_9_reg & TI_INTS_STATUS)) {
		dev_err(&i2c->dev, "intr_status is 0!, ignore interrupt\n");
		ti_usb->attach_state = false;
		return -EINVAL;
	}

	attach_state = ti_usb->status_9_reg & mask;
	ti_usb->attach_state = attach_state >> find_first_bit((void *)&mask, 8);

	return rc;
}

static int tiusb_reset(struct i2c_client *i2c)
{
	int rc;

	rc = i2c_smbus_write_byte_data(i2c, TI_STS_A_REG, 0x08);
	if (rc < 0)
		return -EIO;
	return 0;
}

static int tiusb_update_power_supply(struct power_supply *psy, int limit)
{
	const union power_supply_propval ret = {limit,};

	/* Update USB of max charging current (500 corresponds to bc1.2 */
	if (psy->set_property)
		return psy->set_property(psy,
			POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, &ret);

	return -ENODEV;
}

static void tiusb_update_max_current(struct ti_usb_type_c *ti_usb)
{
	u8 mask = CCD_MASK;
	u8 shift = find_first_bit((void *)&mask, 8);
	u8 chg_mode = ti_usb->status_8_reg & mask;

	chg_mode >>= shift;

	/* update to 0 if type-c UFP detached */
	if (ti_usb->attach_state != TI_ATTACH_TO_DFP) {
		dev_dbg(&ti_usb->client->dev, "attach_state: %x\n",
						ti_usb->attach_state);
		ti_usb->max_current = 0;
		return;
	}

	switch (chg_mode) {
	case CCD_DEFAULT:
		ti_usb->max_current = MAX_CURRENT_BC1P2;
		break;
	case CCD_MEDIUM:
		ti_usb->max_current = MAX_CURRENT_MEDIUM;
		break;
	case CCD_HIGH:
		ti_usb->max_current = MAX_CURRENT_HIGH;
		break;
	default:
		dev_dbg(&ti_usb->client->dev, "wrong chg mode %x\n", chg_mode);
		ti_usb->max_current = 500;
	}

	dev_dbg(&ti_usb->client->dev, "chg mode: %x, mA:%u, attach: %x\n",
			chg_mode, ti_usb->max_current, ti_usb->attach_state);
}
#ifdef CONFIG_TYPE_C_INFO
static void tiusb_manu_set_mode_work(struct work_struct *work)
{
    u8 mode;
    int rc;
    struct type_c_event_data notif_data;

    dev_err(&ti_usb->client->dev, "%s:begin\n",__func__);

    if (ti_usb->irq_set_work){
        dev_err(&ti_usb->client->dev, "irq set work is busy,waiting for its finishing\n");
        schedule_delayed_work(&ti_usb->tiusb_set_mode_dwork, msecs_to_jiffies(500));
    }
    ti_usb->manu_set_work = true;

    if(!(ti_usb->manu_set_DFP_mode) && !(ti_usb->manu_set_UFP_mode) && !(ti_usb->manu_set_DRP_mode))//return if receive nothing from UI.
    {
        dev_err(&ti_usb->client->dev, "manual set mode is not received from UI,so do nothing\n");
           goto exit;
    }

    rc = i2c_smbus_read_byte_data(ti_usb->client, TI_STS_A_REG);
    if (rc < 0)
        goto exit;
    ti_usb->status_10_reg = rc;

    /*case:triger from irq*/
    if (false == ti_usb->manu_set_mode){

        /*case:disconnetc event*/
        if (TI_ATTACH_TO_NONE == ti_usb->attach_state){
            ti_usb->manu_set_DFP_mode = false;
            ti_usb->manu_set_UFP_mode = false;
            ti_usb->manu_set_DRP_mode = false;

            ti_usb->status_10_reg = ((ti_usb->status_10_reg) & ( TI_MODE_MASK)) | (TI_MODE_DEFAULT_MASK);
            dev_err(&ti_usb->client->dev, "ti_usb->status_10_reg = 0x%x\n",ti_usb->status_10_reg);
            dev_err(&ti_usb->client->dev, "device is disconnected from otg,so set type-c as default mode\n");
            rc = i2c_smbus_write_byte_data(ti_usb->client, TI_STS_A_REG, ti_usb->status_10_reg);
            if (rc < 0)
                goto exit;

            if(ti_usb->info_notif_ready){
                mode = MANU_CURR_MODE_DEFAULT;
                notif_data.data = &mode;
                type_c_get_notifier_call_chain(TYPE_C_INFO_MODE_RESULT,&notif_data);
            }
        }
        /*case:connect to an  DFP*/
        else if (TI_ATTACH_TO_DFP == ti_usb->attach_state){
            ti_usb->manu_set_DFP_mode = false;
            ti_usb->manu_set_UFP_mode = false;
            ti_usb->manu_set_DRP_mode = false;

            ti_usb->status_10_reg = ((ti_usb->status_10_reg) & ( TI_MODE_MASK)) | (TI_MODE_DEFAULT_MASK);
            dev_err(&ti_usb->client->dev, "ti_usb->status_10_reg = 0x%x\n",ti_usb->status_10_reg);
            dev_err(&ti_usb->client->dev, "connecting to an UFP device,so set type-c as default mode immediately\n");
            rc = i2c_smbus_write_byte_data(ti_usb->client, TI_STS_A_REG, ti_usb->status_10_reg);
            if (rc < 0)
                goto exit;

            if(ti_usb->info_notif_ready){
                mode = MANU_CURR_MODE_DEFAULT;
                notif_data.data = &mode;
                type_c_get_notifier_call_chain(TYPE_C_INFO_MODE_RESULT,&notif_data);
            }
        }

    }
    /*case:set mode received from UI when no device is connected.*/
    if (true == ti_usb->manu_set_mode){

        if(true == ti_usb->manu_set_DFP_mode){
            ti_usb->status_10_reg = ((ti_usb->status_10_reg) & ( TI_MODE_MASK)) | (TI_MODE_DFP_MASK);
            if(ti_usb->info_notif_ready){
                mode = MANU_CURR_MODE_DFP;
                notif_data.data = &mode;
                type_c_get_notifier_call_chain(TYPE_C_INFO_MODE_RESULT,&notif_data);
            }
        }

        if(true == ti_usb->manu_set_UFP_mode){
            ti_usb->status_10_reg = ((ti_usb->status_10_reg) & ( TI_MODE_MASK)) | (TI_MODE_UFP_MASK);
            if(ti_usb->info_notif_ready){
                mode = MANU_CURR_MODE_UFP;
                notif_data.data = &mode;
                type_c_get_notifier_call_chain(TYPE_C_INFO_MODE_RESULT,&notif_data);
            }
        }

        if(true == ti_usb->manu_set_DRP_mode){
            ti_usb->status_10_reg = ((ti_usb->status_10_reg) & ( TI_MODE_MASK)) | (TI_MODE_DEFAULT_MASK);
            if(ti_usb->info_notif_ready){
                mode = MANU_CURR_MODE_DRP;
                notif_data.data = &mode;
                type_c_get_notifier_call_chain(TYPE_C_INFO_MODE_RESULT,&notif_data);
            }
        }
        dev_err(&ti_usb->client->dev, "set:mode = %d ti_usb->status_10_reg = %d\n", mode,ti_usb->status_10_reg);
        rc = i2c_smbus_write_byte_data(ti_usb->client, TI_STS_A_REG, ti_usb->status_10_reg);
        if (rc < 0)
            goto exit;
    }
exit:

    ti_usb->manu_set_work = false;
}
#endif
static void tiusb_read_regdata_work(struct work_struct *work)
{
	int ret;
    #ifdef CONFIG_TYPE_C_INFO
    u8 dir_state;
    struct type_c_event_data notif_data;
    #endif
    u8 otg_power_on;
    #ifdef CONFIG_TYPE_C_INFO
    if (ti_usb->manu_set_work){
        dev_err(&ti_usb->client->dev, "manu set work is busy,waiting for its finishing\n");
        schedule_delayed_work(&ti_usb->tiusb_read_regdata_dwork, msecs_to_jiffies(500));
    }
    ti_usb->irq_set_work = true;
    #endif
    tiusb_read_regdata(ti_usb->client);

	tiusb_update_max_current(ti_usb);

	ret = tiusb_update_power_supply(ti_usb->usb_psy, ti_usb->max_current);
	if (ret < 0)
		dev_err(&ti_usb->client->dev, "failed to notify USB-%d\n", ret);
	if (0x40 == (ti_usb->status_9_reg & 0xc0)) {
		is_attached_ufp = true;
	}
	if ((0x8 == (ti_usb->status_8_reg & 0x0e))
		|| ((0x0 == (ti_usb->status_8_reg & 0x0e)) && (is_type_c_headset_inserted))) {
		if (0x8 == (ti_usb->status_8_reg & 0x0e)) {
			tiusb_audio_switch_gpio_set(ti_usb, true);
			is_type_c_headset_inserted = true;
		} else {
			tiusb_audio_switch_gpio_set(ti_usb, false);
			is_type_c_headset_inserted = false;
		}
		wcd_mbhc_mech_plug_detect();
	}
    #ifdef CONFIG_TYPE_C_INFO
    if(ti_usb->info_notif_ready){
        dir_state = ti_usb->status_9_reg & TIUSB_TYPE_C_DIR_MASK;
        dir_state=!!dir_state;
        ti_usb->dir_state = dir_state;
        notif_data.data=&dir_state;
        type_c_get_notifier_call_chain(TYPE_C_GET_ORIENTATION,&notif_data);
    }
    #endif

    #ifdef CONFIG_TYPE_C_INFO
    ti_usb->irq_set_work = false;
    if(ti_usb->manu_set_DRP_mode || ti_usb->manu_set_DFP_mode || ti_usb->manu_set_UFP_mode)
        schedule_delayed_work(&ti_usb->tiusb_set_mode_dwork, msecs_to_jiffies(300));
    #endif
    if(ti_usb->info_notif_ready){
        if(TI_ATTACH_TO_UFP == ti_usb->attach_state){
            if(ti_usb->status_8_reg & TI_CURRENT_MODE_MID_MASK){
                otg_power_on = 1;
                ti_usb->status_8_reg = ((ti_usb->status_8_reg) & ( TI_CURRENT_MODE_MASK)) | (TI_CURRENT_MODE_DEFAULT_MASK);
                ret = i2c_smbus_write_byte_data(ti_usb->client, TI_STS_8_REG, ti_usb->status_8_reg);
                if (ret < 0)
                    dev_err(&ti_usb->client->dev, "set current mode in reg8 failed\n");
                notif_data.data=&otg_power_on;
                type_c_otg_power_notifier_call_chain(TYPE_C_SET_OTG_POWER,&notif_data);
            }
        }
        if(TI_ATTACH_TO_NONE == ti_usb->attach_state){
            if(!(ti_usb->status_8_reg & TI_CURRENT_MODE_MID_MASK)){
                otg_power_on = 0;
                ti_usb->status_8_reg = ((ti_usb->status_8_reg) & ( TI_CURRENT_MODE_MASK)) | (TI_CURRENT_MODE_MID_MASK);
                ret = i2c_smbus_write_byte_data(ti_usb->client, TI_STS_8_REG, ti_usb->status_8_reg);
                if (ret < 0)
                    dev_err(&ti_usb->client->dev, "set current mode in reg8 failed\n");
                notif_data.data=&otg_power_on;
                type_c_otg_power_notifier_call_chain(TYPE_C_SET_OTG_POWER,&notif_data);
            }
        }
    }
    wake_unlock(&ti_usb->ti_usb_wake_lock);
}

static irqreturn_t tiusb_irq(int irq, void *data)
{
	struct ti_usb_type_c *ti_usb = (struct ti_usb_type_c *)data;
	unsigned long flags;

	dev_err(&ti_usb->client->dev, "%s\n", __func__);

	wake_lock(&ti_usb->ti_usb_wake_lock);
	spin_lock_irqsave(&ti_usb->irq_enabled_lock, flags);
	ti_usb->manu_set_mode = false;
	if (is_attached_ufp) {
		is_attached_ufp = false;
		usb_audio_digital_swap_analog = 0;
		ti_otg_have_disconnected = true;
	} else {
		ti_otg_have_disconnected = false;
	}
	schedule_delayed_work(&ti_usb->tiusb_read_regdata_dwork, msecs_to_jiffies(200));
	spin_unlock_irqrestore(&ti_usb->irq_enabled_lock, flags);

	return IRQ_HANDLED;
}

static ssize_t tiusb_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int count = 0;
	int i;
	int value;

	for (i = TI_STS_0_REG; i <= TI_STS_A_REG; i++) {
		value = i2c_smbus_read_byte_data(ti_usb->client, i);
		if (value < 0)
			count += sprintf(buf + count, "%02x: XXXX\n",
				i);
		else
			count += sprintf(buf + count, "%02x: %04x\n", i,
				value);

		if (count >= PAGE_SIZE - 1)
			break;
	}

	return count;
}

static ssize_t tiusb_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int val = 0, addr = 0;
	int i;

	for (i = 0; i < count; i++) {
		if (*(buf + i) <= '9' && *(buf + i) >= '0')
			addr = (addr << 4) | (*(buf + i)-'0');
		else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
			addr = (addr << 4) | ((*(buf + i) - 'a') + 0xa);
		else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
			addr = (addr << 4) | ((*(buf + i)-'A') + 0xa);
		else
			break;
	}

	for (i = i + 1 ; i < count; i++) {
		if (*(buf + i) <= '9' && *(buf + i) >= '0')
			val = (val << 4) | (*(buf + i) - '0');
		else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
			val = (val << 4) | ((*(buf + i) - 'a') + 0xa);
		else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
			val = (val << 4) | ((*(buf + i) - 'A') + 0xa);
		else
			break;
	}

	if (addr > TI_STS_A_REG || val > 0xffff || val < 0)
		return count;

	if (i == count)
		pr_info("0x%02x = 0x%04x\n", addr,
			i2c_smbus_read_byte_data(ti_usb->client, addr));
	else
		i2c_smbus_write_byte_data(ti_usb->client, addr, val);

	return count;
}
static DEVICE_ATTR(tiusb_reg, 0666, tiusb_reg_show, tiusb_reg_store);

static void tiusb_init(int irq, void *data)
{
	int ret;
	struct ti_usb_type_c *ti_usb = (struct ti_usb_type_c *)data;

	tiusb_read_regdata(ti_usb->client);

	tiusb_update_max_current(ti_usb);

	ret = tiusb_update_power_supply(ti_usb->usb_psy, ti_usb->max_current);
	if (ret < 0)
		dev_err(&ti_usb->client->dev, "failed to notify USB-%d\n", ret);
        if(!(ti_usb->status_8_reg & TI_CURRENT_MODE_MID_MASK)){
            ti_usb->status_8_reg = ((ti_usb->status_8_reg) & ( TI_CURRENT_MODE_MASK)) | (TI_CURRENT_MODE_MID_MASK);
            ret = i2c_smbus_write_byte_data(ti_usb->client, TI_STS_8_REG, ti_usb->status_8_reg);
            if (ret < 0)
                dev_err(&ti_usb->client->dev, "set current mode in reg8 failed\n");
        }
}

/*
static int tiusb_gpio_config(struct ti_usb_type_c *ti, bool enable)
{
	int ret = 0;

	if (!enable) {
		gpio_set_value(ti_usb->enb_gpio, !ti_usb->enb_gpio_polarity);
		return 0;
	}

	ret = devm_gpio_request(&ti->client->dev, ti->enb_gpio,
					"ti_typec_enb_gpio");
	if (ret) {
		pr_err("unable to request gpio [%d]\n", ti->enb_gpio);
		return ret;
	}

	ret = gpio_direction_output(ti->enb_gpio, ti->enb_gpio_polarity);
	if (ret) {
		dev_err(&ti->client->dev, "set dir[%d] failed for gpio[%d]\n",
			ti->enb_gpio_polarity, ti->enb_gpio);
		return ret;
	}
	dev_dbg(&ti->client->dev, "set dir[%d] for gpio[%d]\n",
			ti->enb_gpio_polarity, ti->enb_gpio);

	gpio_set_value(ti->enb_gpio, ti->enb_gpio_polarity);
	msleep(TI_I2C_DELAY_MS);

	return ret;
}
*/

static int tiusb_ldo_init(struct ti_usb_type_c *ti, bool init)
{
	int rc = 0;

	if (!init) {
		regulator_set_voltage(ti->i2c_1p8, 0, TIUSB_1P8_VOL_MAX);
		rc = regulator_disable(ti->i2c_1p8);
		return rc;
	}

	ti->i2c_1p8 = devm_regulator_get(&ti->client->dev, "vdd_io");
	if (IS_ERR(ti->i2c_1p8)) {
		rc = PTR_ERR(ti->i2c_1p8);
		dev_err(&ti->client->dev, "unable to get 1p8(%d)\n", rc);
		return rc;
	}
	rc = regulator_set_voltage(ti->i2c_1p8, TIUSB_1P8_VOL_MAX,
					TIUSB_1P8_VOL_MAX);
	if (rc) {
		dev_err(&ti->client->dev, "unable to set voltage(%d)\n", rc);
		goto put_1p8;
	}

	rc = regulator_enable(ti->i2c_1p8);
	if (rc) {
		dev_err(&ti->client->dev, "unable to enable 1p8-reg(%d)\n", rc);
		return rc;
	}

	return 0;

put_1p8:
	regulator_set_voltage(ti->i2c_1p8, 0, TIUSB_1P8_VOL_MAX);
	return rc;
}

static int was4732d_ldo_init(struct ti_usb_type_c *ti, bool init)
{
	int rc = 0;

	if (!init) {
		regulator_set_voltage(ti->i2c_2p95, 0, TIUSB_2P95_VOL_MAX);
		rc = regulator_disable(ti->i2c_2p95);
		return rc;
	}

	ti->i2c_2p95 = devm_regulator_get(&ti->client->dev, "vdd");
	if (IS_ERR(ti->i2c_2p95)) {
		rc = PTR_ERR(ti->i2c_2p95);
		dev_err(&ti->client->dev, "unable to get 2p95(%d)\n", rc);
		return rc;
	}
	rc = regulator_set_voltage(ti->i2c_2p95, TIUSB_2P95_VOL_MAX,
					TIUSB_2P95_VOL_MAX);
	if (rc) {
		dev_err(&ti->client->dev, "unable to set voltage(%d)\n", rc);
		goto put_2p95;
	}

	rc = regulator_enable(ti->i2c_2p95);
	if (rc) {
		dev_err(&ti->client->dev, "unable to enable 2p95-reg(%d)\n", rc);
		return rc;
	}

	return 0;

put_2p95:
	regulator_set_voltage(ti->i2c_2p95, 0, TIUSB_2P95_VOL_MAX);
	return rc;
}

static int tiusb_parse_dt(struct device *dev, struct ti_usb_type_c *pdata)
{
	struct device_node *np = dev->of_node;

        pdata->irq_gpio = of_get_named_gpio(np, "ti,irq-gpio", 0);
        if ((!gpio_is_valid(pdata->irq_gpio)))
            return -EINVAL;

        return 0;

}

int tiusb_audio_switch_gpio_set(struct ti_usb_type_c *pdata, bool value)
{
	pr_info("%s\n", __func__);

	if (!gpio_is_valid(pdata->audio_swch_gpio)) {
		pr_err("%s: Invalid gpio: %d", __func__, pdata->audio_swch_gpio);
		return false;
	}
	gpio_direction_output(pdata->audio_swch_gpio, value);
	pr_info("%s: audio switch gpio sets to %d\n", __func__, value);

	if (!gpio_is_valid(pdata->hph_lr_swch_gpio)) {
		pr_err("%s: Invalid gpio: %d", __func__, pdata->hph_lr_swch_gpio);
		return false;
	}
	gpio_direction_output(pdata->hph_lr_swch_gpio, value);
	pr_info("%s: hph lr switch gpio sets to %d\n", __func__, value);
	return 0;
}

void tiusb_audio_mode_set(bool mode)
{
	struct ti_usb_type_c *pdata = ti_usb;

	pr_info("%s\n", __func__);

	if (mode && !letv_audio_mode_supported(NULL)) {
		pr_info("%s:VR, will not change to analogic mode\n", __func__);
		return;
	}

	/*true for analog mode and false for digital mode*/
	if (!gpio_is_valid(pdata->audio_swch_gpio)) {
		pr_err("%s: Invalid gpio: %d", __func__, pdata->audio_swch_gpio);
		return;
	}
	if (!gpio_is_valid(pdata->hph_lr_swch_gpio)) {
		pr_err("%s: Invalid gpio: %d", __func__, pdata->hph_lr_swch_gpio);
		return;
	}
	if (!gpio_is_valid(pdata->vconn1_swch_gpio)) {
		pr_err("%s: Invalid gpio: %d", __func__,
				pdata->vconn1_swch_gpio);
		return;
	}
	if (!gpio_is_valid(pdata->vconn2_swch_gpio)) {
		pr_err("%s: Invalid gpio: %d", __func__,
				pdata->vconn2_swch_gpio);
		return;
	}
	mutex_lock(&ti_usb->ti_usb_lock);
	cclogic_set_audio_mode(mode);
	if (mode) {
		usb_audio_digital_swap_analog = 1;
		gpio_direction_output(pdata->audio_swch_gpio, 1);
		pr_info("%s: audio switch gpio sets to high\n", __func__);
		gpio_direction_output(pdata->hph_lr_swch_gpio, 1);
		pr_info("%s: hph lr switch gpio sets to high\n", __func__);
		if (0x20 == (ti_usb->status_9_reg & 0x20)) {
			pr_info("%s: use CC1 to switch to analog mode\n", __func__);
			gpio_direction_output(pdata->vconn1_swch_gpio, !pdata->active_low);
			pr_info("%s: vconn1 switch gpio set to %d\n", __func__, !pdata->active_low);
		} else {
			pr_info("%s: use CC2 to switch to analog mode\n", __func__);
			gpio_direction_output(pdata->vconn2_swch_gpio, !pdata->active_low);
			pr_info("%s: vconn2 switch gpio set to %d\n", __func__, !pdata->active_low);
		}
		is_type_c_headset_inserted = true;
	} else {
		usb_audio_digital_swap_analog = 2;
		gpio_direction_output(pdata->audio_swch_gpio, 0);
		pr_info("%s: audio switch gpio sets to low\n", __func__);
		gpio_direction_output(pdata->hph_lr_swch_gpio, 0);
		pr_info("%s: hph lr switch gpio sets to low\n", __func__);
		gpio_direction_output(pdata->vconn1_swch_gpio, pdata->active_low);
		gpio_direction_output(pdata->vconn2_swch_gpio, pdata->active_low);
		pr_info("%s: vconn1 switch gpio set to %d\n", __func__, pdata->active_low);
		pr_info("%s: vconn2 switch gpio set to %d\n", __func__, pdata->active_low);
		if (is_type_c_headset_inserted) {
		    if (ti_otg_have_disconnected) {
		        pr_info("%s: ******************avoid crash\n", __func__);
		        ti_otg_have_disconnected = false;
		    } else {
		        pr_info("%s: ******************normal exe\n", __func__);
		        msm_otg_vbus_power_set(false);
		        msleep(100);
		        msm_otg_vbus_power_set(true);
		    }
		}
		is_type_c_headset_inserted = false;
	}
	wcd_mbhc_mech_plug_detect();
	mutex_unlock(&ti_usb->ti_usb_lock);
	return;
}

static int tiusb_ext_gpios_init(struct device *dev, struct ti_usb_type_c *pdata)
{
	int ret;

	pr_info("%s\n", __func__);

	pdata->audio_swch_gpio = of_get_named_gpio(dev->of_node,
						"qcom,audio-swch-gpios", 0);
	if (!gpio_is_valid(pdata->audio_swch_gpio)) {
		pr_err("%s: Invalid gpio: %d", __func__,
				pdata->audio_swch_gpio);
		return -EINVAL;
	}

	ret = gpio_request(pdata->audio_swch_gpio,
					   "audio_swch_gpio");
	if (ret) {
		pr_err("failed to request audio switch gpio\n");
		return ret;
	}

	ret = pinctrl_select_state(pinctrl_info.pinctrl,
				pinctrl_info.audio_switch_act);
	if (ret < 0) {
		pr_err("failed to enable audio switch gpio\n");
		return ret;
	}

	pdata->hph_lr_swch_gpio = of_get_named_gpio(dev->of_node,
						"qcom,hph-lr-swch-gpios", 0);
	if (!gpio_is_valid(pdata->hph_lr_swch_gpio)) {
		pr_err("%s: Invalid gpio: %d", __func__,
				pdata->hph_lr_swch_gpio);
		return -EINVAL;
	}

	ret = gpio_request(pdata->hph_lr_swch_gpio,
					   "hph_lr_swch_gpio");
	if (ret) {
		pr_err("failed to request hph lr switch gpio\n");
		return ret;
	}

	ret = pinctrl_select_state(pinctrl_info.pinctrl,
				pinctrl_info.hph_lr_switch_act);
	if (ret < 0) {
		pr_err("failed to enable hph lr switch pinctrl\n");
		return ret;
	}

	pdata->vconn1_swch_gpio = of_get_named_gpio(dev->of_node,
					"qcom,vconn1-swch-gpios", 0);
	if (pdata->vconn1_swch_gpio < 0) {
		dev_dbg(dev,
			"property %s in node %s not found %d\n",
			"qcom,vconn1-swch-gpios", dev->of_node->full_name,
			pdata->vconn1_swch_gpio);
	} else {
		if (!gpio_is_valid(pdata->vconn1_swch_gpio)) {
			pr_err("%s: Invalid gpio: %d", __func__,
					pdata->vconn1_swch_gpio);
			return -EINVAL;
		} else {
			ret = pinctrl_select_state(pinctrl_info.pinctrl,
						pinctrl_info.vconn1_swch_act);
			if (ret < 0) {
				pr_err("failed to enable vconn1 switch gpio\n");
				return ret;
			}
			ret = gpio_request(pdata->vconn1_swch_gpio,
							   "vconn1_swch_gpio");
			if (ret) {
				pr_err("failed to request vconn1 switch gpio\n");
				return ret;
			}
			gpio_direction_output(pdata->vconn1_swch_gpio, pdata->active_low);

			pr_info("%s: vconn1 switch gpio set to %d\n", __func__, pdata->active_low);
		}
	}

	pdata->vconn2_swch_gpio = of_get_named_gpio(dev->of_node,
					"qcom,vconn2-swch-gpios", 0);
	if (pdata->vconn2_swch_gpio < 0) {
		dev_dbg(dev,
			"property %s in node %s not found %d\n",
			"qcom,vconn2-swch-gpios", dev->of_node->full_name,
			pdata->vconn2_swch_gpio);
	} else {
		if (!gpio_is_valid(pdata->vconn2_swch_gpio)) {
			pr_err("%s: Invalid gpio: %d", __func__,
					pdata->vconn2_swch_gpio);
			return -EINVAL;
		} else {
			ret = pinctrl_select_state(pinctrl_info.pinctrl,
						pinctrl_info.vconn2_swch_act);
			if (ret < 0) {
				pr_err("failed to enable vconn1 switch gpio\n");
				return ret;
			}
			ret = gpio_request(pdata->vconn2_swch_gpio,
							   "vconn2_swch_gpio");
			if (ret) {
				pr_err("failed to request vconn2 switch gpio\n");
				return ret;
			}
			gpio_direction_output(pdata->vconn2_swch_gpio, pdata->active_low);

			pr_info("%s: vconn2 switch gpio set to %d\n", __func__, pdata->active_low);
		}
	}

	pdata->uart_tx_gpio = of_get_named_gpio(dev->of_node,
					"qcom,uart-tx", 0);
	if (pdata->uart_tx_gpio < 0) {
		dev_dbg(dev,
			"property %s in node %s not found %d\n",
			"qcom,uart-tx", dev->of_node->full_name,
			pdata->uart_tx_gpio);
	} else {
		if (!gpio_is_valid(pdata->uart_tx_gpio)) {
			pr_err("%s: Invalid gpio: %d", __func__,
					pdata->uart_tx_gpio);
			return -EINVAL;
		} else {
			ret = pinctrl_select_state(pinctrl_info.pinctrl,
						pinctrl_info.uart_console_sus);
			if (ret < 0) {
				pr_err("failed to enable uart console gpio\n");
				return ret;
			}
			ret = gpio_request(pdata->uart_tx_gpio,
							   "uart_tx_gpio");
			if (ret) {
				pr_err("failed to request uart tx gpio\n");
				return ret;
			}
			gpio_direction_output(pdata->uart_tx_gpio, 0);

			pr_info("%s: uart tx gpio set to low\n", __func__);
		}
	}

	pdata->uart_rx_gpio = of_get_named_gpio(dev->of_node,
					"qcom,uart-rx", 0);
	if (pdata->uart_rx_gpio < 0) {
		dev_dbg(dev,
			"property %s in node %s not found %d\n",
			"qcom,uart-rx", dev->of_node->full_name,
			pdata->uart_rx_gpio);
	} else {
		if (!gpio_is_valid(pdata->uart_rx_gpio)) {
			pr_err("%s: Invalid gpio: %d", __func__,
					pdata->uart_rx_gpio);
			return -EINVAL;
		} else {
			ret = pinctrl_select_state(pinctrl_info.pinctrl,
						pinctrl_info.uart_console_sus);
			if (ret < 0) {
				pr_err("failed to enable uart console gpio\n");
				return ret;
			}
			ret = gpio_request(pdata->uart_rx_gpio,
							   "uart_rx_gpio");
			if (ret) {
				pr_err("failed to request uart rx gpio\n");
				return ret;
			}
			gpio_direction_output(pdata->uart_rx_gpio, 0);

			pr_info("%s: uart rx gpio set to low\n", __func__);
		}
	}

	return 0;
}
/**
*Name: get_board_id
*Author: lichuangchuang
*Date: 20160121
*Param: dev, pdata
*Return: not use
*Purpose: get board id
*/
static int get_board_id(struct device *dev, struct ti_usb_type_c *pdata)
{
	int ret;
	int board_id1_pull_up;
	int board_id2_pull_up;
	int board_id3_pull_up;
	int board_id1_pull_down;
	int board_id2_pull_down;
	int board_id3_pull_down;
	int board_id1_value;
	int board_id2_value;
	int board_id3_value;
	int board_id;
#ifdef CONFIG_GET_HARDWARE_INFO
	char board_id_info[125];
#endif

	pr_info("%s\n", __func__);

	pdata->board_id1_gpio = of_get_named_gpio(dev->of_node,
						"qcom,board-id1-gpios", 0);
	if (!gpio_is_valid(pdata->board_id1_gpio)) {
		pr_err("%s: Invalid gpio: %d", __func__,
				pdata->board_id1_gpio);
		return -EINVAL;
	}

	pdata->board_id2_gpio = of_get_named_gpio(dev->of_node,
						"qcom,board-id2-gpios", 0);
	if (!gpio_is_valid(pdata->board_id2_gpio)) {
		pr_err("%s: Invalid gpio: %d", __func__,
				pdata->board_id2_gpio);
		return -EINVAL;
	}

	pdata->board_id3_gpio = of_get_named_gpio(dev->of_node,
						"qcom,board-id3-gpios", 0);
	if (!gpio_is_valid(pdata->board_id3_gpio)) {
		pr_err("%s: Invalid gpio: %d", __func__,
				pdata->board_id3_gpio);
		return -EINVAL;
	}

	ret = pinctrl_select_state(pinctrl_info.pinctrl,
				pinctrl_info.board_id_act);
	if (ret < 0) {
		pr_err("failed to enable audio switch gpio\n");
		return ret;
	}

	board_id1_pull_up = __gpio_get_value(pdata->board_id1_gpio);
	board_id2_pull_up = __gpio_get_value(pdata->board_id2_gpio);
	board_id3_pull_up = __gpio_get_value(pdata->board_id3_gpio);
	ret = pinctrl_select_state(pinctrl_info.pinctrl,
				pinctrl_info.board_id_sus);
	if (ret < 0) {
		pr_err("failed to enable audio switch gpio\n");
		return ret;
	}
	board_id1_pull_down = __gpio_get_value(pdata->board_id1_gpio);
	board_id2_pull_down = __gpio_get_value(pdata->board_id2_gpio);
	board_id3_pull_down = __gpio_get_value(pdata->board_id3_gpio);
	if((board_id1_pull_up == 1) && (board_id1_pull_down == 0)){
		board_id1_value = 2;
	}else if(board_id1_pull_up == board_id1_pull_down){
		board_id1_value = board_id1_pull_up;
	}else if(board_id1_pull_up == board_id1_pull_down){
		pr_err("failed to get board_id1_value\n");
	}
	if((board_id2_pull_up == 1) && (board_id2_pull_down == 0)){
		board_id2_value = 2;
	}else if(board_id2_pull_up == board_id2_pull_down){
		board_id2_value = board_id2_pull_up;
	}else if(board_id2_pull_up == board_id2_pull_down){
		pr_err("failed to get board_id1_value\n");
	}
	if((board_id3_pull_up == 1) && (board_id3_pull_down == 0)){
		board_id3_value = 2;
	}else if(board_id3_pull_up == board_id3_pull_down){
		board_id3_value = board_id3_pull_up;
	}else if(board_id3_pull_up == board_id3_pull_down){
		pr_err("failed to get board_id1_value\n");
	}
	board_id = (board_id1_value << 4) | (board_id2_value << 2) | board_id3_value;
	switch(board_id){
		case 0x20:
		case 0x21:
		case 0x00:
			pdata->active_low = 1;
			break;
		default:
			pdata->active_low = 0;
			break;
	}
	pr_info("get board_id_value = %d\n", board_id);
	pr_info("get board_id1_pull_up = %d\n", board_id1_pull_up);
	pr_info("get board_id2_pull_up = %d\n", board_id2_pull_up);
	pr_info("get board_id3_pull_up = %d\n", board_id3_pull_up);
	pr_info("get board_id1_pull_down = %d\n", board_id1_pull_down);
	pr_info("get board_id2_pull_down = %d\n", board_id2_pull_down);
	pr_info("get board_id3_pull_down = %d\n", board_id3_pull_down);
	pr_info("get board_id1_value = %d\n", board_id1_value);
	pr_info("get board_id2_value = %d\n", board_id2_value);
	pr_info("get board_id3_value = %d\n", board_id3_value);
#ifdef CONFIG_GET_HARDWARE_INFO
	snprintf(board_id_info, 100,"V%d.%d.%d" ,
		board_id1_value, board_id2_value, board_id3_value);
	register_hardware_info(BOARD_ID, board_id_info);
#endif
	return 0;
}
static int tiusb_pinctrl_info_init(struct pinctrl *pinctrl)
{
	pr_info("%s\n", __func__);

	pinctrl_info.audio_switch_act = pinctrl_lookup_state(pinctrl,
		"audio_swch_act");
	if (IS_ERR(pinctrl_info.audio_switch_act)) {
		pr_err("%s: Unable to get pinctrl disable state handle\n",
							__func__);
		return -EINVAL;
	}
	pinctrl_info.audio_switch_sus = pinctrl_lookup_state(pinctrl,
		"audio_swch_sus");
	if (IS_ERR(pinctrl_info.audio_switch_sus)) {
		pr_err("%s: Unable to get pinctrl disable state handle\n",
							__func__);
		return -EINVAL;
	}
	pinctrl_info.vconn1_swch_act = pinctrl_lookup_state(pinctrl,
		"vconn1_swch_act");
	if (IS_ERR(pinctrl_info.vconn1_swch_act)) {
		pr_err("%s: Unable to get pinctrl disable state handle\n",
							__func__);
		return -EINVAL;
	}
	pinctrl_info.vconn1_swch_sus = pinctrl_lookup_state(pinctrl,
		"vconn1_swch_sus");
	if (IS_ERR(pinctrl_info.vconn1_swch_sus)) {
		pr_err("%s: Unable to get pinctrl disable state handle\n",
							__func__);
		return -EINVAL;
	}
	pinctrl_info.vconn2_swch_act = pinctrl_lookup_state(pinctrl,
		"vconn2_swch_act");
	if (IS_ERR(pinctrl_info.vconn2_swch_act)) {
		pr_err("%s: Unable to get pinctrl disable state handle\n",
							__func__);
		return -EINVAL;
	}
	pinctrl_info.vconn2_swch_sus = pinctrl_lookup_state(pinctrl,
		"vconn2_swch_sus");
	if (IS_ERR(pinctrl_info.vconn2_swch_sus)) {
		pr_err("%s: Unable to get pinctrl disable state handle\n",
							__func__);
		return -EINVAL;
	}
	pinctrl_info.board_id_act = pinctrl_lookup_state(pinctrl,
		"board_id_act");
	if (IS_ERR(pinctrl_info.board_id_act)) {
		pr_err("%s: Unable to get pinctrl disable state handle\n",
							__func__);
		return -EINVAL;
	}
	pinctrl_info.board_id_sus = pinctrl_lookup_state(pinctrl,
		"board_id_sus");
	if (IS_ERR(pinctrl_info.board_id_sus)) {
		pr_err("%s: Unable to get pinctrl disable state handle\n",
							__func__);
		return -EINVAL;
	}

	pinctrl_info.usb_irq_act = pinctrl_lookup_state(pinctrl,"usb_irq_act");
	if (IS_ERR(pinctrl_info.usb_irq_act)) {
		pr_err("%s: Unable to get usb_irq_active pinctrl disable\n",
							__func__);
		return -EINVAL;
	}
	pinctrl_info.hph_lr_switch_act = pinctrl_lookup_state(pinctrl,
		"hph_lr_swch_act");
	if (IS_ERR(pinctrl_info.hph_lr_switch_act)) {
		pr_err("%s: Unable to get pinctrl disable state handle\n",
							__func__);
		return -EINVAL;
	}
	pinctrl_info.hph_lr_switch_sus = pinctrl_lookup_state(pinctrl,
		"hph_lr_swch_sus");
	if (IS_ERR(pinctrl_info.hph_lr_switch_sus)) {
		pr_err("%s: Unable to get pinctrl disable state handle\n",
							__func__);
		return -EINVAL;
	}
	pinctrl_info.uart_console_sus = pinctrl_lookup_state(pinctrl,
		"uart_console_sus");
	if (IS_ERR(pinctrl_info.uart_console_sus)) {
		pr_err("%s: Unable to get pinctrl disable state handle\n",
							__func__);
		return -EINVAL;
	}

	pr_info("%s: finished\n", __func__);
	return 0;
}

static void tiusb_free(struct ti_usb_type_c *ti_usb)
{
	gpio_free(ti_usb->board_id1_gpio);
	gpio_free(ti_usb->board_id2_gpio);
	gpio_free(ti_usb->board_id3_gpio);
	gpio_free(ti_usb->audio_swch_gpio);
	gpio_free(ti_usb->hph_lr_swch_gpio);
	gpio_free(ti_usb->vconn1_swch_gpio);
	gpio_free(ti_usb->vconn2_swch_gpio);
}

static int tiusb_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret;
	struct power_supply *usb_psy;
	struct pinctrl *pinctrl;
	int irqn;
	int reg_10;
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&i2c->dev, "USB power_supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	ti_usb = devm_kzalloc(&i2c->dev, sizeof(struct ti_usb_type_c),
				GFP_KERNEL);
	if (!ti_usb)
		return -ENOMEM;

	i2c_set_clientdata(i2c, ti_usb);
	ti_usb->client = i2c;
	ti_usb->usb_psy = usb_psy;
    #ifdef CONFIG_TYPE_C_INFO
    ti_usb->at_bootup = true;
    ti_usb->manu_set_DRP_mode = false;
    ti_usb->manu_set_DFP_mode = false;
    ti_usb->manu_set_UFP_mode = false;
    ti_usb->manu_set_work = false;
    ti_usb->manu_curr_mode = MANU_CURR_MODE_DEFAULT;
    #endif

	/* override with module-param */
	/*
	if (!disable_on_suspend)
		disable_on_suspend = of_property_read_bool(np,
						"ti,disable-on-suspend");
	ti_usb->enb_gpio = of_get_named_gpio_flags(np, "ti,enb-gpio", 0,
							&flags);
	if (!gpio_is_valid(ti_usb->enb_gpio)) {
		dev_dbg(&i2c->dev, "enb gpio_get fail:%d\n", ti_usb->enb_gpio);
	} else {
		ti_usb->enb_gpio_polarity = !(flags & OF_GPIO_ACTIVE_LOW);
		ret = tiusb_gpio_config(ti_usb, true);
		if (ret)
			goto out;
	}
	*/

	if (i2c->dev.of_node) {
		ret = tiusb_parse_dt(&i2c->dev, ti_usb);
		if (ret) {
			dev_err(&i2c->dev,
				"unable to parse device tree.(%d)\n", ret);
			goto out;
		}
	} else {
		dev_err(&i2c->dev, "device tree not found.\n");
		ret = -ENODEV;
		goto out;
	}
	INIT_DELAYED_WORK(&ti_usb->tiusb_set_mode_dwork, tiusb_manu_set_mode_work);

	INIT_DELAYED_WORK(&ti_usb->tiusb_read_regdata_dwork, tiusb_read_regdata_work);

	ret = tiusb_ldo_init(ti_usb, true);
	if (ret) {
		dev_err(&ti_usb->client->dev, "i2c ldo init failed\n");
		goto out;
	}

	ret = was4732d_ldo_init(ti_usb, true);
	if (ret) {
		dev_err(&ti_usb->client->dev, "was4732d ldo init failed\n");
		goto ti_ldo_disable;
	}
	reg_10 = i2c_smbus_read_byte_data(ti_usb->client, TI_STS_A_REG);
	ret = tiusb_reset(i2c);
	if (ret == -EIO) {
		dev_err(&ti_usb->client->dev, "tiusb reset failed\n");
		goto was4732d_ldo_disable;
	}

	ret = tiusb_read_regdata(i2c);
	if (ret == -EIO) {
		dev_err(&ti_usb->client->dev, "i2c access failed\n");
		ret = -EPROBE_DEFER;
		goto was4732d_ldo_disable;
	}

	ret = device_create_file(&i2c->dev, &dev_attr_tiusb_reg);
	if (ret < 0) {
		dev_err(&i2c->dev, "failed to add tiusb_reg sysfs files\n");
		goto out;
	}

	/* Update initial state to USB */
	tiusb_init(i2c->irq, ti_usb);

	pinctrl = devm_pinctrl_get(&i2c->dev);
	if (IS_ERR(pinctrl)) {
		pr_err("%s: Unable to get pinctrl handle\n",
				__func__);
		return -EINVAL;
	}
	pinctrl_info.pinctrl = pinctrl;
	ret = tiusb_pinctrl_info_init(pinctrl);
	if (ret < 0) {
		pr_err("%s: failed to get the tusb320 gpio's %d\n",
				__func__, ret);
		goto was4732d_ldo_disable;
	}

	if (gpio_is_valid(ti_usb->irq_gpio)) {
		ret = gpio_request(ti_usb->irq_gpio, "ti_usb_enb_gpio");
		if (ret) {
		dev_err(&i2c->dev, "%s: unable to request gpio [%d]\n",
			__func__, ti_usb->irq_gpio);
		goto was4732d_ldo_disable;
		}

		ret = pinctrl_select_state(pinctrl_info.pinctrl,
					pinctrl_info.usb_irq_act);
		if (ret < 0) {
			pr_err("failed to enable usb_irq_act gpio\n");
		}

		ret = gpio_direction_input(ti_usb->irq_gpio);
		if (ret) {
			dev_err(&i2c->dev,
				"%s: unable to set direction for gpio [%d]\n",
				__func__, ti_usb->irq_gpio);
			goto err_irq;
		}
		irqn = gpio_to_irq(ti_usb->irq_gpio);
		if (irqn < 0) {
			ret = irqn;
			goto err_irq;
		}
		i2c->irq = irqn;
        } else {
            dev_err(&i2c->dev, "%s: irq gpio not provided\n", __func__);
            goto was4732d_ldo_disable;
        }

	ret = get_board_id(&i2c->dev, ti_usb);
	if (ret < 0) {
		pr_err("%s: failed to get_board_id %d\n",
				__func__, ret);
		goto err_i2c;
	}
	ret = tiusb_ext_gpios_init(&i2c->dev, ti_usb);
	if (ret < 0) {
		pr_err("%s: failed to vconn12_switch_gpio_init %d\n",
				__func__, ret);
		goto err_i2c;
	}
	spin_lock_init(&ti_usb->irq_enabled_lock);
	wake_lock_init(&ti_usb->ti_usb_wake_lock, WAKE_LOCK_SUSPEND, "ti_usb_wake_lock");
	mutex_init(&ti_usb->ti_usb_lock);

	ret = devm_request_threaded_irq(&i2c->dev, i2c->irq, NULL, tiusb_irq,
					IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
					TYPE_C_I2C_NAME, ti_usb);
	if (ret) {
		dev_err(&i2c->dev, "irq(%d) req failed-%d\n", i2c->irq, ret);
		goto err_i2c;
	}
      #ifdef CONFIG_TYPE_C_INFO
	ti_usb->typec_set_notif.notifier_call=typec_set_notifier;
       ret = type_c_set_register_client(&ti_usb->typec_set_notif);
       if (ret)
            dev_err(&i2c->dev, "Unable to register typec_set_notif: %d\n",ret);
	 #endif
    ti_usb->typec_reboot_notif.notifier_call=typec_reboot_notifier;
    ret = register_reboot_notifier(&ti_usb->typec_reboot_notif);
    if (ret)
        dev_err(&i2c->dev, "Unable to register typec_reboot_notif: %d\n",ret);
    #ifdef CONFIG_TYPE_C_INFO
    ti_usb->at_bootup = false;
    #endif
	dev_dbg(&i2c->dev, "%s finished, addr:%d\n", __func__, i2c->addr);
   if((reg_10&0x30) == 0x10){
		mdelay(300);
		i2c_smbus_write_byte_data(ti_usb->client, 0x45, 0x40);
		mdelay(100);
		i2c_smbus_write_byte_data(ti_usb->client, 0x45, 0x00);
		mdelay(100);
		i2c_smbus_write_byte_data(ti_usb->client, 0x45, 0x40);
		mdelay(100);
		i2c_smbus_write_byte_data(ti_usb->client, 0x45, 0x00);
     }
	is_ti_type_c_register = true;
	return 0;

err_i2c:
	tiusb_free(ti_usb);
err_irq:
	gpio_free(ti_usb->irq_gpio);
was4732d_ldo_disable:
		was4732d_ldo_init(ti_usb, false);
ti_ldo_disable:
		tiusb_ldo_init(ti_usb, false);
out:
	return ret;
}

static int tiusb_remove(struct i2c_client *i2c)
{
	struct ti_usb_type_c *ti_usb = i2c_get_clientdata(i2c);

	tiusb_ldo_init(ti_usb, false);
	gpio_free(ti_usb->irq_gpio);
	wake_lock_destroy(&ti_usb->ti_usb_wake_lock);
#ifdef CONFIG_TYPE_C_INFO
	type_c_set_unregister_client(&ti_usb->typec_set_notif);
#endif
	type_c_set_unregister_client(&ti_usb->typec_reboot_notif);
	devm_kfree(&i2c->dev, ti_usb);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tiusb_i2c_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct ti_usb_type_c *ti = i2c_get_clientdata(i2c);

	dev_err(dev, "ti_usb PM suspend.. attach(%d) disable(%d)\n",
			ti->attach_state, disable_on_suspend);
	disable_irq(ti->client->irq);
	/* Keep type-c chip enabled during session */
	if (ti->attach_state)
		return 0;

	regulator_set_voltage(ti->i2c_1p8, 0, TIUSB_1P8_VOL_MAX);
	regulator_disable(ti->i2c_1p8);

	if (disable_on_suspend)
		gpio_set_value(ti->enb_gpio, !ti->enb_gpio_polarity);

	return 0;
}

static int tiusb_i2c_resume(struct device *dev)
{
	int rc;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct ti_usb_type_c *ti = i2c_get_clientdata(i2c);

	dev_dbg(dev, "ti_usb PM resume\n");
	/* suspend was no-op, just re-enable interrupt */
	if (ti->attach_state) {
		enable_irq(ti->client->irq);
		return 0;
	}

	if (disable_on_suspend) {
		gpio_set_value(ti->enb_gpio, ti->enb_gpio_polarity);
		msleep(TI_I2C_DELAY_MS);
	}

	rc = regulator_set_voltage(ti->i2c_1p8, TIUSB_1P8_VOL_MAX,
					TIUSB_1P8_VOL_MAX);
	if (rc)
		dev_err(&ti->client->dev, "unable to set voltage(%d)\n", rc);

	rc = regulator_enable(ti->i2c_1p8);
	if (rc)
		dev_err(&ti->client->dev, "unable to enable 1p8-reg(%d)\n", rc);

	enable_irq(ti->client->irq);

	return rc;
}
#endif

static SIMPLE_DEV_PM_OPS(tiusb_i2c_pm_ops, tiusb_i2c_suspend,
			  tiusb_i2c_resume);

static const struct i2c_device_id tiusb_id[] = {
	{ TYPE_C_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tiusb_id);

#ifdef CONFIG_OF
static const struct of_device_id tiusb_of_match[] = {
	{ .compatible = "ti,usb-type-c", },
	{},
};
MODULE_DEVICE_TABLE(of, tiusb_of_match);
#endif

static struct i2c_driver tiusb_driver = {
	.driver = {
		.name = TYPE_C_I2C_NAME,
		.of_match_table = of_match_ptr(tiusb_of_match),
	},
	.probe		= tiusb_probe,
	.remove		= tiusb_remove,
	.id_table	= tiusb_id,
};

module_i2c_driver(tiusb_driver);

MODULE_DESCRIPTION("TI TypeC Detection driver");
MODULE_LICENSE("GPL v2");
