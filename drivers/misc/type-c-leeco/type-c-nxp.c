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
#include <linux/wakelock.h>
#include "type-c-nxp.h"
#include "type-c-info.h"
#include "../../../sound/soc/codecs/wcd-mbhc-v2.h"
#ifdef CONFIG_GET_HARDWARE_INFO
#include <asm/hardware_info.h>
#endif
#ifdef CONFIG_TYPE_C_INFO
#include <linux/type-c_notify.h>
#endif
#include <linux/reboot.h>
static struct ptn5150_usb_pinctrl_info pinctrl_info;
struct ptn5150_usb_type_c *ptn5150_usb;
extern bool is_nxp_type_c_register;
extern bool is_attached_ufp;
extern int usb_audio_digital_swap_analog;
static bool nxp_otg_have_disconnected = false;

#define Try_Sink_Idle_DRP					0
#define Try_Sink_Attached_Wait_Src			1
#define Try_Sink_Source_To_Sink				2
#define Try_Sink_Attached_Wait_Src_Detached	3
#define Try_Sink_Attached_As_UFP			4
#define Try_Sink_tDRPTry_Expire				5
#define Try_Sink_Try_Wait_Src				6
#define Try_Sink_Attached_As_DFP			7
#define Try_Sink_Try_Wait_Src_Expire		8

static int Try_Sink_State = Try_Sink_Idle_DRP;
static DEFINE_MUTEX(ptn5150_i2c_lock);

extern void msm_otg_vbus_power_set(bool enable);
#ifdef CONFIG_TYPE_C_INFO
static int typec_set_notifier(struct notifier_block *self,unsigned long action, void *data)
{
    int *mode;
    u8 curr_mode;
    struct type_c_event_data *evdata = data;

    if(ptn5150_usb->at_bootup)//do not set DRP or other modes manually at bootup.
        return 0;

    if (evdata && evdata->data && action == TYPE_C_SET_MODE){
        mode=evdata->data;
        pr_err("%s: mode = %d\n",__func__, *mode);

        /*mode = 0: set DFP mode from UI.*/
        /*mode = 1: set UFP mode from UI.*/
        /*mode = 2: set DRP mode from UI.*/
        /*mode = 3 or other value: set default(DRP) mode from UI.*/

        if (0 == (*mode))
            curr_mode = NXP_MANU_CURR_MODE_DFP;
        else if (1 == (*mode))
            curr_mode = NXP_MANU_CURR_MODE_UFP;
        else if (2 == (*mode))
            curr_mode = NXP_MANU_CURR_MODE_DRP;
        else
            curr_mode = NXP_MANU_CURR_MODE_DEFAULT;

        pr_err("%s: ptn5150_usb->manu_curr_mode = %d\n",__func__, ptn5150_usb->manu_curr_mode);

        /*so do nothing when as the same as last time.*/
        if(ptn5150_usb->manu_curr_mode == curr_mode){
            pr_err("so do nothing as same as current mode,current_mode=%d\n",ptn5150_usb->manu_curr_mode);
            return 0;
        }

        /*do nothing when any usb device is connected*/
        if (NXP_ATTACH_TO_NONE != ptn5150_usb->attach_state){
            pr_err("usb device is connected,do not set mode manually,ptn5150_usb->attach_state=%d\n",ptn5150_usb->attach_state);
            return 0;
       }

	ptn5150_usb->manu_curr_mode = curr_mode;
	if (ptn5150_usb->manu_curr_mode == NXP_MANU_CURR_MODE_DFP){
	    ptn5150_usb->manu_set_DFP_mode = true;
	    ptn5150_usb->manu_set_DRP_mode = false;
	    ptn5150_usb->manu_set_UFP_mode = false;
	}

	if (ptn5150_usb->manu_curr_mode == NXP_MANU_CURR_MODE_DRP){
	    ptn5150_usb->manu_set_DRP_mode = true;
	    ptn5150_usb->manu_set_DFP_mode = false;
	    ptn5150_usb->manu_set_UFP_mode = false;
	}

	if (ptn5150_usb->manu_curr_mode == NXP_MANU_CURR_MODE_DEFAULT){
	    ptn5150_usb->manu_set_DRP_mode = true;
	    ptn5150_usb->manu_set_DFP_mode = false;
	    ptn5150_usb->manu_set_UFP_mode = false;
	}
	ptn5150_usb->manu_set_mode = true;
	schedule_delayed_work(&ptn5150_usb->nxpusb_set_mode_dwork, msecs_to_jiffies(10));
    }
    /*indicate that notifier is ready*/
    if (evdata && evdata->data && action == TYPE_C_INFO_NOTIFY_READY)
    {
        ptn5150_usb->info_notif_ready = true;
    }
    return NOTIFY_OK;
}

static int typec_reboot_notifier(struct notifier_block *self,unsigned long code, void *data)
{
    int rc;

    if ((code == SYS_DOWN) ||(code == SYS_HALT) || (code == SYS_POWER_OFF)){
        ptn5150_usb->status_2_reg = ((ptn5150_usb->status_2_reg) & ( NXP_MODE_MASK)) | (NXP_MODE_UFP_MASK);
        dev_err(&ptn5150_usb->client->dev, "set UFP mode during poweroff process\n");
        rc = i2c_smbus_write_byte_data(ptn5150_usb->client, NXP_STS_A_REG, ptn5150_usb->status_2_reg);
        if (rc < 0)
            dev_err(&ptn5150_usb->client->dev, "set UFP mode failed during poweroff process\n");
    }
    mdelay(900);
    return NOTIFY_DONE;
}
static void nxpusb_manu_set_mode_work(struct work_struct *work)
{
    u8 mode;
    int rc;
    struct type_c_event_data notif_data;

    dev_err(&ptn5150_usb->client->dev, "%s:begin\n",__func__);

    if (ptn5150_usb->irq_set_work){
        dev_err(&ptn5150_usb->client->dev, "irq set work is busy,waiting for its finishing\n");
        schedule_delayed_work(&ptn5150_usb->nxpusb_set_mode_dwork, msecs_to_jiffies(500));
    }
    ptn5150_usb->manu_set_work = true;

    if(!(ptn5150_usb->manu_set_DFP_mode) && !(ptn5150_usb->manu_set_UFP_mode) && !(ptn5150_usb->manu_set_DRP_mode))//return if receive nothing from UI.
    {
        dev_err(&ptn5150_usb->client->dev, "manual set mode is not received from UI,so do nothing\n");
        goto exit;
    }

    rc = i2c_smbus_read_byte_data(ptn5150_usb->client, NXP_STS_A_REG);
    if (rc < 0)
        goto exit;
    ptn5150_usb->status_2_reg = rc;

    /*case:triger from irq*/
    if (false == ptn5150_usb->manu_set_mode){

        /*case:disconnect event*/
        if (NXP_ATTACH_TO_NONE == ptn5150_usb->attach_state){
            ptn5150_usb->manu_set_DFP_mode = false;
            ptn5150_usb->manu_set_UFP_mode = false;
            ptn5150_usb->manu_set_DRP_mode = false;

            ptn5150_usb->status_2_reg = ((ptn5150_usb->status_2_reg) & ( NXP_MODE_MASK)) | (NXP_MODE_DRP_MASK);
            dev_err(&ptn5150_usb->client->dev, "ptn5150_usb->status_2_reg = 0x%x\n",ptn5150_usb->status_2_reg);
            dev_err(&ptn5150_usb->client->dev, "device is disconnected from otg,so set type-c as default mode\n");
            rc = i2c_smbus_write_byte_data(ptn5150_usb->client, NXP_STS_A_REG, ptn5150_usb->status_2_reg);
            if (rc < 0)
                goto exit;

            if(ptn5150_usb->info_notif_ready){
                mode = NXP_MANU_CURR_MODE_DEFAULT;
                notif_data.data = &mode;
                type_c_get_notifier_call_chain(TYPE_C_INFO_MODE_RESULT,&notif_data);
            }
        }
        /*case:connect to an  DFP*/
        else if (NXP_ATTACH_TO_DFP == ptn5150_usb->attach_state){
            ptn5150_usb->manu_set_DFP_mode = false;
            ptn5150_usb->manu_set_UFP_mode = false;
            ptn5150_usb->manu_set_DRP_mode = false;

            ptn5150_usb->status_2_reg = ((ptn5150_usb->status_2_reg) & ( NXP_MODE_MASK)) | (NXP_MODE_DRP_MASK);
            dev_err(&ptn5150_usb->client->dev, "ptn5150_usb->status_2_reg = 0x%x\n",ptn5150_usb->status_2_reg);
            dev_err(&ptn5150_usb->client->dev, "connecting to an UFP device,so set type-c as default mode immediately\n");
            rc = i2c_smbus_write_byte_data(ptn5150_usb->client, NXP_STS_A_REG, ptn5150_usb->status_2_reg);
            if (rc < 0)
                goto exit;

            if(ptn5150_usb->info_notif_ready){
                mode = NXP_MANU_CURR_MODE_DEFAULT;
                notif_data.data = &mode;
                type_c_get_notifier_call_chain(TYPE_C_INFO_MODE_RESULT,&notif_data);
            }
        }

    }
    /*case:set mode received from UI when no device is connected.*/
    if ((NXP_ATTACH_TO_NONE == ptn5150_usb->attach_state) && (true == ptn5150_usb->manu_set_mode)){

        if(true == ptn5150_usb->manu_set_DFP_mode){
            ptn5150_usb->status_2_reg = ((ptn5150_usb->status_2_reg) & ( NXP_MODE_MASK)) | (NXP_MODE_DFP_MASK);
            if(ptn5150_usb->info_notif_ready){
                mode = NXP_MANU_CURR_MODE_DFP;
                notif_data.data = &mode;
                type_c_get_notifier_call_chain(TYPE_C_INFO_MODE_RESULT,&notif_data);
            }
        }

        if(true == ptn5150_usb->manu_set_UFP_mode){
            ptn5150_usb->status_2_reg = ((ptn5150_usb->status_2_reg) & ( NXP_MODE_MASK)) | (NXP_MODE_UFP_MASK);
            if(ptn5150_usb->info_notif_ready){
                mode = NXP_MANU_CURR_MODE_UFP;
                notif_data.data = &mode;
                type_c_get_notifier_call_chain(TYPE_C_INFO_MODE_RESULT,&notif_data);
            }
        }

        if(true == ptn5150_usb->manu_set_DRP_mode){
            ptn5150_usb->status_2_reg = ((ptn5150_usb->status_2_reg) & ( NXP_MODE_MASK)) | (NXP_MODE_DEFAULT_MASK);
            if(ptn5150_usb->info_notif_ready){
                mode = NXP_MANU_CURR_MODE_DRP;
                notif_data.data = &mode;
                type_c_get_notifier_call_chain(TYPE_C_INFO_MODE_RESULT,&notif_data);
            }
        }
        dev_err(&ptn5150_usb->client->dev, "set %s mode manually\n", ptn5150_usb->manu_set_DFP_mode?"DFP":"default");
        rc = i2c_smbus_write_byte_data(ptn5150_usb->client, NXP_STS_A_REG, ptn5150_usb->status_2_reg);
        if (rc < 0)
            goto exit;
    }

exit:

    ptn5150_usb->manu_set_work = false;
}
#endif

static ssize_t ptn5150_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int count = 0;
	int i;
	int value;

	for (i = NXP_STS_ID_REG; i <= NXP_STS_INT_REG; i++) {
		value = i2c_smbus_read_byte_data(ptn5150_usb->client, i);
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

static ssize_t ptn5150_reg_store(struct device *dev,
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

	if (addr > NXP_STS_INT_REG || val > 0xffff || val < 0)
		return count;

	if (i == count)
		pr_info("0x%02x = 0x%04x\n", addr,
			i2c_smbus_read_byte_data(ptn5150_usb->client, addr));
	else
		i2c_smbus_write_byte_data(ptn5150_usb->client, addr, val);

	return count;
}
static DEVICE_ATTR(ptn5150_reg, 0666, ptn5150_reg_show, ptn5150_reg_store);

static int ptn5150_usb_ldo_init(struct ptn5150_usb_type_c *ptn5150, bool init)
{
	int rc = 0;

	if (!init) {
		regulator_set_voltage(ptn5150->i2c_1p8, 0, USB_1P8_VOL_MAX);
		rc = regulator_disable(ptn5150->i2c_1p8);
		return rc;
	}

	ptn5150->i2c_1p8 = devm_regulator_get(&ptn5150->client->dev, "vdd_io");
	if (IS_ERR(ptn5150->i2c_1p8)) {
		rc = PTR_ERR(ptn5150->i2c_1p8);
		dev_err(&ptn5150->client->dev, "unable to get 1p8(%d)\n", rc);
		return rc;
	}
	rc = regulator_set_voltage(ptn5150->i2c_1p8, USB_1P8_VOL_MAX,
					USB_1P8_VOL_MAX);
	if (rc) {
		dev_err(&ptn5150->client->dev, "unable to set voltage(%d)\n", rc);
		goto put_1p8;
	}

	rc = regulator_enable(ptn5150->i2c_1p8);
	if (rc) {
		dev_err(&ptn5150->client->dev, "unable to enable 1p8-reg(%d)\n", rc);
		return rc;
	}

	return 0;

put_1p8:
	regulator_set_voltage(ptn5150->i2c_1p8, 0, USB_1P8_VOL_MAX);
	return rc;
}

static int was4732d_ldo_init(struct ptn5150_usb_type_c *ptn5150, bool init)
{
	int rc = 0;

	if (!init) {
		regulator_set_voltage(ptn5150->i2c_2p95, 0, USB_2P95_VOL_MAX);
		rc = regulator_disable(ptn5150->i2c_2p95);
		return rc;
	}

	ptn5150->i2c_2p95 = devm_regulator_get(&ptn5150->client->dev, "vdd");
	if (IS_ERR(ptn5150->i2c_2p95)) {
		rc = PTR_ERR(ptn5150->i2c_2p95);
		dev_err(&ptn5150->client->dev, "unable to get 2p95(%d)\n", rc);
		return rc;
	}
	rc = regulator_set_voltage(ptn5150->i2c_2p95, USB_2P95_VOL_MAX,
					USB_2P95_VOL_MAX);
	if (rc) {
		dev_err(&ptn5150->client->dev, "unable to set voltage(%d)\n", rc);
		goto put_2p95;
	}

	rc = regulator_enable(ptn5150->i2c_2p95);
	if (rc) {
		dev_err(&ptn5150->client->dev, "unable to enable 2p95-reg(%d)\n", rc);
		return rc;
	}

	return 0;

put_2p95:
	regulator_set_voltage(ptn5150->i2c_2p95, 0, USB_2P95_VOL_MAX);
	return rc;
}

static int ptn5150_usb_parse_dt(struct device *dev, struct ptn5150_usb_type_c *pdata)
{
	struct device_node *np = dev->of_node;

        pdata->irq_gpio = of_get_named_gpio(np, "nxp,irq-gpio", 0);
        if ((!gpio_is_valid(pdata->irq_gpio)))
            return -EINVAL;

        return 0;

}

int usb_audio_switch_gpio_set(struct ptn5150_usb_type_c *pdata, bool value)
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

void ptn5150_usb_audio_mode_set(bool mode)
{
	struct ptn5150_usb_type_c *pdata = ptn5150_usb;

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
	mutex_lock(&ptn5150_usb->ptn5150_lock);
	cclogic_set_audio_mode(mode);
	if (mode) {
		usb_audio_digital_swap_analog = 1;
		gpio_direction_output(pdata->audio_swch_gpio, 1);
		pr_info("%s: audio switch gpio sets to high\n", __func__);
		gpio_direction_output(pdata->hph_lr_swch_gpio, 1);
		pr_info("%s: hph lr switch gpio sets to high\n", __func__);
		if (0x02 == (ptn5150_usb->status_4_reg & 0x03)) {
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
		    if (nxp_otg_have_disconnected) {
		        pr_info("%s: ******************avoid crash\n", __func__);
		        nxp_otg_have_disconnected = false;
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
	mutex_unlock(&ptn5150_usb->ptn5150_lock);
	return;
}

static int ptn5150_usb_ext_gpios_init(struct device *dev, struct ptn5150_usb_type_c *pdata)
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

			pr_info("%s: vconn1 switch gpio set to high\n", __func__);
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

			pr_info("%s: vconn2 switch gpio set to high\n", __func__);
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
static int get_board_id(struct device *dev, struct ptn5150_usb_type_c *pdata)
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
static int ptn5150_usb_pinctrl_info_init(struct pinctrl *pinctrl)
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


static irqreturn_t ptn5150_cc_eint_interrupt_handler(int irq, void *data)
{
	struct ptn5150_usb_type_c *ptn5150_usb = (struct ptn5150_usb_type_c *)data;
	unsigned long flags;

	t_printk(K_INFO, "ptn5150_cc_eint_interrupt_handler interrupt!!!\n");

	wake_lock(&ptn5150_usb->ptn5150_wake_lock);
	spin_lock_irqsave(&ptn5150_usb->irq_enabled_lock, flags);
#ifdef CONFIG_TYPE_C_INFO
	ptn5150_usb->manu_set_mode = false;
#endif
	schedule_delayed_work(&ptn5150_usb->eint_work,0);
	spin_unlock_irqrestore(&ptn5150_usb->irq_enabled_lock, flags);

	return IRQ_RETVAL(IRQ_HANDLED);
}

static int ptn5150_i2c_read(struct i2c_client *client, u8 buf[], u8 reg)
{
	int ret;

	mutex_lock(&ptn5150_i2c_lock);

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		printk("ptn5150_i2c_read,failed to read 0x%.2x\n", reg);
		mutex_unlock(&ptn5150_i2c_lock);
		return ret;
	}

	buf[0] = (unsigned char)ret;

	mutex_unlock(&ptn5150_i2c_lock);

	return 0;
}

static int ptn5150_i2c_write(struct i2c_client *client, u8 reg,u8 const buf[])
{
	int ret;

	mutex_lock(&ptn5150_i2c_lock);

	ret = i2c_smbus_write_byte_data(client, reg, buf[0]);
	if(ret < 0)
	{
		t_printk(K_ERR, "ptn5150_i2c_write,failed to write 0x%.2x\n", reg);
	}

	mutex_unlock(&ptn5150_i2c_lock);

  return ret;
}

static void ptn5150_eint_work(struct work_struct *work)
{
	//u8 attached_value = 0;
	u8 Register_Value = 0;
	int attachment_status;
	u8 reg1=0x15,reg2=0x11;
	u8 reg_val0=0x24;//aftung3
	u8 reg_val1=0xc0;//aftung3
#ifdef CONFIG_TYPE_C_INFO
	u8 dir_state;
	struct type_c_event_data notif_data;
	if (ptn5150_usb->manu_set_work){
		dev_err(&ptn5150_usb->client->dev, "manu set work is busy,waiting for its finishing\n");
		schedule_delayed_work(&ptn5150_usb->nxpusb_set_mode_dwork, msecs_to_jiffies(500));
	}
	ptn5150_usb->irq_set_work = true;
#endif
	t_printk(K_INFO, "ptn5150_eint_work->NXP\n");
	ptn5150_i2c_read(ptn5150_usb->client, &Register_Value, 0x19);
	t_printk(K_INFO, "ptn5150 0x19 register value = 0x%x\n", Register_Value);

	if (Register_Value & 0x8)//role changed interupt
	{
		ptn5150_i2c_read(ptn5150_usb->client, &Register_Value, 0x04);
		attachment_status = (Register_Value >> 2) & 0x07;
		ptn5150_usb->status_4_reg = Register_Value;
#ifdef CONFIG_TYPE_C_INFO
		if(ptn5150_usb->info_notif_ready){
			dir_state = ptn5150_usb->status_4_reg & 0x03;
			if(dir_state==0x01){
				dir_state=0;
			}else if(dir_state==0x02){
				dir_state=1;
			}
			printk("%s:dir_state = %d\n",__func__,dir_state);
			ptn5150_usb->dir_state = dir_state;
			notif_data.data=&dir_state;
			type_c_get_notifier_call_chain(TYPE_C_GET_ORIENTATION,&notif_data);
		}
		ptn5150_usb->irq_set_work = false;
		if(ptn5150_usb->manu_set_DRP_mode || ptn5150_usb->manu_set_DFP_mode || ptn5150_usb->manu_set_UFP_mode)
		schedule_delayed_work(&ptn5150_usb->nxpusb_set_mode_dwork, msecs_to_jiffies(300));
#endif
		ptn5150_usb->attach_state = attachment_status;
		t_printk(K_INFO, "ptn5150 0x04 register value = 0x%x, attachment_status=%d, Try_Sink_State=%d\n", Register_Value, attachment_status, Try_Sink_State);
		switch (attachment_status)
		{
		case 0:
			// If Cable Disconnected
			is_attached_ufp = false;
			usb_audio_digital_swap_analog = 0;
			nxp_otg_have_disconnected = true;
			if (Try_Sink_State == Try_Sink_Source_To_Sink)
			{
			       t_printk(K_INFO, "case 0 try sink\n");
				schedule_delayed_work(&ptn5150_usb->trysnk_work1, msecs_to_jiffies(100));
				Try_Sink_State = Try_Sink_Attached_Wait_Src_Detached;
			}
			// aftung simplify
			//if ((Try_Sink_State == Try_Sink_Attached_As_DFP) || (Try_Sink_State == Try_Sink_Attached_As_UFP)||(Try_Sink_State == No_Trysink_Analog_In))
			else // this will including any kind of disconnect, when disconnect, we go back to DRP, aind state=try sink idle drp
			{
				reg_val0=0x24;//aftung3, restore to default enable accessory detect
				reg_val1=0xc0;//aftung3, restore to default
				ptn5150_i2c_write(ptn5150_usb->client, 0x4c, &reg_val0 );	//aftung3
				ptn5150_i2c_write(ptn5150_usb->client, 0x43, &reg_val1);		//aftung3
				ptn5150_i2c_write(ptn5150_usb->client, 0x02, &reg1);
				// Force to run in DRP mode
				Try_Sink_State = Try_Sink_Idle_DRP;

				t_printk(K_INFO, "detect cable removal\n");
				/*ptn5150_cable_disconnect();

				if (g_exttypec->device_driver &&
					(g_exttypec->device_driver->on == ENABLE))
					ptn5150_trigger_driver(g_exttypec, DEVICE_TYPE, DISABLE);
				else if (g_exttypec->host_driver &&
					(g_exttypec->host_driver->on == ENABLE))
					ptn5150_trigger_driver(g_exttypec, HOST_TYPE, DISABLE);

				pinctrl_select_state(g_exttypec->pinctrl,g_exttypec->pin_cfg->ptn5150_pins_cc1_high);
				pinctrl_select_state(g_exttypec->pinctrl,g_exttypec->pin_cfg->ptn5150_pins_cc2_high);
				pinctrl_select_state(g_exttypec->pinctrl,g_exttypec->pin_cfg->uart0_rx_pins_set_cfg);
				pinctrl_select_state(g_exttypec->pinctrl,g_exttypec->pin_cfg->uart0_tx_pins_set_cfg);*/
				t_printk(K_INFO, "clr rx tx\n");
			}
			if (is_type_c_headset_inserted) {
				is_type_c_headset_inserted = false;
				usb_audio_switch_gpio_set(ptn5150_usb, false);
				wcd_mbhc_mech_plug_detect();
			}
			break;
		case 1:
			// if DFP is attached
			if (Try_Sink_State == Try_Sink_Attached_Wait_Src_Detached)
			{
				cancel_delayed_work(&ptn5150_usb->trysnk_work1);
				Try_Sink_State = Try_Sink_Attached_As_UFP;
			}
			else
			{
				Try_Sink_State = Try_Sink_Attached_As_UFP;
			}
			// aftung: I move the following here, since this needs to be done when we are UFP

			t_printk(K_INFO, " case 1 DFP is attached!\n");
			//ptn5150_trigger_driver(g_exttypec, DEVICE_TYPE,ENABLE);
			break;
		case 2:
			// if UFP is attached
			is_attached_ufp = true;
			nxp_otg_have_disconnected = false;
			if (Try_Sink_State == Try_Sink_Idle_DRP)
				// in this state, you don't want to enable VBus output right now, instead, you want to first try sink
			{
				Try_Sink_State = Try_Sink_Attached_Wait_Src;
				reg_val0=0x34;//aftung3, try sink setting, disable accessory detect
				reg_val1=0x40;//aftung3: 0x40
				ptn5150_i2c_write(ptn5150_usb->client, 0x4c, &reg_val0 );	//aftung3
				ptn5150_i2c_write(ptn5150_usb->client, 0x43, &reg_val1);		//aftung3
				ptn5150_i2c_write(ptn5150_usb->client, 0x02, &reg2);
				// Force to run in UFP mode
				Try_Sink_State = Try_Sink_Source_To_Sink;
			}
			else if (Try_Sink_State == Try_Sink_Try_Wait_Src)
			{
				cancel_delayed_work(&ptn5150_usb->trysnk_work2);
				Try_Sink_State = Try_Sink_Attached_As_DFP;

				// aftung: in this state, you have already try sink, but not successful, so you are back in DFP, and you see a UFP attached, so then you output VBus, etc.
				/*pinctrl_select_state(g_exttypec->pinctrl,g_exttypec->pin_cfg->uart0_tx_pins_clr_cfg);
				pinctrl_select_state(g_exttypec->pinctrl,g_exttypec->pin_cfg->uart0_rx_pins_clr_cfg);
				t_printk(K_INFO, "UFP is attached!\n");
				t_printk(K_ERR, "clr rx tx\n");
				ptn5150_trigger_driver(g_exttypec, HOST_TYPE, ENABLE);*/
				t_printk(K_INFO, "UFP is attached!\n");
			}
			break;
		case 3: // aftung: analog audio accessory
			// in this case, we don't need to look at the try sink states., just go ahead and handle audio
			t_printk(K_INFO, "audio Attached to an accessory!\n");
			is_type_c_headset_inserted = true;
			usb_audio_switch_gpio_set(ptn5150_usb, true);
			wcd_mbhc_mech_plug_detect();
		         break;
		};

	}
	wake_unlock(&ptn5150_usb->ptn5150_wake_lock);
}

static void ptn5150_trysnk_work1(struct work_struct *work)
{
	//u8 reg;
	u8 Register_Value;
	static int rp_miss_count = 0;
        u8 reg = 0x03, reg1;
		t_printk(K_INFO, "%s begin\n", __func__);
	if (Try_Sink_State == Try_Sink_Attached_Wait_Src_Detached)
	{
		// Check if there is any Rp connected externally
		ptn5150_i2c_read(ptn5150_usb->client, &Register_Value, 0x49);
		t_printk(K_INFO, "%s ptn5150 reg 0x49 value = 0x%x\n", __func__, Register_Value);
		Register_Value &= 0x14;
		t_printk(K_INFO, "%s ptn5150 reg 0x49 value&0x14 = 0x%x\n", __func__, Register_Value);
		if ((Register_Value == 0x04) || (Register_Value == 0x10))
		{
		   //external has device attached and maybe in DRP mode
			rp_miss_count = 0;
			schedule_delayed_work(&ptn5150_usb->trysnk_work1, msecs_to_jiffies(10));
		}
		else if ((Register_Value == 0x14) || (Register_Value == 0x00))
		{
			rp_miss_count++;
			t_printk(K_INFO, "ptn5150 rp_miss_count = %d\n", rp_miss_count);
			if(rp_miss_count >= 3)
			{//finally external is rd or cable is removed
				rp_miss_count = 0;
				t_printk(K_INFO, "ptn5150 Try_Sink_State = %d, will do DRP mode\n", Try_Sink_State);
				ptn5150_i2c_read(ptn5150_usb->client, &reg1, 0x4A);
				t_printk(K_INFO, "%s ptn5150 reg 0x4A value = 0x%x\n", __func__, reg1);
				Try_Sink_State = Try_Sink_tDRPTry_Expire;
				ptn5150_i2c_write(ptn5150_usb->client, 0x02, &reg);
				Try_Sink_State = Try_Sink_Try_Wait_Src;
				//if no Cable external in 200ms ,excute trysnk_work2;
				//if UFP detected,an interrupt occures,cancel trysnk_work2
				t_printk(K_INFO, "%s schedule work2\n", __func__);
				schedule_delayed_work(&ptn5150_usb->trysnk_work2, msecs_to_jiffies(200));
			}
			else
			{
				t_printk(K_INFO, "ptn5150 rp_miss_count = %d schedule trysnd work1\n", rp_miss_count);
				schedule_delayed_work(&ptn5150_usb->trysnk_work1, msecs_to_jiffies(10));
			}
		}
	}
}

static void ptn5150_trysnk_work2(struct work_struct *work)
{
	u8 reg=0x15;
	u8 reg_val0=0x24;//aftung3, restore to default enable accessory detect
	u8 reg_val1=0xc0;//aftung3, restore to default
	t_printk(K_INFO, "%s ptn5150 Try_Sink_State =%d\n", __func__, Try_Sink_State);

	if (Try_Sink_State == Try_Sink_Try_Wait_Src)
	{
	      t_printk(K_INFO, "%s in work2\n", __func__);
		Try_Sink_State = Try_Sink_Try_Wait_Src_Expire;
		ptn5150_i2c_write(ptn5150_usb->client, 0x4c, &reg_val0 );	//aftung3
		ptn5150_i2c_write(ptn5150_usb->client, 0x43, &reg_val1);		//aftung3
		ptn5150_i2c_write(ptn5150_usb->client, 0x02, &reg);
		Try_Sink_State = Try_Sink_Idle_DRP;
	}
}

static void usb_free(struct ptn5150_usb_type_c *ptn5150_usb)
{
	gpio_free(ptn5150_usb->board_id1_gpio);
	gpio_free(ptn5150_usb->board_id2_gpio);
	gpio_free(ptn5150_usb->board_id3_gpio);
	gpio_free(ptn5150_usb->audio_swch_gpio);
	gpio_free(ptn5150_usb->hph_lr_swch_gpio);
	gpio_free(ptn5150_usb->vconn1_swch_gpio);
	gpio_free(ptn5150_usb->vconn2_swch_gpio);
}

static int ptn5150_usb_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int ret;
	struct power_supply *usb_psy;
	struct pinctrl *pinctrl;
	u8 reg_val0, reg_val1, reg_val2, reg_val3;
	u8 reg_0, reg_1, reg_2, reg_3;
	u8 device_id_0;
	int irqn;

	pr_info("%s: ptn5150_probe\n", __func__);
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&i2c->dev, "USB power_supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	ptn5150_usb = devm_kzalloc(&i2c->dev, sizeof(struct ptn5150_usb_type_c),
				GFP_KERNEL);
	if (!ptn5150_usb)
		return -ENOMEM;
#ifdef CONFIG_TYPE_C_INFO
	ptn5150_usb->at_bootup = true;
	ptn5150_usb->manu_set_DRP_mode = false;
	ptn5150_usb->manu_set_DFP_mode = false;
	ptn5150_usb->manu_set_UFP_mode = false;
	ptn5150_usb->manu_set_work = false;
	ptn5150_usb->manu_curr_mode = NXP_MANU_CURR_MODE_DEFAULT;
#endif
	i2c_set_clientdata(i2c, ptn5150_usb);
	ptn5150_usb->client = i2c;
	ptn5150_usb->usb_psy = usb_psy;

	if (i2c->dev.of_node) {
		ret = ptn5150_usb_parse_dt(&i2c->dev, ptn5150_usb);
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

	ret = ptn5150_usb_ldo_init(ptn5150_usb, true);
	if (ret) {
		dev_err(&ptn5150_usb->client->dev, "i2c ldo init failed\n");
		goto out;
	}

	ret = was4732d_ldo_init(ptn5150_usb, true);
	if (ret) {
		dev_err(&ptn5150_usb->client->dev, "was4732d ldo init failed\n");
		goto ptn5150_ldo_disable;
	}

	ret=ptn5150_i2c_read(ptn5150_usb->client, &device_id_0, 0x01);
	if (ret < 0) {
		dev_err(&ptn5150_usb->client->dev, "ptn5150 usb i2c read device id failed: %d\n", ret);
		goto was4732d_ldo_disable;
	} else {
		if(device_id_0!=0xb)
		{
			pr_err( "read device_id_0 fail ret=%d\n", ret);
			goto err_i2c;
		}
		else
			pr_info("device_id_0=0x%x\n", device_id_0);
	}

	ret = device_create_file(&i2c->dev, &dev_attr_ptn5150_reg);
	if (ret < 0) {
		dev_err(&i2c->dev, "failed to add ptn5150_reg sysfs files\n");
		goto was4732d_ldo_disable;
	}

	pinctrl = devm_pinctrl_get(&i2c->dev);
	if (IS_ERR(pinctrl)) {
		pr_err("%s: Unable to get pinctrl handle\n",
				__func__);
		return -EINVAL;
	}
	pinctrl_info.pinctrl = pinctrl;
	ret = ptn5150_usb_pinctrl_info_init(pinctrl);
	if (ret < 0) {
		pr_err("%s: failed to get the ptn5150 gpio's %d\n",
				__func__, ret);
		goto was4732d_ldo_disable;
	}

	if (gpio_is_valid(ptn5150_usb->irq_gpio)) {
		ret = gpio_request(ptn5150_usb->irq_gpio, "ptn5150_usb_enb_gpio");
		if (ret) {
		dev_err(&i2c->dev, "%s: unable to request gpio [%d]\n",
			__func__, ptn5150_usb->irq_gpio);
		goto was4732d_ldo_disable;
		}

		ret = pinctrl_select_state(pinctrl_info.pinctrl,
					pinctrl_info.usb_irq_act);
		if (ret < 0) {
			pr_err("failed to enable usb_irq_act gpio\n");
		}

		ret = gpio_direction_input(ptn5150_usb->irq_gpio);
		if (ret) {
			dev_err(&i2c->dev,
				"%s: unable to set direction for gpio [%d]\n",
				__func__, ptn5150_usb->irq_gpio);
			goto err_irq;
		}
		irqn = gpio_to_irq(ptn5150_usb->irq_gpio);
		if (irqn < 0) {
			ret = irqn;
			goto err_irq;
		}
		i2c->irq = irqn;
        } else {
            dev_err(&i2c->dev, "%s: irq gpio not provided\n", __func__);
            goto was4732d_ldo_disable;
        }

	ret = get_board_id(&i2c->dev, ptn5150_usb);
	if (ret < 0) {
		pr_err("%s: failed to get_board_id %d\n",
				__func__, ret);
		goto err_i2c;
	}
	ret = ptn5150_usb_ext_gpios_init(&i2c->dev, ptn5150_usb);
	if (ret < 0) {
		pr_err("%s: failed to vconn12_switch_gpio_init %d\n",
				__func__, ret);
		goto err_i2c;
	}

	// aftung: end of move

	INIT_DELAYED_WORK(&ptn5150_usb->eint_work, ptn5150_eint_work);
	INIT_DELAYED_WORK(&ptn5150_usb->trysnk_work1, ptn5150_trysnk_work1);
	INIT_DELAYED_WORK(&ptn5150_usb->trysnk_work2, ptn5150_trysnk_work2);
#ifdef CONFIG_TYPE_C_INFO
	ptn5150_usb->typec_set_notif.notifier_call=typec_set_notifier;
	ret = type_c_set_register_client(&ptn5150_usb->typec_set_notif);
	if (ret)
		dev_err(&i2c->dev, "Unable to register typec_set_notif: %d\n",ret);
	INIT_DELAYED_WORK(&ptn5150_usb->nxpusb_set_mode_dwork, nxpusb_manu_set_mode_work);
	spin_lock_init(&ptn5150_usb->irq_enabled_lock);
	wake_lock_init(&ptn5150_usb->ptn5150_wake_lock, WAKE_LOCK_SUSPEND, "ptn5150_wake_lock");
	mutex_init(&ptn5150_usb->ptn5150_lock);
	ptn5150_usb->at_bootup = false;
#endif

    ptn5150_usb->typec_reboot_notif.notifier_call=typec_reboot_notifier;
    ret = register_reboot_notifier(&ptn5150_usb->typec_reboot_notif);
    if (ret)
        dev_err(&i2c->dev, "Unable to register typec_reboot_notif: %d\n",ret);

	ret = devm_request_threaded_irq(&i2c->dev, i2c->irq, NULL, ptn5150_cc_eint_interrupt_handler,
					IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
					NXP_TYPE_C_I2C_NAME, ptn5150_usb);
	if (ret != 0)
	{
		pr_err("%s:EINT IRQ LINE NOT AVAILABLE-gyg\n", __func__);
		goto err_i2c;
	}
	else
	{
		pr_err("%s:ptn5150 set EINT finished, ptn5150_irq=%d\n", __func__, ptn5150_usb->client->irq);
	}

	pr_info("ptn5150 run here!\n");
	schedule_delayed_work(&ptn5150_usb->eint_work, 3 * HZ);

	dev_dbg(&i2c->dev, "%s finished, addr:%d\n", __func__, i2c->addr);
	is_nxp_type_c_register = true;
	ptn5150_i2c_read(ptn5150_usb->client, &reg_0, 0x04);
	reg_0 = (reg_0 >> 2) & 0x07;
	switch (reg_0)
	{
	case 0: // No Connect - you can do anything you like here, set to UFP, then set back to DRP
		   {
				  reg_val0 = 0x34;
				  reg_val1 = 0x40;
				  reg_val2 = 0x11;
				  reg_val3 = 0x00;
				  ptn5150_i2c_write(ptn5150_usb->client, 0x4c, &reg_val0);
				  ptn5150_i2c_write(ptn5150_usb->client, 0x43, &reg_val1);
				  ptn5150_i2c_write(ptn5150_usb->client, 0x02, &reg_val2);
				  ptn5150_i2c_write(ptn5150_usb->client, 0x18, &reg_val3);

				  reg_val0 = 0x24;
				  reg_val1 = 0xc0;
				  reg_val2 = 0x15;
				  reg_val3 = 0x00;
				  ptn5150_i2c_write(ptn5150_usb->client, 0x4c, &reg_val0);
				  ptn5150_i2c_write(ptn5150_usb->client, 0x43, &reg_val1);
				  ptn5150_i2c_write(ptn5150_usb->client, 0x02, &reg_val2);
				  ptn5150_i2c_write(ptn5150_usb->client, 0x18, &reg_val3);

				  ptn5150_i2c_read(ptn5150_usb->client, &reg_0, 0x4c);
				  t_printk(K_INFO, "ptn5150 0x4c register value = 0x%x\n", reg_0);
				  ptn5150_i2c_read(ptn5150_usb->client, &reg_1, 0x43);
				  t_printk(K_INFO, "ptn5150 0x43 register value = 0x%x\n", reg_1);
				  ptn5150_i2c_read(ptn5150_usb->client, &reg_2, 0x02);
				  t_printk(K_INFO, "ptn5150 0x02 register value = 0x%x\n", reg_2);
				  ptn5150_i2c_read(ptn5150_usb->client, &reg_3, 0x18);
				  t_printk(K_INFO, "ptn5150 0x18 register value = 0x%x\n", reg_3);
		   }
		   break;
	default:// if there is a connection already
		   {
				  ptn5150_i2c_read(ptn5150_usb->client, &reg_val2, 0x02);
				  reg_val2 |= 0x01;// mask the interrupt
				  reg_val0 = 0x24;
				  reg_val1 = 0xc0;
				  reg_val3 = 0x00;
				  ptn5150_i2c_write(ptn5150_usb->client, 0x4c, &reg_val0);
				  ptn5150_i2c_write(ptn5150_usb->client, 0x43, &reg_val1);
				  ptn5150_i2c_write(ptn5150_usb->client, 0x02, &reg_val2);
				  ptn5150_i2c_write(ptn5150_usb->client, 0x18, &reg_val3);
				  ptn5150_i2c_read(ptn5150_usb->client, &reg_0, 0x4c);
				  t_printk(K_INFO, "ptn5150 0x4c register value = 0x%x\n", reg_0);
				  ptn5150_i2c_read(ptn5150_usb->client, &reg_1, 0x43);
				  t_printk(K_INFO, "ptn5150 0x43 register value = 0x%x\n", reg_1);
				  ptn5150_i2c_read(ptn5150_usb->client, &reg_2, 0x02);
				  t_printk(K_INFO, "ptn5150 0x02 register value = 0x%x\n", reg_2);
				  ptn5150_i2c_read(ptn5150_usb->client, &reg_3, 0x18);
				  t_printk(K_INFO, "ptn5150 0x18 register value = 0x%x\n", reg_3);
		   }
		   break;
	}
	printk("ptn5150 end\n");

	return 0;

err_i2c:
	usb_free(ptn5150_usb);
err_irq:
	gpio_free(ptn5150_usb->irq_gpio);
was4732d_ldo_disable:
		was4732d_ldo_init(ptn5150_usb, false);
ptn5150_ldo_disable:
		ptn5150_usb_ldo_init(ptn5150_usb, false);
out:
	return ret;
}

static int ptn5150_usb_remove(struct i2c_client *i2c)
{
	struct ptn5150_usb_type_c *ptn5150_usb = i2c_get_clientdata(i2c);
	wake_lock_destroy(&ptn5150_usb->ptn5150_wake_lock);
#ifdef CONFIG_TYPE_C_INFO
	type_c_set_unregister_client(&ptn5150_usb->typec_set_notif);
#endif
    type_c_set_unregister_client(&ptn5150_usb->typec_reboot_notif);
	ptn5150_usb_ldo_init(ptn5150_usb, false);
	gpio_free(ptn5150_usb->irq_gpio);
	devm_kfree(&i2c->dev, ptn5150_usb);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ptn5150_i2c_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct ptn5150_usb_type_c *ptn5150_usb = i2c_get_clientdata(i2c);

	dev_dbg(dev, "ptn5150_usb PM suspend.. attach(%d)\n",
			ptn5150_usb->attach_state);
	disable_irq(ptn5150_usb->client->irq);
	/* Keep type-c chip enabled during session */
	if (ptn5150_usb->attach_state)
		return 0;

	regulator_set_voltage(ptn5150_usb->i2c_1p8, 0, USB_1P8_VOL_MAX);
	regulator_disable(ptn5150_usb->i2c_1p8);

	return 0;
}

static int ptn5150_i2c_resume(struct device *dev)
{
	int rc;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct ptn5150_usb_type_c *ptn5150_usb = i2c_get_clientdata(i2c);

	dev_dbg(dev, "ptn5150 PM resume\n");
	/* suspend was no-op, just re-enable interrupt */
	if (ptn5150_usb->attach_state) {
		enable_irq(ptn5150_usb->client->irq);
		return 0;
	}

	rc = regulator_set_voltage(ptn5150_usb->i2c_1p8, USB_1P8_VOL_MAX,
					USB_1P8_VOL_MAX);
	if (rc)
		dev_err(&ptn5150_usb->client->dev, "unable to set voltage(%d)\n", rc);

	rc = regulator_enable(ptn5150_usb->i2c_1p8);
	if (rc)
		dev_err(&ptn5150_usb->client->dev, "unable to enable 1p8-reg(%d)\n", rc);

	enable_irq(ptn5150_usb->client->irq);

	return rc;
}
#endif

static SIMPLE_DEV_PM_OPS(ptn5150_i2c_pm_ops, ptn5150_i2c_suspend,
			  ptn5150_i2c_resume);


static const struct i2c_device_id ptn5150_usb_id[] = {
	{ NXP_TYPE_C_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tiusb_id);

#ifdef CONFIG_OF
static const struct of_device_id ptn5150_usb_of_match[] = {
	{ .compatible = "nxp,usb-type-c", },
	{},
};
MODULE_DEVICE_TABLE(of, ptn5150_usb_of_match);
#endif

static struct i2c_driver ptn5150_usb_driver = {
	.driver = {
		.name = NXP_TYPE_C_I2C_NAME,
		.of_match_table = of_match_ptr(ptn5150_usb_of_match),
	},
	.probe		= ptn5150_usb_probe,
	.remove		= ptn5150_usb_remove,
	.id_table	= ptn5150_usb_id,
};

module_i2c_driver(ptn5150_usb_driver);

MODULE_DESCRIPTION("NXP TypeC Detection driver");
MODULE_LICENSE("GPL v2");
