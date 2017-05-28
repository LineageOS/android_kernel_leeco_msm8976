/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fb.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/of_device.h>
#include <linux/type-c_notify.h>
#include "type-c-info.h"
#include "type-c-ti.h"
#include "type-c-nxp.h"
#include <linux/usb.h>

#define DEVICE_NAME  "type-c-info"

extern int usb_for_each_dev(void *data, int (*fn)(struct usb_device *, void *));

#define LETV_USB_AUDIO_VID  0x262A
#define LETV_USB_AUDIO_PID_0  0x1530
#define LETV_USB_AUDIO_PID_1  0x1532
#define LETV_USB_AUDIO_PID_2  0x1534

static const struct usb_device_id letv_usb_audio_id[] = {
	{ USB_DEVICE(LETV_USB_AUDIO_VID, LETV_USB_AUDIO_PID_0) },
	{ USB_DEVICE(LETV_USB_AUDIO_VID, LETV_USB_AUDIO_PID_1) },
	{ USB_DEVICE(LETV_USB_AUDIO_VID, LETV_USB_AUDIO_PID_2) },
	{ } /* Terminating entry */
};
int headset_vbus_status = 0;
static struct typec_info_data *typec_info;

static ssize_t orientation_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
    return size;
}

static ssize_t orientation_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    int len = 0;

    len += sprintf(buf + len, "%d", typec_info->typec_dir);
    return  len;
}
static ssize_t vbus_status_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
 unsigned int data;
 sscanf(buf, "%d", &data);
 printk("%s:data = %d\n",__func__,data);
 if(data <= 0){
	headset_vbus_status = 0;
 }else{
	headset_vbus_status = 1;
 }
    return size;
}

static ssize_t vbus_status_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    int len = 0;
    len += sprintf(buf + len, "%d", headset_vbus_status);
    return  len;
}
static ssize_t ready_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
    unsigned int data;
    struct type_c_event_data notif_data;
    sscanf(buf, "%d", &data);
    printk("%s:data = %d\n",__func__,data);
    typec_info->typec_notif_ready=data;
    notif_data.data=&data;
    type_c_set_notifier_call_chain(TYPE_C_INFO_NOTIFY_READY,&notif_data);
    return size;
}

static ssize_t ready_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    int len = 0;

    len += sprintf(buf + len, "%d", typec_info->typec_notif_ready);
    return  len;
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    int len = 0;

    len += sprintf(buf + len, "%d", typec_info->typec_mode);
    return  len;
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
    unsigned int data;
    struct type_c_event_data notif_data;
    sscanf(buf, "%d", &data);
    printk("%s:data = %d\n",__func__,data);
    typec_info->typec_mode=data;
    notif_data.data=&data;
    type_c_set_notifier_call_chain(TYPE_C_SET_MODE,&notif_data);
    return size;
}
static ssize_t result_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
    return size;
}

static ssize_t result_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    int len = 0;

    len += sprintf(buf + len, "%d", typec_info->typec_mode_result);
    return  len;
}
static DEVICE_ATTR(vbus_status, 0644, vbus_status_show, vbus_status_store);
static DEVICE_ATTR(orientation, 0644, orientation_show, orientation_store);
static DEVICE_ATTR(mode, 0664, mode_show, mode_store);
static DEVICE_ATTR(result, 0644, result_show, result_store);
static DEVICE_ATTR(ready, 0664, ready_show, ready_store);

static int check_letv_usb_dev(struct usb_device *udev, void *data)
{
	bool ret;

	if (!udev || !udev->config ||!udev->parent)
		return 0;

	usb_lock_device(udev);
	if (udev->actconfig && udev->actconfig->interface[0] &&
		udev->actconfig->interface[0]->unregistering)
		//This device has already disconnected
		goto out;

	ret = ((udev->parent->parent == NULL) && (udev->config->interface[0]) &&
		   (usb_match_id(udev->config->interface[0], letv_usb_audio_id)));

	if (ret)
		data = (void *)udev;
out:
	usb_unlock_device(udev);
	return ret;
}

/**
* @data: reserved for future usage
* Iterate over all USB devices. If there is only one Letv usb
* audio device on bus returns 1 otherwise returns 0
*/
int letv_audio_mode_supported(void *data)
{
	int ret;
	ret = usb_for_each_dev(data, check_letv_usb_dev);
	printk("%s:+++++ret=0x%x\n", __func__, ret);
	return ret;
}

static ssize_t letv_supported_dev_show(struct device *pdev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
			letv_audio_mode_supported(NULL));
}

void cclogic_set_audio_mode(bool mode)
{
	if ((mode == TYPEC_AUDIO_ANALOG) && !letv_audio_mode_supported(NULL))
		return;

	/*true for Analog mode and false for Digit mode*/
	if (is_nxp_type_c_register) {
		ptn5150_usb->typec_audio_mode = mode;
	}
	if (is_ti_type_c_register) {
		ti_usb->typec_audio_mode = mode;
	}
	return;
}

int cclogic_get_audio_mode(void)
{
	bool audio_mode;
	if (is_nxp_type_c_register) {
		audio_mode = ptn5150_usb->typec_audio_mode;
	}
	if (is_ti_type_c_register) {
		audio_mode = ti_usb->typec_audio_mode;
	}
	return audio_mode;
}

static DEVICE_ATTR(supported_dev, 0444, letv_supported_dev_show, NULL);

static int typec_get_notifier(struct notifier_block *self,unsigned long action, void *data);
static struct notifier_block typec_get_notif = {
    .notifier_call = typec_get_notifier,
};

static int typec_get_notifier(struct notifier_block *self,unsigned long action, void *data)
{
    struct type_c_event_data *evdata = data;
    u8 *dir;
    u8 *mode_result;
    if (evdata && evdata->data && action == TYPE_C_GET_ORIENTATION)
    {
        if(!typec_info){
            pr_err("typec_info is null %s\n", __func__);
        }else{
            dir=evdata->data;
            typec_info->typec_dir=*dir;
        }
        printk("%s:usb typec %s insert\n",__func__,(!typec_info->typec_dir)?"top":"bottom");
    }
    if (evdata && evdata->data && action == TYPE_C_INFO_MODE_RESULT)
    {
        if(!typec_info){
            pr_err("typec_info is null %s\n", __func__);
        }else{
            mode_result=evdata->data;
            typec_info->typec_mode_result=*mode_result;
        }
        printk("%s:mode set result %s\n",__func__,(!typec_info->typec_mode_result)?"DFP":"default");
    }
    return NOTIFY_OK;
}

static int typec_info_dev_init(void)
{
    struct device *typec_info_dev;
    struct class *typec_info_class;
    int rc;

    typec_info = kzalloc(sizeof(struct typec_info_data), GFP_KERNEL);
    if (NULL == typec_info)
    {
        pr_err("Failed to data allocate %s\n", __func__);
        rc = -ENOMEM;
        goto err_free_mem;
    }
    typec_info->typec_mode = 1;//default mode.
    typec_info->typec_mode_result = 2;//default mode.
    typec_info_class = class_create(THIS_MODULE, "type-c");
    if (IS_ERR(typec_info_class))
    {
        pr_err("Failed to create typec-info class!\n");
        return PTR_ERR(typec_info_class);
    }
    typec_info_dev = device_create(typec_info_class, NULL, 0, typec_info, "info");
    if (IS_ERR(typec_info_dev))
        pr_err("Failed to create typec_info_dev device\n");
    if (device_create_file(typec_info_dev, &dev_attr_orientation) < 0)
        pr_err("Failed to create device file(%s)!\n",dev_attr_orientation.attr.name);
    if (device_create_file(typec_info_dev, &dev_attr_mode) < 0)
        pr_err("Failed to create device file(%s)!\n",dev_attr_mode.attr.name);
    if (device_create_file(typec_info_dev, &dev_attr_ready) < 0)
        pr_err("Failed to create device file(%s)!\n",dev_attr_mode.attr.name);

    if (device_create_file(typec_info_dev, &dev_attr_result) < 0)
        pr_err("Failed to create device file(%s)!\n",dev_attr_result.attr.name);
    if (device_create_file(typec_info_dev, &dev_attr_supported_dev) < 0)
        pr_err("Failed to create device file(%s)!\n",dev_attr_result.attr.name);
    if (device_create_file(typec_info_dev, &dev_attr_vbus_status) < 0)
        pr_err("Failed to create device file(%s)!\n",dev_attr_result.attr.name);
    rc = type_c_get_register_client(&typec_get_notif);
    if (rc)
        pr_err("Unable to register typec_get_notif: %d\n",rc);

err_free_mem:
    kfree(typec_info);

    return rc;
}
static int  typec_info_probe(struct platform_device *pdev)
{
    int ret;

    if (pdev->dev.of_node)
    {
        typec_info = devm_kzalloc(&pdev->dev,sizeof(struct typec_info_data), GFP_KERNEL);
        if (!typec_info)
        {
            dev_err(&pdev->dev,"typec_info Failed to allocate memory for pdata\n");
            return -ENOMEM;
        }
    }

    ret = typec_info_dev_init();
    if (ret)
    {
        dev_err(&pdev->dev,"typec_info_dev_init failed! ret = %d\n",ret);
        return ret;
    }
    return 0;
}
static struct of_device_id typec_info_match_table[] = {
    {.compatible = "type-c-info"},
    {}
};
static struct platform_driver typec_info_driver = {
    .probe = typec_info_probe,
    .driver = {
        .name = DEVICE_NAME,
        .of_match_table = typec_info_match_table,
    },
};

static int __init typec_info_init(void)
{
    return platform_driver_register(&typec_info_driver);
}

static void __exit typec_info_exit(void)
{
    platform_driver_unregister(&typec_info_driver);
}

module_init(typec_info_init);
module_exit(typec_info_exit);

MODULE_DEVICE_TABLE(usb, letv_usb_audio_id);
MODULE_DESCRIPTION("USB Type-c info Driver");
MODULE_LICENSE("GPL v2");
