/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/of_device.h>

#include "maxq656.h"

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define DEVICE_NAME				"irremote_maxq656"
#define SYSFS_REMOTE_PATH   "remote"
static struct ir_remocon_data	*ir_data;

struct class *maxq656_class;
static int power_supply_gpio = 1003;
static int maxq656_remote_open(struct inode *inode, struct file *file) {


    printk("maxq656---open OK---\n");
    return 0;
}

static int maxq656_remote_close(struct inode *inode, struct file *file) {


    printk("maxq656---close---\n");
	return 0;
}


static struct file_operations maxq656_remote_ops = {
	.owner			= THIS_MODULE,
	.open			= maxq656_remote_open,
	.release		= maxq656_remote_close,
#if 0
	.write          = ET_remote_write,
	.read			=	ET_remote_read,
	.poll			= 	ET_remote_poll,
	.unlocked_ioctl	=	ET_remote_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ET_remote_ioctl,
#endif
#endif

};
static ssize_t power_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{

	unsigned int data;
    sscanf(buf, "%d", &data);
    printk("%s:data = %d\n",__func__,data);
	if(data <= 0){
		gpio_set_value(power_supply_gpio, 0);
	}else{
	 	gpio_set_value(power_supply_gpio, 1);
	}
	return count;
}

static ssize_t power_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	int len = 0;
	char buf_ir_test=0;
    buf_ir_test = gpio_get_value(power_supply_gpio);
	len += sprintf(buf + len, "%d", buf_ir_test);
	return  len;
}
static struct kobj_attribute maxq656_attrs[] = {
	__ATTR(enable, (S_IRUGO | S_IWUSR | S_IWGRP),
					power_show,
					power_store),
};

static struct miscdevice maxq656_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &maxq656_remote_ops,
};

static int maxq656_gpio_init(void)
{
	int ret;
	ret = gpio_request(ir_data->maxq656_power_supply, "maxq656_power_supply");
	if (ret) {
		pr_err("request maxq656_power_supply(%d) gpio failed, rc=%d\n", ir_data->maxq656_power_supply, ret);
	}else{
		ret = gpio_direction_output(ir_data->maxq656_power_supply, 1);
	    if (ret) {
	            pr_err("%s: Set direction for maxq656_power_supply failed, ret=%d\n",
	                            __func__, ret);
	    }

	}
	return 0;
}
static int maxq656_dev_init(void)
{
	//struct device *ir_remocon_dev;
	int rc;
	ir_data = kzalloc(sizeof(struct ir_remocon_data), GFP_KERNEL);
	if (NULL == ir_data) {
		printk("Failed to data allocate %s\n", __func__);
		rc = -ENOMEM;
		goto err_free_mem;
	}
	ir_data->maxq656_obj = kobject_create_and_add(SYSFS_REMOTE_PATH, NULL);
	if (!ir_data->maxq656_obj) {
		printk("Failed to create maxq656_obj\n");
		rc = -ENOMEM;
		goto err_free_mem;
	}
	rc = sysfs_create_file(ir_data->maxq656_obj,&maxq656_attrs[0].attr);
	if (rc) {
		printk("Failed to create maxq656 device file!\n");
		rc = -ENOMEM;
		goto err_create_sysfs;
	}
	if((rc = misc_register(&maxq656_misc_dev)))
	{
		printk("maxq656: misc_register register failed\n");
		rc = -ENOMEM;
		goto err_misc_register;
	}
	return 0;
err_misc_register:
		misc_deregister(&maxq656_misc_dev);
err_create_sysfs:
		sysfs_remove_file(ir_data->maxq656_obj, &maxq656_attrs[0].attr);
err_free_mem:
		kfree(ir_data);

	return rc;
}
static int  maxq656_misc_dev_probe(struct platform_device *pdev) {

	int ret;
	if (pdev->dev.of_node) {
	ir_data = devm_kzalloc(&pdev->dev,
		sizeof(struct ir_remocon_data), GFP_KERNEL);
			if (!ir_data) {
				dev_err(&pdev->dev,
					"maxq656 Failed to allocate memory for pdata\n");
				return -ENOMEM;
			}
    }
	ir_data->maxq656_power_supply = of_get_named_gpio(pdev->dev.of_node,"max656,power-gpio", 0);
	power_supply_gpio = ir_data->maxq656_power_supply;
	if (!gpio_is_valid(ir_data->maxq656_power_supply)) {
		pr_err("%s: maxq656 power gpio not specified\n", __func__);
	}else{

	ret = maxq656_gpio_init();
		if (ret){
			pr_err("maxq656_gpio_init failed! ret = %d\n",ret);
			return ret;
		}
	}

	ret = maxq656_dev_init();
	if (ret){
		pr_err("maxq656_dev_init failed! ret = %d\n",ret);
		return ret;
	}

	return 0;
}
static struct of_device_id maxq656_misc_dev_match_table[] = {
	{.compatible = "max,maxq656"},
	{}
};
static struct platform_driver maxq656_misc_dev_driver = {
	.probe = maxq656_misc_dev_probe,
	.driver = {
		.name = "max-maxq656",
		.of_match_table = maxq656_misc_dev_match_table,
	},
};

static int __init maxq656_remote_dev_init(void)
{
        printk("%s:E\n",__func__);
	return platform_driver_register(&maxq656_misc_dev_driver);
}

static void __exit maxq656_remote_dev_exit(void)
{
	platform_driver_unregister(&maxq656_misc_dev_driver);
}

module_init(maxq656_remote_dev_init);
module_exit(maxq656_remote_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("maxq656 Inc.");
MODULE_DESCRIPTION("maxq656 consumerir  Driver");

