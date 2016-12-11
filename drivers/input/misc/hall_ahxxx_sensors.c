/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/ratelimit.h>
#include <linux/cpuidle.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#if defined(CONFIG_HALL_NOTIFY)
#include <linux/hall_notify.h>
#endif
#include <linux/wakelock.h>
#include "hall_ahxxx_sensors.h"
#define HALL_VDD_MIN_UV 	1750000
#define HALL_VDD_MAX_UV	1950000

#define FIRST_HALL_WORK_DELAY 5000

static int hall_factory_flag=0;

struct hall_sensor_data *g_hall_sensor_pdata;
static u8  report_val=0;
struct hall_sensor_data *pdev_data;
static int sensor_platform_hw_power_on(bool on)
{
	int err;

	if (pdev_data == NULL)
		return -ENODEV;

	if (!IS_ERR_OR_NULL(pdev_data->pinctrl)) {
		if (on)
			err = pinctrl_select_state(pdev_data->pinctrl,
				pdev_data->pin_default);

		if (err)
			printk(KERN_ERR"Can't select pinctrl state\n");
	}

	return 0;
}
static int  hall_sensor_parse_dt(struct device *dev,
			struct hall_sensor_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int rc;
	pdata->power_on = sensor_platform_hw_power_on;
	dev_info(dev, "Detected %s(),parse device tree\n",__func__);

	rc = of_property_read_string(np, "label", &pdata->name);
	if (rc) {
		dev_err(dev, "Failed to read label\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(np, "qcom,debounce", &pdata->debounce);
	if (rc) {
		dev_err(dev, "Failed to read debounce time x\n");
		return -EINVAL;
	}

	pdata->irq_gpio = of_get_named_gpio(np,"qcom,irq-gpio", 0);


	return 0;
}

static int hall_request_input_dev(struct device *dev,struct hall_sensor_platform_data *pdata)
{
       int  err=0;
	pdata->input_dev = input_allocate_device();
	 if (pdata->input_dev == NULL) {
		dev_err(dev,
				"Failed to allocate input device.\n");
		return -ENOMEM;
	}
			pdata->input_dev->name = "hall_sensors";
			pdata->input_dev->id.vendor = 0x0001;
			pdata->input_dev->id.product = 0x0001;
			pdata->input_dev->id.version = 0x0010;

			set_bit(EV_SW, pdata->input_dev->evbit);
			set_bit(SW_LID, pdata->input_dev->swbit);

			err = input_register_device(pdata->input_dev);
			if (err < 0)
			{
		              dev_err(dev,
				       "input device regist failed.\n");
					  return -ENOMEM;
			}
			return 0;

}

static irqreturn_t hall_sensors_irq_handle(int irq, void *dev_id)
{

    disable_irq_nosync(g_hall_sensor_pdata->pdata->irq);
    wake_lock_timeout(&(g_hall_sensor_pdata->pdata->hall_lock), 2*HZ);
    schedule_delayed_work(&g_hall_sensor_pdata->pdata->work,0);
    return IRQ_HANDLED;

}

static void hall_sensors_work_func(struct work_struct *work)
{
		static int   gpio_val;
		static int  delay_count=0;

#if defined(CONFIG_HALL_NOTIFY)
	struct hall_event event;
#endif
		gpio_val = gpio_get_value(g_hall_sensor_pdata->pdata->irq_gpio);
	 	msleep(10);
		if(delay_count<g_hall_sensor_pdata->pdata->debounce/100)
	      {
		    if(gpio_val !=  gpio_get_value(g_hall_sensor_pdata->pdata->irq_gpio))
		    {
                       delay_count=0;
			  printk("%s(),hall sensor status changed,so do not report key value\n",__func__);
		         enable_irq(g_hall_sensor_pdata->pdata->irq);
		    }
		    else
		    {
			  msleep(10);
			  if(gpio_val !=  gpio_get_value(g_hall_sensor_pdata->pdata->irq_gpio))
		  	  {
		  	      delay_count = 0;
			      enable_irq(g_hall_sensor_pdata->pdata->irq);

		  	  }
			  else
			  {
					delay_count++;
					schedule_delayed_work(&g_hall_sensor_pdata->pdata->work,0);
			  }
		    }
		    return;
            }

              report_val=!gpio_val;
		if(report_val == g_hall_sensor_pdata->last_report_val)
		{
			enable_irq(g_hall_sensor_pdata->pdata->irq);
			delay_count = 0;
			return;
		}
		else
			g_hall_sensor_pdata->last_report_val = report_val;

		if(gpio_val)
		{
			input_report_switch(g_hall_sensor_pdata->pdata->input_dev, SW_LID, report_val);
			input_sync(g_hall_sensor_pdata->pdata->input_dev);
		}
		else
		{
			input_report_switch(g_hall_sensor_pdata->pdata->input_dev, SW_LID, report_val);
			input_sync(g_hall_sensor_pdata->pdata->input_dev);
		}
			printk(KERN_ERR"hall sensors %s,report_val %d\n",gpio_val?"far away":"near by",report_val);
			if(1==hall_factory_flag)
			{
					enable_irq(g_hall_sensor_pdata->pdata->irq);
					delay_count = 0;
					return ;
			}
#if defined(CONFIG_HALL_NOTIFY)
			else
			{
				event.data = &report_val;
				ret = hall_notifier_call_chain(HALL_EVENT_REPORT, &event);
			}
#endif
		enable_irq(g_hall_sensor_pdata->pdata->irq);
		delay_count = 0;

}
static ssize_t hall_show_gpio_value(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	static int   gpio_val;
	gpio_val = gpio_get_value(g_hall_sensor_pdata->pdata->irq_gpio);
	return sprintf(buf, "%d\n", gpio_val);
}

static DEVICE_ATTR(hall_gpio_value,0664,hall_show_gpio_value,NULL);

static void hall_sensors_work_test(struct work_struct *work)
{
	printk(KERN_ERR"hall_sensor: run in test now! report value = %d\n",g_hall_sensor_pdata->pdata->run_rpt_val);
	input_report_switch(g_hall_sensor_pdata->pdata->input_dev, SW_LID,g_hall_sensor_pdata->pdata->run_rpt_val);
	input_sync(g_hall_sensor_pdata->pdata->input_dev);
	g_hall_sensor_pdata->pdata->run_rpt_val = !(g_hall_sensor_pdata->pdata->run_rpt_val);
	schedule_delayed_work(&g_hall_sensor_pdata->pdata->test_work,msecs_to_jiffies(g_hall_sensor_pdata->pdata->delay_msc));
}


static ssize_t hall_store_runin_test(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	g_hall_sensor_pdata->pdata->run_rpt_val = 1;

	if(!val)
	{
		cancel_delayed_work(&g_hall_sensor_pdata->pdata->test_work);
	}
	else
	{
		g_hall_sensor_pdata->pdata->delay_msc = val*1000;
		schedule_delayed_work(&g_hall_sensor_pdata->pdata->test_work,msecs_to_jiffies(g_hall_sensor_pdata->pdata->delay_msc));
	}
	return count;
}
static ssize_t hall_store_factory(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);
	hall_factory_flag=val;

	return count;
}
static DEVICE_ATTR(hall_run_in_test,0664,NULL,hall_store_runin_test);
static DEVICE_ATTR(hall_factory,0664,NULL,hall_store_factory);


static struct attribute *hall_sensor_attributes[] = {
	&dev_attr_hall_gpio_value.attr,
	&dev_attr_hall_run_in_test.attr,
	&dev_attr_hall_factory.attr,
	NULL
};

static const struct attribute_group hall_sensor_attr_group = {
	.attrs = hall_sensor_attributes,
};
static int hall_pinctrl_init(struct device *dev,struct hall_sensor_data *data)
{

	data->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		dev_err(dev, "Failed to get pinctrl\n");
		return PTR_ERR(data->pinctrl);
	}

	data->pin_default =
		pinctrl_lookup_state(data->pinctrl, "default");
	if (IS_ERR_OR_NULL(data->pin_default)) {
		dev_err(dev, "Failed to look up default state\n");
		return PTR_ERR(data->pin_default);
	}
	return 0;
}
static int hall_power_init(struct device *dev, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(g_hall_sensor_pdata->vdd) > 0)
			regulator_set_voltage(g_hall_sensor_pdata->vdd, 0, HALL_VDD_MAX_UV);

		regulator_put(g_hall_sensor_pdata->vdd);

	} else {
		g_hall_sensor_pdata->vdd = regulator_get(dev, "vdd");
		if (IS_ERR(g_hall_sensor_pdata->vdd)) {
			rc = PTR_ERR(g_hall_sensor_pdata->vdd);
			printk(KERN_ERR"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(g_hall_sensor_pdata->vdd) > 0) {
			rc = regulator_set_voltage(g_hall_sensor_pdata->vdd, HALL_VDD_MIN_UV,
						   HALL_VDD_MAX_UV);
			if (rc) {
				printk(KERN_ERR"Regulator set failed vdd rc=%d\n",rc);
				goto reg_vdd_put;
			}
		}
	}

	return 0;

reg_vdd_put:
	regulator_put(g_hall_sensor_pdata->vdd);
	return rc;
}
static int hall_power_on_off(struct device *dev, bool on)
{
	int rc = 0;
	if (!on) {
		rc = regulator_disable(g_hall_sensor_pdata->vdd);
		if (rc) {
			dev_err(dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}

	} else {
		rc = regulator_enable(g_hall_sensor_pdata->vdd);
		if (rc) {
			dev_err(dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}

	}

	msleep(10);
	return rc;
}
static int hall_sensor_probe(struct platform_device *pdev)
{
	int ret;
	struct hall_sensor_data *hall_sensor;
	static int   gpio_val;

	hall_sensor = kzalloc(sizeof(struct hall_sensor_data), GFP_KERNEL);
	if (!hall_sensor) {
		dev_err(&pdev->dev, "hall sensor not enough memory for ts\n");
		return -ENOMEM;
	}

	pdev_data = hall_sensor;
	if (pdev->dev.of_node) {
		hall_sensor->pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct hall_sensor_platform_data), GFP_KERNEL);
		if (!hall_sensor->pdata) {
			dev_err(&pdev->dev,
				"hall sensor Failed to allocate memory for pdata\n");
			return -ENOMEM;
		}

		ret = hall_sensor_parse_dt(&pdev->dev, hall_sensor->pdata);
		if (ret)
			return ret;
	} else {
		hall_sensor->pdata = pdev->dev.platform_data;
	}

	g_hall_sensor_pdata=hall_sensor;

	ret = hall_request_input_dev(&pdev->dev,hall_sensor->pdata);
	if (ret) {
		dev_err(&pdev->dev, "hall sensor request input dev failed.\n");
		return ret;
	}
	ret = hall_power_init(&pdev->dev, true);
	if (ret < 0) {
		dev_err(&pdev->dev, "power init failed! err=%d", ret);
	}
	ret = hall_power_on_off(&pdev->dev, true);
	if (ret < 0) {
		dev_err(&pdev->dev, "power on failed! err=%d\n", ret);
		hall_power_init(&pdev->dev, false);
	}
	ret= sysfs_create_group(&g_hall_sensor_pdata->pdata->input_dev->dev.kobj, &hall_sensor_attr_group);
	if (ret)
	{
		printk("%s sysfs_create_groupX\n", __func__);
		goto failed;
	}
       INIT_DELAYED_WORK(&(g_hall_sensor_pdata->pdata->work), hall_sensors_work_func);
	INIT_DELAYED_WORK(&(g_hall_sensor_pdata->pdata->test_work),hall_sensors_work_test);
	wake_lock_init(&(hall_sensor->pdata->hall_lock),WAKE_LOCK_SUSPEND,"hall wakelock");
    ret = hall_pinctrl_init(&pdev->dev,hall_sensor);
	if (ret)
	{
		dev_err(&pdev->dev, "Can't initialize pinctrl\n");
		goto failed;
	}
	ret = pinctrl_select_state(hall_sensor->pinctrl, hall_sensor->pin_default);
	if (ret)
	{
		dev_err(&pdev->dev,
			"Can't select pinctrl default state\n");
		goto failed;
	}
       ret =gpio_request(hall_sensor->pdata->irq_gpio,"hall-sensor-irq-gpio");
       if(ret<0)
		{
			dev_err(&pdev->dev,"hall sensor request gpio%d failed\n",hall_sensor->pdata->irq_gpio);
			goto   failed;
		}

       ret =gpio_direction_input(hall_sensor->pdata->irq_gpio);
       if(ret<0)
		{
			dev_err(&pdev->dev,"hall sensor gpio%d input direction \n",hall_sensor->pdata->irq_gpio);
		}
      dev_err(&pdev->dev,"hall sensor1 gpio%d input direction \n",hall_sensor->pdata->irq_gpio);

       hall_sensor->pdata->irq=gpio_to_irq(hall_sensor->pdata->irq_gpio);

		ret= hall_sensor->pdata->power_on(true);
      // ret =request_threaded_irq(hall_sensor->pdata->irq,NULL,hall_sensors_irq_handle,IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,"hall_sensors",NULL);
       ret = request_irq(hall_sensor->pdata->irq,hall_sensors_irq_handle, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING| IRQF_ONESHOT,
			 "hall_sensors",NULL);
       if(ret<0)
    {
            dev_err(&pdev->dev,"hall sensor gpio%d request threaded irq failed\n",hall_sensor->pdata->irq_gpio);
    }

	ret =enable_irq_wake(hall_sensor->pdata->irq);
       if(ret<0)
    {
            dev_err(&pdev->dev,"hall sensor gpio%d request threaded irq failed\n",hall_sensor->pdata->irq_gpio);
    }
	gpio_val = gpio_get_value(hall_sensor->pdata->irq_gpio);

	if(gpio_val == 0)
	{
        disable_irq(hall_sensor->pdata->irq);
              schedule_delayed_work(&g_hall_sensor_pdata->pdata->work,msecs_to_jiffies(FIRST_HALL_WORK_DELAY));
		printk(KERN_ERR"hall_sensor:report in %s,hall sensor gpio value is %d",__func__,gpio_val);
	}
	return 0;
failed:
	gpio_free(hall_sensor->pdata->irq_gpio);
	if (hall_sensor->pdata->power_on)
		hall_sensor->pdata->power_on(false);
	kfree(hall_sensor);
	pdev_data = NULL;
	return 0;
}
static int hall_sensor_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&g_hall_sensor_pdata->pdata->input_dev->dev.kobj, &hall_sensor_attr_group);
	return 0;
}
static int hall_sensor_suspend(struct platform_device *pdev,pm_message_t mesg)
{
	return 0;
}

static int hall_sensor_resume(struct platform_device *pdev)
{
	return 0;
}
static struct of_device_id hall_sensor_match_table[] = {
	{ .compatible = "qcom,hall-sensor",},
	{ },
};

static struct platform_driver hall_sensor_driver = {
	.probe = hall_sensor_probe,
	.remove = hall_sensor_remove,
	.suspend = hall_sensor_suspend,
	.resume	= hall_sensor_resume,
	.driver = {
		.owner = THIS_MODULE,
		.name = "hall_sensor",
		.of_match_table = hall_sensor_match_table,
	},
};
module_platform_driver(hall_sensor_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Hall Sensor Driver");
