/* Copyright (c) 2012-2012, OEM Telecom. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <asm/system_misc.h>

#include <asm/hardware_info.h>

#include <../../net/wireless/wcnss/detect_wcnss_fem.h>

static struct platform_device *hardware_info_pdev = NULL;
static DEFINE_MUTEX(registration_lock);


#define HARDWARE_INFO_ATTR(name)\
	static ssize_t name##_show(struct device *dev, struct device_attribute *attr, char *buf)\
	{\
		struct hardware_info_pdata *hardware_info = dev_get_drvdata(dev);\
		if(hardware_info->name==NULL)\
			snprintf(hardware_info->name,125,"sorry,not detected");\
		return sprintf(buf,  "%s\n", hardware_info->name);\
	}\
	static ssize_t name##_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)\
	{\
		struct hardware_info_pdata *hardware_info = dev_get_drvdata(dev);\
		snprintf(hardware_info->name,125, buf);\
		return count;\
	}\
	static DEVICE_ATTR(name, 0664, name##_show, name##_store)

	HARDWARE_INFO_ATTR(lcm);
	HARDWARE_INFO_ATTR(ctp);
	HARDWARE_INFO_ATTR(main_cam);
	HARDWARE_INFO_ATTR(sub_cam);
	HARDWARE_INFO_ATTR(flash);
	HARDWARE_INFO_ATTR(gsensor);
	HARDWARE_INFO_ATTR(msensor);
	HARDWARE_INFO_ATTR(als_ps);
	HARDWARE_INFO_ATTR(gyro);
	HARDWARE_INFO_ATTR(emcp);
	HARDWARE_INFO_ATTR(rom);
	HARDWARE_INFO_ATTR(ram);
	HARDWARE_INFO_ATTR(nfc);
	HARDWARE_INFO_ATTR(wifi);
	HARDWARE_INFO_ATTR(bluetooth);
	HARDWARE_INFO_ATTR(board_id);


static struct hardware_info_pdata hardware_info_data =
{
	.lcm = "lcm",
	.ctp = "ctp",
	.main_cam = "main_cam",
	.sub_cam = "sub_cam",
	.flash = "flash",
	.gsensor = "gsensor",
	.msensor = "msensor",
	.als_ps = "als_ps",
	.gyro = "gyro",
	.emcp = "emcp",
	.rom = "rom",
	.ram = "ram",
	.nfc = "nfc",
	.wifi="wifi",
	.bluetooth="bluetooth",
	.bluetooth="board_id",
};

static int hardware_info_probe(struct platform_device *pdev)
{

	struct hardware_info_pdata *hardware_pdata;
	hardware_pdata = pdev->dev.platform_data;
	platform_set_drvdata(pdev, hardware_pdata);
	hardware_info_pdev = pdev;
	register_hardware_info(GSENSOR, "sorry,gsensor not detected");
	register_hardware_info(MSENSOR, "sorry,msensor not detected");
	register_hardware_info(ALS_PS, "sorry,als and ps not detected");
	register_hardware_info(GYRO, "sorry,gyro not detected");
	register_hardware_info(FLASH, "PHILIPS Dual LED LXCL-SW08/SA08-0001 1A");
	return 0;
}

static int hardware_info_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef COMPATIBLE_WCNSS_NV
static char board_id_buf[BOARD_ID_SIZE+1];

void obtain_hw_board_id(char *buf)
{
	strncpy(buf, board_id_buf, BOARD_ID_SIZE);
}
EXPORT_SYMBOL(obtain_hw_board_id);
#endif

int register_hardware_info(HARDWARE_ID id, char* name)
{
	int ret=0;

	struct hardware_info_pdata *hardware_info;

	if(NULL == hardware_info_pdev){
		return -1;
	}
	printk("%s:hardware id=%d, name=%s\n", __func__, id, name);
	hardware_info = platform_get_drvdata(hardware_info_pdev);
	mutex_lock(&registration_lock);
	switch(id){
	case LCM:
		snprintf(hardware_info->lcm,125, name);
		ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_lcm);
		if (ret) {
			printk("%s:error on device_create_file %s\n", __func__, name);
		}
		break;
	case CTP:
		snprintf(hardware_info->ctp,125,name);
		ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_ctp);
		if (ret) {
			printk("%s:error on device_create_file %s\n", __func__, name);
		}
		break;
	case MAIN_CAM:
		snprintf(hardware_info->main_cam,125, name);
		ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_main_cam);
		if (ret) {
			printk("%s:error on device_create_file %s\n", __func__, name);
		}
		break;
	case SUB_CAM:
		snprintf(hardware_info->sub_cam, 125,name);
		ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_sub_cam);
		if (ret) {
			printk("%s:error on device_create_file %s\n", __func__, name);
		}
		break;
	case FLASH:
		snprintf(hardware_info->flash,125,name);
		ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_flash);
		if (ret) {
			printk("%s:error on device_create_file %s\n", __func__, name);
		}
		break;
	case GSENSOR:
		snprintf(hardware_info->gsensor, 125,name);
		ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_gsensor);
		if (ret) {
			printk("%s:error on device_create_file %s\n", __func__, name);
		}
		break;
	case MSENSOR:
		snprintf(hardware_info->msensor,125,name);
		ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_msensor);
		if (ret) {
			printk("%s:error on device_create_file %s\n", __func__, name);
		}
		break;
	case ALS_PS:
		snprintf(hardware_info->als_ps, 125,name);
		ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_als_ps);
		if (ret) {
			printk("%s:error on device_create_file %s\n", __func__, name);
		}
		break;
	case GYRO:
		snprintf(hardware_info->gyro, 125,name);
		ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_gyro);
		if (ret) {
			printk("%s:error on device_create_file %s\n", __func__, name);
		}
		break;
	case EMCP:
		snprintf(hardware_info->emcp,125, name);
		ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_emcp);
		if (ret) {
			printk("%s:error on device_create_file %s\n", __func__, name);
		}
		break;
	case ROM:
		snprintf(hardware_info->rom, 125,name);
		ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_rom);
		if (ret) {
			printk("%s:error on device_create_file %s\n", __func__, name);
		}
		break;
	case RAM:
		snprintf(hardware_info->ram, 125,name);
		ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_ram);
		if (ret) {
			printk("%s:error on device_create_file %s\n", __func__, name);
		}
		break;
	case NFC:
		snprintf(hardware_info->nfc, 125,name);
		ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_nfc);
		if (ret) {
			printk("%s:error on device_create_file %s\n", __func__, name);
		}
		break;
	case WIFI:
		snprintf(hardware_info->wifi, 125,name);
		ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_wifi);
		if (ret) {
			printk("%s:error on device_create_file %s\n", __func__, name);
		}
		break;
	case BLUETOOTH:
			snprintf(hardware_info->bluetooth, 125,name);
			ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_bluetooth);
			if (ret) {
			  printk("%s:error on device_create_file %s\n", __func__, name);
			}
		break;
	case BOARD_ID:
			snprintf(hardware_info->board_id, 125,name);
			ret = device_create_file(&hardware_info_pdev->dev, &dev_attr_board_id);
			if (ret) {
			  printk("%s:error on device_create_file %s\n", __func__, name);
			}
			#ifdef COMPATIBLE_WCNSS_NV
			memset(board_id_buf, 0, sizeof(board_id_buf));
			strncpy(board_id_buf, name, BOARD_ID_SIZE);
			printk("%s board id is: %s\n", __func__, board_id_buf);
			#endif
		break;

	default:
		ret = -1;
		break;
}

	mutex_unlock(&registration_lock);

	return ret;
}

EXPORT_SYMBOL(register_hardware_info);

static struct of_device_id hardware_info_table[] = {
	{ .compatible ="oem,hardware_info"},
	{ },
};
static struct platform_device hardware_info_device = {
	.name = "hardware_info",
	.id = -1,
	.dev = {
	.platform_data = &hardware_info_data,
},
};
static struct platform_driver hardware_info_driver = {
	.probe = hardware_info_probe,
	.remove = hardware_info_remove,
	.shutdown = NULL,
	.driver = {
		.name = "hardware_info",
		.owner = THIS_MODULE,
		.of_match_table = hardware_info_table,
		},
};
static int __init hardware_info_driver_init(void)
{
	int rc = 0;
	platform_device_register(&hardware_info_device);
    rc = platform_driver_register(&hardware_info_driver);
	return rc;
}

postcore_initcall(hardware_info_driver_init);
/*
MODULE_AUTHOR("oem");
MODULE_DESCRIPTION("get all hardware device info driver");
MODULE_LICENSE("GPL");

module_init(hardware_info_driver_init);
module_exit(hardware_info_driver_exit);
*/

