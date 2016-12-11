/* Copyright (c) 2012-2012, YEP Telecom. All rights reserved.
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


#ifndef HARDWARE_INFO_H
#define HARDWARE_INFO_H

typedef enum {
	LCM = 0,
	CTP,
	MAIN_CAM,
	SUB_CAM,
	FLASH,
	GSENSOR,/*ACCELEROMETER*/
	MSENSOR,/*MAGNETIC_FIELD*/
	ALS_PS,/*LIGHT and PROXIMITY*/
	GYRO,/*GYROSCOPE*/
    EMCP,
    ROM, /* ROM INFO */
    RAM, /* RAM INFO */
        NFC, /* NFC INFO */
// add by shihuijun for "wifi bluetooth"hardware devices version info start2013-10-30
    WIFI,/*WIFI INFO*/
    BLUETOOTH,/*BLUETOOTH INFO*/
// add by shihuijun for "wifi bluetooth"hardware devices version info  end 2013-10-31
	HARDWARE_ID_MAX
} HARDWARE_ID;
/*Begin  by yangyongfeng for camera sensor hardware_info (ql1530) 20151222 */
struct hardware_info_pdata{
	char lcm[125];
	char ctp[125];
	char main_cam[125];
	char sub_cam[125];
	char flash[125];
	char gsensor[125];
	char msensor[125];
	char als_ps[125];
	char gyro[125];
    char emcp[125];
    char rom[125];
    char ram[125];
    char nfc[125];
// add by shihuijun for "wifi bluetooth"  hardware devices version info  start 2013-10-30
    char wifi[125];
    char bluetooth[125];
// add by shihuijun for "wifi bluetooth" hardware devices version info  end 2013-10-31

};
/*End  by yangyongfeng for camera sensor hardware_info (ql1530) 20151222 */
int register_hardware_info(HARDWARE_ID id, char* name);

#endif /* HARDWARE_INFO_H */

