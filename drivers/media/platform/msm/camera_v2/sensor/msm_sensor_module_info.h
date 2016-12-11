/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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

 #ifndef MSM_SENSOR_MODULE_INFO_H
#define MSM_SENSOR_MODULE_INFO_H
int msm_sensor_module_info_set(enum msm_sensor_camera_id_t position, char* module_info);
int msm_sensor_module_info_get(enum msm_sensor_camera_id_t position, char* module_info);
#endif
