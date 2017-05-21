/*
** =============================================================================
** Copyright (c) 2014  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**
** File:
**	 drv2604l.c
**
** Description:
**	 DRV2604L chip driver
**
** =============================================================================
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/firmware.h>
#include "drv2604l.h"
#include <linux/of_gpio.h>
#define CONFIG_HAPTICS_LRA_SEMCO1030 1
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend early_suspend;
#endif
static struct DRV2604L_data *pDRV2604Ldata = NULL;
static struct DRV2604L_platform_data  drv2604l_plat_data = {
	.GpioEnable = 0,			//enable the chip
	.GpioTrigger = 0,			//external trigger pin, (0: internal trigger)
#if defined(CONFIG_HAPTICS_LRA_SEMCO1030)
	//rated = 1.5Vrms, ov=2.1Vrms, f=204hz
	.loop = CLOSE_LOOP,
	.RTPFormat = Signed,
	.BIDIRInput = BiDirectional,
	.actuator = {
		.device_type = LRA,
		.rated_vol = 0x0a,
		.over_drive_vol = 0x87,
		.LRAFreq = 204,
	},
#elif defined(CONFIG_HAPTICS_ERM_EVOWAVE_Z4TJGB1512658)
	//rated vol = 3.0 v, ov = 3.6 v, risetime = 150 ms
	.loop = CLOSE_LOOP,
	.RTPFormat = Signed,
	.BIDIRInput = BiDirectional,
	.actuator = {
		.device_type = ERM,
		.rated_vol = 0x8d,
		.over_drive_vol = 0xa9,
	},
#else
#error "please define actuator type"
#endif
};
static int drv2604l_reg_read(struct DRV2604L_data *pDrv2604ldata, unsigned int reg)
{
	unsigned int val;
	int ret;

	ret = regmap_read(pDrv2604ldata->regmap, reg, &val);

	if (ret < 0)
		return ret;
	else
		return val;
}

static int drv2604l_reg_write(struct DRV2604L_data *pDrv2604ldata, unsigned char reg, char val)
{
	return regmap_write(pDrv2604ldata->regmap, reg, val);
}

static int drv2604l_bulk_read(struct DRV2604L_data *pDrv2604ldata, unsigned char reg, unsigned int count, u8 *buf)
{
	return regmap_bulk_read(pDrv2604ldata->regmap, reg, buf, count);
}

static int drv2604l_bulk_write(struct DRV2604L_data *pDrv2604ldata, unsigned char reg, unsigned int count, const u8 *buf)
{
	return regmap_bulk_write(pDrv2604ldata->regmap, reg, buf, count);
}

static int drv2604l_set_bits(struct DRV2604L_data *pDrv2604ldata, unsigned char reg, unsigned char mask, unsigned char val)
{
	return regmap_update_bits(pDrv2604ldata->regmap, reg, mask, val);
}

static int drv2604l_set_go_bit(struct DRV2604L_data *pDrv2604ldata, unsigned char val)
{
	return drv2604l_reg_write(pDrv2604ldata, GO_REG, (val&0x01));
}
#if 0
static void drv2604l_poll_go_bit(struct DRV2604L_data *pDrv2604ldata)
{
	while (drv2604l_reg_read(pDrv2604ldata, GO_REG) == GO)
	  schedule_timeout_interruptible(msecs_to_jiffies(GO_BIT_POLL_INTERVAL));
}
#endif
static int drv2604l_set_rtp_val(struct DRV2604L_data *pDrv2604ldata, char value)
{
	/* please be noted: in unsigned mode, maximum is 0xff, in signed mode, maximum is 0x7f */
	return drv2604l_reg_write(pDrv2604ldata, REAL_TIME_PLAYBACK_REG, value);
}

static int drv2604l_set_waveform_sequence(struct DRV2604L_data *pDrv2604ldata, unsigned char* seq, unsigned int size)
{
	return drv2604l_bulk_write(pDrv2604ldata, WAVEFORM_SEQUENCER_REG, (size>WAVEFORM_SEQUENCER_MAX)?WAVEFORM_SEQUENCER_MAX:size, seq);
}

static void drv2604l_change_mode(struct DRV2604L_data *pDrv2604ldata, char work_mode, char dev_mode)
{
	/* please be noted : LRA open loop cannot be used with analog input mode */
	if(dev_mode == DEV_IDLE){
		pDrv2604ldata->dev_mode = dev_mode;
		pDrv2604ldata->work_mode = work_mode;
	}else if(dev_mode == DEV_STANDBY){
		if(pDrv2604ldata->dev_mode != DEV_STANDBY){
			pDrv2604ldata->dev_mode = DEV_STANDBY;
			drv2604l_reg_write(pDrv2604ldata, MODE_REG, MODE_STANDBY);
			schedule_timeout_interruptible(msecs_to_jiffies(WAKE_STANDBY_DELAY));
		}
		pDrv2604ldata->work_mode = WORK_IDLE;
	}else if(dev_mode == DEV_READY){
		if((work_mode != pDrv2604ldata->work_mode)
			||(dev_mode != pDrv2604ldata->dev_mode)){
			pDrv2604ldata->work_mode = work_mode;
			pDrv2604ldata->dev_mode = dev_mode;
			if((pDrv2604ldata->work_mode == WORK_VIBRATOR)
				||(pDrv2604ldata->work_mode == WORK_PATTERN_RTP_ON)
				||(pDrv2604ldata->work_mode == WORK_SEQ_RTP_ON)
				||(pDrv2604ldata->work_mode == WORK_RTP)){
					drv2604l_reg_write(pDrv2604ldata, MODE_REG, MODE_REAL_TIME_PLAYBACK);
			}else if(pDrv2604ldata->work_mode == WORK_CALIBRATION){
				drv2604l_reg_write(pDrv2604ldata, MODE_REG, AUTO_CALIBRATION);
			}else{
				drv2604l_reg_write(pDrv2604ldata, MODE_REG, MODE_INTERNAL_TRIGGER);
				schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));
			}
		}
	}
}

static void play_effect(struct DRV2604L_data *pDrv2604ldata)
{
	switch_set_state(&pDrv2604ldata->sw_dev, SW_STATE_SEQUENCE_PLAYBACK);
	drv2604l_change_mode(pDrv2604ldata, WORK_SEQ_PLAYBACK, DEV_READY);
	drv2604l_set_waveform_sequence(pDrv2604ldata, pDrv2604ldata->sequence, WAVEFORM_SEQUENCER_MAX);
	pDrv2604ldata->vibrator_is_playing = YES;
	drv2604l_set_go_bit(pDrv2604ldata, GO);

	while((drv2604l_reg_read(pDrv2604ldata, GO_REG) == GO) && (pDrv2604ldata->should_stop == NO)){
		schedule_timeout_interruptible(msecs_to_jiffies(GO_BIT_POLL_INTERVAL));
	}

	if(pDrv2604ldata->should_stop == YES){
		drv2604l_set_go_bit(pDrv2604ldata, STOP);
	}

	drv2604l_change_mode(pDrv2604ldata, WORK_IDLE, DEV_STANDBY);
	switch_set_state(&pDrv2604ldata->sw_dev, SW_STATE_IDLE);
	pDrv2604ldata->vibrator_is_playing = NO;
	wake_unlock(&pDrv2604ldata->wklock);
}

static void play_Pattern_RTP(struct DRV2604L_data *pDrv2604ldata)
{
	if(pDrv2604ldata->work_mode == WORK_PATTERN_RTP_ON){
		drv2604l_change_mode(pDrv2604ldata, WORK_PATTERN_RTP_OFF, DEV_READY);
		if(pDrv2604ldata->repeat_times == 0){
			drv2604l_change_mode(pDrv2604ldata, WORK_IDLE, DEV_STANDBY);
			pDrv2604ldata->vibrator_is_playing = NO;
			switch_set_state(&pDrv2604ldata->sw_dev, SW_STATE_IDLE);
			wake_unlock(&pDrv2604ldata->wklock);
		}else{
			hrtimer_start(&pDrv2604ldata->timer, ns_to_ktime((u64)pDrv2604ldata->silience_time * NSEC_PER_MSEC), HRTIMER_MODE_REL);
		}
	}else if(pDrv2604ldata->work_mode == WORK_PATTERN_RTP_OFF){
		pDrv2604ldata->repeat_times--;
		drv2604l_change_mode(pDrv2604ldata, WORK_PATTERN_RTP_ON, DEV_READY);
		hrtimer_start(&pDrv2604ldata->timer, ns_to_ktime((u64)pDrv2604ldata->vibration_time * NSEC_PER_MSEC), HRTIMER_MODE_REL);
	}
}

static void play_Seq_RTP(struct DRV2604L_data *pDrv2604ldata)
{
	if(pDrv2604ldata->RTPSeq.RTPindex < pDrv2604ldata->RTPSeq.RTPCounts){
		int RTPTime = pDrv2604ldata->RTPSeq.RTPData[pDrv2604ldata->RTPSeq.RTPindex] >> 8;
		int RTPVal = pDrv2604ldata->RTPSeq.RTPData[pDrv2604ldata->RTPSeq.RTPindex] & 0x00ff ;

		pDrv2604ldata->vibrator_is_playing = YES;
		pDrv2604ldata->RTPSeq.RTPindex++;
		drv2604l_change_mode(pDrv2604ldata, WORK_SEQ_RTP_ON, DEV_READY);
		drv2604l_set_rtp_val(pDrv2604ldata,  RTPVal);

		hrtimer_start(&pDrv2604ldata->timer, ns_to_ktime((u64)RTPTime * NSEC_PER_MSEC), HRTIMER_MODE_REL);
	}else{
		drv2604l_change_mode(pDrv2604ldata, WORK_IDLE, DEV_STANDBY);
		pDrv2604ldata->vibrator_is_playing = NO;
		switch_set_state(&pDrv2604ldata->sw_dev, SW_STATE_IDLE);
		wake_unlock(&pDrv2604ldata->wklock);
	}
}

static void vibrator_off(struct DRV2604L_data *pDrv2604ldata)
{
	if (pDrv2604ldata->vibrator_is_playing) {
		pDrv2604ldata->vibrator_is_playing = NO;
		drv2604l_set_go_bit(pDrv2604ldata, STOP);
		drv2604l_change_mode(pDrv2604ldata, WORK_IDLE, DEV_STANDBY);
		switch_set_state(&pDrv2604ldata->sw_dev, SW_STATE_IDLE);
		wake_unlock(&pDrv2604ldata->wklock);
	}
}

static void drv2604l_stop(struct DRV2604L_data *pDrv2604ldata)
{
	if(pDrv2604ldata->vibrator_is_playing){
		if((pDrv2604ldata->work_mode == WORK_VIBRATOR)
				||(pDrv2604ldata->work_mode == WORK_PATTERN_RTP_ON)
				||(pDrv2604ldata->work_mode == WORK_PATTERN_RTP_OFF)
				||(pDrv2604ldata->work_mode == WORK_SEQ_RTP_ON)
				||(pDrv2604ldata->work_mode == WORK_SEQ_RTP_OFF)
				||(pDrv2604ldata->work_mode == WORK_RTP)){
			vibrator_off(pDrv2604ldata);
		}else if(pDrv2604ldata->work_mode == WORK_SEQ_PLAYBACK){
		}else{
			printk("%s, err mode=%d \n", __FUNCTION__, pDrv2604ldata->work_mode);
		}
	}
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct DRV2604L_data *pDrv2604ldata = container_of(dev, struct DRV2604L_data, to_dev);

	if (hrtimer_active(&pDrv2604ldata->timer)) {
		ktime_t r = hrtimer_get_remaining(&pDrv2604ldata->timer);
		return ktime_to_ms(r);
	}

	return 0;
}

static void vibrator_enable( struct timed_output_dev *dev, int value)
{
	struct DRV2604L_data *pDrv2604ldata = container_of(dev, struct DRV2604L_data, to_dev);

	pDrv2604ldata->should_stop = YES;
	hrtimer_cancel(&pDrv2604ldata->timer);
	cancel_work_sync(&pDrv2604ldata->vibrator_work);

	mutex_lock(&pDrv2604ldata->lock);

	drv2604l_stop(pDrv2604ldata);

	if (value > 0) {
		wake_lock(&pDrv2604ldata->wklock);

		drv2604l_change_mode(pDrv2604ldata, WORK_VIBRATOR, DEV_READY);
		pDrv2604ldata->vibrator_is_playing = YES;
		switch_set_state(&pDrv2604ldata->sw_dev, SW_STATE_RTP_PLAYBACK);

		value = (value>MAX_TIMEOUT)?MAX_TIMEOUT:value;
		hrtimer_start(&pDrv2604ldata->timer, ns_to_ktime((u64)value * NSEC_PER_MSEC), HRTIMER_MODE_REL);
	}

	mutex_unlock(&pDrv2604ldata->lock);
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct DRV2604L_data *pDrv2604ldata = container_of(timer, struct DRV2604L_data, timer);

	schedule_work(&pDrv2604ldata->vibrator_work);

	return HRTIMER_NORESTART;
}

static void vibrator_work_routine(struct work_struct *work)
{
	struct DRV2604L_data *pDrv2604ldata = container_of(work, struct DRV2604L_data, vibrator_work);
	mutex_lock(&pDrv2604ldata->lock);
	if((pDrv2604ldata->work_mode == WORK_VIBRATOR)
		||(pDrv2604ldata->work_mode == WORK_RTP)){
		vibrator_off(pDrv2604ldata);
	}else if(pDrv2604ldata->work_mode == WORK_SEQ_PLAYBACK){
		play_effect(pDrv2604ldata);
	}else if((pDrv2604ldata->work_mode == WORK_PATTERN_RTP_ON)||(pDrv2604ldata->work_mode == WORK_PATTERN_RTP_OFF)){
		play_Pattern_RTP(pDrv2604ldata);
	}else if((pDrv2604ldata->work_mode == WORK_SEQ_RTP_ON)||(pDrv2604ldata->work_mode == WORK_SEQ_RTP_OFF)){
		play_Seq_RTP(pDrv2604ldata);
	}
	mutex_unlock(&pDrv2604ldata->lock);
}

static int fw_chksum(const struct firmware *fw){
	int sum = 0;
	int i=0;
	int size = fw->size;
	const unsigned char *pBuf = fw->data;

	for (i=0; i< size; i++){
		if((i>11) && (i<16)){

		}else{
			sum += pBuf[i];
		}
	}

	return sum;
}

/* drv2604l_firmware_load:   This function is called by the
 *		request_firmware_nowait function as soon
 *		as the firmware has been loaded from the file.
 *		The firmware structure contains the data and$
 *		the size of the firmware loaded.
 * @fw: pointer to firmware file to be dowloaded
 * @context: pointer variable to DRV2604L_data
 *
 *
 */
static void drv2604l_firmware_load(const struct firmware *fw, void *context)
{
	struct DRV2604L_data *pDrv2604ldata = context;
	int size = 0, fwsize = 0, i=0;
	const unsigned char *pBuf = NULL;

	if(fw != NULL){
		pBuf = fw->data;
		size = fw->size;

		memcpy(&(pDrv2604ldata->fw_header), pBuf, sizeof(struct DRV2604L_fw_header));
		if((pDrv2604ldata->fw_header.fw_magic != DRV2604L_MAGIC)
			||(pDrv2604ldata->fw_header.fw_size != size)
			||(pDrv2604ldata->fw_header.fw_chksum != fw_chksum(fw))){
			printk("%s, ERROR!! firmware not right:Magic=0x%x,Size=%d,chksum=0x%x\n",
				__FUNCTION__, pDrv2604ldata->fw_header.fw_magic,
				pDrv2604ldata->fw_header.fw_size, pDrv2604ldata->fw_header.fw_chksum);
		}else{
			printk("%s, firmware good\n", __FUNCTION__);

			drv2604l_change_mode(pDrv2604ldata, WORK_IDLE, DEV_READY);

			pBuf += sizeof(struct DRV2604L_fw_header);

			drv2604l_reg_write(pDrv2604ldata, RAM_ADDR_UPPER_BYTE_REG, 0);
			drv2604l_reg_write(pDrv2604ldata, RAM_ADDR_LOWER_BYTE_REG, 0);

			fwsize = size - sizeof(struct DRV2604L_fw_header);
			for(i = 0; i < fwsize; i++){
				drv2604l_reg_write(pDrv2604ldata, RAM_DATA_REG, pBuf[i]);
			}

			drv2604l_change_mode(pDrv2604ldata, WORK_IDLE, DEV_STANDBY);
		}
	}else{
		printk("%s, ERROR!! firmware not found\n", __FUNCTION__);
	}
}

static int dev2604_open (struct inode * i_node, struct file * filp)
{
	if(pDRV2604Ldata == NULL){
		return -ENODEV;
	}

	filp->private_data = pDRV2604Ldata;
	return 0;
}

static ssize_t dev2604_read(struct file* filp, char* buff, size_t length, loff_t* offset)
{
	struct DRV2604L_data *pDrv2604ldata = (struct DRV2604L_data *)filp->private_data;
	int ret = 0;

	if(pDrv2604ldata->ReadLen > 0){
		ret = copy_to_user(buff, pDrv2604ldata->ReadBuff, pDrv2604ldata->ReadLen);
		if (ret != 0){
			printk("%s, copy_to_user err=%d \n", __FUNCTION__, ret);
		}else{
			ret = pDrv2604ldata->ReadLen;
		}
		pDrv2604ldata->ReadLen = 0;
	}else{
		printk("%s, nothing to read\n", __FUNCTION__);
	}

	return ret;
}

static bool isforDebug(int cmd){
	return ((cmd == HAPTIC_CMDID_REG_WRITE)
		||(cmd == HAPTIC_CMDID_REG_READ)
		||(cmd == HAPTIC_CMDID_REG_SETBIT));
}

static ssize_t dev2604_write(struct file* filp, const char* buff, size_t len, loff_t* off)
{
	struct DRV2604L_data *pDrv2604ldata = (struct DRV2604L_data *)filp->private_data;

	if(isforDebug(buff[0])){
	}else{
		pDrv2604ldata->should_stop = YES;
		hrtimer_cancel(&pDrv2604ldata->timer);
		cancel_work_sync(&pDrv2604ldata->vibrator_work);
	}

	mutex_lock(&pDrv2604ldata->lock);

	if(isforDebug(buff[0])){
	}else{
		drv2604l_stop(pDrv2604ldata);
	}

	switch(buff[0])
	{
		case HAPTIC_CMDID_PLAY_SINGLE_EFFECT:
		case HAPTIC_CMDID_PLAY_EFFECT_SEQUENCE:
		{
			memset(&pDrv2604ldata->sequence, 0, WAVEFORM_SEQUENCER_MAX);
			if (!copy_from_user(&pDrv2604ldata->sequence, &buff[1], len - 1))
			{
				wake_lock(&pDrv2604ldata->wklock);

				pDrv2604ldata->should_stop = NO;
				drv2604l_change_mode(pDrv2604ldata, WORK_SEQ_PLAYBACK, DEV_IDLE);
				schedule_work(&pDrv2604ldata->vibrator_work);
			}
			break;
		}
		case HAPTIC_CMDID_PLAY_TIMED_EFFECT:
		{
			unsigned int value = 0;
			value = buff[2];
			value <<= 8;
			value |= buff[1];

			if (value > 0)
			{
				wake_lock(&pDrv2604ldata->wklock);
				switch_set_state(&pDrv2604ldata->sw_dev, SW_STATE_RTP_PLAYBACK);
				pDrv2604ldata->vibrator_is_playing = YES;
				value = (value > MAX_TIMEOUT)?MAX_TIMEOUT:value;
				drv2604l_change_mode(pDrv2604ldata, WORK_RTP, DEV_READY);

				hrtimer_start(&pDrv2604ldata->timer, ns_to_ktime((u64)value * NSEC_PER_MSEC), HRTIMER_MODE_REL);
			}
			break;
		}

	   case HAPTIC_CMDID_PATTERN_RTP:
		{
			unsigned char strength = 0;

			pDrv2604ldata->vibration_time = (int)((((int)buff[2])<<8) | (int)buff[1]);
			pDrv2604ldata->silience_time = (int)((((int)buff[4])<<8) | (int)buff[3]);
			strength = buff[5];
			pDrv2604ldata->repeat_times = buff[6];

			if(pDrv2604ldata->vibration_time > 0){
				wake_lock(&pDrv2604ldata->wklock);
				switch_set_state(&pDrv2604ldata->sw_dev, SW_STATE_RTP_PLAYBACK);
				pDrv2604ldata->vibrator_is_playing = YES;
				if(pDrv2604ldata->repeat_times > 0)
					pDrv2604ldata->repeat_times--;
				if (pDrv2604ldata->vibration_time > MAX_TIMEOUT)
					pDrv2604ldata->vibration_time = MAX_TIMEOUT;
				drv2604l_change_mode(pDrv2604ldata, WORK_PATTERN_RTP_ON, DEV_READY);
				drv2604l_set_rtp_val(pDrv2604ldata, strength);

				hrtimer_start(&pDrv2604ldata->timer, ns_to_ktime((u64)pDrv2604ldata->vibration_time * NSEC_PER_MSEC), HRTIMER_MODE_REL);
			}
			break;
		}

		case HAPTIC_CMDID_RTP_SEQUENCE:
		{
			memset(&pDrv2604ldata->RTPSeq, 0, sizeof(struct RTP_Seq));
			if(((len-1)%2) == 0){
				pDrv2604ldata->RTPSeq.RTPCounts = (len-1)/2;
				if((pDrv2604ldata->RTPSeq.RTPCounts <= MAX_RTP_SEQ)&&(pDrv2604ldata->RTPSeq.RTPCounts>0)){
					if(copy_from_user(pDrv2604ldata->RTPSeq.RTPData, &buff[1], pDrv2604ldata->RTPSeq.RTPCounts*2) != 0){
						printk("%s, rtp_seq copy seq err\n", __FUNCTION__);
						break;
					}

					wake_lock(&pDrv2604ldata->wklock);
					switch_set_state(&pDrv2604ldata->sw_dev, SW_STATE_RTP_PLAYBACK);
					drv2604l_change_mode(pDrv2604ldata, WORK_SEQ_RTP_OFF, DEV_IDLE);
					schedule_work(&pDrv2604ldata->vibrator_work);
				}else{
					printk("%s, rtp_seq count error,maximum=%d\n", __FUNCTION__,MAX_RTP_SEQ);
				}
			}else{
				printk("%s, rtp_seq len error\n", __FUNCTION__);
			}
			break;
		}

		case HAPTIC_CMDID_STOP:
		{
			break;
		}

		case HAPTIC_CMDID_UPDATE_FIRMWARE:
		{
			struct firmware fw;
			unsigned char *fw_buffer = (unsigned char *)kzalloc(len-1, GFP_KERNEL);
			int result = -1;

			if(fw_buffer != NULL){
				fw.size = len-1;

				wake_lock(&pDrv2604ldata->wklock);
				result = copy_from_user(fw_buffer, &buff[1], fw.size);
				if(result == 0){
					printk("%s, fwsize=%ld, f:%x, l:%x\n", __FUNCTION__, fw.size, buff[1], buff[len-1]);
					fw.data = (const unsigned char *)fw_buffer;
					drv2604l_firmware_load(&fw, (void *)pDrv2604ldata);
				}
				wake_unlock(&pDrv2604ldata->wklock);

				kfree(fw_buffer);
			}
			break;
		}

		case HAPTIC_CMDID_READ_FIRMWARE:
		{
			int i;
			if(len == 3){
				pDrv2604ldata->ReadLen = 1;
				drv2604l_reg_write(pDrv2604ldata, RAM_ADDR_UPPER_BYTE_REG, buff[2]);
				drv2604l_reg_write(pDrv2604ldata, RAM_ADDR_LOWER_BYTE_REG, buff[1]);
				pDrv2604ldata->ReadBuff[0] = drv2604l_reg_read(pDrv2604ldata, RAM_DATA_REG);
			}else if(len == 4){
				drv2604l_reg_write(pDrv2604ldata, RAM_ADDR_UPPER_BYTE_REG, buff[2]);
				drv2604l_reg_write(pDrv2604ldata, RAM_ADDR_LOWER_BYTE_REG, buff[1]);
				pDrv2604ldata->ReadLen = (buff[3]>MAX_READ_BYTES)?MAX_READ_BYTES:buff[3];
				for(i=0; i < pDrv2604ldata->ReadLen; i++){
					pDrv2604ldata->ReadBuff[i] = drv2604l_reg_read(pDrv2604ldata, RAM_DATA_REG);
				}
			}else{
				printk("%s, read fw len error\n", __FUNCTION__);
			}
			break;
		}
		case HAPTIC_CMDID_REG_READ:
		{
			if(len == 2){
				pDrv2604ldata->ReadLen = 1;
				pDrv2604ldata->ReadBuff[0] = drv2604l_reg_read(pDrv2604ldata, buff[1]);
			}else if(len == 3){
				pDrv2604ldata->ReadLen = (buff[2]>MAX_READ_BYTES)?MAX_READ_BYTES:buff[2];
				drv2604l_bulk_read(pDrv2604ldata, buff[1], pDrv2604ldata->ReadLen, pDrv2604ldata->ReadBuff);
			}else{
				printk("%s, reg_read len error\n", __FUNCTION__);
			}
			break;
		}

		case HAPTIC_CMDID_REG_WRITE:
		{
			if((len-1) == 2){
				drv2604l_reg_write(pDrv2604ldata, buff[1], buff[2]);
			}else if((len-1)>2){
				unsigned char *data = (unsigned char *)kzalloc(len-2, GFP_KERNEL);
				if(data != NULL){
					if(copy_from_user(data, &buff[2], len-2) != 0){
						printk("%s, reg copy err\n", __FUNCTION__);
					}else{
						drv2604l_bulk_write(pDrv2604ldata, buff[1], len-2, data);
					}
					kfree(data);
				}
			}else{
				printk("%s, reg_write len error\n", __FUNCTION__);
			}
			break;
		}

		case HAPTIC_CMDID_REG_SETBIT:
		{
			int i=1;
			for(i=1; i< len; ){
				drv2604l_set_bits(pDrv2604ldata, buff[i], buff[i+1], buff[i+2]);
				i += 3;
			}
			break;
		}
	default:
		printk("%s, unknown HAPTIC cmd\n", __FUNCTION__);
	  break;
	}

	mutex_unlock(&pDrv2604ldata->lock);

	return len;
}


static struct file_operations fops =
{
	.open = dev2604_open,
	.read = dev2604_read,
	.write = dev2604_write,
};
#ifdef CONFIG_HAS_EARLYSUSPEND
void drv2604l_early_suspend(struct early_suspend *h){
	struct DRV2604L_data *pDrv2604ldata = container_of(h, struct DRV2604L_data, early_suspend);

	pDrv2604ldata->should_stop = YES;
	hrtimer_cancel(&pDrv2604ldata->timer);
	cancel_work_sync(&pDrv2604ldata->vibrator_work);

	mutex_lock(&pDrv2604ldata->lock);

	drv2604l_stop(pDrv2604ldata);

	mutex_unlock(&pDrv2604ldata->lock);
	return ;
}

void drv2604l_late_resume(struct early_suspend *h) {
	struct DRV2604L_data *pDrv2604ldata = container_of(h, struct DRV2604L_data, early_suspend);

	mutex_lock(&pDrv2604ldata->lock);
	mutex_unlock(&pDrv2604ldata->lock);
	return ;
 }
 #endif
static int Haptics_init(struct DRV2604L_data *pDrv2604ldata)
{
	int reval = -ENOMEM;

	pDrv2604ldata->version = MKDEV(0,0);
	reval = alloc_chrdev_region(&pDrv2604ldata->version, 0, 1, HAPTICS_DEVICE_NAME);
	if (reval < 0)
	{
		printk(KERN_ALERT"drv2604: error getting major number %d\n", reval);
		goto fail0;
	}

	pDrv2604ldata->class = class_create(THIS_MODULE, HAPTICS_DEVICE_NAME);
	if (!pDrv2604ldata->class)
	{
		printk(KERN_ALERT"drv2604: error creating class\n");
		goto fail1;
	}

	pDrv2604ldata->device = device_create(pDrv2604ldata->class, NULL, pDrv2604ldata->version, NULL, HAPTICS_DEVICE_NAME);
	if (!pDrv2604ldata->device)
	{
		printk(KERN_ALERT"drv2604: error creating device 2604\n");
		goto fail2;
	}

	cdev_init(&pDrv2604ldata->cdev, &fops);
	pDrv2604ldata->cdev.owner = THIS_MODULE;
	pDrv2604ldata->cdev.ops = &fops;
	reval = cdev_add(&pDrv2604ldata->cdev, pDrv2604ldata->version, 1);
	if (reval)
	{
		printk(KERN_ALERT"drv2604: fail to add cdev\n");
		goto fail3;
	}

	pDrv2604ldata->sw_dev.name = "haptics";
	reval = switch_dev_register(&pDrv2604ldata->sw_dev);
	if (reval < 0) {
		printk(KERN_ALERT"drv2604: fail to register switch\n");
		goto fail4;
	}

	pDrv2604ldata->to_dev.name = "vibrator";
	pDrv2604ldata->to_dev.get_time = vibrator_get_time;
	pDrv2604ldata->to_dev.enable = vibrator_enable;

	if (timed_output_dev_register(&(pDrv2604ldata->to_dev)) < 0)
	{
		printk(KERN_ALERT"drv2604: fail to create timed output dev\n");
		goto fail3;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	pDrv2604ldata->early_suspend.suspend = drv2604l_early_suspend;
	pDrv2604ldata->early_suspend.resume = drv2604l_late_resume;
	pDrv2604ldata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
	register_early_suspend(&pDrv2604ldata->early_suspend);
#endif

	hrtimer_init(&pDrv2604ldata->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pDrv2604ldata->timer.function = vibrator_timer_func;
	INIT_WORK(&pDrv2604ldata->vibrator_work, vibrator_work_routine);

	wake_lock_init(&pDrv2604ldata->wklock, WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&pDrv2604ldata->lock);

	return 0;

fail4:
	switch_dev_unregister(&pDrv2604ldata->sw_dev);
fail3:
	device_destroy(pDrv2604ldata->class, pDrv2604ldata->version);
fail2:
	class_destroy(pDrv2604ldata->class);
fail1:
	unregister_chrdev_region(pDrv2604ldata->version, 1);
fail0:
	return reval;
}

static void dev_init_platform_data(struct DRV2604L_data *pDrv2604ldata)
{
	struct DRV2604L_platform_data *pDrv2604Platdata = &pDrv2604ldata->PlatData;
	struct actuator_data actuator = pDrv2604Platdata->actuator;
	unsigned char temp = 0;
	//OTP memory saves data from 0x16 to 0x1a
	if(pDrv2604ldata->OTP == 0) {
		if(actuator.rated_vol != 0){
			drv2604l_reg_write(pDrv2604ldata, RATED_VOLTAGE_REG, actuator.rated_vol);
		}else{
			printk("%s, ERROR Rated ZERO\n", __FUNCTION__);
		}

		if(actuator.over_drive_vol != 0){
			drv2604l_reg_write(pDrv2604ldata, OVERDRIVE_CLAMP_VOLTAGE_REG, actuator.over_drive_vol);
		}else{
			printk("%s, ERROR OverDriveVol ZERO\n", __FUNCTION__);
		}

		drv2604l_set_bits(pDrv2604ldata,
						FEEDBACK_CONTROL_REG,
						FEEDBACK_CONTROL_DEVICE_TYPE_MASK
							|FEEDBACK_CONTROL_FB_BRAKE_MASK
							|FEEDBACK_CONTROL_LOOP_GAIN_MASK,
						(((actuator.device_type == LRA)?FEEDBACK_CONTROL_MODE_LRA:FEEDBACK_CONTROL_MODE_ERM)
							|FB_BRAKE_FACTOR
							|LOOP_GAIN)
						);
	}else{
		printk("%s, OTP programmed\n", __FUNCTION__);
	}

	if(actuator.device_type == LRA){
		unsigned char DriveTime = 5*(1000 - actuator.LRAFreq)/actuator.LRAFreq;
		drv2604l_set_bits(pDrv2604ldata,
				Control1_REG,
				Control1_REG_DRIVE_TIME_MASK,
				DriveTime);
		printk("%s, LRA = %d, DriveTime=0x%x\n", __FUNCTION__, actuator.LRAFreq, DriveTime);
	}

	if(pDrv2604Platdata->loop == OPEN_LOOP){
		temp = BIDIR_INPUT_BIDIRECTIONAL;
	}else{
		if(pDrv2604Platdata->BIDIRInput == UniDirectional){
			temp = BIDIR_INPUT_UNIDIRECTIONAL;
		}else{
			temp = BIDIR_INPUT_BIDIRECTIONAL;
		}
	}

	drv2604l_set_bits(pDrv2604ldata,
				Control2_REG,
				Control2_REG_BIDIR_INPUT_MASK|BLANKING_TIME_MASK|IDISS_TIME_MASK,
				temp|BLANKING_TIME|IDISS_TIME);

	if((pDrv2604Platdata->loop == CLOSE_LOOP)&&(actuator.device_type == LRA))
	{
		drv2604l_set_bits(pDrv2604ldata,
				Control2_REG,
				AUTO_RES_SAMPLE_TIME_MASK,
				AUTO_RES_SAMPLE_TIME_300us);
	}

	if((pDrv2604Platdata->loop == OPEN_LOOP)&&(actuator.device_type == LRA))
	{
		temp = LRA_OpenLoop_Enabled;
	}
	else if((pDrv2604Platdata->loop == OPEN_LOOP)&&(actuator.device_type == ERM))
	{
		temp = ERM_OpenLoop_Enabled;
	}
	else
	{
		temp = ERM_OpenLoop_Disable|LRA_OpenLoop_Disable;
	}

	if((pDrv2604Platdata->loop == CLOSE_LOOP) &&(pDrv2604Platdata->BIDIRInput == UniDirectional))
	{
		temp |= RTP_FORMAT_UNSIGNED;
		drv2604l_reg_write(pDrv2604ldata, REAL_TIME_PLAYBACK_REG, 0xff);
	}
	else
	{
		if(pDrv2604Platdata->RTPFormat == Signed)
		{
			temp |= RTP_FORMAT_SIGNED;
			drv2604l_reg_write(pDrv2604ldata, REAL_TIME_PLAYBACK_REG, 0x7f);
		}
		else
		{
			temp |= RTP_FORMAT_UNSIGNED;
			drv2604l_reg_write(pDrv2604ldata, REAL_TIME_PLAYBACK_REG, 0xff);
		}
	}

	drv2604l_set_bits(pDrv2604ldata,
					Control3_REG,
					Control3_REG_LOOP_MASK|Control3_REG_FORMAT_MASK,
					temp);

	drv2604l_set_bits(pDrv2604ldata,
			Control4_REG,
			Control4_REG_CAL_TIME_MASK|Control4_REG_ZC_DET_MASK,
			AUTO_CAL_TIME|ZC_DET_TIME
			);

	drv2604l_set_bits(pDrv2604ldata,
			Control5_REG,
			BLANK_IDISS_MSB_MASK,
			BLANK_IDISS_MSB_CLEAR
			);

	if(actuator.device_type == LRA)
	{
		/* please refer to the equations in DRV2604L data sheet */
		unsigned int temp = 9846 * actuator.LRAFreq;
		unsigned char R20 = (unsigned char)(100000000 / temp);
		drv2604l_reg_write(pDrv2604ldata, LRA_OPENLOOP_PERIOD_REG, R20);
	}
}
#if 0
static int dev_auto_calibrate(struct DRV2604L_data *pDrv2604ldata)
{
	int err = 0, status=0;

	drv2604l_change_mode(pDrv2604ldata, WORK_CALIBRATION, DEV_READY);
	drv2604l_set_go_bit(pDrv2604ldata, GO);

	/* Wait until the procedure is done */
	drv2604l_poll_go_bit(pDrv2604ldata);
	/* Read status */
	status = drv2604l_reg_read(pDrv2604ldata, STATUS_REG);

	printk("%s, calibration status =0x%x\n", __FUNCTION__, status);

	/* Read calibration results */
	drv2604l_reg_read(pDrv2604ldata, AUTO_CALI_RESULT_REG);
	drv2604l_reg_read(pDrv2604ldata, AUTO_CALI_BACK_EMF_RESULT_REG);
	drv2604l_reg_read(pDrv2604ldata, FEEDBACK_CONTROL_REG);

	return err;
}
#endif
static struct regmap_config drv2604l_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

static void HapticsFirmwareLoad(const struct firmware *fw, void *context)
{
	drv2604l_firmware_load(fw, context);
	release_firmware(fw);
}

static int drv2604l_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
	struct DRV2604L_data *pDrv2604ldata;

	int err = 0;
	int status = 0;
	int ret = -1;
	printk("%s,%d\n",__func__,__LINE__);
	if (client->dev.of_node) {
		pDrv2604ldata = devm_kzalloc(&client->dev,
			sizeof(struct DRV2604L_data), GFP_KERNEL);
		if (!pDrv2604ldata) {
			dev_err(&client->dev,
				"DRV2604L_data Failed to allocate memory for pdata\n");
			return -ENOMEM;
		}else{
			memcpy(&pDrv2604ldata->PlatData, &drv2604l_plat_data, sizeof(struct DRV2604L_platform_data));
			pDrv2604ldata->PlatData.GpioEnable = of_get_named_gpio(client->dev.of_node,"ti,drv2604l-enable-gpio", 0);
			if (!gpio_is_valid(pDrv2604ldata->PlatData.GpioEnable)) {
					pr_err("%s: drv2604l chip enable not specified\n", __func__);
			}
		}
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR"%s:I2C check failed\n", __FUNCTION__);
		return -ENODEV;
	}
	pDrv2604ldata->regmap = devm_regmap_init_i2c(client, &drv2604l_i2c_regmap);
	if (IS_ERR(pDrv2604ldata->regmap)) {
		err = PTR_ERR(pDrv2604ldata->regmap);
		printk(KERN_ERR"%s:Failed to allocate register map: %d\n",__FUNCTION__,err);
		return err;
	}

	i2c_set_clientdata(client,pDrv2604ldata);

	if(pDrv2604ldata->PlatData.GpioTrigger){
		err = gpio_request(pDrv2604ldata->PlatData.GpioTrigger,HAPTICS_DEVICE_NAME"Trigger");
		if(err < 0){
			printk(KERN_ERR"%s: GPIO request Trigger error\n", __FUNCTION__);
			goto exit_gpio_request_failed;
		}
	}

	if(pDrv2604ldata->PlatData.GpioEnable){
		err = gpio_request(pDrv2604ldata->PlatData.GpioEnable,HAPTICS_DEVICE_NAME"Enable");
		if(err < 0){
			printk(KERN_ERR"%s: GPIO request enable error\n", __FUNCTION__);
			goto exit_gpio_request_failed;
		}else{
		/* Enable power to the chip */
			ret = gpio_direction_output(pDrv2604ldata->PlatData.GpioEnable, 1);
		/* Wait 30 us */
			udelay(30);
			if (ret) {
		   		pr_err("%s: Set direction for Drv2604l GpioEnable failed, ret=%d\n",
								__func__, ret);
					}
		}
	}
	err = drv2604l_reg_read(pDrv2604ldata, STATUS_REG);
	if(err < 0){
		printk("%s, i2c bus fail (%d)\n", __FUNCTION__, err);
		goto exit_gpio_request_failed;
	}else{
		printk("%s, i2c status (0x%x)\n", __FUNCTION__, err);
		status = err;
	}
	/* Read device ID */
	pDrv2604ldata->device_id = (status & DEV_ID_MASK);
	switch (pDrv2604ldata->device_id)
	{
		case DRV2605_VER_1DOT1:
		printk("drv2604 driver found: drv2605 v1.1.\n");
		break;
		case DRV2605_VER_1DOT0:
		printk("drv2604 driver found: drv2605 v1.0.\n");
		break;
		case DRV2604:
		printk(KERN_ALERT"drv2604 driver found: drv2604.\n");
		break;
		case DRV2604L:
		printk(KERN_ALERT"drv2604 driver found: drv2604L.\n");
		break;
		case DRV2605L:
		printk(KERN_ALERT"drv2604 driver found: drv2605L.\n");
		break;
		default:
		printk(KERN_ERR"drv2604 driver found: unknown.\n");
		break;
	}
	if((pDrv2604ldata->device_id != DRV2604L)&&(pDrv2604ldata->device_id != DRV2604)){
		printk("%s, status(0x%x),device_id(%d) fail\n",
			__FUNCTION__, status, pDrv2604ldata->device_id);
		goto exit_gpio_request_failed;
	}else{
	if(0){
		err = request_firmware_nowait(THIS_MODULE,
										FW_ACTION_HOTPLUG,
										"drv2604.bin",
										&(client->dev),
										GFP_KERNEL,
										pDrv2604ldata,
										HapticsFirmwareLoad);
		}
	}

	drv2604l_change_mode(pDrv2604ldata, WORK_IDLE, DEV_READY);
	schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));

	pDrv2604ldata->OTP = drv2604l_reg_read(pDrv2604ldata, Control4_REG) & Control4_REG_OTP_MASK;

	dev_init_platform_data(pDrv2604ldata);
#if 0
	if(pDrv2604ldata->OTP == 0){
		err = dev_auto_calibrate(pDrv2604ldata);
		if(err < 0){
			printk("%s, ERROR, calibration fail\n",__FUNCTION__);
		}
	}
#endif
	/* Put hardware in standby */
	drv2604l_change_mode(pDrv2604ldata, WORK_IDLE, DEV_STANDBY);

	Haptics_init(pDrv2604ldata);

	pDRV2604Ldata = pDrv2604ldata;
	printk("drv2604 probe succeeded\n");

	return 0;

exit_gpio_request_failed:
	if(pDrv2604ldata->PlatData.GpioTrigger){
		gpio_free(pDrv2604ldata->PlatData.GpioTrigger);
	}

	if(pDrv2604ldata->PlatData.GpioEnable){
		gpio_free(pDrv2604ldata->PlatData.GpioEnable);
	}

	printk(KERN_ERR"%s failed, err=%d\n",__FUNCTION__, err);
	return err;
}

static int drv2604l_remove(struct i2c_client* client)
{
	struct DRV2604L_data *pDrv2604ldata = i2c_get_clientdata(client);

	device_destroy(pDrv2604ldata->class, pDrv2604ldata->version);
	class_destroy(pDrv2604ldata->class);
	unregister_chrdev_region(pDrv2604ldata->version, 1);

	if(pDrv2604ldata->PlatData.GpioTrigger)
		gpio_free(pDrv2604ldata->PlatData.GpioTrigger);

	if(pDrv2604ldata->PlatData.GpioEnable)
		gpio_free(pDrv2604ldata->PlatData.GpioEnable);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&pDrv2604ldata->early_suspend);
#endif

	printk(KERN_ALERT"drv2604 remove");

	return 0;
}

static struct i2c_device_id drv2604l_id_table[] =
{
	{ HAPTICS_DEVICE_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, drv2604l_id_table);
static struct of_device_id drv2604l_match_table[] =
{
		{.compatible = "ti,haptic_drv2604l" },
		{},
};
static struct i2c_driver drv2604l_driver =
{
	.driver = {
		.name = HAPTICS_DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = drv2604l_match_table,
	},
	.id_table = drv2604l_id_table,
	.probe = drv2604l_probe,
	.remove = drv2604l_remove,
};

static int __init drv2604l_init(void)
{
	return i2c_add_driver(&drv2604l_driver);
}

static void __exit drv2604l_exit(void)
{
	i2c_del_driver(&drv2604l_driver);
}

module_init(drv2604l_init);
module_exit(drv2604l_exit);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("Driver for "HAPTICS_DEVICE_NAME);
