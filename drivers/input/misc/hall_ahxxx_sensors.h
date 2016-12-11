#ifndef _HALL_AHXXX_SENSOR_H
#define _HALL_AHXXX_SENSOR_H
#include <linux/regulator/consumer.h>

struct hall_sensor_platform_data {
	struct delayed_work	work;
	u32 delay_msc;
	u8 run_rpt_val;
	struct delayed_work	test_work;
	struct input_dev  *input_dev;
	struct workqueue_struct *hall_wq;
	const char *name;
	int irq_gpio;
	int irq;
	u32 debounce;
	int (*power_on)(bool);
	struct  wake_lock hall_lock;
};

struct hall_sensor_data {

    struct hall_sensor_platform_data *pdata;
    u8 last_report_val;

    struct pinctrl *pinctrl;
    struct pinctrl_state *pin_default;

    #if defined(CONFIG_FB)
	struct notifier_block fb_notif;
    #elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
    #endif
    struct regulator *vdd;
};


#endif
