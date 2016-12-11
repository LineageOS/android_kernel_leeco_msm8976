#ifndef _LINUX_USB_H
#define _LINUX_USB_H

#include <linux/kgdb.h>


#include <linux/fs.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/list.h>
#include <linux/backlight.h>
#include <linux/slab.h>
#include <asm/io.h>

#define USB_MS_DEVICE_ADD		0x0001
#define USB_MS_DEVICE_REMOVE	0x0002


extern int usb_register_client(struct notifier_block *nb);
extern void usb_unregister_client(struct notifier_block *nb);
extern int usb_notifier_call_chain(unsigned long val);

#endif /* _LINUX_USB_H */
