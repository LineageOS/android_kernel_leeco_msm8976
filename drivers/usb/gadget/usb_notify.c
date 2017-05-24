/*
 *  linux/drivers/video/usb_notify.c
 *
 *  Copyright (C) 2006 Antonino Daplas <adaplas@pol.net>
 *
 *	2001 - Documented with DocBook
 *	- Brad Douglas <brad@neruo.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */
#include <linux/notifier.h>
#include <linux/export.h>

static BLOCKING_NOTIFIER_HEAD(usb_ms_notifier_list);

/**
 *	usb_register_client - register a client notifier
 *	@nb: notifier block to callback on events
 */
int usb_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&usb_ms_notifier_list, nb);
}
EXPORT_SYMBOL(usb_register_client);

/**
 *	usb_unregister_client - unregister a client notifier
 *	@nb: notifier block to callback on events
 */
void usb_unregister_client(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&usb_ms_notifier_list, nb);
}
EXPORT_SYMBOL(usb_unregister_client);

/**
 * usb_notifier_call_chain - notify clients of usb_events
 *
 */

int usb_notifier_call_chain(unsigned long val)
{
	return blocking_notifier_call_chain(&usb_ms_notifier_list, val, NULL);
}
EXPORT_SYMBOL_GPL(usb_notifier_call_chain);
