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

static BLOCKING_NOTIFIER_HEAD(type_c_get_notifier_list);
static BLOCKING_NOTIFIER_HEAD(type_c_set_notifier_list);
static BLOCKING_NOTIFIER_HEAD(type_c_otg_power_notifier_list);
/**
 *	type_c_register_client - register a client notifier
 *	@nb: notifier block to callback on events
 */
int type_c_get_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&type_c_get_notifier_list, nb);
}
EXPORT_SYMBOL(type_c_get_register_client);

int type_c_set_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&type_c_set_notifier_list, nb);
}
EXPORT_SYMBOL(type_c_set_register_client);

int type_c_otg_power_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&type_c_otg_power_notifier_list, nb);
}
EXPORT_SYMBOL(type_c_otg_power_register_client);
/**
 *	type_c_unregister_client - unregister a client notifier
 *	@nb: notifier block to callback on events
 */
void type_c_get_unregister_client(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&type_c_get_notifier_list, nb);
}
EXPORT_SYMBOL(type_c_get_unregister_client);

void type_c_set_unregister_client(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&type_c_set_notifier_list, nb);
}
EXPORT_SYMBOL(type_c_set_unregister_client);

void type_c_otg_power_unregister_client(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&type_c_otg_power_notifier_list, nb);
}
EXPORT_SYMBOL(type_c_otg_power_unregister_client);

/**
 * usb_notifier_call_chain - notify clients of usb_events
 *
 */

int type_c_get_notifier_call_chain(unsigned long val,void *v)
{
	return blocking_notifier_call_chain(&type_c_get_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(type_c_get_notifier_call_chain);

int type_c_set_notifier_call_chain(unsigned long val,void *v)
{
	return blocking_notifier_call_chain(&type_c_set_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(type_c_set_notifier_call_chain);

int type_c_otg_power_notifier_call_chain(unsigned long val,void *v)
{
	return blocking_notifier_call_chain(&type_c_otg_power_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(type_c_otg_power_notifier_call_chain);

