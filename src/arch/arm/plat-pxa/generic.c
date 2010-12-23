/*
 *  linux/arch/arm/plat-pxa/generic.c
 *
 *  Code to PXA processor lines
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <plat/generic.h>
#include <linux/module.h>

static int android_project = 0;
static int __init android_setup(char *__unused)
{
	android_project = 1;
	return 1;
}
__setup("android", android_setup);

int is_android(void)
{
	return android_project;
}

EXPORT_SYMBOL(is_android);


