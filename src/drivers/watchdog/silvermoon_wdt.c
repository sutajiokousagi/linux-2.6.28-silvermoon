/*
 *	Watchdog driver for the SA11x0/PXA2xx
 *
 *      (c) Copyright 2000 Oleg Drokin <green@crimea.edu>
 *          Based on SoftDog driver by Alan Cox <alan@lxorguk.ukuu.org.uk>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 *	Neither Oleg Drokin nor iXcelerator.com admit liability nor provide
 *	warranty for any of this software. This material is provided
 *	"AS-IS" and at no charge.
 *
 *	(c) Copyright 2000           Oleg Drokin <green@crimea.edu>
 *
 *      27/11/2000 Initial release
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/uaccess.h>

#include <mach/regs-timers.h>

#include <asm/io.h>

static long timeout = 15*256;	/* (secs) */
static unsigned long wdt_users;
static unsigned long boot_status;

static void wdt_write(unsigned long value, unsigned long reg) {
	writel(0xbaba, TIMERS1_VIRT_BASE + TMR_WFAR);
	writel(0xeb10, TIMERS1_VIRT_BASE + TMR_WSAR);
	writel(value, TIMERS1_VIRT_BASE + reg);
}


/*
 * Sets the timeout and starts the timer running
 */
static void program_timeout(unsigned long time) {
	/* Reset the old value */
	wdt_write(1,    TMR_WCR);

	/* Program the new time */
	wdt_write(time, TMR_WMR);

	/* Enable the WDT, and set it to generate a reset signal */
	wdt_write(3,    TMR_WMER);
}


/*
 *	Allow only one person to hold it open
 */
static int silvermoon_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(1, &wdt_users))
		return -EBUSY;

	program_timeout(timeout);
	return nonseekable_open(inode, file);
}


/*
 * This ought to disable the watchdog
 */
static int silvermoon_wdt_release(struct inode *inode, struct file *file)
{
	wdt_write(0, TMR_WMER);
	clear_bit(1, &wdt_users);
	return 0;
}

static ssize_t silvermoon_wdt_write(struct file *file, const char __user *data,
						size_t len, loff_t *ppos)
{
	if (len)
		wdt_write(1, TMR_WCR);
	return len;
}

static const struct watchdog_info ident = {
	.options	= WDIOF_CARDRESET | WDIOF_SETTIMEOUT
				| WDIOF_KEEPALIVEPING,
	.identity	= "Silvermoon/PXA168 Watchdog",
};

static long silvermoon_wdt_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	int ret = -ENOTTY;
	int time;
	void __user *argp = (void __user *)arg;
	int __user *p = argp;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		ret = copy_to_user(argp, &ident,
				   sizeof(ident)) ? -EFAULT : 0;
		break;

	case WDIOC_GETSTATUS:
		ret = put_user(0, p);
		break;

	case WDIOC_GETBOOTSTATUS:
		ret = put_user(boot_status, p);
		break;

	case WDIOC_KEEPALIVE:
		wdt_write(1, TMR_WCR);
		ret = 0;
		break;

	case WDIOC_SETTIMEOUT:
		ret = get_user(time, p);
		if (ret)
			break;

		/* The register is 16-bits.  Limit to 0xffff. */
		time *= 256;
		if (time <= 0 || time > 0xffff) {
			ret = -EINVAL;
			break;
		}
		timeout = time;

		program_timeout(timeout);
		/*fall through*/

	case WDIOC_GETTIMEOUT:
		ret = put_user(timeout, p);
		break;
	}
	return ret;
}

static const struct file_operations silvermoon_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= silvermoon_wdt_write,
	.unlocked_ioctl	= silvermoon_wdt_ioctl,
	.open		= silvermoon_wdt_open,
	.release	= silvermoon_wdt_release,
};

static struct miscdevice silvermoon_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &silvermoon_wdt_fops,
};

static int __init silvermoon_wdt_init(void)
{
	int ret;

	/*
	 * Read the reset status, and save it for later.  If
	 * we suspend, RCSR will be cleared, and the watchdog
	 * reset reason will be lost.
	 */
	boot_status = (readl(TIMERS1_VIRT_BASE + TMR_WSR)&1) ?
				WDIOF_CARDRESET : 0;

	ret = misc_register(&silvermoon_wdt_miscdev);
	if (ret != 0)
		return ret;

	return ret;
}

static void __exit silvermoon_wdt_exit(void)
{
	misc_deregister(&silvermoon_wdt_miscdev);
}

module_init(silvermoon_wdt_init);
module_exit(silvermoon_wdt_exit);

MODULE_AUTHOR("Sean Cross <sean@chumby.com>");
MODULE_DESCRIPTION("Silvermoon/PXA168 Watchdog");

module_param(timeout, long, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds (default 15s)");

MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
