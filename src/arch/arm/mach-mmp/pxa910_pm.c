/*
 * PXA910 Power Management Routines
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2009 Marvell International Ltd.
 * All Rights Reserved
 */

#undef DEBUG
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/kobject.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/cputype.h>
#include <mach/pxa910_pm.h>

static unsigned long pm_state;

static int pxa910_pm_enter(suspend_state_t state)
{
	return -EINVAL;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int pxa910_pm_prepare(void)
{
	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void pxa910_pm_finish(void)
{
	pm_state = PM_SUSPEND_ON;
}

static int pxa910_pm_valid(suspend_state_t state)
{
	int ret = 1;

	if (state == PM_SUSPEND_STANDBY) {
		pm_state = PM_SUSPEND_STANDBY;
	} else {
		ret = 0;
	}
	return ret;
}

/*
 * Set to PM_DISK_FIRMWARE so we can quickly veto suspend-to-disk.
 */
static struct platform_suspend_ops pxa910_pm_ops = {
	.valid		= pxa910_pm_valid,
	.prepare	= pxa910_pm_prepare,
	.enter		= pxa910_pm_enter,
	.finish		= pxa910_pm_finish,
};

static int __init pxa910_pm_init(void)
{
	if (!cpu_is_pxa910())
		return -EIO;
	suspend_set_ops(&pxa910_pm_ops);
	return 0;
}

late_initcall(pxa910_pm_init);
