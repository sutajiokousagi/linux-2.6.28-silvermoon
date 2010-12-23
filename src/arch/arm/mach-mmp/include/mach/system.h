/*
 * linux/arch/arm/mach-mmp/include/mach/system.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_SYSTEM_H
#define __ASM_MACH_SYSTEM_H

#include <mach/regs-mpmu.h>
#include <mach/regs-timers.h>

#define MPMU_APRR_WDTR	(1<<4)

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode)
{
	u32 reg;

	/* negate hardware reset to the WDT after system reset */
	reg = readl(MPMU_APRR) | MPMU_APRR_WDTR;
	writel(reg, MPMU_APRR);

	/* reset/enable WDT clock */
	writel(0x7, MPMU_WDTPCR);
	readl(MPMU_WDTPCR);
	writel(0x3, MPMU_WDTPCR);
	readl(MPMU_WDTPCR);

	/* clear previous WDT status */
	writel(0xbaba, TIMERS1_VIRT_BASE + TMR_WFAR);
	writel(0xeb10, TIMERS1_VIRT_BASE + TMR_WSAR);
	writel(0, TIMERS1_VIRT_BASE + TMR_WSR);

	/* set match counter */
	writel(0xbaba, TIMERS1_VIRT_BASE + TMR_WFAR);
	writel(0xeb10, TIMERS1_VIRT_BASE + TMR_WSAR);
	writel(0xf, TIMERS1_VIRT_BASE + TMR_WMR);

	/* enable WDT reset */
	writel(0xbaba, TIMERS1_VIRT_BASE + TMR_WFAR);
	writel(0xeb10, TIMERS1_VIRT_BASE + TMR_WSAR);
	writel(0x3, TIMERS1_VIRT_BASE + TMR_WMER);

	/* cpu_reset(0); */
}
#endif /* __ASM_MACH_SYSTEM_H */
