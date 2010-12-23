#ifndef __ASM_MACH_CPUTYPE_H
#define __ASM_MACH_CPUTYPE_H

#include <asm/io.h>
#include <asm/cputype.h>
#include <mach/addr-map.h>

#define CHIP_ID		(AXI_VIRT_BASE + 0x82c00)

/*
 *  CPU   Stepping   OLD_ID       CPU_ID      CHIP_ID
 *
 * PXA168    A0    0x41159263   0x56158400   0x00A0A333
 * PXA910    Y0    0x41159262   0x56158000   0x00F0C910
 */

#ifndef CONFIG_CPU_MOHAWK_OLD_ID

#ifdef CONFIG_CPU_PXA168
#  define __cpu_is_pxa168(id)	\
	({ unsigned int _id = ((id) >> 8) & 0xff; _id == 0x84; })
#else
#  define __cpu_is_pxa168(id)	(0)
#endif

#ifdef CONFIG_CPU_PXA910
#  define __cpu_is_pxa910(id)	\
	({ unsigned int _id = ((id) >> 8) & 0xff; _id == 0x80; })
#else
#  define __cpu_is_pxa910(id)	(0)
#endif

#else

#ifdef CONFIG_CPU_PXA168
#  define __cpu_is_pxa168(id)	\
	({ unsigned int _id = (id) & 0xffff; _id == 0x9263; })
#else
#  define __cpu_is_pxa168(id)	(0)
#endif

#ifdef CONFIG_CPU_PXA910
#  define __cpu_is_pxa910(id)	\
	({ unsigned int _id = (id) & 0xffff; _id == 0x9262; })
#else
#  define __cpu_is_pxa910(id)	(0)
#endif

#endif /* CONFIG_CPU_MOHAWK_OLD_ID */

#ifdef CONFIG_MACH_CHUMBY_SILVERMOON
#warning Hacking the CPU type here, because it's coming back wrong
#define cpu_is_pxa168() 1
#define cpu_is_pxa910() 0
#else
#define cpu_is_pxa168()		({ __cpu_is_pxa168(read_cpuid_id()); })
#define cpu_is_pxa910()		({ __cpu_is_pxa910(read_cpuid_id()); })
#endif

static inline int cpu_is_pxa910_z0(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	if (cpu_is_pxa910() && ((chip_id & 0x00f00000) == 0x00a00000))
		return 1;
	else
		return 0;
}

static inline int cpu_is_pxa168_S0(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	if (cpu_is_pxa168() && ((chip_id & 0x0000ffff) == 0x0000c910))
		return 1;
	else
		return 0;
}

static inline int cpu_is_pxa168_A0(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	if (cpu_is_pxa168() && ((chip_id & 0x0000ffff) == 0x0000a168))
		return 1;
	else
		return 0;
}


#endif /* __ASM_MACH_CPUTYPE_H */
