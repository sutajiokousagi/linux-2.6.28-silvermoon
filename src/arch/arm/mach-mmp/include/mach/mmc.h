#ifndef ASMARM_ARCH_MMC_H
#define ASMARM_ARCH_MMC_H

#include <linux/mmc/host.h>
#include <linux/interrupt.h>

struct device;
struct mmc_host;

struct pxasdh_platform_data {
	unsigned int ocr_mask;			/* available voltages */
	unsigned long detect_delay;		/* delay in jiffies before detecting cards after interrupt */
	unsigned long bus_width;
	int (*init)(struct device *, irq_handler_t , void *);
	void (*setpower)(struct device *, unsigned int);
	void (*exit)(struct device *, void *);
	void (*mfp_config)(void);

};

#endif
