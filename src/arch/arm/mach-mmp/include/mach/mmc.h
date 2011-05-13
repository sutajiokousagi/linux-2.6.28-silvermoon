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
	unsigned int card_detect_ok;
	int (*init)(struct device *, irq_handler_t , void *);
	void (*setpower)(struct device *, unsigned int);
	void (*exit)(struct device *, void *);
	void (*mfp_config)(void);

	/* only way to meet 74 init clk requirement with this controller is  */
	/* to explicitly generate them.                                      */
	/* note: keep these fields in the slot structure, since it is each   */
	/* card that needs the init clocks, not just the controller.         */
	unsigned int sent_init_clks;		/* true after init clks sent */
	unsigned int cmdpad_raw_pin;		/* gpio used for mmc_cmd     */
	unsigned int cmdpad_as_cmd;		/* pad val for mmc_cmd mode  */
	unsigned int cmdpad_as_gpio;		/* pad val for gpio mode     */
	unsigned int clkpad_raw_pin;		/* gpio used for mmc_clk     */
	unsigned int clkpad_as_clk;		/* pad val for mmc_clk mode  */
	unsigned int clkpad_as_gpio;		/* pad val for gpio mode     */
};

#endif
