/*
 * am300epd.c -- Platform device for AM300 EPD kit
 *
 * Copyright (C) 2008, Jaya Kumar
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 * This work was made possible by help and equipment support from E-Ink
 * Corporation. http://support.eink.com/community
 *
 * This driver is written to be used with the Broadsheet display controller.
 * on the AM300 EPD prototype kit/development kit with an E-Ink 800x600
 * Vizplex EPD on a Gumstix board using the Broadsheet interface board.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>

#include <mach/mfp-pxa168.h>

#include "common.h"

#include <video/broadsheetfb.h>

#define USE_READY 1

#define SMC_BASE        0xD4283800
#define SMC_CSDFICFGX   0x0090       /* DFI Configuration Control
                                                 * Register for Chip
                                                 * Selects Start
                                                 */
// SMC registers

#define SMC(r) (*(volatile unsigned short *)((char*)_mapSMC+(r)))
static void * _mapSMC;

const off_t SMC_ADDR_BASE = 0x80000000;
const off_t SMC_ACCESS_DATA = 0x10;
const off_t SMC_WRITE_COMMAND = 0x0;

static unsigned int panel_type = 6;
static struct platform_device *am300_device;
static struct broadsheet_board am300_board;

static unsigned long am300_pin_config[] __initdata = {
	GPIO20_GPIO,
	GPIO27_GPIO,
#ifdef USE_READY
	GPIO28_GPIO,
#endif
	GPIO35_GPIO,
	GPIO36_GPIO
};

/* register offsets for gpio control */
#define PWR_GPIO_PIN    36
#define CFG_GPIO_PIN    20
#define RST_GPIO_PIN    35
#define IRQ_GPIO_PIN    27
#ifdef USE_READY
#define RDY_GPIO_PIN    28
#endif
static int gpios[] = { PWR_GPIO_PIN, CFG_GPIO_PIN, RST_GPIO_PIN, 
			IRQ_GPIO_PIN
#ifdef USE_READY
, RDY_GPIO_PIN
#endif
 };
static char *gpio_names[] = { "PWR", "CFG", "RST", "IRQ"
#ifdef USE_READY
, "RDY"
#endif
 };

#ifdef USE_READY
static int am300_wait_for_ready( void )
{
	int d = __gpio_get_value(RDY_GPIO_PIN);
	int count = 0;
	while ( d == 0 && count < 100000) {
		d = __gpio_get_value(RDY_GPIO_PIN);
		count++;
	}
	return d;
}
#endif

static void am300_reset(void)
{
  int count = 0;
  printk("am300 reset\n");
	gpio_direction_output(CFG_GPIO_PIN, 1);
	gpio_direction_output(RST_GPIO_PIN, 0);
	msleep(1000);
	gpio_direction_output(RST_GPIO_PIN, 1);
	while (!__gpio_get_value(RDY_GPIO_PIN)) count++;
	printk("count = %d\n", count);
}

static void am300_set_gpio_regs(struct broadsheetfb_par *par)
{
	int i;
	/* go into command mode */
	gpio_set_value(CFG_GPIO_PIN, 1);
	gpio_set_value(RST_GPIO_PIN, 0);
	msleep(500);
	gpio_set_value(RST_GPIO_PIN, 1);
	msleep(500);
}

static int am300_init_gpio_regs(struct broadsheetfb_par *par)
{
	int i;
	int err;
	for (i = 0; i < ARRAY_SIZE(gpios); i++) {
		err = gpio_request(gpios[i], gpio_names[i]);
		if (err) {
			dev_err(&am300_device->dev, "failed requesting "
				"gpio %s, err=%d\n", gpio_names[i], err);
			goto err_req_gpio;
		}
	}

	/* setup the outputs and init values */
	gpio_direction_output(PWR_GPIO_PIN, 1);   /*** power off  ***/
	msleep(1000);
	gpio_direction_output(PWR_GPIO_PIN, 0);   /*** power on  ***/
	gpio_direction_output(CFG_GPIO_PIN, 1);
	gpio_direction_output(RST_GPIO_PIN, 0);
	msleep(1000);

	/* setup the inputs */
	gpio_direction_input(IRQ_GPIO_PIN);
#ifdef USE_READY
	gpio_direction_input(RDY_GPIO_PIN);
#endif
	am300_set_gpio_regs(par);

	return 0;
err_req_gpio:
	while (i > 0)
		gpio_free(gpios[i--]);
	return err;
}

static int am300_SMCsetup(void) {

        void *base;
	int retval = 0;

        /* set up the  SMC configuration register for suspend resume */
        base = ioremap(SMC_BASE, PAGE_SIZE);
        if (base == NULL) {
           dev_err(&am300_device->dev, "failed to remap I/O memory\n");
           retval = -ENXIO;
        } else {
          (*(volatile unsigned int *)(base + SMC_CSDFICFGX)) = 0x52010018;
	  iounmap(base);
	}

#define SMC_CLOCK 0xd42828d4
#define SMC_CLOCK_VALUE 0x5b

        /* set up the SMC clock */
        base = ioremap(SMC_CLOCK, 4);
        if (base == NULL) {
           dev_err(&am300_device->dev, "failed to remap I/O memory\n");
           retval = -ENXIO;
        } else {
	  (*(volatile unsigned int *)base) = SMC_CLOCK_VALUE;
	  iounmap(base);
	}

        printk(KERN_INFO "Broadsheet frame buffer SMC setup complete\n");
	return retval;
}

static int am300_init_board(struct broadsheetfb_par *par)
{
	unsigned int res_start = SMC_ADDR_BASE;
	unsigned int res_size = 0x1000;
	int retval = am300_SMCsetup();

        if (! retval) {
	  _mapSMC = ioremap(res_start, res_size);
	  printk("_mapSMC = 0x%x,  res_start = 0x%x,  res_size = 0x%x\n",
		 (unsigned int)_mapSMC, res_start, res_size );
	  if (_mapSMC == NULL) {
	    dev_err(&am300_device->dev, "failed to remap I/O memory\n");
	    retval = -ENXIO;
	  }
	}
	
	return retval ? retval : am300_init_gpio_regs(par);
}

static int am300_suspend(struct broadsheetfb_par *par)
{
        return 0;
}

static int am300_resume(struct broadsheetfb_par *par)
{
	am300_set_gpio_regs(par);
        return am300_SMCsetup();
}

static void am300_cleanup(struct broadsheetfb_par *par)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(gpios); i++)
		gpio_free(gpios[i]);

}

static u16 am300_read(struct broadsheetfb_par *par)
{
	u16 res;

#ifdef USE_READY
	am300_wait_for_ready( );
#endif
	res =  SMC(SMC_ACCESS_DATA);

	return res;
}

static void am300_writedata(struct broadsheetfb_par *par, u16 data)
{
#ifdef USE_READY
	am300_wait_for_ready( );
#endif
	SMC(SMC_ACCESS_DATA)= data;
}


static void am300_writecmd(struct broadsheetfb_par *par, u16 data)
{
#ifdef USE_READY
	am300_wait_for_ready( );
#endif
	SMC(SMC_WRITE_COMMAND)= data;
}

static int am300_get_panel_type(void)
{
	return panel_type;
}

static irqreturn_t am300_handle_irq(int irq, void *dev_id)
{
	struct broadsheetfb_par *par = dev_id;

	wake_up(&par->waitq);
	return IRQ_HANDLED;
}

static int am300_setup_irq(struct fb_info *info)
{
#if 0
	int ret;
	struct broadsheetfb_par *par = info->par;

	ret = request_irq(IRQ_GPIO(IRQ_GPIO_PIN), am300_handle_irq,
			  IRQF_DISABLED|IRQF_TRIGGER_RISING,
			  "AM300", par);
	if (ret)
		dev_err(&am300_device->dev, "request_irq failed: %d\n", ret);

	return ret;
#else
        return 0;
#endif
}

static struct broadsheet_board am300_board = {
	.owner			= THIS_MODULE,
	.init			= am300_init_board,
	.cleanup		= am300_cleanup,
	.writedata		= am300_writedata,
	.read			= am300_read,
	.writecmd      		= am300_writecmd,
	.get_panel_type		= am300_get_panel_type,
	.setup_irq		= am300_setup_irq,
	.waitready              = am300_wait_for_ready,
	.reset                  = am300_reset,
	.resume                 = am300_resume,
};

int __init am300_init(void)
{
	int ret;
#if 0
	mfp_config(ARRAY_AND_SIZE(am300_pin_config));
#endif
	/* request our platform independent driver */
	request_module("broadsheetfb");

	am300_device = platform_device_alloc("broadsheetfb", -1);
	if (!am300_device)
		return -ENOMEM;

	/* the am300_board that will be seen by broadsheetfb is a copy */
	platform_device_add_data(am300_device, &am300_board,
					sizeof(am300_board));

	ret = platform_device_add(am300_device);

	if (ret) {
		platform_device_put(am300_device);
		return ret;
	}

	return 0;
}

module_param(panel_type, uint, 0);
MODULE_PARM_DESC(panel_type, "Select the panel type: 6, 8, 97");

MODULE_DESCRIPTION("board driver for am300 epd kit");
MODULE_AUTHOR("Jaya Kumar");
MODULE_LICENSE("GPL");

