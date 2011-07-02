/*
 * linux/drivers/input/gpio_ir.c
 *
 * Support for the Aspenite /Zyloniteii /Teton development Platforms
 *
 * Copyright (C) 2008 Marvell International Ltd.
 *
 * 2008-010-12: Ofer Zaarur <ofer.zaarur@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define DEBUG

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/irq.h>

#include <mach/gpio_ir.h>
#include <mach/ir_key_def.h>
#include <mach/timer_services.h>

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/gpio.h>

MODULE_LICENSE("GPL");

/*
 * Nec SC33 code:
 * ___                  _-_-_-_-_-_-_-_~~
 * Pre 0 1 1 1 0 1 1 0  1 0 0 0 1 0 0 1
 */


/*
 * for Nex protocol analysis
 */
#define ENDED 		100000 	/*uSec*/
#define END_MIN		35000 	/*uSec*/
#define END_MAX		42000 	/*uSec*/
#define PREAM_MIN	12000	/*uSec*/
#define PREAM_MAX	21000	/*uSec*/
#define ONE_MIN		2100	/*uSec*/
#define ONE_MAX		4100	/*uSec*/
#define ZERO_MIN	800	/*uSec*/
#define ZERO_MAX	2100	/*uSec*/
#define word_size 	32     	/*word size according to the protocol */
#define calibration_n 	2167    /* timer resolution eq=1500 OS timer HZ */


#define IR_SHIFT(x)     (x % (sizeof(int) * 8)) /* Shift6*/
#define GPIO_BIT(x)     (1 << ((x) & 0x1f))

/*
 * Circular buffer
 */
#define CIRC_BUFF_MASK 0x3ff
#define CIRC_BUFF_LENGTH 0x400
static unsigned int cbuffer[CIRC_BUFF_LENGTH];
static unsigned int cb_start=0, cb_end=0;


/*
 * Table of IR-signal-code and key
 */
ir_key_table_t nikon_key_table[] = {
	{0xb946f685, KEY_CONFIG},
	{0xba45f685, KEY_VENDOR},
	{0xbc43f685, KEY_UP},
	{0xbd42f685, KEY_LEFT},
	{0xbf40f685, KEY_ENTER},
	{0xbe41f685, KEY_RIGHT},
	{0xbb44f685, KEY_DOWN},
	{0xb748f685, KEY_1},
	{0xb847f685, KEY_2},
	{0,          0},
};

ir_key_table_t *ir_key_table = nikon_key_table;



static int decode_command(struct cir_device *cir, unsigned long cmd)
{
	ir_key_table_t *key = ir_key_table;
	while (key->ir_encode) {
		if (key->ir_encode == cmd) {
			dev_dbg(&cir->pdev->dev,
				"Found key %d\n", key->ir_key);
			input_report_key(cir->input_dev, key->ir_key , 1);
			input_report_key(cir->input_dev, key->ir_key , 0);
			return 0;
		}
		key++;
	}
	dev_err(&cir->pdev->dev, "No key found for code 0x%08x\n", cmd);
	return 1;
}

/*
 * This function should be called in an independent thread.
 * It reads out bit-wised code.
 */
static void process_times(unsigned long data)
{
	struct cir_device *cir = (struct cir_device *)data;
	int event_count = 0;
	unsigned long cmd;
	unsigned int cmd_shift;
	unsigned long length, active;

	cmd = 0;
	cmd_shift = 0;

	while (cb_start != cb_end) {
		event_count++;
		length = cbuffer[cb_start] & ~0x80000000L;
		active = !(cbuffer[cb_start] >> 31);

		/* Advance to the next spot in the buffer */
		cb_start = (cb_start+1) & CIRC_BUFF_MASK;

		/* For this protocol, we only consider the rising edges */
		if (!active)
			continue;

		/* Convert from ticks to usecs */
		length = (length*10000)>>15;


		/* Start-of-packet messages are about 4.5 ms */
		if (length > 2000 || length == 0) {
			cmd_shift = 0;
			continue;
		}

		else if (length <= 2000 && length > 1300)
			cmd |= (1<<cmd_shift++);
		else if (length <= 700 && length > 400)
			cmd |= (0<<cmd_shift++);
		else
			dev_err(&cir->pdev->dev,
				"Event %lu doesn't match anything\n", length);

		//dev_dbg(&cir->pdev->dev, "Event %lu usec long: %lu\n",
		//	length, active);

	}
	if (cmd)
		decode_command(cir, cmd);

	if (cmd)
		dev_dbg(&cir->pdev->dev, "Read out %d events: 0x%08x\n",
			event_count, cmd);
	return;
}

DEFINE_TIMER(process_time_timer, process_times, 0, 0);


static irqreturn_t cir_interrupt(int irq, void *dev_id)
{
	struct cir_device *cir = (struct cir_device *)dev_id;
	static unsigned long this, prev;

	prev = this;
	this = timer_services_counter_read(COUNTER_0);

	/* This is true when the buffer has just been emptied, so
	 * the time will need to be 0.
	 */
	if (cb_end == cb_start)
		prev = this;

	cbuffer[cb_end] = ((this - prev) & 0x7fffffffL)
		 	| ((__raw_readl(cir->mmio_base + 0x00) & 0x40) << 25);

	cb_end = (cb_end+1) & CIRC_BUFF_MASK;
	if (cb_end == cb_start)
		cb_start = (cb_start+1) & CIRC_BUFF_MASK;

	process_time_timer.data = (unsigned long)cir;
	mod_timer(&process_time_timer, jiffies + msecs_to_jiffies(20));

	return IRQ_HANDLED;
}

/**
 * cir_enable - enable the cir -unmask
 *
 * Turn on the cir.
 */
void cir_enable(struct cir_dev *dev)
{
	struct cir_device *cir = dev->cir;
	__raw_writel(GPIO_BIT(IR_SHIFT(cir->pin)), cir->mmio_base + GAPMASK0);
	__raw_writel(GPIO_BIT(IR_SHIFT(cir->pin)), cir->mmio_base + GSFER0);
	__raw_writel(GPIO_BIT(IR_SHIFT(cir->pin)), cir->mmio_base + GSRER0);
}

EXPORT_SYMBOL(cir_enable);

/**
 * cir_disable - shut down the cir - mask
 *
 * Turn off the cir port.
 */
void cir_disable(struct cir_dev *dev)
{
	struct cir_device *cir = dev->cir;

	__raw_writel(GPIO_BIT(IR_SHIFT(cir->pin)), cir->mmio_base + GCPMASK0);
}

EXPORT_SYMBOL(cir_disable);

#ifdef CONFIG_PM
/*
 * Basic power management.
 */

static int cir_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* require timer services can be disabled */
	return 0;
}

static int cir_resume(struct platform_device *pdev)
{
	/* require timer services resume to be enabled */
	return 0;
}
#else
#define cir_suspend	NULL
#define cir_resume	NULL
#endif

static int __devinit cir_probe(struct platform_device *pdev)
{
	struct cir_device *cir;
	struct resource *res;
	int ret = 0;
	int key;

   	cir = kzalloc(sizeof(struct cir_device), GFP_KERNEL);
   	if (cir == NULL) {
      		dev_err(&pdev->dev, "failed to allocate memory\n");
      		return -ENOMEM;
   	}
	
	/* Allocate an input device data structure */
	cir->input_dev = input_allocate_device();
	if (!cir->input_dev) {
		kfree(cir);
		return -ENOMEM;
	}

	/* Allocate GPIO memory range */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev,"no memory resource defined\n");
		input_free_device(cir->input_dev);
		kfree(cir);
		ret = -ENODEV;
	}
	cir->mmio_base = ioremap_nocache(res->start, SZ_256);
	dev_dbg(&pdev->dev, "cir->address 0x%p", cir->mmio_base);

	/* Figure out which GPIO IRQ to use */
	cir->irq = platform_get_irq(pdev, 0);
	dev_dbg(&pdev->dev, "cir->irq : %d \n", cir->irq);
	if (cir->irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource defined\n");
		ret = -ENODEV;
		goto err_free_io;
	}

	/* Actually set up the IRQ */
	ret = request_irq(cir->irq, cir_interrupt,
			  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			  "IR port", cir);
	dev_dbg(&pdev->dev, "ret from request irq. %d\n", ret);
   	if (ret) {
		dev_err(&pdev->dev, "Unable to request IRQ: %d\n", ret);
		goto err_free_io;
	}

	cir->pin = IRQ_TO_GPIO(cir->irq);

	platform_set_drvdata(pdev, cir);
	dev_dbg(&pdev->dev, " Initialize CIR driver completed \n");

	/* setup input device */
	cir->input_dev->name = "aspenite_cir";
	cir->input_dev->phys = "aspenite_cir/input2";
	cir->input_dev->dev.parent = &pdev->dev;
	cir->input_dev->evbit[0] = BIT(EV_KEY);
	cir->pdev = pdev;

	/* Announce that the CIR will generate  key map */
	for (key=0; ir_key_table[key].ir_key; key++)
		set_bit(ir_key_table[key].ir_key, cir->input_dev->keybit);
	dev_dbg(&pdev->dev, "Key map was set \n");

	/* Register with the input subsystem */
	if (input_register_device(cir->input_dev)) 
		dev_err(&pdev->dev, "Can't register CIR_input Driver.\n");
	dev_dbg(&pdev->dev, "Driver Initialized.\n");

	return 0;
	
err_free_io:
	if (cir && cir->irq > 0)
		free_irq(cir->irq, cir);
	if (cir && cir->input_dev)
		input_free_device(cir->input_dev);
	if (cir)
		kfree(cir);
	return ret;
}

static int __devexit cir_remove(struct platform_device *pdev)
{
	struct cir_device *cir;
	
	cir = platform_get_drvdata(pdev);
	if (cir == NULL)
		return -ENODEV;

	if (cir->irq > 0)
		free_irq(cir->irq, cir);

	if (cir->input_dev)
		input_unregister_device(cir->input_dev);

	kfree(cir);
	return 0;
}

static struct platform_driver aspenite_cir_driver = {
	.driver		= {
		.name	= "aspenite-cir",
	},
	.probe		= cir_probe,
	.remove		= __devexit_p(cir_remove),
	.suspend	= cir_suspend,
	.resume		= cir_resume,
};


static int __init aspenite_cir_init(void)
{
	int ret = 0;
	ret = platform_driver_register(&aspenite_cir_driver);
	if (ret) {
		printk(KERN_ERR "failed to register aspenite_cir_driver");
		return ret;
	}
	return ret;
}

static void __exit aspenite_cir_exit(void)
{
	platform_driver_unregister(&aspenite_cir_driver);
}

late_initcall(aspenite_cir_init);
module_exit(aspenite_cir_exit);
