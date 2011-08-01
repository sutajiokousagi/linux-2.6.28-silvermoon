/*
 * LEDs driver for GPIOs
 *
 * Copyright (C) 2007 8D Technologies inc.
 * Raphael Assenat <raph@8d.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <asm/gpio.h>
#include <linux/io.h>
#include <asm/io.h>


struct netv_led_data {
	struct led_classdev cdev;
	struct resource *res;
	struct clk *clk;
	unsigned char __iomem *membase;
};

static void netv_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct netv_led_data *led_dat =
		container_of(led_cdev, struct netv_led_data, cdev);

	writel(4*(255-value), 4 + led_dat->membase);
}

static int netv_led_probe(struct platform_device *pdev)
{
	struct netv_led_data *leds_data;
	int ret = -ENOMEM;

	leds_data = kzalloc(sizeof(struct netv_led_data), GFP_KERNEL);
	if (!leds_data) {
		dev_err(&pdev->dev, "unable to allocate memory for leds_data");
		return -ENOMEM;
	}

	leds_data->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!leds_data->res) {
		dev_err(&pdev->dev, "could not find memory resource block");
		goto err;
	}

	leds_data->membase = ioremap(leds_data->res->start, leds_data->res->end - leds_data->res->start + 1);
        if (!leds_data->membase) {
		dev_err(&pdev->dev, "could not remap PWM space");
                goto err;
	}

	leds_data->clk = clk_get(&pdev->dev, "LEDS-CLK");
	if (IS_ERR(leds_data->clk)) {
		ret = PTR_ERR(leds_data->clk);
		dev_err(&pdev->dev, "could not find clock");
		goto err;
	}
	clk_enable(leds_data->clk);


	leds_data->cdev.name = "generic:green:blink";
	leds_data->cdev.brightness_set = netv_led_set;
	leds_data->cdev.brightness = LED_OFF;

	ret = led_classdev_register(&pdev->dev, &leds_data->cdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not register LED class");
		goto err;
	}

	/* Set an arbitrary scale of CLK/4.  It doesn't really matter what. */
	writel(3, 0 + leds_data->membase);

	/* Set the maximum length for PWM period */
	writel(1022, 8 + leds_data->membase);

	/* Default the LED to be off */
	writel(1023, 4 + leds_data->membase);

	platform_set_drvdata(pdev, leds_data);

	return 0;

err:
	kfree(leds_data);

	return ret;
}

static int __devexit netv_led_remove(struct platform_device *pdev)
{
	struct netv_led_data *leds_data;

	leds_data = platform_get_drvdata(pdev);
	led_classdev_unregister(&leds_data->cdev);
	kfree(leds_data);

	return 0;
}

#ifdef CONFIG_PM
static int netv_led_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct netv_led_data *leds_data;

	leds_data = platform_get_drvdata(pdev);
	led_classdev_suspend(&leds_data->cdev);

	return 0;
}

static int netv_led_resume(struct platform_device *pdev)
{
	struct netv_led_data *leds_data;

	leds_data = platform_get_drvdata(pdev);
	led_classdev_resume(&leds_data->cdev);

	return 0;
}
#else
#define netv_led_suspend NULL
#define netv_led_resume NULL
#endif

static struct platform_driver netv_led_driver = {
	.probe		= netv_led_probe,
	.remove		= __devexit_p(netv_led_remove),
	.suspend	= netv_led_suspend,
	.resume		= netv_led_resume,
	.driver		= {
		.name	= "leds-netv",
		.owner	= THIS_MODULE,
	},
};

static int __init netv_led_init(void)
{
	return platform_driver_register(&netv_led_driver);
}

static void __exit netv_led_exit(void)
{
	platform_driver_unregister(&netv_led_driver);
}

module_init(netv_led_init);
module_exit(netv_led_exit);

MODULE_AUTHOR("Sean Cross <sean@chumby.com>");
MODULE_DESCRIPTION("NeTV LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-netv");
