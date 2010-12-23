/*
 *  Backlight Driver for Chumby Silvermoon
 *
 *  Copyright (c) 2009 Sean Cross
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <mach/mfp-pxa168.h>
#include <mach/regs-apbc.h>

#define HW_LEVEL_MAX 0x400
#define HW_LEVEL_MIN 0x0

#define PWM1_BASE (APB_VIRT_BASE + 0x1a400)

/*
 * Nasty, platform-specific stuff that probably oughtn't go here.
 * We use PWM2 (sometimes confusingly called PWM1 in the docs about half
 * the time, at called PWM0 at least once).
 */
#define PWM1_CRX PWM1_BASE+0L
#define PWM1_DCR PWM1_BASE+4L
#define PWM1_PCR PWM1_BASE+8L

static int silvermoon_set_intensity(struct backlight_device *bd)
{
	int intensity = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;

	/* Normalize the intensity, to make lower values brightner */
	intensity = (intensity*intensity)/100;

	__raw_writel((intensity+HW_LEVEL_MIN)*HW_LEVEL_MAX/100, PWM1_DCR);

	return 0;
}

static int silvermoon_get_intensity(struct backlight_device *bd)
{
	return (__raw_readl(PWM1_DCR) - HW_LEVEL_MIN)*100/(HW_LEVEL_MAX-HW_LEVEL_MIN);
}

static struct backlight_ops silvermoon_ops = {
	.get_brightness = silvermoon_get_intensity,
	.update_status = silvermoon_set_intensity,
};

static int silvermoon_probe(struct platform_device *pdev)
{
	struct backlight_device *silvermoon_backlight_device;
	static mfp_cfg_t backlight_pin[] = {
		MFP_CFG(GPIO84, AF2),
	};

	/* Use GPIO84 as PWM2 */
	mfp_config(backlight_pin, ARRAY_SIZE(backlight_pin));

	/*
	 * Bring PWM0 and PWM1 (sic) out of reset.
	 * Not sure why both are required.
	 */
	__raw_writel(3, APBC_PXA168_PWM0);
	__raw_writel(3, APBC_PXA168_PWM1);

	/* Set up PWM2 (sic) to have a period of 64, the longest it can be */
	__raw_writel(0x3f, PWM1_CRX);

	/* Set the duty cycle as long as it can be */
	__raw_writel(0x3ff, PWM1_DCR);

	/* Set the period cycle as long as it can be */
	__raw_writel(0x3ff, PWM1_PCR);

	silvermoon_backlight_device = backlight_device_register(pdev->name,
							     &pdev->dev, NULL,
							     &silvermoon_ops);

	if (IS_ERR(silvermoon_backlight_device))
		return PTR_ERR(silvermoon_backlight_device);

	platform_set_drvdata(pdev, silvermoon_backlight_device);

	silvermoon_backlight_device->props.power = FB_BLANK_UNBLANK;
	silvermoon_backlight_device->props.brightness = 100;
	silvermoon_backlight_device->props.max_brightness = 100;
	silvermoon_set_intensity(silvermoon_backlight_device);

	return 0;
}

static int silvermoon_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	backlight_device_unregister(bd);

	return 0;
}

static struct platform_driver silvermoon_driver = {
	.probe = silvermoon_probe,
	.remove = silvermoon_remove,
	.driver = {
		   .name = "silvermoon-bl",
		   },
};

static struct platform_device *silvermoon_device;

static int __init silvermoon_init(void)
{
	return platform_driver_register(&silvermoon_driver);
}

static void __exit silvermoon_exit(void)
{
	platform_device_unregister(silvermoon_device);
	platform_driver_unregister(&silvermoon_driver);
}

module_init(silvermoon_init);
module_exit(silvermoon_exit);

MODULE_AUTHOR("Sean Cross <sean@chumby.com>");
MODULE_DESCRIPTION("chumby Silvermoon Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:silvermoon-bl");
