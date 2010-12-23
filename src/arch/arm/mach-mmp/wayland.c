/*
 *  linux/arch/arm/mach-mmp/wayland.c
 *
 *  Support for the Marvell PXA168-based Wayland platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/smc91x.h>
#include <linux/i2c/pca953x.h>
#include <linux/card.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <plat/pxa_u2o.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa168.h>
#include <mach/pxa168.h>
#include <mach/gpio.h>
#include <mach/pxa3xx_nand.h>
#include <mach/camera.h>
#include <mach/mfp.h>

#include <plat/part_table.h>
#include <plat/generic.h>
#include <plat/pxa3xx_pmic.h>
#include <plat/pxa_u2o.h>
#include <plat/pxa3xx_otg.h>

#include "common.h"
/*used by expander max7319, 8 pins gpio expander */
#define GPIO_EXT0(x)		(NR_BUILTIN_GPIO + (x))

static unsigned long wayland_pin_config[] __initdata = {

#if defined(CONFIG_MTD_NAND) || defined(CONFIG_PXA168_CF)
	/* Data Flash Interface */
	GPIO0_DFI_D15,
	GPIO1_DFI_D14,
	GPIO2_DFI_D13,
	GPIO3_DFI_D12,
	GPIO4_DFI_D11,
	GPIO5_DFI_D10,
	GPIO6_DFI_D9,
	GPIO7_DFI_D8,
	GPIO8_DFI_D7,
	GPIO9_DFI_D6,
	GPIO10_DFI_D5,
	GPIO11_DFI_D4,
	GPIO12_DFI_D3,
	GPIO13_DFI_D2,
	GPIO14_DFI_D1,
	GPIO15_DFI_D0,
	GPIO16_ND_nCS0,

	/* Static Memory Controller */
#ifndef CONFIG_PXA168_CF
	GPIO18_SMC_nCS0,
	GPIO34_SMC_nCS1,
	GPIO23_SMC_nLUA,
	GPIO25_SMC_nLLA,
	GPIO28_SMC_RDY,
	GPIO29_SMC_SCLK,
	GPIO35_SMC_BE1,
	GPIO36_SMC_BE2,
#endif
#endif

#if defined(CONFIG_FB_BROADSHEET)
	/* am300epd */
	GPIO20_GPIO,
	GPIO27_GPIO,
	GPIO28_GPIO,
	GPIO35_GPIO,
	GPIO36_GPIO,
#endif

	/* i2c bus */
	GPIO105_CI2C_SDA,
	GPIO106_CI2C_SCL,

	/* UART1 */
	GPIO107_UART1_TXD,
	GPIO108_UART1_RXD,

	/* UART2 */
	GPIO88_UART2_SOUT,
	GPIO89_UART2_SIN,

	/* MMC1 */
	GPIO51_MMC1_DAT3,
	GPIO52_MMC1_DAT2,
	GPIO40_MMC1_DAT1,
	GPIO41_MMC1_DAT0,
	GPIO49_MMC1_CMD,
	GPIO43_MMC1_CLK,
	GPIO53_MMC1_CD,
	GPIO46_MMC1_WP,

#if defined(CONFIG_WLAN_8688_SDIO)
	/* sdh MMC2, wlan*/
	GPIO90_MMC2_DAT3,
	GPIO91_MMC2_DAT2,
	GPIO92_MMC2_DAT1,
	GPIO93_MMC2_DAT0,
	GPIO94_MMC2_CMD,
	GPIO95_MMC2_CLK,
#endif

};

static struct i2c_pxa_platform_data pwri2c_info __initdata = {
	.use_pio		= 1,
};

static struct i2c_board_info i2c_board_info[] =
{

};

static struct i2c_board_info pwri2c_board_info[] =
{
#if defined(CONFIG_TSC2007)
       {
	       .type	= "tsc2007",
	       .addr	= 0x48,                      /* 0x90/0x91 */
	       .irq	= IRQ_GPIO(GPIO_EXT0(7)),    /* IO7 of TSC2007 */
       },
#endif

};

static unsigned int wayland_matrix_key_map[] = {
	KEY(0, 7, KEY_LEFT),
	KEY(4, 7, KEY_RIGHT),
	KEY(0, 6, KEY_HOME),
	KEY(4, 6, KEY_END),
	KEY(1, 7, KEY_ENTER),	/* keypad action */
	KEY(1, 6, KEY_SEND),
};

static struct pxa27x_keypad_platform_data wayland_keypad_info __initdata = {
	.matrix_key_rows	= 8,
	.matrix_key_cols	= 8,
	.matrix_key_map		= wayland_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(wayland_matrix_key_map),
	.debounce_interval	= 30,
};

DECLARE_512M_V75_FS256M_PARTITIONS(generic_512m_v75_partitions);
static struct pxa3xx_nand_platform_data wayland_nand_info;
static void __init wayland_add_nand(void)
{
	wayland_nand_info.parts[0] = generic_512m_v75_partitions;
	wayland_nand_info.nr_parts[0] = ARRAY_SIZE(generic_512m_v75_partitions);

	wayland_nand_info.use_dma = 0;
	wayland_nand_info.enable_arbiter = 1;
	pxa168_add_nand((struct flash_platform_data *) &wayland_nand_info);
}

#if defined(CONFIG_MMC_PXA_SDH)

static struct pxasdh_platform_data wayland_sdh_platform_data_MMC1 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_29_30 | MMC_VDD_30_31,
};

#if defined(CONFIG_BT_HCIUART)
static mfp_cfg_t wayland_bt_uart_pins[] = {
	GPIO98_UART_SOUT,
	GPIO99_UART_SIN,
	GPIO100_UART_RTS,
	GPIO101_UART_CTS,
};

static void bt_uart_mfp_config(void)
{
	mfp_config(ARRAY_AND_SIZE(wayland_bt_uart_pins));
	return;
}

static void __init wayland_bt_init(void)
{
	bt_uart_mfp_config();
}

#endif

#if defined(CONFIG_WLAN_8688_SDIO)
static int poweron_8688(void)
{
	int gpio_power = 0;
	int gpio_reset = 0;
	int gpio_wake = 0;
	int gpio_h_wake = 0;

	gpio_power = 102;
	gpio_reset = 103;
	gpio_wake = 104;
	gpio_h_wake = 4;

	if (gpio_request(gpio_power, "8688 wlan power down")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_power);
		return -1;
	}

	if (gpio_request(gpio_reset, "8688 wlan reset")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_reset);
		gpio_free(gpio_power);
		return -1;
	}
	if (gpio_request(gpio_wake, "8688 wlan gpio_wake")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_wake);
		gpio_free(gpio_power);
		gpio_free(gpio_reset);
		return -1;
	}
	if (gpio_request(gpio_h_wake, "8688 wlan card gpio_h_wake")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_h_wake);
		gpio_free(gpio_power);
		gpio_free(gpio_reset);
		gpio_free(gpio_wake);
		return -1;
	}

	gpio_direction_output(gpio_power, 0);
	gpio_direction_output(gpio_reset, 0);
	gpio_direction_output(gpio_wake, 0);
	gpio_direction_input(gpio_h_wake);
	mdelay(500);
	gpio_direction_output(gpio_reset, 1);
	gpio_direction_output(gpio_power, 1);
	gpio_direction_output(gpio_wake, 0);

	gpio_free(gpio_power);
	gpio_free(gpio_reset);
	gpio_free(gpio_wake);
	gpio_free(gpio_h_wake);
	return 0;
}

static struct pxasdh_platform_data wayland_sdh_platform_data_MMC2 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_29_30 | MMC_VDD_30_31,
};
#endif
#endif

#ifdef CONFIG_USB_GADGET_PXA_U2O
static int gpio_usb_otg_pen = GPIO_EXT0(0);
static int gpio_usb_otg_stat1 = GPIO_EXT0(8);
static int gpio_usb_otg_stat2 = GPIO_EXT0(9);

static int wayland_u2o_vbus_status(unsigned base)
{
	int otg_stat1, otg_stat2, otg_pen, status = VBUS_HIGH;
	unsigned long flags;

#ifdef CONFIG_USB_OTG
	/* FIXME on aspenite R0 boards otg stat1/stat2 could not
	 * reflect VBUS status yet, check with U2O itself instead
	 */
	if (u2o_get(base, U2xOTGSC) & U2xOTGSC_BSV)
		status = VBUS_HIGH;
	else
		status = VBUS_LOW;

	return status;
#endif

	local_irq_save(flags);

	if (gpio_request(gpio_usb_otg_pen, "USB OTG Power Enable")) {
		printk(KERN_ERR "%s Max7312 USB_OTG_PEN GPIO Request"
			" Failed\n", __func__);
		return -1;
	}

	if (gpio_request(gpio_usb_otg_stat1, "USB OTG VBUS stat1")) {
		printk(KERN_ERR "%s Max7312 USB_OTG_STAT1 GPIO Request"
			" Failed\n", __func__);
		return -1;
	}

	if (gpio_request(gpio_usb_otg_stat2, "USB OTG Power Enable")) {
		printk(KERN_ERR "%s Max7312 USB_OTG_STAT2 GPIO Request"
			" Failed\n", __func__);
		return -1;
	}

	gpio_direction_input(gpio_usb_otg_pen);
	gpio_direction_input(gpio_usb_otg_stat1);
	gpio_direction_input(gpio_usb_otg_stat2);

	otg_pen = __gpio_get_value(gpio_usb_otg_pen);
	otg_stat1 = __gpio_get_value(gpio_usb_otg_stat1);
	otg_stat2 = __gpio_get_value(gpio_usb_otg_stat2);

	if (otg_pen) {
		status = VBUS_CHARGE;
		if (otg_stat1 && otg_stat2) {
			status |= VBUS_HIGH;
		}
	} else {
		if (!otg_stat1 && !otg_stat2) {
			status = VBUS_HIGH;
		}
	}

	printk(KERN_DEBUG "%s otg_pen %d stat1 %d stat2 %d status %d\n\n",
			__func__, otg_pen, otg_stat1, otg_stat2, status);
	gpio_free(gpio_usb_otg_pen);
	gpio_free(gpio_usb_otg_stat1);
	gpio_free(gpio_usb_otg_stat2);

	local_irq_restore(flags);
	return status;
}

static int wayland_u2o_vbus_set(int vbus_type)
{
	unsigned long flags;

	local_irq_save(flags);

	if (gpio_request(gpio_usb_otg_pen, "USB OTG Power Enable")) {
		printk(KERN_ERR "%s Max7312 USB_OTG_PEN GPIO Request"
			" Failed\n", __func__);
		return -1;
	}

	switch (vbus_type) {
	case VBUS_SRP:
		gpio_direction_output(gpio_usb_otg_pen, 1);
		udelay(10);
		gpio_direction_output(gpio_usb_otg_pen, 0);
		break;
	case VBUS_HIGH:
		gpio_direction_output(gpio_usb_otg_pen, 1);
		break;
	case VBUS_LOW:
		gpio_direction_output(gpio_usb_otg_pen, 0);
		break;
	default:
		break;
	}
	gpio_free(gpio_usb_otg_pen);

	local_irq_restore(flags);

	return 0;
}
static int wayland_otg_init(void)
{
	int gpio_usb_otg_stat1 = 0, gpio_usb_otg_stat2 = 0;

	if (gpio_request(gpio_usb_otg_stat1, "USB OTG Host Status 1") &&
	    gpio_request(gpio_usb_otg_stat2, "USB OTG Host Status 2")) {
		printk(KERN_ERR "Max7312 USB OTG Status GPIO Request Failed\n");
		return -EAGAIN;
	}

	gpio_direction_input(gpio_usb_otg_stat1);
	gpio_direction_input(gpio_usb_otg_stat2);
	gpio_free(gpio_usb_otg_stat1);
	gpio_free(gpio_usb_otg_stat2);
	return 0;
}

static int wayland_u2o_vbus_set_ic(int function)
{
	printk(KERN_DEBUG "%s %d not implemented yet\n", __func__, function);
	return 0;
}

static struct otg_pmic_ops wayland_otg_ops = {
	.otg_vbus_init          = wayland_otg_init,
	.otg_set_vbus           = wayland_u2o_vbus_set,
	.otg_set_vbus_ic        = wayland_u2o_vbus_set_ic,
	.otg_get_vbus_state     = wayland_u2o_vbus_status,
};

struct otg_pmic_ops *init_wayland_otg_ops(void)
{
	return &wayland_otg_ops;
}

static struct pxa_usb_plat_info wayland_u2o_info = {
	.phy_init	= pxa168_usb_phy_init,
	.vbus_set	= wayland_u2o_vbus_set,
	.vbus_status	= wayland_u2o_vbus_status,
	.init_pmic_ops	= init_wayland_otg_ops,
	.is_otg		= 1,
};
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H
/* USB 2.0 Host Controller */
static int wayland_u2h_vbus_set(int enable)
{
	int gpio_u2h_vbus_on = GPIO_EXT0(4);
	int gpio_u2h_vbus_flt_n = GPIO_EXT0(14);

	if (gpio_request(gpio_u2h_vbus_on, "USB Host VBUS_ON")) {
		printk(KERN_ERR "Max7312 VBUS_ON GPIO Request Failed\n");
		return -EIO;
	}
	if (gpio_request(gpio_u2h_vbus_flt_n, "USB Host VBUS_FLT_N")) {
		printk(KERN_ERR "Max7312 VBUS_FLT_N GPIO Request Failed\n");
		return -EIO;
	}
	if (gpio_u2h_vbus_on && gpio_u2h_vbus_flt_n) {
		if (enable)
			gpio_direction_output(gpio_u2h_vbus_on, 1);
		else
			gpio_direction_output(gpio_u2h_vbus_on, 0);

		gpio_direction_input(gpio_u2h_vbus_flt_n);
		gpio_free(gpio_u2h_vbus_on);
		gpio_free(gpio_u2h_vbus_flt_n);
	}

	return 0;
}

static struct pxa_usb_plat_info wayland_u2h_info = {
	.phy_init	= pxa168_usb_phy_init,
	.vbus_set	= wayland_u2h_vbus_set,
};
#endif

#if defined(CONFIG_FB_BROADSHEET)
int __attribute__((weak)) am300_init(void)
{
	return 0;
}
#endif

static void __init wayland_init(void)
{
	mfp_config(ARRAY_AND_SIZE(wayland_pin_config));

	/* on-chip devices */
	pxa168_add_uart(1);
	pxa168_add_uart(2);
#if defined(CONFIG_BT_HCIUART)
	pxa168_add_uart(3);
#endif
/*	wayland_add_nand(); */
	pxa168_add_twsi(0, &pwri2c_info,
			   ARRAY_AND_SIZE(i2c_board_info));
	pxa168_add_twsi(1, &pwri2c_info,
			   ARRAY_AND_SIZE(pwri2c_board_info));
	pxa168_add_keypad(&wayland_keypad_info);

#ifdef CONFIG_USB_GADGET_PXA_U2O
	pxa168_add_u2o(&wayland_u2o_info);
#endif

#ifdef CONFIG_USB_OTG
	pxa168_add_u2ootg(&wayland_u2o_info);
	pxa168_add_u2oehci(&wayland_u2o_info);
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H
	pxa168_add_u2h(&wayland_u2h_info);
#endif

#if defined(CONFIG_MMC_PXA_SDH)
	pxa168_add_sdh(0, &wayland_sdh_platform_data_MMC1);
#if defined(CONFIG_WLAN_8688_SDIO)
	poweron_8688();
	pxa168_add_sdh(1, &wayland_sdh_platform_data_MMC2);
#endif
#endif
	pxa168_add_freq();

#if defined(CONFIG_FB_BROADSHEET)
	am300_init();
#endif


}

MACHINE_START(WAYLAND, "PXA168-based Wayland Development Platform")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq       = pxa168_init_irq,
	.timer          = &pxa168_timer,
	.init_machine   = wayland_init,
MACHINE_END

