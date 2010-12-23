/*
 * linux/arch/arm/mach-pxa/silvermoon.c
 *
 * Support for the Chumby Silvermoon PXA168-based DPF board
 *
 * Copyright (C) 2009 Chumby Industries Inc.
 *
 * 2009-05-11  henry@chumby.com - stubbed out initial version
 * 2009-05-12  henry@chumby.com - cloned from arch/arm/mach-mmp/aspenite.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/smc91x.h>
#include <linux/i2c/pca953x.h>
#include <mach/pxa168_eth.h>
#include <linux/card.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <plat/pxa_u2o.h>

#include <asm/cputype.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa168.h>
#include <mach/pxa168.h>
#include <mach/gpio.h>
#include <mach/max8660.h>
#include <mach/pxa3xx_nand.h>
#include <mach/camera.h>

#include <plat/part_table.h>
#include <plat/generic.h>
#include <plat/pxa3xx_pmic.h>
#include <plat/pxa_u2o.h>
#include <plat/pxa3xx_otg.h>



#define CHLOG(format, arg...)            \
    printk("silvermoon.c - %s():%d - " format, __func__, __LINE__, ## arg)


#include "common.h"

// henry@chumby.com - constants used to be defined in asm-arm/arch-pxa/pxa910.h,
// now in include/mach/mfp-pxa168.h (relative to this dir)

//#include <asm-arm/arch-pxa/irqs.h>
#include <mach/irqs.h>
// Previously, we used 49, which is now used by GPIOX (GPIO multiplexed IRQ)
// This magic number (which is 52) is configured explicitly in silvermoon-ts.c
//#define IRQ_GPIO_AP	IRQ_PXA168CS_TOUCHSCREEN

#if defined(CONFIG_MACH_CHUMBY_SILVERMOON)
extern int dbg_pxa_mfp_verbose;
#endif

static unsigned long silvermoon_pin_config[] __initdata = {
	// Note that the range of GPIO0-GPIO09 conflicts with MMC3
	// See below for definition of MMC3
#if !defined(CONFIG_CHUMBY_SILVERMOON_SDBOOT) && !defined(CONFIG_MMC3)
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
#endif

#if defined(CONFIG_MMC3)
	GPIO0_MMC3_DAT7,
	GPIO1_MMC3_DAT6,
	GPIO2_MMC3_DAT5,
	GPIO3_MMC3_DAT4,
	GPIO4_MMC3_DAT3,
	GPIO5_MMC3_DAT2,
	GPIO6_MMC3_DAT1,
	GPIO7_MMC3_DAT0,
	GPIO8_MMC3_CLK,
	GPIO9_MMC3_CMD,
	GPIO16_SMC_nCS0_DIS,
#endif

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
	GPIO27_GPIO,	/* Ethernet IRQ */
#endif


	/* LCD */
	MFP_CFG_DRV_PULL(GPIO56, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO57, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO58, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO59, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO60, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO61, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO62, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO63, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO64, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO65, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO66, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO67, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO68, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO69, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO70, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO71, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO72, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO73, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO74, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO75, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO76, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO77, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO78, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO79, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO80, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO81, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO82, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO83, AF1, FAST, LOW),
	/*
	CSM_GPIO56_LCD_VS,
	CSM_GPIO57_LCD_HS,
	CSM_GPIO58_LCD_CLK,
	CSM_GPIO59_LCD_DEN,
	CSM_GPIO60_LCD_R0,
	CSM_GPIO61_LCD_R1,
	CSM_GPIO62_LCD_R2,
	CSM_GPIO63_LCD_R3,
	CSM_GPIO64_LCD_R4,
	CSM_GPIO65_LCD_R5,
	CSM_GPIO66_LCD_G0,
	CSM_GPIO67_LCD_G1,
	CSM_GPIO68_LCD_G2,
	CSM_GPIO69_LCD_G3,
	CSM_GPIO70_LCD_G4,
	CSM_GPIO71_LCD_G5,
	CSM_GPIO72_LCD_B0,
	CSM_GPIO73_LCD_B1,
	CSM_GPIO74_LCD_B2,
	CSM_GPIO75_LCD_B3,
	CSM_GPIO76_LCD_B4,
	CSM_GPIO77_LCD_B5,
	*/
	

	CSM_GPIO85_XotgDRV_VBUS,


	// henry@chumby.com -these two were left out before
	/* i2c bus */
	GPIO105_CI2C_SDA,
	GPIO106_CI2C_SCL,

	/* UART1 */
	/*********
	GPIO107_UART1_RXD,
	GPIO108_UART1_TXD,
	**********/

	//Added to ensure UART1 input works
	MFP_CFG(GPIO109, AF0),

	/* SSP0 */
	GPIO113_I2S_MCLK,
	GPIO114_I2S_FRM,
	GPIO115_I2S_BCLK,
	GPIO116_I2S_RXD,
	GPIO117_I2S_TXD,

#if defined(CONFIG_WLAN_8688_SDIO) || defined(CONFIG_WLAN_8688_SDIO_MODULE)

	/* sdh MMC2, wlan*/

	// henry@chumby.com - the Silvermoon board with eSD boot uses MMC2
#if defined(CONFIG_CHUMBY_SILVERMOON_MMC2_WIFI)
 	GPIO90_MMC2_DAT3,
	GPIO91_MMC2_DAT2,
	GPIO92_MMC2_DAT1,
	GPIO93_MMC2_DAT0,
	GPIO94_MMC2_CMD,
	GPIO95_MMC2_CLK,
#else
	// atb -this is for older NAND-based silvermoon's wifi connection.
	CSM_DFI11_MMC3_DAT3,
	CSM_DFI10_MMC3_DAT2,
	CSM_DFI09_MMC3_DAT1,
	CSM_DFI08_MMC3_DAT0,
	CSM_DFI15_MMC3_CMD,
	CSM_DFI14_MMC3_CLK,
#endif

#endif

	// Turn on power to LCD
	CSM_GPIO84_LCD_PWM,

	// Touchscreen
	CSM_GPIO118_TS_SCLK,
	CSM_GPIO119_TS_CS_LV,
	CSM_GPIO120_TS_MISO,
	CSM_GPIO121_TS_MOSI,

	// Bend sensor detect
	CSM_GPIO89_CHUMBY_BEND,

	// Headphone presence detect
	CSM_GPIO97_HP_IN,

	// CP UART
	CSM_GPIO98_HOST_TO_CP,
	CSM_GPIO99_CP_TO_HOST,

	// Media insert detect
	CSM_GPIO100_SDCDN,
	CSM_GPIO101_XDCDN,
	CSM_GPIO102_MSINS,
	CSM_GPIO103_CFCDN,

	CSM_GPIO105_Xi2cSDA,
	CSM_GPIO106_Xi2cSCL,

};

#if defined(CONFIG_TOUCHSCREEN_ASPEN) || defined(CONFIG_TOUCHSCREEN_ASPEN_MODULE)
// from 2.6.24 arch/arm/mach-ttc/ttc_fpga.c (bunnie)
static struct resource silvermoon_resource_touchscreen[] = {
  [0] = {
    .start = IRQ_PXA168CS_TOUCHSCREEN,
    .end = IRQ_PXA168CS_TOUCHSCREEN,
    .flags = IORESOURCE_IRQ,
  },
};

struct platform_device silvermoon_device_touchscreen = {
  .name = "silvermoon-ts",
  .id   = 0,
  .resource     = silvermoon_resource_touchscreen,
  .num_resources        = ARRAY_SIZE(silvermoon_resource_touchscreen),
};

static int silvermoon_touchscreen_init(void)
{
	platform_device_register( &silvermoon_device_touchscreen );
	return 0;
}


// Initialize ssp - needed for silvermoon touchscreen
static int __init silvermoon_ssp_init(void)
{
	// If we do this now, we need to do all I/O through the driver in silvermoon-ts
	// Note that device is origin:0
	printk( "%s() adding ssp2\n", __FUNCTION__ );
	pxa168_add_ssp(1);
	printk( "%s() exit\n", __FUNCTION__ );
	return 0;
}

#endif

/*
 * mfp is shared in card, cam and tw9907, only one is effective
 */
typedef enum{
	SW_CARD    = 0x01,
	SW_CAM_ON  = 0x02,
	SW_CAM_OFF = 0x03,
	SW_TW9907  = 0x04,
} SW_TYPE_T;


#if defined(CONFIG_PXA168_CF)

static struct resource pxa168_cf_resources[] = {
	[0] = {
		.start	= 0xD4285000,
		.end	= 0xD4285800,
		.flags	= IORESOURCE_MEM,
	},

	[1] = {
		.start	= IRQ_PXA168_CF,
		.end	= IRQ_PXA168_CF,
		.flags	= IORESOURCE_IRQ,
		},
	[2] = {
		.start	= IRQ_GPIO(32),
		.end	= IRQ_GPIO(32),
		.flags	= IORESOURCE_IRQ,
		},
};

struct platform_device silvermoon_cf_device = {
	.name		= "pxa168-cf",
	.resource	= pxa168_cf_resources,
	.num_resources	= ARRAY_SIZE(pxa168_cf_resources),
};

static void __init pxa168_cf_init(void)
{
	platform_device_register(&silvermoon_cf_device);
}
#endif

#if defined(CONFIG_PXA168_CAMERA) || defined(CONFIG_PXA168_CAMERA_MODULE)
static mfp_cfg_t aspenite_cam_pins[] = {
	GPIO37_CAM_DAT7,
	GPIO38_CAM_DAT6,
	GPIO39_CAM_DAT5,
	GPIO40_CAM_DAT4,
	GPIO41_CAM_DAT3,
	GPIO42_CAM_DAT2,
	GPIO43_CAM_DAT1,
	GPIO44_CAM_DAT0,
	GPIO46_CAM_VSYNC,
	GPIO48_CAM_HSYNC,
	GPIO54_CAM_MCLK,
	GPIO55_CAM_PCLK,
};

/* sensor init */
static int sensor_power_onoff(int on, int unused)
{
	/*
	 * on, 1, power on
	 * on, 0, power off
	 */
	int ret = 0;
	if(on) {
		mfp_config(ARRAY_AND_SIZE(aspenite_cam_pins));
	}
	return ret;
}

static struct sensor_platform_data ov7670_sensor_data = {
	.id = SENSOR_LOW,
	.power_on = sensor_power_onoff,
};

/* sensor init over */
#endif


static struct i2c_pxa_platform_data pwri2c_info __initdata = {
	.use_pio		= 0,
};

static struct i2c_pxa_platform_data xi2c_info __initdata = {
	.use_pio		= 0,
};

// These values are mostly taken from aspenite.c which has the only
// example of video mode(s) set for 800x600 (albeit not in RGB565 DMA mode)
#if defined(CONFIG_CHUMBY_800_480_VIDEO)
#warning "----------------------->Using silvermoon_tablet config<---------------------------"
static struct fb_videomode video_modes_aspen[] = {
  [0] = {
    .pixclock       = 24500,                                                                        
    //    .pixclock       = 12820,
    .refresh        = 60,
    .xres           = 800,
    .yres           = 480,
    .hsync_len      = 5 /*128*/,
    .left_margin    = 16 /*215*/,
    .right_margin   = 8,
    .vsync_len      = 2 /*4*/,
    .upper_margin   = 8, // 23                                                                                  
    .lower_margin   = 5, // 1                                                                                   
	// henry - if these are set, invert vsync / hsync are turned OFF:
	// FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
    .sync           = 0,
  },

};
#else
static struct fb_videomode video_modes_aspen[] = {
	[0] = {
		.pixclock       = 25641,
		.refresh        = 60,
		.xres           = 800,
		.yres           = 600,
		.hsync_len      = 128 /*128*/,
		.left_margin    = 215 /*215*/,
		.right_margin   = 40,
		.vsync_len      = 4 /*4*/,
		.upper_margin   = 34, // 23
		.lower_margin   = 14, // 1

		// henry - if these are set, invert vsync / hsync are turned OFF:
		// FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
		.sync           = FB_SYNC_VERT_HIGH_ACT,
	},

};
#endif

static void silvermoon_lcd_power(struct pxa168fb_info *fbi, unsigned int spi_gpio_cs, unsigned int spi_gpio_reset, int on)
{
	printk( "%s() %s\n", __FUNCTION__, on ? "ON" : "off" );
}

/* SPI Control Register. */
#define     CFG_SCLKCNT(div)                    (div<<24)  /* 0xFF~0x2 */
#define     CFG_RXBITS(rx)                      (rx<<16)   /* 0x1F~0x1 */
#define     CFG_TXBITS(tx)                      (tx<<8)    /* 0x1F~0x1, 0x1: 2bits ... 0x1F: 32bits */
#define     CFG_SPI_ENA(spi)                    (spi<<3)
#define     CFG_SPI_SEL(spi)                    (spi<<2)   /* 1: port1; 0: port0 */
#define     CFG_SPI_3W4WB(wire)                 (wire<<1)  /* 1: 3-wire; 0: 4-wire */

// Stole this from tavor, which is also 800x600 RGB565
// Is this 312 million picoseconds (ps) = 0.312ms = 0.312s ?
// Apparently hz, eg 312mhz
#define LCD_SCLK (312000000UL)
struct pxa168fb_mach_info silvermoon_lcd_info __initdata = {
	.id                     = "Base-silvermoon",
	.modes                  = video_modes_aspen,
	.num_modes              = ARRAY_SIZE(video_modes_aspen),
	.pix_fmt                = PIX_FMT_RGBA888,
	.io_pin_allocation_mode = PIN_MODE_DUMB_18_GPIO | 0x20, // No crossing 1K boundary
	.dumb_mode              = DUMB_MODE_RGB666,	// LCD_SPU_DUMB_CTRL[31:28]
	.active                 = 1, // LCD active (boolean)
	.pxa168fb_lcd_power     = silvermoon_lcd_power,
	.gpio_output_data       = 0x10,	// LCD_SPU_DUMB_CTRL[27:20]
	.gpio_output_mask       = 0xff,	// LCD_SPU_DUMB_CTRL[19:12]
#if defined(CONFIG_CHUMBY_800_480_VIDEO)
#warning "----------------------->non-inverted pixclk<---------------------------"
	.invert_pixclock        = 0,
#else
	.invert_pixclock        = 1,
#endif
	.invert_vsync           = 1,
	.invert_hsync           = 1,
// New to 2.6.28 bsp
	.sclk_clock             = LCD_SCLK,
	.panel_rgb_reverse_lanes= 0,
	//.gpio_output_data       = 1,
	//.gpio_output_mask       = 0xff,
	.invert_composite_blank = 0,
	.invert_pix_val_ena     = 0,
	.panel_rbswap           = 0,
	.enable_lcd             = 1,
	.spi_gpio_cs            = 0,
	.spi_gpio_reset         = 0,
};

struct pxa168fb_mach_info silvermoon_lcd_ovly_info __initdata = {
        .id                     = "Ovly-silvermoon",
        .modes                  = video_modes_aspen,
        .num_modes              = ARRAY_SIZE(video_modes_aspen),
        .pix_fmt                = PIX_FMT_RGBA888,
        .io_pin_allocation_mode = PIN_MODE_DUMB_18_GPIO | 0x20, // No crossing 1K boundary
		.dumb_mode              = DUMB_MODE_RGB666,	// LCD_SPU_DUMB_CTRL[31:28]
        .active                 = 1,
        //.spi_ctrl               = CFG_SCLKCNT(2) | CFG_TXBITS(15) | CFG_SPI_SEL(1) | CFG_SPI_3W4WB(1) | CFG_SPI_ENA(1),
        //.spi_gpio_cs            = 0 /*GPIO_EXT1(14)*/,
        //.spi_gpio_reset         = -1,
        .pxa168fb_lcd_power     = silvermoon_lcd_power,
        .gpio_output_data       = 0x10,	// LCD_SPU_DUMB_CTRL[27:20]
        .gpio_output_mask       = 0xff,	// LCD_SPU_DUMB_CTRL[19:12]
        .invert_pixclock        = 1,
        .invert_vsync           = 1,
        .invert_hsync           = 1,
// New to 2.6.28 bsp
		.sclk_clock             = LCD_SCLK,
		.panel_rgb_reverse_lanes= 0,
        .invert_composite_blank = 0,
        .invert_pix_val_ena     = 0,
        .panel_rbswap           = 0,
        .enable_lcd             = 1,
        .spi_gpio_cs            = 0,
        .spi_gpio_reset         = 0,
};


static struct i2c_board_info pwri2c_board_info[] =
{

	{
		.type		= "wm8961",
		.addr		= 0x4a, // wm8961 data sheet says 0x94, but should probably be 0x4A since this is a 7 bit address
	},


#if defined(CONFIG_TSC2007)
       {
	       .type	= "tsc2007",
	       .addr	= 0x48,                                 /* 0x90/0x91 */
	       .irq	= IRQ_GPIO(GPIO_EXT0(7)),               /* IO7 of TSC2007 */
       },
#endif

};

static struct i2c_board_info xi2c_board_info[] =
{

	{
		.type		= "dcid",
		.addr		= 0x50,
	},
};


#if defined(CONFIG_MMC_PXA_SDH)  || defined(CONFIG_MMC_PXA_SDH_MODULE)

static mfp_cfg_t aspenite_sdh_pins[] = {
#if defined(CONFIG_CHUMBY_SILVERMOON_SDBOOT) || defined(CONFIG_MMC3)
// Silvermoon uses MMC3 for eSD (embedded SD) for boot
// and MMC2 for wifi
	// MMC3 data interface
	// Using 4 bit-wide interface
	/***** Using values above in silvermoon_pins ****
	CSM_GPIO0_MMC3_DAT7,
	CSM_GPIO1_MMC3_DAT6,
	CSM_GPIO2_MMC3_DAT5,
	CSM_GPIO3_MMC3_DAT4,
	CSM_GPIO4_MMC3_DAT3,
	CSM_GPIO5_MMC3_DAT2,
	CSM_GPIO6_MMC3_DAT1,
	CSM_GPIO7_MMC3_DAT0,
	CSM_GPIO8_MMC3_CLK,
	CSM_GPIO9_MMC3_CMD,
	CSM_GPIO16_MMC3_CD,
	****************************/
#else
	GPIO37_MMC1_DAT7,
	GPIO38_MMC1_DAT6,
	GPIO54_MMC1_DAT5,
	GPIO48_MMC1_DAT4,
	GPIO51_MMC1_DAT3,
	GPIO52_MMC1_DAT2,
	GPIO40_MMC1_DAT1,
	GPIO41_MMC1_DAT0,
	GPIO49_MMC1_CMD,
	GPIO43_MMC1_CLK,
	GPIO53_MMC1_CD,
	GPIO46_MMC1_WP,
#endif
};

static void sdh_mfp_config(void)
{
	mfp_config(ARRAY_AND_SIZE(aspenite_sdh_pins));
}

static struct pxasdh_platform_data aspenite_sdh_platform_data_MMC1 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_29_30 | MMC_VDD_30_31,
	.mfp_config	= sdh_mfp_config,
	.bus_width = 4,
};

static struct pxasdh_platform_data aspenite_sdh_platform_data_MMC2 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_29_30 | MMC_VDD_30_31,
	.bus_width = 4,
};

static struct pxasdh_platform_data aspenite_sdh_platform_data_MMC3 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_29_30 | MMC_VDD_30_31,
	.bus_width = 4,
};

#endif

#ifdef CONFIG_USB_GADGET_PXA_U2O

static int aspenite_u2o_vbus_status(unsigned base)
{
	return 0;
}

static int aspenite_u2o_vbus_set(int vbus_type)
{
#if !defined(CONFIG_MACH_CHUMBY_SILVERMOON)
	unsigned long flags;

	local_irq_save(flags);

	if (gpio_request(gpio_usb_otg_pen, "USB OTG Power Enable")) {
		printk(KERN_ERR "%s Max7312 USB_OTG_PEN GPIO Request"
			" Failed\n", __func__);
		return -1;
	}

	switch (vbus_type) {
	case VBUS_SRP:
		gpio_direction_output(gpio_usb_otg_pen,1);
		udelay(10);
		gpio_direction_output(gpio_usb_otg_pen,0);
		break;
	case VBUS_HIGH:
		gpio_direction_output(gpio_usb_otg_pen,1);
		break;
	case VBUS_LOW:
 		gpio_direction_output(gpio_usb_otg_pen,0);
		break;
	default:
		break;
	}
	gpio_free(gpio_usb_otg_pen);

	local_irq_restore(flags);
#endif

	return 0;
}
static int aspenite_otg_init(void)
{
	return 0;
}

static int aspenite_u2o_vbus_set_ic(int function)
{
	printk(KERN_DEBUG "%s %d not implemented yet\n", __func__, function);
	return 0;
}

#if defined(CONFIG_USB_GADGET_PXA_U2O)
static struct otg_pmic_ops aspenite_otg_ops = {
	.otg_vbus_init          = aspenite_otg_init,
	.otg_set_vbus           = aspenite_u2o_vbus_set,
	.otg_set_vbus_ic        = aspenite_u2o_vbus_set_ic,
	.otg_get_vbus_state     = aspenite_u2o_vbus_status,
};

struct otg_pmic_ops *init_silvermoon_otg_ops(void)
{
	return &aspenite_otg_ops;
}
#endif

#if defined(CONFIG_USB_GADGET_PXA_U2O)
static struct pxa_usb_plat_info aspenite_u2o_info = {
	.phy_init	= pxa168_usb_phy_init,
	.vbus_set	= aspenite_u2o_vbus_set,
	.vbus_status	= aspenite_u2o_vbus_status,
	.init_pmic_ops	= init_silvermoon_otg_ops,
	.is_otg		= 1,
};
#endif

#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H
/* USB 2.0 Host Controller */
static int aspenite_u2h_vbus_set (int enable)
{
	return 0;
}

static struct pxa_usb_plat_info aspenite_u2h_info = {
#if defined(CONFIG_USB_GADGET_PXA_U2O)
	.phy_init	= pxa168_usb_phy_init,
#endif
	.vbus_set	= aspenite_u2h_vbus_set,
};
#endif


struct platform_device silvermoon_bl_device = {
	.name		= "silvermoon-bl",
	.id		= -1,
};


static void __init silvermoon_init(void)
{
	int gpio_109 = 109;
    CHLOG("Just so you know, the CPU type is 0x%08x\n", read_cpuid_id());

	mfp_config(ARRAY_AND_SIZE(silvermoon_pin_config));

	pxa168_set_vdd_iox(VDD_IO0, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO1, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO2, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO3, VDD_IO_1P8V);
	pxa168_set_vdd_iox(VDD_IO4, VDD_IO_3P3V);
	pxa168_mfp_set_fastio_drive(MFP_DS02X);


	//Make sure the GPIO 109 is set as output, level low


	if(gpio_request(gpio_109, "GPIO109")) 
	{
		printk(KERN_ERR "GPIO109 Request Failed\n");
	}
	else
	{
		gpio_direction_output(gpio_109, 1);
		gpio_set_value(gpio_109, 0);
		gpio_free(gpio_109);
	}


	/* on-chip devices */
	pxa168_add_uart(1);
	// Used for cryptoprocessor
	pxa168_add_uart(3);

	//turn on the I2C bus for the audio codec (and others)
	pxa168_add_twsi(0, &xi2c_info, ARRAY_AND_SIZE(xi2c_board_info));
	pxa168_add_twsi(1, &pwri2c_info, ARRAY_AND_SIZE(pwri2c_board_info));
	pxa168_add_ssp(0);
#if defined(CONFIG_TOUCHSCREEN_ASPEN) || defined(CONFIG_TOUCHSCREEN_ASPEN_MODULE)
	// SSP2 used for CP-based touchscreen
	printk( "%s() - attempting to add SSP2 for touchscreen\n", __FUNCTION__ );
	silvermoon_ssp_init();
#if defined(CONFIG_TOUCHSCREEN_ASPEN)
       // Register platform device now iff not loading as module
        silvermoon_touchscreen_init();
#endif
#else
	printk( "%s() - CONFIG_TOUCHSCREEN_ASPEN not set??\n", __FUNCTION__ );
#endif

#ifdef CONFIG_USB_GADGET_PXA_U2O
 	pxa168_add_u2o(&aspenite_u2o_info);
#endif

#ifdef CONFIG_USB_OTG
	pxa168_add_u2ootg(&aspenite_u2o_info);
	pxa168_add_u2oehci(&aspenite_u2o_info);
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H
	printk( "%s() - adding u2h ehci usb hub\n", __FUNCTION__ );
 	pxa168_add_u2h(&aspenite_u2h_info);
#endif
//	pxa168_add_mfu(&pxa168_eth_data);
#if defined(CONFIG_MMC_PXA_SDH) || defined(CONFIG_MMC_PXA_SDH_MODULE)
#if defined(CONFIG_CHUMBY_SILVERMOON_SDBOOT)
#define WIFI_SDIO_ADDR	1 // MMC2
#else
#define WIFI_SDIO_ADDR	2 // MMC3
#endif
#if defined(CONFIG_WLAN_8688_SDIO) || defined(CONFIG_WLAN_8688_SDIO_MODULE)
#define ESD_SDIO_ADDR	2
#else
#define ESD_SDIO_ADDR	1
#endif
	// This is not present but may be needed for the driver to work properly with MMC2/MMC3
	pxa168_add_sdh(0, &aspenite_sdh_platform_data_MMC1);
#if defined(CONFIG_WLAN_8688_SDIO) || defined(CONFIG_WLAN_8688_SDIO_MODULE)
	pxa168_add_sdh(WIFI_SDIO_ADDR, &aspenite_sdh_platform_data_MMC2);
#endif
#if defined(CONFIG_CHUMBY_SILVERMOON_SDBOOT)
	// Addr=2 for MMC3
	pxa168_add_sdh(ESD_SDIO_ADDR, &aspenite_sdh_platform_data_MMC3);
	printk( "%s() - adding MMC3 init[%d], wifi MMC%d\n", __FUNCTION__, ESD_SDIO_ADDR, WIFI_SDIO_ADDR );
#endif
#endif
#if defined(CONFIG_PXA168_CF)
	/* pxa168_add_cf(); */
	pxa168_cf_init();
#endif

	pxa168_add_freq();

	pxa168_add_fb_ovly(&silvermoon_lcd_ovly_info);
	pxa168_add_fb(&silvermoon_lcd_info);

#if defined(CONFIG_PXA168_CAMERA) || defined(CONFIG_PXA168_CAMERA_MODULE)
	pxa168_add_cam();
#endif

	platform_device_register(&silvermoon_bl_device);

	platform_device_register(&pxa910_device_rtc);
}

MACHINE_START(CHUMBY_SILVERMOON, "PXA168-based Chumby Silvermoon platform")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq       = pxa168_init_irq,
	.timer          = &pxa168_timer,
	.init_machine   = silvermoon_init,
MACHINE_END


static int global_chumbybrand = 1001;
int chumby_brand(void)
{
	return global_chumbybrand;
}
EXPORT_SYMBOL(chumby_brand);

static int __init chumbybrand_setup(char *str)
{
	global_chumbybrand = simple_strtoul(str, NULL, 0);
}
__setup("brandnum=", chumbybrand_setup);

