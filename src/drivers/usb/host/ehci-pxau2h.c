/*
 * (C) Marvell Inc. 2008 (kvedere@marvell.com)
 * Code Based on ehci-fsl.c & ehci-pxa9xx.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <plat/pxa_u2o.h>

static struct pxa_usb_plat_info *info;

static int pxau2h_ehci_clk_set(int en)
{
	struct clk *clk = NULL;

	clk = clk_get(NULL, "U2HCLK");
	if (IS_ERR(clk)) {
	        printk(KERN_ERR "Cannot get USB clk\n");
		return PTR_ERR(clk);
	}
	if (en)
		clk_enable(clk);
	else
		clk_disable(clk);
	return 0;
}

#ifdef CONFIG_CHUMBY_SILVERMOON_MEDIA
#include <linux/irq.h>
#include <linux/delay.h>
#include <mach/gpio.h>

#define SD_INSERT_GPIO		100
#define XD_INSERT_GPIO		101
#define MS_INSERT_GPIO		102
#define CF_INSERT_GPIO		103

#define SD_INSERT_IRQ 		IRQ_GPIO(SD_INSERT_GPIO)
#define XD_INSERT_IRQ 		IRQ_GPIO(XD_INSERT_GPIO)
#define MS_INSERT_IRQ 		IRQ_GPIO(MS_INSERT_GPIO)
#define CF_INSERT_IRQ 		IRQ_GPIO(CF_INSERT_GPIO)

#define MY_GPIO_REGBANK(x) 	( (x/32) )    
#define MY_GPIO_BASE_OFFSET(x) 	(  MY_GPIO_REGBANK(x) > 2 ? 0x100: MY_GPIO_REGBANK(x) * 0x4 )	 // extra offset added to basic base to select the right bank register
#define MY_GPIO_MASK(x) 	(1 << (x - ( MY_GPIO_REGBANK(x) * 32)))		// bit mask  for GPIO


#define MY_GPIO_PLR  0x0000  // pin level readback

struct my_gpio {
  void __iomem  *mmio_base;
  unsigned long phys_base;
  unsigned long size;
} my_gpio;
static struct timer_list event_timer;
static int timer_queued;


#endif //CONFIG_CHUMBY_SILVERMOON_MEDIA


/* called during probe() after chip reset completes */
static int pxau2h_ehci_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;

#if defined(CONFIG_MACH_CHUMBY_SILVERMOON) && defined(CONFIG_FORCE_USB_FULLSPEED_HUB)
	set_bit(1, &ehci->companion_ports); // bunnie -- temporarily force companion mode for full-speed
#endif


	/* EHCI registers start at offset 0x100 */
	ehci->caps = hcd->regs + U2x_CAPREGS_OFFSET;
	ehci->regs = hcd->regs + U2x_CAPREGS_OFFSET + 
		HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));
	dbg_hcs_params(ehci, "reset");
	dbg_hcc_params(ehci, "reset");

	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	retval = ehci_halt(ehci);
	if (retval)
		return retval;

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

	hcd->has_tt = 1;
	ehci->sbrn = 0x20;
	ehci_reset(ehci);

	return retval;
}

static const struct hc_driver pxau2h_ehci_hc_driver = {
	.description = hcd_name,
	.product_desc = "Marvell PXA SOC EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset = pxau2h_ehci_setup,
	.start = ehci_run,
#ifdef	CONFIG_PM
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
#endif
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
	.relinquish_port = ehci_relinquish_port,
	.port_handed_over = ehci_port_handed_over,
};

#ifdef CONFIG_CHUMBY_SILVERMOON_MEDIA


static inline int read_gpio_line(int gpio)
{
    // read the card insertion interrupt line: bit off means card is inserted
	return !((__raw_readl(GPIO_REGS_VIRT+0x100)) & GPIO_bit(gpio));
}


enum card_type {
	CARD_NONE,
	CARD_CF,
	CARD_MS,
	CARD_XD,
	CARD_SD,
	CARD_UNKNOWN,
};

enum card_type silvermoon_media_inserted(void)
{
	if(read_gpio_line(SD_INSERT_GPIO))
		return CARD_SD;
	if(read_gpio_line(XD_INSERT_GPIO))
		return CARD_XD;
	if(read_gpio_line(MS_INSERT_GPIO))
		return CARD_MS;
	if(read_gpio_line(CF_INSERT_GPIO))
		return CARD_CF;
	return CARD_UNKNOWN;
}

static char *CARD_TYPES[] = {
	"none",
	"CF",
	"MS",
	"XD",
	"SD",
	"unknown",
};

static void insertion_alert(unsigned long dev_id)
{
	char *envp[2];
	char card_type_string[48];
	char card_type_tmp[32];
	int  types_found = 0;
	struct usb_hcd *hcd = (struct usb_hcd *)dev_id;

	memset(card_type_tmp, 0, sizeof(card_type_tmp));

	if(read_gpio_line(SD_INSERT_GPIO)) {
		if(types_found) {
			char card_type_tmp2[32];
			strcpy(card_type_tmp2, card_type_tmp);
			snprintf(card_type_tmp, sizeof(card_type_tmp), "%s:%s",
					card_type_tmp2, CARD_TYPES[CARD_SD]);
		}
		else {
			snprintf(card_type_tmp, sizeof(card_type_tmp), CARD_TYPES[CARD_SD]);
		}
		types_found++;
	}
	if(read_gpio_line(XD_INSERT_GPIO)) {
		if(types_found) {
			char card_type_tmp2[32];
			strcpy(card_type_tmp2, card_type_tmp);
			snprintf(card_type_tmp, sizeof(card_type_tmp), "%s:%s",
					card_type_tmp2, CARD_TYPES[CARD_XD]);
		}
		else {
			snprintf(card_type_tmp, sizeof(card_type_tmp), CARD_TYPES[CARD_XD]);
		}
		types_found++;
	}
	if(read_gpio_line(MS_INSERT_GPIO)) {
		if(types_found) {
			char card_type_tmp2[32];
			strcpy(card_type_tmp2, card_type_tmp);
			snprintf(card_type_tmp, sizeof(card_type_tmp), "%s:%s",
					card_type_tmp2, CARD_TYPES[CARD_MS]);
		}
		else {
			snprintf(card_type_tmp, sizeof(card_type_tmp), CARD_TYPES[CARD_MS]);
		}
		types_found++;
	}
	if(read_gpio_line(CF_INSERT_GPIO)) {
		if(types_found) {
			char card_type_tmp2[32];
			strcpy(card_type_tmp2, card_type_tmp);
			snprintf(card_type_tmp, sizeof(card_type_tmp), "%s:%s",
					card_type_tmp2, CARD_TYPES[CARD_CF]);
		}
		else {
			snprintf(card_type_tmp, sizeof(card_type_tmp), CARD_TYPES[CARD_CF]);
		}
		types_found++;
	}

	if(!types_found)
		strcpy(card_type_tmp, CARD_TYPES[CARD_UNKNOWN]);


	// Copy the discovered card type to the user.
	snprintf(card_type_string, sizeof(card_type_string),
			"CARD_TYPE=%s", card_type_tmp);
	envp[0] = card_type_string;
	envp[1] = NULL;

	// SMC -- So we'd like to just reset the port.  At one point I had
	// lots of code in here to try and reset the port.  Then I realized
	// we already have hub-ctrl that can do that.  So rather than resetting
	// the port in kernelspace, we do it in userspace.  Everyone's happy.
	kobject_uevent_env(&hcd->self.controller->kobj, KOBJ_CHANGE, envp);
	timer_queued = 0;
	return;
}

/*
 * ISR for the CardDetect Pin
 */
static irqreturn_t card_detect_isr(int irq, void *dev_id)
{
	struct usb_hcd *hcd = (struct usb_hcd *)dev_id;
	int msecs;


	/* Debounce the port by queueing the handler to happen in 2 msecs */
	msecs = 2;
    if(!timer_queued) {
        timer_queued = 1;
        init_timer(&event_timer);
        event_timer.data     = (unsigned long)hcd;
        event_timer.function = insertion_alert;
        event_timer.expires  = jiffies + msecs_to_jiffies(msecs);
        add_timer(&event_timer);
    }

	return IRQ_HANDLED;
}

#endif // CONFIG_CHUMBY_SILVERMOON_MEDIA


static int pxau2h_ehci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
        struct resource	    *res;
	struct usb_hcd *hcd;
	int irq, retval, tmp;

	dev_dbg(&pdev->dev,"Initializing PXA EHCI-SOC USB Controller(U2H)\n");
	info = dev->platform_data;

	pxau2h_ehci_clk_set(1);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "u2h");
	info->regbase = (unsigned int)ioremap_nocache(res->start, res_size(res));
	if (!info->regbase) {
	        printk(KERN_ERR "Cannot get regbase 0x%p\n", (void *)info->regbase);
	        return -ENOMEM;
	}
	
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "u2hphy");
	info->phybase = (unsigned int)ioremap_nocache(res->start, res_size(res));
	if (!info->phybase) {
	        printk(KERN_ERR "Cannot get phybase 0x%p\n", (void *)info->phybase);
	        retval = -ENODEV;
		goto err1;
	}
	
	irq = platform_get_irq(pdev, 0);
	if (!irq) {
	        printk( KERN_ERR "Cannot get irq %x\n", irq);
	        retval = -ENODEV;
		goto err1;
	}

	printk("u2h regbase 0x%p phybase 0x%p irq %d\n", (void *)info->regbase,
			(void *)info->phybase, irq);

	if (!info->phy_init || info->phy_init((unsigned int)info->phybase)) {
		printk(KERN_ERR "unable to init pxa usb 2.0 host controller phy: %p\n", 
				info->phy_init);
		retval = -EBUSY; 
		goto err1;
	}

	if (!info->vbus_set || info->vbus_set(1)) {
		printk(KERN_ERR "Unable to power USB Host Controller.\n");
		retval = -EBUSY; 
		goto err1;
	}

	hcd = usb_create_hcd(&pxau2h_ehci_hc_driver, &pdev->dev, 
			pdev->dev.bus_id);
	if (!hcd) {
		retval = -ENOMEM;
		goto err1;
	}

	hcd->rsrc_start = virt_to_phys((void *)info->regbase);
	hcd->rsrc_len = 0x1ff;
	hcd->regs = (void __iomem*)info->regbase;
	hcd->irq = (unsigned int)irq;

	/* @USBCMD, Reset USB core */
	tmp = readl(hcd->regs + U2xUSBCMD);
	tmp |= U2xUSBCMD_RST;
	writel(tmp,hcd->regs + U2xUSBCMD);
	udelay(1000);
	
	retval = usb_add_hcd(hcd, hcd->irq, IRQF_DISABLED | IRQF_SHARED);
	if (retval != 0) {
		goto err2;
	}

#ifdef CONFIG_CHUMBY_SILVERMOON_MEDIA
	printk( "[ehci-pxau2h.c] Silvermoon media insertion / removal detection enabled\n" );

	set_irq_type(SD_INSERT_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	set_irq_type(XD_INSERT_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	set_irq_type(CF_INSERT_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	set_irq_type(MS_INSERT_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING );


	if (request_irq(CF_INSERT_IRQ, card_detect_isr, IRQF_DISABLED,
					"compact-detect", hcd)) {
		dev_err(&pdev->dev, "failed to request compact-flash detect interrupt\n" );
		goto err_irq;
	}

	if (request_irq(MS_INSERT_IRQ, card_detect_isr, IRQF_DISABLED,
					"memstick-detect", hcd)) {
		dev_err(&pdev->dev, "failed to request memstick detect interrupt\n" );
		goto err_irq;
	}

	if (request_irq(XD_INSERT_IRQ, card_detect_isr, IRQF_DISABLED,
					"xd-card-detect", hcd)) {
		dev_err(&pdev->dev, "failed to request XD-card detect interrupt\n" );
		goto err_irq;
	}

	if (request_irq(SD_INSERT_IRQ, card_detect_isr, IRQF_DISABLED,
					"sd-card-detect", hcd)) {
		dev_err(&pdev->dev, "failed to request SD-card detect interrupt\n" );
		goto err_irq;
	}

	/* Inform userspace in 8 seconds that a card was inserted */
	timer_queued = 1;
	init_timer(&event_timer);
	event_timer.data     = (unsigned long)hcd;
	event_timer.function = insertion_alert;
	event_timer.expires  = jiffies + msecs_to_jiffies(8000);
	add_timer(&event_timer);

#endif

	platform_set_drvdata(pdev, hcd);
	return retval;

#ifdef CONFIG_CHUMBY_SILVERMOON_MEDIA
	iounmap(my_gpio.mmio_base);
	release_mem_region(my_gpio.phys_base, my_gpio.size);
err_irq:
	retval = -ENOENT;
	free_irq(XD_INSERT_IRQ, hcd);
	free_irq(SD_INSERT_IRQ, hcd);
	free_irq(MS_INSERT_IRQ, hcd);
	free_irq(CF_INSERT_IRQ, hcd);
#endif

err2:
		usb_put_hcd(hcd);
err1:
		dev_err(&pdev->dev, "init %s fail, %d\n", pdev->dev.bus_id, 
			retval);
	return retval;
}

static int pxau2h_ehci_remove(struct platform_device *pdev)
{
	struct usb_hcd  *hcd = platform_get_drvdata(pdev);

#ifdef CONFIG_CHUMBY_SILVERMOON_MEDIA
	iounmap(my_gpio.mmio_base);
	release_mem_region(my_gpio.phys_base, my_gpio.size);

	free_irq(XD_INSERT_IRQ, hcd);
	free_irq(SD_INSERT_IRQ, hcd);
	free_irq(MS_INSERT_IRQ, hcd);
	free_irq(CF_INSERT_IRQ, hcd);
#endif

	if (HC_IS_RUNNING(hcd->state))
		hcd->state = HC_STATE_QUIESCING;

	usb_disconnect(&hcd->self.root_hub);
	hcd->driver->stop(hcd);

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	iounmap((void *)info->phybase);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	pxau2h_ehci_clk_set(0);
	return 0;
}

#ifdef CONFIG_PM
static int pxau2h_driver_suspend (struct platform_device *pdev, pm_message_t message)
{
	struct usb_hcd  *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	unsigned long flags;
	int    rc = 0;

	if (time_before(jiffies, ehci->next_statechange))
		msleep(10);

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible, bail out if RH has been resumed. Use
	 * the spinlock to properly synchronize with possible pending
	 * RH suspend or resume activity.
	 *
	 * This is still racy as hcd->state is manipulated outside of
	 * any locks =P But that will be a different fix.
	 */
	spin_lock_irqsave(&ehci->lock, flags);
	if (hcd->state != HC_STATE_SUSPENDED) {
		rc = -EINVAL;
		goto bail;
	}
	ehci_writel(ehci, 0, &ehci->regs->intr_enable);
	(void)ehci_readl(ehci, &ehci->regs->intr_enable);

	/* make sure snapshot being resumed re-enumerates everything */
	if (message.event == PM_EVENT_PRETHAW) {
		ehci_halt(ehci);
		ehci_reset(ehci);
	}

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	
	pxau2h_ehci_clk_set(0);
bail:
	spin_unlock_irqrestore(&ehci->lock, flags);

	// could save FLADJ in case of Vaux power loss
	// ... we'd only use it to handle clock skew

	return rc;
}

static int pxau2h_driver_resume (struct platform_device *pdev)
{
	struct usb_hcd  *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	pxau2h_ehci_clk_set(1);

	if (!info->phy_init || info->phy_init((unsigned int)info->phybase)) {
		printk("%s phy_init failed\n", __func__);
		return 0;
	}

	if (time_before(jiffies, ehci->next_statechange))
		udelay(100);

	/* Mark hardware accessible again */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

#if 0
	/* If CF is still set, we maintained PCI Vaux power.
	 * Just undo the effect of ehci_pci_suspend().
	 */
	if (ehci_readl(ehci, &ehci->regs->configured_flag) == FLAG_CF) {
		int	mask = INTR_MASK;

		if (!hcd->self.root_hub->do_remote_wakeup)
			mask &= ~STS_PCD;
		ehci_writel(ehci, mask, &ehci->regs->intr_enable);
		ehci_readl(ehci, &ehci->regs->intr_enable);
		printk("%s end--------------------------------\n", __func__);
		return 0;
	}
#endif

	usb_root_hub_lost_power(hcd->self.root_hub);
	ehci_halt(ehci);
	ehci_reset(ehci);

	/* emptying the schedule aborts any urbs */
	spin_lock_irq(&ehci->lock);
	if (ehci->reclaim)
		end_unlink_async(ehci);
	ehci_work(ehci);
	spin_unlock_irq(&ehci->lock);

	ehci_writel(ehci, ehci->command, &ehci->regs->command);
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command);	/* unblock posted writes */

	/* here we "know" root ports should always stay powered */
	ehci_port_power(ehci, 1);

	hcd->state = HC_STATE_SUSPENDED;

	return 0;
}
#endif

MODULE_ALIAS("pxau2h-ehci");

static struct platform_driver pxau2h_ehci_driver = {
	.probe = pxau2h_ehci_probe,
	.remove = pxau2h_ehci_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.driver = {
		   .name = "pxau2h-ehci",
		   .bus = &platform_bus_type
		   },
#ifdef CONFIG_PM
	.suspend = pxau2h_driver_suspend,
	.resume = pxau2h_driver_resume,
#endif
};
