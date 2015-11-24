/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2002 David Brownell <dbrownell@users.sourceforge.net>
 * (C) Copyright 2002 Hewlett-Packard Company
 *
 * Bus Glue for AMD Alchemy Au1xxx
 *
 * Written by Christopher Hoover <ch@hpl.hp.com>
 * Based on fragments of previous driver by Rusell King et al.
 *
 * Modified for LH7A404 from ohci-sa1111.c
 *  by Durgesh Pattamatta <pattamattad@sharpsec.com>
 * Modified for AMD Alchemy Au1xxx
 *  by Matt Porter <mporter@kernel.crashing.org>
 *
 * This file is licenced under the GPL.
 */

#include <linux/platform_device.h>

#undef CONFIG_PM

/*-------------------------------------------------------------------------*/
static int ehci_xls_setup(struct usb_hcd *hcd)
{
    int    retval;
    struct ehci_hcd *ehci = hcd_to_ehci(hcd);

    ehci->caps = hcd->regs;
    ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));
    dbg_hcs_params(ehci, "reset");

    /* cache this readonly data; minimize chip reads */
    ehci->hcs_params = readl(&ehci->caps->hcs_params);

    retval = ehci_halt(ehci);
    if (retval)
        return retval;

    /* data structure init */
    retval = ehci_init(hcd);
    if (retval)
        return retval;
	//ehci_reset(ehci);
    return retval;
}

/*-------------------------------------------------------------------------*/
int ehci_xls_probe_internal(const struct hc_driver *driver,
        struct platform_device *pdev) {

    struct usb_hcd  *hcd;       
    struct resource *res;
    int retval;
    int irq;

    /* Get our IRQ from an earlier registered Platform Resource */
    res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (!res) {
        dev_err(&pdev->dev, "Found HC with no IRQ. Check %s setup!\n",
                dev_name(&pdev->dev));
        return -ENODEV;
    }
    irq = res->start;

    hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
    if (!hcd) {
        retval = -ENOMEM;
        goto err1;
    }

    /* Get our Memory Handle */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "Error: MMIO Handle %s setup!\n", 
                dev_name(&pdev->dev));
        retval = -ENODEV;
        goto err2;
    }

    hcd->rsrc_start = res->start;
    hcd->rsrc_len = res->end - res->start + 1;

    if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
                driver->description)) {
        dev_dbg(&pdev->dev, "controller already in use\n");
        retval = -EBUSY;
        goto err2;
    }
    hcd->regs = ioremap_nocache(hcd->rsrc_start, hcd->rsrc_len);

    if (hcd->regs == NULL) {
        dev_dbg(&pdev->dev, "error mapping memory\n");
        retval = -EFAULT;
        goto err3;
    }

    retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
    if (retval != 0)
        goto err4;
    return retval;

err4:
    iounmap(hcd->regs);
err3:
    release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err2:
    usb_put_hcd(hcd);
err1:
    dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev),
            retval);
    return retval;
}

/*-------------------------------------------------------------------------*/
static struct hc_driver ehci_xls_hc_driver = {
    .description      = hcd_name,
    .product_desc     = "XLS EHCI Host Controller",
    .hcd_priv_size    = sizeof(struct ehci_hcd),
    .irq              = ehci_irq,
    .flags            = HCD_USB2 | HCD_MEMORY,
    .reset            = ehci_xls_setup,
    .start            = ehci_run,
#ifdef  CONFIG_PM
    .suspend          = ehci_bus_suspend,
    .resume           = ehci_bus_resume,
#endif
    .stop             = ehci_stop,
    .shutdown         = ehci_shutdown,

    .urb_enqueue      = ehci_urb_enqueue,
    .urb_dequeue      = ehci_urb_dequeue,
    .endpoint_disable = ehci_endpoint_disable,
    .get_frame_number = ehci_get_frame,
    .hub_status_data  = ehci_hub_status_data,
    .hub_control      = ehci_hub_control,
    .bus_suspend      = ehci_bus_suspend,
    .bus_resume       = ehci_bus_resume,
};

/*-------------------------------------------------------------------------*/
static int ehci_xls_probe(struct platform_device *pdev)
{
	if (usb_disabled())
		return -ENODEV;
    return ehci_xls_probe_internal(&ehci_xls_hc_driver, pdev);
}
    
/*-------------------------------------------------------------------------*/
static int ehci_xls_remove(struct platform_device *pdev)
{
    struct usb_hcd *hcd = platform_get_drvdata(pdev);

    usb_remove_hcd(hcd);
    iounmap (hcd->regs);
    release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
    usb_put_hcd (hcd);
    return 0;
}

/*-------------------------------------------------------------------------*/
MODULE_ALIAS("ehci-xls");

static struct platform_driver ehci_xls_driver = {
    .probe      = ehci_xls_probe,
    .remove     = ehci_xls_remove,
    .shutdown   = usb_hcd_platform_shutdown,       
    .driver     = {
          .name = "ehci-xls",
    },
};
/*-------------------------------------------------------------------------*/
