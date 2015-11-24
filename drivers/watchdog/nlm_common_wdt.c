/*-
 * Copyright (c) 2003-2014 Broadcom Corporation
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY BROADCOM ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL BROADCOM OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * #BRCM_2# */

/* XLP Watchdog Timer driver */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/jiffies.h>
#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <nlm_hal_xlp_dev.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/netlogic/xlp_irq.h>
#include <linux/smp_lock.h>

#define DEVNAME				"watchdog-xlp"	// Device file name
#define DEFAULT_TIMEOUT		60				// Default timeout used if userspace doesn't supply one
//#define WDT_DEBUG							// Uncomment to enable debug messages to console

/* Below will also be needed by userspace */
#define NLM_IOC_WDT_ENABLE	(0x1003 + 1)	// IOCTL to enable a timer
#define NLM_IOC_WDT_DISABLE	(0x1003 + 2)	// IOCTL to disable a timer
#define NLM_WDT_MAGIC		(0xBAD0F00D)	// Magic number used to validate an IOCTL
struct nlm_wdt
{
	uint32_t response;		// For IOCTLs that retrieve data from the driver
	uint32_t wd_id;			// Timer number (0 or 1)
	uint32_t node;			// Node (cpu mask applies to this node)
	uint32_t timeout;		// Timeout in seconds
	uint32_t cpu_mask;		// Bitmap of PHYSICAL CPUs on this node
	uint32_t magic_close;	// Disable timer when device flie closes if set
	uint32_t magic;			// To protect the kernel from bad IOCTL
};
/* End what userspace needs */

#ifdef WDT_DEBUG
#define WDT_DBG		printk
#else
#define WDT_DBG(...)
#endif

/* Magic number used to make sure file descriptor private_data is really set by us.
 * bits 7:4 used for node, bits 3:0 used for timer number */
#define MAGIC 		0x7E3B5C00

static int running[2];						// Set if timer is running
static int magic_close[2];
static uint32_t timeout[2];
static uint32_t cpu_mask[2];
static const char devname[] = DEVNAME;

/* Called when a watchdog timer interrupt fires.  The watchdog timer number can be extracted
 * from the LSB of the IRQ number.  XLP_PIC_STATUS bits 1:0 will be set for each watchdog
 * timer that has expired.  Write a 1 to the set bit(s) to ack the interrupt. */
static irqreturn_t xlp_wdt_handler(int irq, void *p)
{
	int node = hard_smp_processor_id() / NLM_NCPUS_PER_NODE;
	int wd_status = (int)(nlh_pic_r64r(node, XLP_PIC_STATUS) & 0x3ULL);
	int wd_id = (irq - XLP_WD_IRQ(node, 0)) & 0x1;

	printk(KERN_NOTICE "%s: Watchdog timer %d interrupt, status = 0x%X\n",
			devname, wd_id, wd_status);

	/* Insert actions here */

	/* Ack the interrupt */
	nlh_pic_w64r(node, XLP_PIC_STATUS, wd_status);
	return IRQ_HANDLED;
}

/* Keepalive for a group of CPUs.  Assume caller has sanity checked the passed struct. */
static void xlp_wdt_keepalive(struct nlm_wdt *wdt)
{
	int i = 0;
	uint32_t cpu_mask = wdt->cpu_mask;

	WDT_DBG(KERN_INFO "%s: Keepalive for timer %d, cpu_mask = 0x%08X\n",
			devname, wdt->wd_id, cpu_mask);

	/* Write WATCHDOG[01]_BEATCMD register */
	while(cpu_mask) {
		if(cpu_mask & 0x1)
			nlh_pic_w64r(wdt->node, XLP_PIC_WD_BEATCMD(wdt->wd_id), i);
		i++;
		cpu_mask = cpu_mask >> 1;
	}
}

/* Enable the specified watchdog timer.  Assume caller has sanity checked the passed struct. */
int xlp_wdt_enable(struct nlm_wdt *wdt)
{
	int ret;
	u64 val, max_val;
	uint32_t mask = wdt->cpu_mask;
	int id = wdt->wd_id;
	int node = wdt->node;

	/* Is the requested watchdog already enabled? */
	if (running[id]) {
		printk(KERN_WARNING "%s: Watchdog timer %d already enabled\n", devname, id);
		return -EBUSY;
	}

	/* Register IRQ handler.  Note new flag IRQF_WATCHDOG - defined in kernel/interrupt.h and
	 * interpreted in kernel/irq/manage.c.  Also change in xlp/irq.c - don't disable interrupts
	 * in nlm_irq_startup - will be called with interrupts disabled if  necessary. */
	WDT_DBG(KERN_INFO "%s: Requesting IRQ for watchdog %d\n", devname, id);
	ret = request_irq(XLP_WD_IRQ(node, id), xlp_wdt_handler, IRQF_WATCHDOG, "xlp_wdt", NULL);
	if (ret < 0)
		printk(KERN_WARNING "%s: Unable to register IRQ for watchdog %d\n", devname, id);

	if(!wdt->timeout)
		wdt->timeout = DEFAULT_TIMEOUT;

	cpu_mask[id]    = mask;
	timeout[id]     = wdt->timeout;
	magic_close[id] = wdt->magic_close;

	/* Set WATCHDOG[01]_MAXVALUE register for the max value of count down register */
	max_val = (u64)wdt->timeout * (u64)nlm_hal_get_xlp_pit_tick_rate();
	nlh_pic_w64r(node, XLP_PIC_WD_MAXVAL(id), max_val);

	/* Set the (WATCHDOG[01]_ENABLE[01]) mask of CPUs which need to be monitored */
	switch (node) {
	case 0:
		val = nlh_pic_r64r(node, XLP_PIC_WD_THREN0(id));
		val &= 0xffffffff00000000;
		val |= mask;
		nlh_pic_w64r(node, XLP_PIC_WD_THREN0(id), val);
		break;
	case 1:
		val = nlh_pic_r64r(node, XLP_PIC_WD_THREN0(id));
		val &= 0x00000000ffffffff;
		val |= ((u64)mask << 32);
		nlh_pic_w64r(node, XLP_PIC_WD_THREN0(id), val);
		break;
	case 2:
		val = nlh_pic_r64r(node, XLP_PIC_WD_THREN1(id));
		val &= 0xffffffff00000000;
		val |= mask;
		nlh_pic_w64r(node, XLP_PIC_WD_THREN1(id), val);
		break;
	case 3:
		val = nlh_pic_r64r(node, XLP_PIC_WD_THREN1(id));
		val &= 0x00000000ffffffff;
		val |= ((u64)mask << 32);
		nlh_pic_w64r(node, XLP_PIC_WD_THREN1(id), val);
		break;
	}

	/* Enable the timer and set the escalation strategy.  Bits 0:1 enable the timers.
	 * Bits 5:4 (timer 1), 3:2 (timer 0) specify timeout count to generate NMI.
	 * Bits 9:8 (timer 1), 7:6 (timer 0) specify timeout count to reset the chip. */
#define WDT_ENABLE_0	   (  1ULL       | (3ULL << 2) | (1ULL << 6) )
#define WDT_MASK_0		(~ (  1ULL       | (3ULL << 2) | (3ULL << 6) ))
#define WDT_ENABLE_1	   ( (1ULL << 1) | (3ULL << 4) | (1ULL << 8) )
#define WDT_MASK_1		(~ ( (1ULL << 1) | (3ULL << 6) | (3ULL << 8) ))
	val = nlh_pic_r64r(node, XLP_PIC_CTRL);
	if(id) {
		val &= WDT_MASK_1;
		val |= WDT_ENABLE_1;
	} else {
		val &= WDT_MASK_0;
		val |= WDT_ENABLE_0;
	}
	nlh_pic_w64r(node, XLP_PIC_CTRL, val);
	WDT_DBG(KERN_INFO "%s: XLP_PIC_CTRL reg = 0x%016llX\n", devname, val);

	/* Clear the interrupt bits */
	val = nlh_pic_r64r(node, XLP_PIC_STATUS);
	nlh_pic_w64r(node, XLP_PIC_STATUS, val | 0xf);

	/* Simulate a keepalive for each CPU */
	xlp_wdt_keepalive(wdt);

	printk(KERN_INFO "%s: Watchdog timer %d enabled, timeout = %d seconds, cpu mask = 0x%08X"
			", magic close %s\n", devname, id, wdt->timeout, mask,
			wdt->magic_close ? "enabled" : "disabled");
	running[id] = 1;

#ifdef WDT_DEBUG
	/* Read the resulting PIC IRT entries */
  {
	int i;
	for (i = 0; i < 4; i++)
		printk(KERN_INFO "%s: PIC IRT Entry %d: 0x%016llX\n", devname, i,
				nlh_pic_r64r(node, XLP_PIC_IRT_ENTRY(i)) );
  }
#endif

	return 0;
}

/* Disable timer and free IRQs */
void xlp_wdt_disable(struct nlm_wdt *wdt)
{
	int id = wdt->wd_id;
	int node = wdt->node;
	u64 val = nlh_pic_r64r(node, XLP_PIC_CTRL) & ~(1 << id);
	nlh_pic_w64r(node, XLP_PIC_CTRL, val);
	WDT_DBG(KERN_INFO "%s: disable\n", devname);

	/* Unregister interrupt handler if timer is running */
	if(running[id]) {
		WDT_DBG(KERN_INFO "%s: Releasing IRQ for watchdog %d\n", devname, id);
		free_irq(XLP_WD_IRQ(node, id), NULL);

		printk(KERN_INFO "%s: Watchdog timer %d disabled\n", devname, id);
		running[id] = 0;
	}
}

static int xlp_wdt_open(struct inode *inode, struct file *file)
{
	WDT_DBG(KERN_INFO "%s: open\n", devname);
	return nonseekable_open(inode, file);
}

static ssize_t xlp_wdt_write(struct file *file, const char __user *buffer, size_t len, loff_t *off)
{
	char hb;
	int id;
	struct nlm_wdt wdt;

	WDT_DBG(KERN_INFO "%s: Write\n", devname);

	if( ((u32)(unsigned long)file->private_data & 0xFFFFFF00) != MAGIC) {
		printk(KERN_WARNING "%s: Write to device file with fd that did not enable a timer\n", devname);
		return -ENODEV;
	}

	wdt.node = ((u32)(unsigned long)file->private_data & 0xf0) >> 4;
	wdt.wd_id = (u32)(unsigned long)file->private_data & 0x03;
	id = wdt.wd_id;
	copy_from_user(&hb, (char __user*)buffer, sizeof(hb));

	if (hb == 'w') {
		wdt.cpu_mask = cpu_mask[id];
		xlp_wdt_keepalive(&wdt);
		return 0;
	}

	if (hb == 'V') {
		WDT_DBG(KERN_INFO "%s: Magic close set for timer %d\n", devname, id);
		magic_close[id] = 1;
		return 0;
	}

	if (hb < NLM_NCPUS_PER_NODE) {
		if(cpu_mask[id] & (1 << hb)) {
			WDT_DBG(KERN_INFO "%s: Keepalive for timer %d, cpu = %d\n", devname, id, hb);
			nlh_pic_w64r(wdt.node, XLP_PIC_WD_BEATCMD(id), hb);
			return 0;
		} else {
			printk(KERN_WARNING "%s: Write to device file with CPU# %d that is not in the enabled CPU mask\n",
					devname, hb);
			return -EINVAL;
		}
	}

	printk(KERN_NOTICE "%s: Unrecognized write to device file\n", devname);
	return -EINVAL;
}

static struct watchdog_info ident = {
	.options	= WDIOF_MAGICCLOSE | WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	.identity	= "XLP Watchdog",
};

static int sanity_check(struct nlm_wdt *wdt, unsigned long arg, int size)
{
	if(wdt) {
		if(wdt->magic != NLM_WDT_MAGIC) {
			printk(KERN_NOTICE "%s: Bad nlm_wdt struct passed in IOCTL\n", devname);
			return -EINVAL;
		}
		if(wdt->wd_id > 1) {
			printk(KERN_NOTICE "%s: Invalid watchdog timer number %d passed in IOCTL\n",
					devname, wdt->wd_id);
			return -EINVAL;
		}
	}
	if (arg)
		if( access_ok(VERIFY_WRITE, (uint32_t *)arg, sizeof(uint32_t)) == 0 ) {
			printk(KERN_NOTICE "%s IOCTL: Can't write to userspace\n", devname);
			return -EFAULT;
		}

	return 0;
}

int xlp_wdt_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;
	struct nlm_wdt *wdt;
	unsigned char temp[sizeof(struct nlm_wdt)];

	if( access_ok(VERIFY_READ, arg, sizeof(struct nlm_wdt)) == 0 ) {
		printk(KERN_WARNING "%s IOCTL: Can't read from userspace\n", devname);
		return -EFAULT;
	}
	ret = copy_from_user(temp, (unsigned char __user*)arg, sizeof(struct nlm_wdt));
	if(ret) {
		printk(KERN_WARNING "%s: ioctl error - copy_from_user failed (err=%d)\n", devname, ret);
		return -EFAULT;
	}

	wdt = (struct nlm_wdt *)temp;

	switch (cmd) {
	  case WDIOC_GETSUPPORT:
		WDT_DBG(KERN_INFO "%s: IOCTL GETSUPPORT\n", devname);
		if(sanity_check(NULL, arg, sizeof(ident)))
			return -EINVAL;
		ret = copy_to_user((struct watchdog_info *)arg, &ident, sizeof(ident));
		break;

	  case WDIOC_GETSTATUS: {
		uint32_t status = running[0] | (running[1] << 1)
				        | (magic_close[0] << 2) | (magic_close[1] << 3);
		WDT_DBG(KERN_INFO "%s: IOCTL GETSTATUS\n", devname);
		if(sanity_check(NULL, arg, sizeof(uint32_t)))
			return -EINVAL;
		ret = put_user(status, (uint32_t *)arg);
		break;
	  }

	  case WDIOC_SETTIMEOUT:
		WDT_DBG(KERN_INFO "%s: IOCTL SETTIMEOUT\n", devname);
		if(sanity_check(wdt, 0, 0))
			return -EINVAL;
		if(wdt->timeout) {
			u64 max_val = (u64)wdt->timeout * (u64)nlm_hal_get_xlp_pit_tick_rate();
			nlh_pic_w64r(wdt->node, XLP_PIC_WD_MAXVAL(wdt->wd_id), max_val);
			timeout[wdt->wd_id] = wdt->timeout;
			printk(KERN_INFO "%s: Watchdog timer %d timeout now %d seconds\n",
					devname, wdt->wd_id, wdt->timeout);
			xlp_wdt_keepalive(wdt);
		} else
			ret = -EINVAL;
		break;

	case WDIOC_GETTIMEOUT:
		WDT_DBG(KERN_INFO "%s: IOCTL GETTIMEOUT\n", devname);
		if(sanity_check(wdt, arg, sizeof(uint32_t)))
			return -EINVAL;
		ret = put_user(timeout[wdt->wd_id], (uint32_t *)arg);
		break;

	case WDIOC_GETTIMELEFT: {
		uint32_t timeleft = 0;
		u64 tick = nlm_hal_get_xlp_pit_tick_rate();
		WDT_DBG(KERN_INFO "%s: IOCTL GETTIMELEFT\n", devname);
		if(!tick)
			BUG();		// Should never happen
		if(sanity_check(wdt, arg, sizeof(uint32_t)))
			return -EINVAL;
		timeleft = nlh_pic_r64r(wdt->node, XLP_PIC_WD_COUNT(wdt->wd_id)) / tick;
		ret = put_user(timeleft, (uint32_t *)arg);
		break;
	}

	case WDIOC_GETBOOTSTATUS:
		if(sanity_check(wdt, arg, sizeof(uint32_t)))
			return -EINVAL;
		ret = put_user( ((uint32_t)read_c0_status() >> 19) & 0x3, (uint32_t *)arg);
		break;
	
	case WDIOC_KEEPALIVE:
		WDT_DBG(KERN_INFO "%s: IOCTL KEEPALIVE\n", devname);
		if(sanity_check(wdt, 0, 0))
			return -EINVAL;
		xlp_wdt_keepalive(wdt);
		break;

	case NLM_IOC_WDT_ENABLE:
		WDT_DBG(KERN_INFO "%s: IOCTL ENABLE\n", devname);
		if(sanity_check(wdt, 0, 0))
			return -EINVAL;
		ret = xlp_wdt_enable(wdt);
		if (!ret) {
			file->private_data = (void *)(unsigned long)(MAGIC | (wdt->node << 4) | (wdt->wd_id) );
		}
		break;

	case NLM_IOC_WDT_DISABLE:
		WDT_DBG(KERN_INFO "%s: IOCTL DISABLE\n", devname);
		if(sanity_check(wdt, 0, 0))
			return -EINVAL;
		xlp_wdt_disable(wdt);
		break;

	default:
		printk(KERN_WARNING "%s: Bad IOCTL\n", devname);
		ret = -EINVAL;
	}

	if(ret < 0)
		printk(KERN_WARNING "%s: IOCTL error (err=%d)\n", devname, ret);
	return ret;
}

long xlp_wdt_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned long ret;

	WDT_DBG(KERN_INFO "%s: IOCTL-compat\n", devname);
	ret = xlp_wdt_ioctl(NULL, filp, cmd, arg);
	return ret;
}

/* Close the fd.  If private_data holds the magic number and CONFIG_WATCHDOG_NOWAYOUT is not
 * set, and a 'V' has been written (magic close) also disable the timer. */
static int xlp_wdt_close(struct inode *inode, struct file *file)
{
	WDT_DBG(KERN_INFO "%s: close\n", devname);
#ifndef CONFIG_WATCHDOG_NOWAYOUT
	if(((u32)(unsigned long)file->private_data & 0xFFFFFF00) == MAGIC) {
		struct nlm_wdt wdt;
		wdt.wd_id = (u32)(unsigned long)file->private_data & 0x03;
		wdt.node = ((u32)(unsigned long)file->private_data & 0xf0) >> 4;
		if(magic_close[wdt.wd_id])
			xlp_wdt_disable(&wdt);
	}
#endif
	return 0;
}

static const struct file_operations xlp_wdt_fops =
{
	.owner			= THIS_MODULE,
	.llseek			= no_llseek,
	.ioctl			= xlp_wdt_ioctl,
	.compat_ioctl   = xlp_wdt_compat_ioctl,
	.open			= xlp_wdt_open,
	.write			= xlp_wdt_write,
	.release		= xlp_wdt_close,
};

static struct miscdevice xlp_wdt_miscdev =
{
	.minor	= WATCHDOG_MINOR,
	.name	= devname,
	.fops	= &xlp_wdt_fops,
};

/* For reboot notification */
/*
static int xlp_notify_sys(struct notifier_block *this, unsigned long code, void *unused)
{
	return NOTIFY_DONE;
}

static struct notifier_block xlp_notifier =
{
	.notifier_call = xlp_notify_sys,
};
*/

static int __init xlp_wdt_init(void)
{
	int ret;

	/* Placeholder code - if we want to be notified when the kernel is shutting down gracefully
	 * we need to register for the reboot notification. */
/*
	ret = register_reboot_notifier(&xlp_notifier);
	if (ret) {
		printk(KERN_WARNING "%s: Cannot register reboot notifier (err=%d)\n", devname, ret);
		return ret;
	}
*/
	ret = misc_register(&xlp_wdt_miscdev);
	if (ret) {
		printk(KERN_WARNING "%s: Cannot register watchdog timers (err=%d)\n", devname, ret);
		return ret;
	}

	printk(KERN_INFO "XLP watchdog driver - device file is /dev/%s\n", devname);
	return 0;
}

static void __exit xlp_wdt_exit(void)
{
	WDT_DBG(KERN_INFO "%s: Unregistering driver\n", devname);
	misc_deregister(&xlp_wdt_miscdev);
/*
	unregister_reboot_notifier(&xlp_notifier);
*/
}

module_init(xlp_wdt_init);
module_exit(xlp_wdt_exit);

MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("XLP Watchdog");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
