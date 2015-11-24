/*-
 * Copyright 2004-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

/* $Id: init.c,v 1.1.2.7 2007-11-15 13:42:00 kmurthy Exp $
 * Performance-monitoring counters driver.
 * Top-level initialisation code.
 *
 * Copyright (C) 1999-2004  Mikael Pettersson
 */
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/sysctl.h>
#include <linux/perfctr.h>

#include <asm/uaccess.h>

#include "cpumask.h"
#include "virtual.h"
#include "version.h"

struct perfctr_info perfctr_info = {
	.abi_version = PERFCTR_ABI_VERSION,
	.driver_version = VERSION,
};

static ssize_t
driver_version_show(struct class *class, char *buf)
{
	return sprintf(buf, "%s\n", perfctr_info.driver_version);
}

static ssize_t
cpu_type_show(struct class *class, char *buf)
{
	return sprintf(buf, "%#x\n", perfctr_info.cpu_type);
}

static ssize_t
cpu_features_show(struct class *class, char *buf)
{
	return sprintf(buf, "%#x\n", perfctr_info.cpu_features);
}

static ssize_t
cpu_khz_show(struct class *class, char *buf)
{
	return sprintf(buf, "%u\n", perfctr_info.cpu_khz);
}

static ssize_t
tsc_to_cpu_mult_show(struct class *class, char *buf)
{
	return sprintf(buf, "%u\n", perfctr_info.tsc_to_cpu_mult);
}

static ssize_t
cpus_online_show(struct class *class, char *buf)
{
	int ret = cpumask_scnprintf(buf, PERFCTR_PAGE_SIZE-1, &cpu_online_map);
	buf[ret++] = '\n';
	return ret;
}

static ssize_t
cpus_forbidden_show(struct class *class, char *buf)
{
#ifdef CONFIG_PERFCTR_CPUS_FORBIDDEN_MASK
	int ret = cpumask_scnprintf(buf, PERFCTR_PAGE_SIZE-1, perfctr_cpus_forbidden_mask);
	buf[ret++] = '\n';
	return ret;
#endif
	return 0;
}

static CLASS_ATTR(driver_version, 0444, driver_version_show, NULL);
static CLASS_ATTR(cpu_type, 0444, cpu_type_show, NULL);
static CLASS_ATTR(cpu_features, 0444, cpu_features_show, NULL);
static CLASS_ATTR(cpu_khz, 0444, cpu_khz_show, NULL);
static CLASS_ATTR(tsc_to_cpu_mult, 0444, tsc_to_cpu_mult_show, NULL);
static CLASS_ATTR(cpus_online, 0444, cpus_online_show, NULL);
static CLASS_ATTR(cpus_forbidden, 0444, cpus_forbidden_show, NULL);

/* static struct class_attribute perfctr_class_attrs[] = {
	__ATTR_RO(driver_version),
	__ATTR_RO(cpu_type),
	__ATTR_RO(cpu_features),
	__ATTR_RO(cpu_khz),
	__ATTR_RO(tsc_to_cpu_mult),
	__ATTR_RO(cpus_online),
	__ATTR_RO(cpus_forbidden),
	__ATTR_NULL
}; */

static struct class perfctr_class = {
	.name		= "perfctr",
	// .class_attrs	= perfctr_class_attrs,
};

char *perfctr_cpu_name __initdata;

static int __init perfctr_class_init(void)
{
	int err;

	err = class_register(&perfctr_class);
	if (err)
		return err;

	err |= class_create_file(&perfctr_class, &class_attr_driver_version);
	err |= class_create_file(&perfctr_class, &class_attr_cpu_type);
	err |= class_create_file(&perfctr_class, &class_attr_cpu_features);
	err |= class_create_file(&perfctr_class, &class_attr_cpu_khz);
	err |= class_create_file(&perfctr_class, &class_attr_tsc_to_cpu_mult);
	err |= class_create_file(&perfctr_class, &class_attr_cpus_online);
	err |= class_create_file(&perfctr_class, &class_attr_cpus_forbidden);

	if (err)
		class_unregister(&perfctr_class);
	return err;
}

extern int	perfctr_cntmode;

ctl_table  perfctr_cntmode_table[] = {
    {
        .ctl_name   	= PERFCTR_CNTMODE,
		.procname   	= "cntmode",
        .data       	= &perfctr_cntmode,
        .maxlen     	= sizeof(int),
        .mode       	= 0644,
        .proc_handler   = &proc_dointvec,
    },
	{0}
};

ctl_table perfctr_sysctl_table[] = {
	{
		.ctl_name	= CTL_PERFCTR,
		.procname	= "perfctr",
		.mode		= 0555,
		.child		= perfctr_cntmode_table,
	},
	{0}
};

static struct ctl_table_header *perfctr_sysctl_table_handle;

static int __init perfctr_init(void)
{
	int err;

	err = perfctr_cpu_init();
	if (err) {
		printk(KERN_INFO "perfctr: not supported by this processor\n");
		return err;
	}
	err = vperfctr_init();
	if (err)
		return err;
	err = perfctr_class_init();
	if (err) {
		printk(KERN_ERR "perfctr: class initialisation failed\n");
		return err;
	}
	printk(KERN_INFO "perfctr: driver %s, cpu type %s at %u kHz\n",
	       perfctr_info.driver_version,
	       perfctr_cpu_name,
	       perfctr_info.cpu_khz);

	// code to create entries in the proc filesystem for perfctr count mode
	if ( (perfctr_sysctl_table_handle = register_sysctl_table (perfctr_sysctl_table)) == NULL) {
		printk (KERN_ERR "register_sysctl_table() for perfctr failed\n");
		printk (KERN_INFO "perfctr count mode defaults to individual threads");
	};

	return 0;
}

static void __exit perfctr_exit(void)
{
	vperfctr_exit();
	perfctr_cpu_exit();
	unregister_sysctl_table(perfctr_sysctl_table_handle);
}

module_init(perfctr_init)
module_exit(perfctr_exit)
