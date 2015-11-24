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

#include <string.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/netlogic/xlp8xx/cpu.h>
#include <asm/netlogic/hal/nlm_hal.h>
#include <asm/netlogic/xlp.h>
#include <libfdt.h>
#include <fdt_helper.h>

#undef NOR_DEBUG
//#define NOR_DEBUG

#ifdef NOR_DEBUG
#define NOR_DEBUG_PRINTK	printk
#else
#define NOR_DEBUG_PRINTK(...)
#endif

#define MAX_PARTITIONS		10
#define NUM_HARDCODED_PARTS	4
#define XLP_NOR_BASEADDR	0x40
#define NOR_FDT_PATH		"/soc/nor_flash@1,"

extern void *fdt;

static int cs;
static uint32_t base, limit;

struct xlp_nor_info {
	struct mtd_partition	*parts;
	struct mtd_info			*mtd;
	struct map_info			map;
};

#define PART_NAME_LEN 32
static char partition_names[MAX_PARTITIONS][PART_NAME_LEN] = {{0}};
static struct mtd_partition xlp_nor_partitions[MAX_PARTITIONS] = {
	{ .name	= partition_names[0] },
	{ .name	= partition_names[1] },
	{ .name	= partition_names[2] },
	{ .name	= partition_names[3] },
	{ .name	= partition_names[4] },
	{ .name	= partition_names[5] },
	{ .name	= partition_names[6] },
	{ .name	= partition_names[7] },
	{ .name	= partition_names[8] },
	{ .name	= partition_names[9] },
};

static __inline__ int32_t nor_reg_read(int node,  int regidx)
{
	uint64_t mmio = nlm_hal_get_dev_base(node, 0, XLP_PCIE_SPI_NOR_FLASH_DEV, XLP_PCIE_SPI_NOR);
	return nlm_hal_read_32bit_reg(mmio, regidx);
}

static __inline__ void nor_reg_write(int node, int regidx, int32_t val)
{
	uint64_t mmio = nlm_hal_get_dev_base(node, 0, XLP_PCIE_SPI_NOR_FLASH_DEV, XLP_PCIE_SPI_NOR);
	nlm_hal_write_32bit_reg(mmio, regidx, val);
}

#if 0
static void nor_dump_reg(void)
{
	int i;

	printk("\nNor Flash memory interface chip select:\n");
	for(i = 0x0; i < 0x7; i++)
	{
		printk("nor flash:  0x%0x = 0x%8x\n", i, nor_reg_read(0, i));
	}

	for(i = 0x40; i < 0x47; i++)
	{
		printk("base addr for cs:%d  0x%0x = 0x%8x\n", i & 0x0f, i, nor_reg_read(0, i));
	}

	for(i = 0x48; i < 0x4f; i++)
	{
		printk("addr limit  cs:%d  0x%0x = 0x%8x\n", i & 0x0f, i, nor_reg_read(0, i));
	}

	for(i = 0x50; i < 0x57; i++)
	{
		printk("device parameter for cs:%d  0x%0x = 0x%8x\n", i & 0x0f, i, nor_reg_read(0, i));
	}

	printk("Nor flash system control 0x68= 0x%8x\n", nor_reg_read(0, 0x68));
}
#endif

static int get_fdt_nor_map_info(void *fdt, struct xlp_nor_info *info, int cs)
{
	int node_offset, lenp;
	uint32_t const *reg;
	char node_path[32];

	NOR_DEBUG_PRINTK("[%s] fdt @ 0x%llx\n", __func__, (unsigned long long)fdt);

	/* Find the node in FDT */
	sprintf(node_path, NOR_FDT_PATH "%d", cs);
	node_offset = fdt_path_offset(fdt, node_path);
	NOR_DEBUG_PRINTK("  offset %d for path %s\n", node_offset, node_path);
	if(!node_offset) {
		printk(KERN_NOTICE "[%s] FDT node %s not found - skipping driver init\n",
				__func__, node_path);
		return -EINVAL;	// FIXME - pick better error code
	}

	/* Find bank-width */
	if(!(reg = fdt_getprop(fdt, node_offset, "bank-width", &lenp)) ) {
		printk(KERN_NOTICE "[%s] FDT node %s missing bank-width property - skipping driver init\n",
				__func__, node_path);
		return -EINVAL;	// FIXME - pick better error code
	}
	info->map.bankwidth = *reg;
	
	/* Find reg property for the node */
	if(!(reg = fdt_getprop(fdt, node_offset, "reg", &lenp)) ) {
		printk(KERN_NOTICE "[%s] FDT node %s missing reg property - skipping driver init\n",
				__func__, node_path);
		return -EINVAL;	// FIXME - pick better error code
	}

	/* reg points to three uint32_t - the third one is the size */
	/* FIXME - need fdt32_to_cpu for little endian */
	info->map.size = reg[2];

	NOR_DEBUG_PRINTK("  NOR device bank width %d size 0x%lx\n",
			info->map.bankwidth, info->map.size);
	return 0;
}

static int get_fdt_nor_partitions(void *fdt, int cs)
{
	int num_parts = 0;
	int depth = 1;
	int i, node_offset, lenp;
	const struct fdt_property *prop_ptr;
	uint32_t offset = 0;
	uint32_t size;
	uint32_t const *reg;
	char node_path[32];
	char const *node_name, *partition_name;

	NOR_DEBUG_PRINTK("[%s] fdt @ 0x%llx\n", __func__, (unsigned long long)fdt);

	/* Find the node in FDT */
	sprintf(node_path, NOR_FDT_PATH "%d", cs);
	node_offset = fdt_path_offset(fdt, node_path);
	NOR_DEBUG_PRINTK("  Offset %d for path %s\n", node_offset, node_path);
	if(!node_offset)
		return 0;

	/* Loop through partitions, setting up the table */
	while (num_parts < MAX_PARTITIONS) {
		node_offset = fdt_next_node(fdt, node_offset, &depth);

		/* Break if EOF (node_offset < 0) or no longer in NOR node (depth == 0) */
		if(node_offset < 0) {
			NOR_DEBUG_PRINTK("  Reached EOF\n");
			break;
		}

		if(depth == 1) {
			NOR_DEBUG_PRINTK("  Reached end of NOR flash node\n");
			break;
		}

		if(!(node_name = fdt_get_name(fdt, node_offset, &lenp)) ) {
			NOR_DEBUG_PRINTK("  Skipping unnamed node\n");
			continue;
		}

		/* Make sure the node name starts with partition@ */
		if(strncasecmp(node_name, "partition@", 10) ) {
			printk(KERN_NOTICE "[%s] Skipping unknown partition node %s in FDT\n",
					__func__, node_name);
			continue;
		}

		/* Found a partition - get the details (copy to avoid compiler warning about const assignment) */
		partition_name = fdt_getprop(fdt, node_offset, "label", &lenp);
		if(!partition_name) {
			printk(KERN_NOTICE "[%s] Skipping partition in FDT with no label\n", __func__);
			continue;
		}
		strncpy(xlp_nor_partitions[num_parts].name, partition_name, PART_NAME_LEN);

		/* reg should be offset then size, skip if size = 0 or not found */
		/* FIXME - little endian needs fdt32_to_cpu call on uint32_t values */
		if(!(reg = fdt_getprop(fdt, node_offset, "reg", &lenp)) ) {
			printk(KERN_NOTICE "[%s] Partition %s in FDT missing reg property - skipping\n",
					__func__, xlp_nor_partitions[num_parts].name);
			xlp_nor_partitions[num_parts].name = NULL;
			continue;
		}

		if( (size = reg[1]) ) {
			if(reg[0] != offset)
				printk(KERN_NOTICE
						"[%s] FDT partition offset 0x%x does not match expected offset %x\n",
						__func__, reg[0], offset);
			offset = reg[0];
			xlp_nor_partitions[num_parts].offset = offset;
			xlp_nor_partitions[num_parts].size = size;
		} else {
			printk(KERN_NOTICE "[%s] Partition %s in FDT size zero or not specified - skipping\n",
					__func__, xlp_nor_partitions[num_parts].name);
			xlp_nor_partitions[num_parts].name = NULL;
			continue;
		}

		prop_ptr = fdt_get_property(fdt, node_offset, "read-only", &lenp);
		NOR_DEBUG_PRINTK("  fdt_get_property for 'read-only' returned 0x%lx\n", 
				(unsigned long)prop_ptr);

		if(prop_ptr == NULL)	// 'read-only' not found, do not mask any flags
			xlp_nor_partitions[num_parts].mask_flags = 0;
		else					// 'read-only' found, mask writeable flag for partition
			xlp_nor_partitions[num_parts].mask_flags = MTD_WRITEABLE;
				
#ifdef NOR_DEBUG
		printk("  Partition %d name = %s, offset = 0x%x, size = 0x%x",
				num_parts, xlp_nor_partitions[num_parts].name, offset, size);
		if(prop_ptr == NULL)
			printk("\n");
		else
			printk(" (RO)\n");
#endif

		num_parts++;
		offset += size;
	};

	/* Zero out remaining entries in hard-coded table */
	if(num_parts && (num_parts < MAX_PARTITIONS) )
		for (i = num_parts; i < MAX_PARTITIONS; i++) {
			xlp_nor_partitions[i].name = NULL;
			xlp_nor_partitions[i].offset = 0;
			xlp_nor_partitions[i].size = 0;
			xlp_nor_partitions[i].mask_flags = 0;
		}

	return num_parts;
}

int __init xlp_nor_probe(struct platform_device *pdev)
{
	struct xlp_nor_info *info;
	int nb_parts = 0;
	int err;

	NOR_DEBUG_PRINTK("[%s]\n", __func__);

	info = kzalloc(sizeof(struct xlp_nor_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	if( (err = get_fdt_nor_map_info(fdt, info, cs)) )
		goto out_info;

	if( ((base << 8) + info->map.size) > ((limit << 8) + 0x100) ) {
		printk(KERN_WARNING "[%s] NOR flash size in FDT exceeds GBU BAR size\n",
				__func__);
		err = -EINVAL;
		goto out_info;
	}

	info->map.name			= dev_name(&pdev->dev);
	info->map.phys			= base << 8;
	info->map.virt			= ioremap(info->map.phys, info->map.size);
	if(!info->map.virt) {
		err = -ENOMEM;
		goto out_info;
	}
	info->mtd				= do_map_probe("cfi_probe", &info->map);
	if (!info->mtd) {
		err = -ENXIO;
		goto out_unmap;
	}
	info->mtd->owner		= THIS_MODULE;
	info->mtd->dev.parent	= &pdev->dev;

	NOR_DEBUG_PRINTK("  Name %s phys_base 0x%llx virt_base 0x%lx\n", info->map.name,
			(unsigned long long)info->map.phys, (unsigned long)info->map.virt);

	if (mtd_has_partitions()) {
		if (mtd_has_cmdlinepart()) {
			static const char *part_probes[] = { "cmdlinepart", NULL, };
			nb_parts = parse_mtd_partitions(info->mtd, part_probes, &info->parts, 0);
		}
		if(nb_parts <= 0) {
			nb_parts = get_fdt_nor_partitions(fdt, cs);
			if(nb_parts <= 0) {
				nb_parts = NUM_HARDCODED_PARTS;		// Nothing in FDT, use hard-coded
				NOR_DEBUG_PRINTK("  Using %d hard-coded partitions\n", nb_parts);
			}
			if(!info->parts)
				info->parts = xlp_nor_partitions;
		}
		err = add_mtd_partitions(info->mtd, info->parts, nb_parts);
	} else {
		err = add_mtd_device(info->mtd);
	}

	platform_set_drvdata(pdev, info);

	return 0;
	
out_unmap:
	iounmap(info->map.virt);
out_info:
	kfree(info);
	return err;
}

static int __exit xlp_nor_remove(struct platform_device *pdev)
{
	struct xlp_nor_info* info = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (info) {
		if(info->parts) {
			del_mtd_partitions(info->mtd);
			kfree(info->parts);
		}
		else
			del_mtd_device(info->mtd);
		map_destroy(info->mtd);
		iounmap(info->map.virt);
		kfree(info);
	}
	return 0;
}

static struct platform_driver xlp_nor_driver = {
	.probe		= xlp_nor_probe,
	.remove		= xlp_nor_remove,
	.driver		= {
		.name		= "nor-xlp",
		.owner		= THIS_MODULE,
	},
};

#define GBU_CS0_BASEADDRESS_REG			0
#define GBU_CS1_BASEADDRESS_REG			1
#define GBU_CS0_BASELIMIT_REG			8
#define GBU_CS1_BASELIMIT_REG			9

static int __init xlp_nor_init(void)
{
	u32 nandboot;
	uint64_t mmio = (uint64_t)cpu_io_mmio(0, SYS);

	nandboot = ((nlm_hal_read_32bit_reg(mmio, 1) & 0xF) == 6) ? 1 : 0;

	mmio = (uint64_t)cpu_io_mmio(0, GBU);
	if (nandboot)	{
		base = nlm_hal_read_32bit_reg(mmio, GBU_CS1_BASEADDRESS_REG);
		limit = nlm_hal_read_32bit_reg(mmio, GBU_CS1_BASELIMIT_REG);
		cs = 1;
	}
	else	{
		base = nlm_hal_read_32bit_reg(mmio, GBU_CS0_BASEADDRESS_REG);
		limit = nlm_hal_read_32bit_reg(mmio, GBU_CS0_BASELIMIT_REG);
		cs = 0;
	}

	printk(KERN_INFO "[%s] NOR chip select = %d base = 0x%x limit = 0x%x\n",
			__func__, cs, (base << 8), ((limit << 8) | 0xff) );

	if((base == 0xFFFFFFFF) && (limit == 0))	{
		return -1;
	}

	return platform_driver_register(&xlp_nor_driver);
}

static void __exit xlp_nor_exit(void)
{
	platform_driver_unregister(&xlp_nor_driver);
}

module_init(xlp_nor_init);
module_exit(xlp_nor_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("XLP SOC NOR MTD driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.2");
MODULE_ALIAS("platform:xlp-nor");
