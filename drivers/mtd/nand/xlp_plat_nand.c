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


#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#ifdef CONFIG_NLM_XLP
#include <asm/netlogic/hal/nlm_hal.h>
#include <asm/netlogic/xlp.h>
#endif
#include "xlp_plat_nand.h"

static int debug = 0;
static int enable_hwecc = 1;

#undef	NAND_DEBUG
#define BUF_SIZE 		(16*1024)
#define dbp_print		if(debug) printk
#if defined(CONFIG_N511) || defined(CONFIG_NLM_XMC_SUPPORT)
#define NAND_DEV_CS		0
#else
#define NAND_DEV_CS		1
#endif
#define DEFAULT_NUM_PARTITIONS	2

/* The oobsize is the area visible to software, and software will read/write in this region.
 * If hardware ecc is enabled, since our implementation of write_page does not calculate
 * hardware ECC, the hardware ECC area should not be overwritten by software.
 *
 * In this particular case, let us foce the oobsize to be 0xc. The hardware will use
 * the area in from spare area offset 0xc to the end of spare area.
 *
 * Why 0xc? This is the minimum space the hardware ECC will not occupy based on the
 * calculation in onfi_init.
 */
#define XLP_HWECC_OOBSIZE       0xc

/*
 * Make the nand file system partition 0 sufficiently large to cover the
 * range used by bootloaders.
 *
 * Currently (as of 09/2012), bootloader uses 12 blocks (x-loader, u-boot, u-boot env)
 * and the maximum observed block size in use is 512KB. Make the first partition
 * to be 16 blocks leaving some extra space in case bootloader needs more.
 */
#if defined(CONFIG_NLM_XMC_SUPPORT)
static struct mtd_partition xlp_nand_partition_info[] = {
	{
        .name = "X-LOADER",
        .offset = 0,
        .size = 4 * 64 * 2048,
        },
        {
        .name = "U-BOOT",
        .offset = MTDPART_OFS_APPEND,
        .size = 8 * 64 * 2048,
        },
        {
        .name = "U-BOOT ENV",
        .offset = MTDPART_OFS_APPEND,
        .size = 1 * 64 * 2048,
        },
        {
        .name = "U-BOOT ENV COPY",
        .offset = MTDPART_OFS_APPEND,
        .size = 1 * 64 * 2048,
        },
        {
         .name = "SYSCONFIG",
         .offset = MTDPART_OFS_APPEND,
         .size = 2 * 64 * 2048,
        },
        {
        .name = "USER",
        .offset = MTDPART_OFS_APPEND ,
        .size = MTDPART_SIZ_FULL ,
        },
};
#else
static struct mtd_partition xlp_nand_partition_info[] = {
        {
        .name = "NAND FS 0",
        .offset = 0,
        .size = 16 * 128 * 4096,
        },
        {
        .name = "NAND FS 1",
        .offset = MTDPART_OFS_APPEND ,
        .size = MTDPART_SIZ_FULL ,
        },
};
#endif

struct xlp_nand_data {
        struct nand_chip        chip;
        struct mtd_info         mtd;
        void __iomem            *io_base;
	int node;
#ifdef CONFIG_MTD_PARTITIONS
        int                     nr_parts;
        struct mtd_partition    *parts;
#endif
};

struct nand_state {
        int cs ;
        uint32_t col_cyc;
        uint32_t row_cyc;
        uint32_t page_size;
        uint32_t block_size ;
        uint32_t pages_per_block;
        uint32_t spare_size ;
        uint32_t last_cmd ;
        int buf_ptr ;
        u8* buf ;
} ;

static int column_prog = 0;
static int page_prog = 0;
static int waitfunc (struct mtd_info *mtd, struct nand_chip *chip) ;

static __inline__ int32_t nand_reg_read(int node,  int regidx)
{
	uint64_t mmio = nlm_hal_get_dev_base(node, 0, XLP_PCIE_SPI_NOR_FLASH_DEV, XLP_PCIE_SPI_NAND);
	return nlm_hal_read_32bit_reg(mmio, regidx);
}

static __inline__ void nand_reg_write(int node, int regidx, int32_t val)
{
	uint64_t mmio = nlm_hal_get_dev_base(node, 0, XLP_PCIE_SPI_NOR_FLASH_DEV, XLP_PCIE_SPI_NAND);
	nlm_hal_write_32bit_reg(mmio, regidx, val);
}

static int dma_wait(int node, int cs)
{
	int timeout = 0xfffff ;
	while ( ( (nand_reg_read(node, NAND_STATUS) & (1 << cs) ) == 0) ||
			( (nand_reg_read(node, NAND_DMA_CTRL) & 0x01) == 0) )  {

		timeout--;
		if (timeout == 0) {
			dbp_print("DMA timed out NAND_STATUS:%x\n", nand_reg_read(node, NAND_STATUS)) ;
			return -1 ;
		}
	}
	return 0;
}

#ifdef NAND_DEBUG
static void print_oob_data(const unsigned char* p)
{
	int i;

	if(!p)
		return;

	for(i = 0; i < 256; i++) {
		printk("%02x",p[i]);
		if((i % 16) == 0xf)
			printk("\n");
	}
        printk("\n");
}

static void print_onfi_params(struct nand_onfi_params * p)
{
	printk("OFNI parameter \n");
	printk("rev info and features block\n");
	printk("revision:%d\n", le16_to_cpu(p->revision));
	printk("features:%x\n", p->features);
	printk("opt_cmd::%x\n", p->opt_cmd);

	printk("\n memory organization block \n");


	printk("byte_per_page:%d\n",le32_to_cpu(p->byte_per_page));
	printk("spare_bytes_per_page: %d\n", le16_to_cpu(p->spare_bytes_per_page));
	printk("data_bytes_per_ppage: %d\n", le32_to_cpu(p->data_bytes_per_ppage));
	printk("spare_bytes_per_ppage: %d\n", le16_to_cpu(p->spare_bytes_per_ppage));
	printk("pages_per_block: %d\n", le32_to_cpu(p->pages_per_block));
	printk("blocks_per_lun: %d\n", le32_to_cpu(p->blocks_per_lun));
	printk("lun_count: %d\n", p->lun_count);
	printk("addr_cycles: %d\n", p->addr_cycles);
	printk("bits_per_cell: %d\n", p->bits_per_cell);
	printk("bb_per_lun: %d\n",  le16_to_cpu(p->bb_per_lun));
	printk("block_endurance: %d\n",  le16_to_cpu(p->block_endurance));
	printk("guaranteed_good_blocks: %d\n", p->guaranteed_good_blocks);
	printk("guaranteed_block_endurance: %d\n", le16_to_cpu(p->guaranteed_block_endurance));
	printk("programs_per_page: %d\n", p->programs_per_page);
	printk("ppage_attr: %d\n", p->ppage_attr);
	printk("ecc_bits: %d\n", p->ecc_bits);
	printk("interleaved_bits: %d\n", p->interleaved_bits);
	printk("interleaved_ops: %d\n", p->interleaved_ops);

	printk("\nelectrical parameter block\n");
	printk("io_pin_capacitance_max: %d\n", p->io_pin_capacitance_max);
	printk("async_timing_mode: %d\n", p->async_timing_mode);
	printk("program_cache_timing_mode: %d\n", p->program_cache_timing_mode);
	printk("t_prog: %d\n", p->t_prog);
	printk("t_bers: %d\n", p->t_bers);
	printk("t_r: %d\n", p->t_r);
	printk("t_ccs: %d\n", p->t_ccs);
	printk("src_sync_timing_mode: %d\n", p->src_sync_timing_mode);
	printk("src_ssync_features: %d\n", p->src_ssync_features);
	printk("clk_pin_capacitance_typ: %d\n", p->clk_pin_capacitance_typ);
	printk("io_pin_capacitance_typ: %d\n", p->io_pin_capacitance_typ);
	printk("input_pin_capacitance_typ: %d\n", p->input_pin_capacitance_typ);
	printk("input_pin_capacitance_max: %d\n", p->input_pin_capacitance_max);
	printk("driver_strenght_support: %d\n", p->driver_strenght_support);
	printk("t_int_r: %d\n", p->t_int_r);
	printk("t_ald: %d\n", p->t_ald);
	printk("crc: %x\n", p->crc);

	printk("\nmanufacturer information block\n");
	printk("manufacturer: %s\n", p->manufacturer);
	printk("model: %s\n",  p->model);
	printk("jedec_id: %x\n",         p->jedec_id);
	printk("date_code: %x\n", p->date_code);

	return;
}
#endif

void onfi_init(int node, struct nand_chip *chip)
{
        struct nand_state *state = chip->priv;
        u8* param_ptr = state->buf ;
        uint32_t page_val ;
        uint32_t block_val;
        uint32_t addr_cyc,addr_val ;
        uint32_t spare_bytes_per_512 ;
        uint32_t ecc_bytes ;
        uint32_t ecc_bits __attribute__((unused)) ;
        uint32_t ecc_val ;
        uint32_t val ;
        uint32_t ecc_offset;
        int i ;

#ifdef NAND_DEBUG
	print_oob_data(state->buf);
#endif
        state->page_size = ( (unsigned int) (param_ptr[80] << 0)  |
                             (unsigned int) (param_ptr[81] << 8)  |
                             (unsigned int) (param_ptr[82] << 16) |
                             (unsigned int) (param_ptr[83] << 24) ) ;

        switch (state->page_size) {
        case 256:  page_val = 0; break;
        case 512:  page_val = 1; break;
        case 1024: page_val = 2; break;
        case 2048: page_val = 3; break;
        case 4096: page_val = 4; break;
        case 8192: page_val = 5; break;
        case 16384: page_val = 6; break;
        default: page_val = 7; break;
        }

        state->pages_per_block  = ( (unsigned int) (param_ptr[92] << 0)  |
                                    (unsigned int) (param_ptr[93] << 8)  |
                                    (unsigned int) (param_ptr[94] << 16) |
                                    (unsigned int) (param_ptr[95] << 24) ) ;

        state->block_size = state->pages_per_block * state->page_size ;


        switch (state->pages_per_block) {
        case 32:  block_val = 0; break;
        case 64:  block_val = 1; break;
        case 128: block_val = 2; break;
        case 256: block_val = 3; break;
        default: block_val = -1 ; break;
        }

        addr_cyc = param_ptr[101] ;
        state->row_cyc = (addr_cyc & 0xf) ;
        state->col_cyc = ( (addr_cyc >> 4) & 0xf) ;
        addr_val = state->row_cyc + state->col_cyc ;

        state->spare_size = ( (unsigned int) (param_ptr[84] << 0)  |
                              (unsigned int) (param_ptr[85] << 8) ) ;

	/* We may allocate too much space for ECC, but since u-boot uses
	 * the same alrogithm to allocate hardware ECC space. Let us
	 * keep it this way. Otherwise, the hardware ECC generated by
	 * linux won't work for u-boot.
	 */
        spare_bytes_per_512 = state->spare_size/(state->page_size/512) ;


        if (spare_bytes_per_512 <= 4) {
                ecc_bytes = 0 ;
                ecc_bits  = 0 ;
                ecc_val   = 0 ;
        } else if (spare_bytes_per_512 <= 8) {
                ecc_bytes = 4 ;
                ecc_bits  = 2 ;
                ecc_val   = 0 ;
        } else if (spare_bytes_per_512 <= 16) {
                ecc_bytes = 13 ;
                ecc_bits  = 8 ;
                ecc_val   = 3 ;
        } else if (spare_bytes_per_512 <= 24) {
                ecc_bytes = 20 ;
                ecc_bits  = 12 ;
                ecc_val   = 5 ;
        } else {
                ecc_bytes = 23 ;
                ecc_bits  = 14 ;
                ecc_val   = 6 ;
        }
        ecc_offset = state->spare_size - ( (state->page_size/512) * ecc_bytes);

	if(enable_hwecc)
	{
		nand_reg_write(node, NAND_ECC_CTRL, (ecc_val << 5));
		nand_reg_write(node, NAND_ECC_OFFSET, state->page_size + ecc_offset);

        	if (ecc_offset < XLP_HWECC_OOBSIZE)
                	printk("Warning: (file %s): adjust XLP_HWECC_OOBSIZE smaller for nand flash driver!\n",
                        	__FILE__);
		nand_reg_write(node, NAND_SPARE_SIZE, XLP_HWECC_OOBSIZE);

		val = nand_reg_read(node, NAND_CTRL);
		val |= (NAND_CTRL_ECC_EN(1)		|
			NAND_CTRL_PAGE_SIZE(page_val)	|
			NAND_CTRL_BLOCK_SIZE(block_val)	|
			NAND_CTRL_ADDR_CYCLE(addr_val))	;
		nand_reg_write(node, NAND_CTRL, val);

		/* The XLP implementation actually does not use chip->ecc data structures
		 * except oobfree fields.
		 * The default read_page/write_page (not xlp specific) indeed uses ecc.bytes/size etc.
		 */
		chip->ecc.size   = 512;
		chip->ecc.bytes  = ecc_bytes;
		chip->ecc.steps	 = state->page_size / 512;
		chip->ecc.total	 = chip->ecc.steps * chip->ecc.bytes;
		chip->ecc.layout = kmalloc(sizeof(struct nand_ecclayout), GFP_KERNEL);
		chip->ecc.layout->eccbytes = ecc_bytes;
		for (i=0; i < ecc_bytes; i++) {
			chip->ecc.layout->eccpos[i] = ecc_offset + i;
		}

		chip->ecc.layout->oobfree[0].offset = 2 ;
		chip->ecc.layout->oobfree[0].length = XLP_HWECC_OOBSIZE - 2;
	}
	else
	{
		nand_reg_write(node, NAND_SPARE_SIZE, state->spare_size);
		val = nand_reg_read(node, NAND_CTRL);
		val &= ~NAND_CTRL_ECC_EN(1);
		val |=  (NAND_CTRL_PAGE_SIZE(page_val)	|
			NAND_CTRL_BLOCK_SIZE(block_val)	|
			NAND_CTRL_ADDR_CYCLE(addr_val))	;
		nand_reg_write(node, NAND_CTRL, val);
	}
}



#ifdef NAND_DEBUG
static void nand_dump_reg(int node)
{
	int i;

	for(i = 0; i < 6; i++) {
		printk("nand 0x%0x = 0x%8x\n", i, nand_reg_read(node ,i));
	}

	for(i = 0x30; i < 0x4A; i++) {
		printk("nand 0x%0x = 0x%8x\n", i, nand_reg_read(node ,i));
	}

	for(i = 0x50; i < 0x5C; i++) {
		printk("nand 0x%0x = 0x%8x\n", i, nand_reg_read(node ,i));
	}
	for(i = 0x60; i < 0x70; i++) {
		printk("nand 0x%0x = 0x%8x\n", i, nand_reg_read(node ,i));
	}

	for(i = 0x80; i < 0x82; i++) {
		printk("nand 0x%0x = 0x%8x\n", i, nand_reg_read(node ,i));
	}
}

static void nand_dump_ioreg( uint32_t * ioaddr)
{
	int i;
	if(!ioaddr)
		return;
	for(i = 0; i < 12; i++)
		printk("nand 0x%0x = 0x%8x\n", i, ioaddr[i]);
	for(i = 25; i < 39; i++)
		printk("nand 0x%0x = 0x%8x\n", i, ioaddr[i]);

}
#endif

static void send_cmd(struct mtd_info *mtd,
                     unsigned int command,
                     int column,
                     int page_addr,
                     int len)
{
	struct xlp_nand_data *priv =  (struct xlp_nand_data*)mtd->priv;
        struct nand_chip *chip = &priv->chip;
        struct nand_state *state = chip->priv;
	uint64_t val;
	int node = priv->node;
	int column2, len2;

	/* The hardware ECC will be generated if the size is mtd->writesize.
	 * So if the write data is more than mtd->writesize, let us break it into two.
	 * hardware ECC will be disabled for the second part.
	 */
        if ((column + len) > mtd->writesize) {
                if (mtd->writesize > 0) {
                        /* initialized */
                        column2 = mtd->writesize;
                        len2 = column + len - mtd->writesize;
                        len = mtd->writesize - column;
                } else {
                        /* not initialized yet, ECC has to be disabled as it has not been configured properly */
                        column2 = column;
                        len2 = len;
                        len = 0;
                }
        } else {
                column2 = 0;
                len2 = 0;
        }

	if (len > 0) {
        	nand_reg_write(node, NAND_DATA_SIZE, len);
        	nand_reg_write(node, NAND_DMA_CNT, len);

		val = (page_addr >> (32 - (state->col_cyc * 8)));
        	nand_reg_write(node, NAND_ADDR0_H, val);
		val = ( (page_addr << (state->col_cyc * 8) ) | column);
        	nand_reg_write(node, NAND_ADDR0_L, val) ;
		val = virt_to_phys((void *) state->buf) + state->buf_ptr;
        	nand_reg_write(node, NAND_DMA_ADDR, val);
		nand_reg_write(node, NAND_DMA_ADDR_H, (val >> 32));

        	if ( (command == NAND_READ_PAGE_CMD) ||
             	     (command == NAND_READ_ID_CMD)   ||
             	     (command == NAND_READ_PARAMETER_CMD) ) {
                	nand_reg_write(node, NAND_DMA_CTRL, (1 << 7) | (1 << 6) | (5 << 2));
        	} else {
                	nand_reg_write(node, NAND_DMA_CTRL, (1 << 7) | (0 << 6) | (5 << 2));
        	}

        	nand_reg_write(node, NAND_CMD, command | NAND_CMD_DMA_FLAG);
        	dma_wait(node, state->cs);
	}

	if (len2 > 0) {
        	nand_reg_write(node, NAND_DATA_SIZE, len2);
        	nand_reg_write(node, NAND_DMA_CNT, len2);

		val = (page_addr >> (32 - (state->col_cyc * 8)));
        	nand_reg_write(node, NAND_ADDR0_H, val);
		val = ( (page_addr << (state->col_cyc * 8) ) | column2);
        	nand_reg_write(node, NAND_ADDR0_L, val) ;
		val = virt_to_phys((void *) state->buf) + state->buf_ptr + len;
        	nand_reg_write(node, NAND_DMA_ADDR, val);
		nand_reg_write(node, NAND_DMA_ADDR_H, (val >> 32));

        	if ( (command == NAND_READ_PAGE_CMD) ||
             	     (command == NAND_READ_ID_CMD)   ||
             	     (command == NAND_READ_PARAMETER_CMD) ) {
                	nand_reg_write(node, NAND_DMA_CTRL, (1 << 7) | (1 << 6) | (5 << 2));
        	} else {
                	nand_reg_write(node, NAND_DMA_CTRL, (1 << 7) | (0 << 6) | (5 << 2));
        	}
		if (enable_hwecc) {
			 val = nand_reg_read(node, NAND_CTRL);
			 val &= ~NAND_CTRL_ECC_EN(1);
			 nand_reg_write(node, NAND_CTRL, val);
		}
        	nand_reg_write(node, NAND_CMD, command | NAND_CMD_DMA_FLAG);
        	dma_wait(node, state->cs);
		if (enable_hwecc) {
			 val = nand_reg_read(node, NAND_CTRL);
			 val |= NAND_CTRL_ECC_EN(1);
			 nand_reg_write(node, NAND_CTRL, val);
		}
	}

        state->last_cmd    = command;
}

static void cmdfunc(struct mtd_info *mtd,
                    unsigned int command,
                    int column,
                    int page_addr)
{
	struct xlp_nand_data *priv =  (struct xlp_nand_data*)mtd->priv;
        struct nand_chip *chip = &priv->chip;
        struct nand_state *state = chip->priv;
	int node = priv->node;
        int len = 0 ;
        uint32_t val;

        if (state->cs < 0)
                return;

        switch (command) {
	/*
	 * READ0 - read in first  256 bytes
	 * READ1 - read in second 256 bytes
	 */
        case NAND_CMD_READ1:
                column += 256;
        case NAND_CMD_READ0:
                state->buf_ptr = 0;
                send_cmd(mtd,
                         NAND_READ_PAGE_CMD,
                         column,
                         page_addr,
                         mtd->writesize);
                state->buf_ptr += mtd->writesize;
                send_cmd(mtd,
                         NAND_READ_PAGE_CMD,
                         (mtd->writesize + column),
                         page_addr,
                         (mtd->oobsize - column));
                state->buf_ptr = 0;
		break;
        /* READOOB reads only the OOB because no ECC is performed. */
        case NAND_CMD_READOOB:
               dbp_print("NAND_CMD_READOOB, "
                    "page_addr: 0x%x, column: 0x%x oobsize: 0x%x.\n",
                    page_addr, column, mtd->oobsize);
                state->buf_ptr = 0;
                send_cmd(mtd,
                         NAND_READ_PAGE_CMD,
                         (mtd->writesize + column),
                         page_addr,
                         (mtd->oobsize - column));
                state->buf_ptr = 0;
		break;
        /* READID must read all 5 possible bytes while CEB is active */
        case NAND_CMD_READID:
                state->buf_ptr = 0;
                send_cmd(mtd,
                         NAND_READ_ID_CMD,
                         column,
                         0,
                         8);
                state->buf_ptr = 0;
		break;
        case NAND_CMD_PARAM:
                state->buf_ptr = 0;
                send_cmd(mtd,
                         NAND_READ_PARAMETER_CMD,
                         0,
                         0,
                         1024);
                onfi_init(node, chip);
		break;
        /* ERASE1 stores the block and page address */
        case NAND_CMD_ERASE1:
                dbp_print("NAND_CMD_ERASE1, "
                    "page_addr: 0x%x, column: 0x%x.\n",
                    page_addr, column);
                val = (page_addr >> (32 - (state->col_cyc*8)));
		nand_reg_write(node, NAND_ADDR0_H, val);
                val = ((page_addr << (state->col_cyc * 8)));
		nand_reg_write(node, NAND_ADDR0_L, val);
		break;
        /* ERASE2 uses the block and page address from ERASE1 */
        case NAND_CMD_ERASE2:
                dbp_print("NAND_CMD_ERASE2, "
                    "addr 0x%x\n",
		nand_reg_read(node, NAND_ADDR0_L));
		nand_reg_write(node, NAND_CMD, NAND_ERASE_BLOCK_CMD);
		state->last_cmd         = NAND_ERASE_BLOCK_CMD ;
		waitfunc(mtd,chip) ;
		break;
        /* SEQIN sets up the addr buffer and all registers except the length */
        case NAND_CMD_SEQIN:
                dbp_print("NAND_CMD_SEQIN/PAGE_PROG, "
                       "page_addr: 0x%x, column: 0x%x.\n",
                       page_addr, column);
                column_prog    = column;
                page_prog      = page_addr;
                state->buf_ptr = 0;
		break;
        /* PAGEPROG reuses all of the setup from SEQIN and adds the length */
        case NAND_CMD_PAGEPROG:
                len = state->buf_ptr;
                state->buf_ptr = 0;
                dbp_print("PAGE_PROG: page 0x%x col 0x%x size %d \n",page_prog,column_prog,len);
                send_cmd(mtd,
                         NAND_PAGE_PROGRAM_CMD,
                         column_prog,
                         page_prog,
                         len);
                waitfunc(mtd,chip);
		break;

        case NAND_CMD_STATUS:
                nand_reg_write(node, NAND_CMD, NAND_READ_STATUS_CMD);
                state->last_cmd = NAND_READ_STATUS_CMD;
                dbp_print("Status read\n");
		break;
        /* RESET command */
        case NAND_CMD_RESET:
                nand_reg_write(node, NAND_CMD, NAND_RESET_CMD);
                state->last_cmd = NAND_RESET_CMD;
                waitfunc(mtd,chip);
		break;

        default:
                dbp_print("netl8xx_nand: unsupported command 0x%x\n",command);
        }
}

static void select_chip(struct mtd_info *mtd, int dev)
{
	struct xlp_nand_data *priv =  (struct xlp_nand_data*)mtd->priv;
        struct nand_chip *chip = &priv->chip;
        struct nand_state *state = chip->priv;
	int node = priv->node;
        if ( (dev >= 0) && (dev < 8) ) {
                nand_reg_write(node, NAND_MEMCTRL, dev + NAND_DEV_CS);
		state->cs = dev + NAND_DEV_CS;
        } else {
                state->cs = -1;
        }
}

static uint8_t read_byte(struct mtd_info *mtd)
{
	struct xlp_nand_data *priv =  (struct xlp_nand_data*)mtd->priv;
        struct nand_chip *chip = &priv->chip;
        struct nand_state *state = chip->priv;
	int node = priv->node;
        uint32_t data ;

        if (state->cs < 0)
                return 0;

        if (state->last_cmd == NAND_READ_STATUS_CMD) {
                data = nand_reg_read(node, NAND_READ_STATUS);
                dbp_print("Status: %x\n",data);
                return nand_reg_read(node, NAND_READ_STATUS);
        } else {
                data = state->buf[state->buf_ptr];
                state->buf_ptr = (state->buf_ptr+1)%BUF_SIZE;
                return data;
        }
}

static void read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct xlp_nand_data *priv =  (struct xlp_nand_data*)mtd->priv;
        struct nand_chip *chip = &priv->chip;
        struct nand_state *state = chip->priv;
        int i;

        if (state->cs < 0)
                return;

        dbp_print("read_buf %p %d %d\n", buf, state->buf_ptr, len);
        for (i = 0; i < len; i++) {
                buf[i] = state->buf[state->buf_ptr];
                state->buf_ptr = (state->buf_ptr + 1) % BUF_SIZE;
        }
        return;
}

static void write_buf(struct mtd_info *mtd, const u8 *buf, int len)
{
        int i=0;
	struct xlp_nand_data *priv =  (struct xlp_nand_data*)mtd->priv;
        struct nand_chip *chip = &priv->chip;
        struct nand_state *state = chip->priv;

        if (state->cs < 0)
                return;

        dbp_print("write_buf %p %d %d\n",buf,state->buf_ptr,len);
        while (len > 0) {
                state->buf[state->buf_ptr] = buf[i++];
                len--;
                state->buf_ptr = (state->buf_ptr + 1) % BUF_SIZE;
        }
}

static int read_page(struct mtd_info *mtd,
                     struct nand_chip *chip,
                     uint8_t *buf, int page)
{
        struct nand_state *state = chip->priv;

        if (state->cs < 0)
                return -1;

        dbp_print ("Read page %s\n",buf);
        read_buf(mtd, buf, mtd->writesize);
	read_buf(mtd, chip->oob_poi, mtd->oobsize);

        return 0;
}

static void write_page(struct mtd_info *mtd,
                       struct nand_chip *chip,
                       const uint8_t *buf)
{
        struct nand_state *state = chip->priv;

        if (state->cs < 0)
                return;
        dbp_print ("Write page %p\n",buf);
        write_buf(mtd, buf, mtd->writesize);
	write_buf(mtd, chip->oob_poi, mtd->oobsize);
}

static int waitfunc(struct mtd_info *mtd, struct nand_chip *chip)
{
	struct xlp_nand_data *priv =  (struct xlp_nand_data*)mtd->priv;
        struct nand_state *state = chip->priv;
        int timeout = 0xfffff;
        uint32_t val;
	int node = priv->node;

        if (state->cs < 0)
                return -1;
		val = nand_reg_read(node, NAND_STATUS);
        while ((val & (1 << state->cs)) == 0) {
                timeout--;
                if (timeout == 0) {
                        dbp_print("wait func out\n");
                        return -1 ;
                }
			val = nand_reg_read(node, NAND_STATUS);
	}

        nand_reg_write(node, NAND_CMD, NAND_READ_STATUS_CMD);
        return nand_reg_read(node, NAND_READ_STATUS);
}

/*
 * Probe for the NAND device.
 */
static int __devinit xlp_plat_nand_probe(struct platform_device *pdev)
{
        struct xlp_nand_data *data;
        struct nand_state *state ;
        int res = 0;
        uint32_t val;
	int node=0;

        /* Allocate memory for the device structure (and zero it) */
        data = kzalloc(sizeof(struct xlp_nand_data), GFP_KERNEL);
        if (!data) {
                dev_err(&pdev->dev, "failed to allocate device structure.\n");
                return -ENOMEM;
        }

	pdev->resource[0].end	= pdev->resource[0].start + 0x1000 -1;
        data->io_base = ioremap(pdev->resource[0].start + 0x100,
                                pdev->resource[0].end - pdev->resource[0].start + 1);
        if (data->io_base == NULL) {
                dev_err(&pdev->dev, "ioremap failed\n");
                kfree(data);
                return -EIO;
        }
	/* nand_dump_ioreg((uint32_t*)  data->io_base);*/

	node = data->node = pdev->id;

        nand_reg_write(node, NAND_CTRL, NAND_CTRL_CUSTOM_XFER_FLAG);

        val = ( NAND_TIME_SEQ0_TWHR(7) |
		NAND_TIME_SEQ0_TRHW(7) |
		NAND_TIME_SEQ0_TADL(7) |
		NAND_TIME_SEQ0_TCCS(7) );

	nand_reg_write(node, NAND_TIME_SEQ0, val);

        val = NAND_TIME_ASYN_TRWH(8) | NAND_TIME_ASYN_TRWP(8);
	nand_reg_write(node, NAND_TIMINGS_ASYN, val);

        state            = kmalloc(sizeof(struct nand_state), GFP_KERNEL);
        state->last_cmd  = 0;
        state->cs        = 0;
        state->buf_ptr   = 0 ;
        state->buf       = kmalloc(BUF_SIZE, GFP_KERNEL);

        data->chip.priv = state;
        data->mtd.priv = data;
        data->mtd.owner = THIS_MODULE;
        data->mtd.name = dev_name(&pdev->dev);

	if (enable_hwecc)
		data->mtd.oobsize = XLP_HWECC_OOBSIZE;
	else
		data->mtd.oobsize = 64;

        data->chip.IO_ADDR_R = data->io_base;
        data->chip.IO_ADDR_W = data->io_base;

        data->chip.read_byte   	= read_byte ;
        data->chip.write_buf 	= write_buf;
        data->chip.read_buf 	= read_buf;
        data->chip.verify_buf  	= NULL ;
        data->chip.select_chip 	= select_chip;
        data->chip.cmdfunc	= cmdfunc ;
        data->chip.waitfunc	= waitfunc ;
        data->chip.chip_delay	= 15;


        data->chip.options      = NAND_NO_READRDY | NAND_NO_AUTOINCR | NAND_USE_FLASH_BBT ;
	if(enable_hwecc)
	        data->chip.ecc.mode = NAND_ECC_HW;
	else
		data->chip.ecc.mode = NAND_ECC_SOFT;

        data->chip.ecc.read_page  = read_page;
        data->chip.ecc.write_page = write_page;

        platform_set_drvdata(pdev, data);

        if (nand_scan(&data->mtd, 1)) {
                res = -ENXIO;
                goto out;
        }

#ifdef NAND_DEBUG
	print_onfi_params(&data->chip.onfi_params);
#endif

#ifdef CONFIG_MTD_PARTITIONS
	if (mtd_has_partitions()) {
                if (mtd_has_cmdlinepart()) {
                        static const char *part_probes[] = { "cmdlinepart", NULL, };
                        data->nr_parts = parse_mtd_partitions(&data->mtd, part_probes, &data->parts, 0);
                }
                if(data->nr_parts <= 0)
                {
                        data->nr_parts = ARRAY_SIZE(xlp_nand_partition_info);
                        if(!data->parts)
                                data->parts = xlp_nand_partition_info;
                }
                if(data->nr_parts > 0)
                {
			/* Register the partitions */
                        res = add_mtd_partitions(&data->mtd, data->parts, data->nr_parts);
                }
        }
	else
#endif
	res = add_mtd_device(&data->mtd);

	/* nand_dump_reg(node);*/

        if (!res)
                return res;

        nand_release(&data->mtd);
out:
        platform_set_drvdata(pdev, NULL);
        iounmap(data->io_base);
        kfree(data);
        return res;
}

/*
 * Remove a NAND device.
 */
static int __devexit xlp_plat_nand_remove(struct platform_device *pdev)
{
        struct xlp_nand_data *data = platform_get_drvdata(pdev);
        struct platform_nand_data *pdata = pdev->dev.platform_data;

        nand_release(&data->mtd);
#ifdef CONFIG_MTD_PARTITIONS
        if (data->parts && data->parts != pdata->chip.partitions)
                kfree(data->parts);
#endif
        if (pdata->ctrl.remove)
                pdata->ctrl.remove(pdev);
        iounmap(data->io_base);
        kfree(data);

        return 0;
}

static struct platform_driver xlp_plat_nand_driver = {
        .probe          = xlp_plat_nand_probe,
        .remove         = __devexit_p(xlp_plat_nand_remove),
        .driver         = {
                .name   = "nand-xlp",
                .owner  = THIS_MODULE,
        },
};

static int __init xlp_plat_nand_init(void)
{
        return platform_driver_register(&xlp_plat_nand_driver);
}
static void __exit xlp_plat_nand_exit(void)
{
        platform_driver_unregister(&xlp_plat_nand_driver);
}

module_init(xlp_plat_nand_init);
module_exit(xlp_plat_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Netlogicmicro System");
MODULE_DESCRIPTION("XLP NAND platform driver");
MODULE_ALIAS("platform:nand-xlp");

