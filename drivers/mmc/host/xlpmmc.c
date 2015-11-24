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

/*
 * linux/drivers/mmc/host/xlpmmc.c - XLP MMC driver
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/leds.h>
#include <linux/mmc/host.h>
#include <linux/delay.h>

#include <asm/io.h>
#ifdef CONFIG_GPIOLIB
#include <asm/gpio.h>
#endif
#include <hal/nlm_hal.h>
#include <hal/nlm_hal_xlp_dev.h>

#include "xlpmmc.h"

#define HOST_MODE_DMA

//#define XLP_MMC_DEBUG
#ifdef XLP_MMC_DEBUG
#define MMC_DEBUG_PRINTK		printk
#else
#define MMC_DEBUG_PRINTK(...)
#endif

struct xlpmmc_host {
	struct mmc_host *mmc;
	struct mmc_request *mrq;

	u32 flags;
	void __iomem *base;
	struct resource *ioarea;
	u32 clock;
	u32 bus_width;
	u32 power_mode;
	spinlock_t irq_lock; /* Prevent races with irq handler */
	int status;
	int present;
	int slot;

#ifdef HOST_MODE_DMA
	struct {
		int len;
		int dir;
	} dma;
#else
	struct {
		int index;
		int offset;
		int len;
	} pio;
#endif
	unsigned long workaround;
	int irq;
	struct timer_list timer;
	struct tasklet_struct card_tasklet;   /* Tasklet structures */

	struct xlpmmc_platform_data *platdata;
	struct platform_device *pdev;
} *xlpmmc_host_data[XLPMMC_MAX_SLOTS]={NULL};

static uint32_t gpio_regread(int regidx) 
{
	uint64_t mmio = nlm_hal_get_dev_base(NODE_0, BUS_0, XLP_PCIE_GIO_DEV, XLP_GIO_GPIO_FUNC);
	return nlm_hal_read_32bit_reg(mmio, regidx);
}

static void gpio_regwrite(int regidx, uint32_t val)
{
	uint64_t mmio = nlm_hal_get_dev_base(NODE_0, BUS_0, XLP_PCIE_GIO_DEV, XLP_GIO_GPIO_FUNC);
	nlm_hal_write_32bit_reg(mmio, regidx, val);
}

/* Remove dependency on GPIO lib */
static int xlp_gpio_get_value(int gpio)
{
	return (gpio_regread(XLP_GPIO_INPUT0) >> gpio) & 1;
}

/* Low-level write-routines*/
static inline void hc_wr32(void *iobase, int offset, u32 data, int slot) {

	int reg_offset = PCIE_HDR_OFFSET + (slot*XLP_SLOT_SIZE);
	volatile u32 *mmc_base = (u32 *)( (u8*)iobase + reg_offset );
	mmc_base[offset>>2] = data;
	return;
}

static inline void hc_wr16(void *iobase, int offset, u16 data, int slot) {

	int reg_offset = PCIE_HDR_OFFSET + (slot*XLP_SLOT_SIZE);
	volatile u16 *mmc_base = (u16 *)( (u8*)iobase + reg_offset );

	mmc_base[offset>>1] = data;
	return;
}

/* Low-level read-routines
 */
static inline u32 hc_rd32(void *iobase,int offset, int slot) {

	int reg_offset = PCIE_HDR_OFFSET + (slot*XLP_SLOT_SIZE);
	volatile u32 *mmc_base = (u32 *)( (u8*)iobase + reg_offset );

	u32 data = mmc_base[offset>>2];
	return data;
}

static inline u32 hc_rd16(void *iobase, int offset, int slot) {

	int reg_offset = PCIE_HDR_OFFSET + (slot*XLP_SLOT_SIZE);
	volatile u16 *mmc_base = (u16 *)( (u8*)iobase + reg_offset );

	u32 data = mmc_base[offset>>1];
	return data;
}

#ifdef XLP_MMC_DEBUG	
static void dump_hc_regs(struct xlpmmc_host *host, int slot)
{
	printk ("SLOT:%d MMC_PRESENT STATE = 0x%x\n", slot, hc_rd32((host->base), HC_PRESENT_STATE_LO, slot));
	printk ("SLOT:%d MMC_HOST_CTL_POWER_CTL = 0x%x\n", slot, hc_rd16((host->base), HC_PC_HC, slot));
	printk ("SLOT:%d MMC_CLOCK_CTL = 0x%x\n", slot, hc_rd16((host->base), HC_CLOCK_CTRL, slot));
	printk ("SLOT:%d MMC_CAP0 = 0x%x\n", slot, hc_rd32((host->base), HC_MMC_CAP0, slot));
	printk ("SLOT:%d MMC_CAP1 = 0x%x\n", slot, hc_rd32((host->base), HC_MMC_CAP1, slot));
}
#endif

static void xlpmmc_set_power(struct xlpmmc_host *host, int state, int slot)
{	
	if(state)
		hc_wr16(host->base, HC_PC_HC, 0x1f00, slot);
	else
		hc_wr16(host->base, HC_PC_HC, 0x1e00, slot);
}		

static int xlpmmc_card_inserted(struct mmc_host *mmc)
{
	#if 1
	// Use this case when using an eMMC.  There is no presence detect.
	return 1;
	#else
	struct xlpmmc_host *host = mmc_priv(mmc);
	int slot = host->slot;
	host->present = !xlp_gpio_get_value(GPIO_MMC_DETECT+slot);
	MMC_DEBUG_PRINTK("xlpmmc: Card present state is %d\n", host->present);
	return host->present;
	#endif
}

static int xlpmmc_card_readonly(struct mmc_host *mmc)
{
	//struct xlpmmc_host *host = mmc_priv(mmc);
	return -ENOSYS;
}

#if 0
static irqreturn_t xlpmmc_det_irq(int irq, void *dev_id)
{
	struct xlpmmc_host **xlpmmc_host_data = dev_id;
	int present, count; 

	MMC_DEBUG_PRINTK("Entered xlpmmc_det_irq\n");
	/*
	* we expect this irq on both insert and remove,
	* and use a short delay to debounce.
	*/
	for (count=0; count<XLPMMC_MAX_SLOTS; count ++) {
		struct xlpmmc_host *host = xlpmmc_host_data[count];
		present = !xlp_gpio_get_value(GPIO_MMC_DETECT+count); 
		gpio_regwrite(XLP_8XX_GPIO_INT_STAT0, gpio_regread(XLP_8XX_GPIO_INT_STAT0) 
				& 0x1<<(GPIO_MMC_DETECT+count));

		if(present)
			gpio_regwrite(XLP_8XX_GPIO_INT_POLAR0, gpio_regread(XLP_8XX_GPIO_INT_POLAR0) 
					& ~(0x1<<(GPIO_MMC_DETECT+count)));
		else
			gpio_regwrite(XLP_8XX_GPIO_INT_POLAR0, gpio_regread(XLP_8XX_GPIO_INT_POLAR0) 
					| 0x1<<(GPIO_MMC_DETECT+count));

		if (present != host->present) {
			host->present = present;
			pr_debug("%s: card %s\n", mmc_hostname(host->mmc),
					present ? "insert" : "remove");
			//tasklet_schedule(&host->card_tasklet);
			mmc_detect_change(host->mmc, msecs_to_jiffies(100));
		}
	}
	return IRQ_HANDLED;
}
#endif

static void xlpmmc_finish_request(struct xlpmmc_host *host)
{
	struct mmc_request *mrq = host->mrq;
	MMC_DEBUG_PRINTK("xlpmmc_finish_request mrq=0x%p slot %d\n**********\n", mrq, host->slot);

	host->mrq = NULL;

#ifdef HOST_MODE_DMA
	host->dma.len = 0;
	host->dma.dir = 0;
#else
	host->pio.index  = 0;
	host->pio.offset = 0;
	host->pio.len = 0;
#endif

	host->flags = 0;
	host->status = HOST_S_IDLE;

	mmc_request_done(host->mmc, mrq);
}

static void xlpmmc_send_command(struct xlpmmc_host *host, int wait,
				struct mmc_command *cmd, struct mmc_data *data, int slot)
{
	int rsp_type;
	u32 mmccmd;
	u32 hcstate;
	
	/* Parse the command and set the CMD and XFER reg accordingly */
	mmccmd = (cmd->opcode << CMD_IDX_SHT);

	/*Command type*/
	rsp_type = mmc_resp_type(cmd);
	switch (rsp_type) {
	case MMC_RSP_NONE:
		mmccmd |= RSP_TYPE_NONE;
		break;
	case MMC_RSP_R1:
		mmccmd |= RSP_TYPE_R1;
		break;
	case MMC_RSP_R1B:
		mmccmd |= RSP_TYPE_R1B;
		break;
	case MMC_RSP_R2:
		mmccmd |= RSP_TYPE_R2;
		break;
	case MMC_RSP_R3:
		mmccmd |= RSP_TYPE_R3;
		break;
	default:
		printk(KERN_INFO "xlpmmc: unhandled response type %02x\n",
			mmc_resp_type(cmd));
		return;
	}

	MMC_DEBUG_PRINTK("  %s: CMD%d data=0x%p rtype=0x%x arg=0x%x\n", __func__,
			cmd->opcode, data, rsp_type, cmd->arg);
	
	/*Data direction and count*/
	if (data) {
		//if((rsp_type != RSP_TYPE_R1B) && (rsp_type != RSP_TYPE_R5B) )/*do we ever come here?*/
		if((rsp_type != RSP_TYPE_R1B) )/*do we ever come here?*/
			mmccmd |= DP_DATA;

		/*enable block count*/
		mmccmd |= (1<<1);

#ifdef HOST_MODE_DMA
		mmccmd |= DMA_EN;
#endif
		
		if (data->blocks == 1){
			mmccmd |= MBS_SINGLE;
		}
		else{
			mmccmd |= MBS_MTPLE; 
			mmccmd |= AUTO_CMD12_EN;  
		}

		if (data->flags & MMC_DATA_READ) {
			mmccmd |= DDIR_READ;
		
		} else if (data->flags & MMC_DATA_WRITE) {
			mmccmd |= DDIR_WRITE;
		}
	} else {
		mmccmd |= DP_NO_DATA; 
	}

	/*Wait for CMD line to free. Safe to poll 
	as HOST CMD line could have been resetted.*/
	hcstate = hc_rd32(host->base, 0x24, slot);
	while (hcstate & 0x1) {
		/*Should never stuck here*/
		hcstate = hc_rd32(host->base, 0x24, slot);
	}

	mod_timer(&host->timer, jiffies + 1 * HZ);

	hc_wr32(host->base, HC_ARG1_LO, cmd->arg, slot);
	hc_wr32(host->base, HC_TX_MODE_COMMAND, mmccmd, slot);	
}

static void xlpmmc_data_complete(struct xlpmmc_host *host)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_data *data;

	if (host->mrq == NULL) {
		printk("SD slot %d data complete with null mrq\n", host->slot);
		dump_stack();
		return;
	}
	
	data = mrq->cmd->data;

	dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len, host->dma.dir);
	
	data->bytes_xfered = 0;

	if (!data->error) {
#ifdef HOST_MODE_DMA
		data->bytes_xfered = (data->blocks * data->blksz); 
#else
		data->bytes_xfered = (data->blocks * data->blksz) - host->pio.len;
#endif
	}
	xlpmmc_finish_request(host);
}

#ifndef HOST_MODE_DMA
static void xlpmmc_send_pio(struct xlpmmc_host *host, int slot)
{
	struct mmc_data *data;
	int sg_len, max, count;
	unsigned char *sg_ptr;
	struct scatterlist *sg;

	data = host->mrq->data;

	if (!(host->flags & HOST_F_XMIT))
		return;
	
	/* This is the pointer to the data buffer */
	sg = &data->sg[host->pio.index];
	sg_ptr = sg_virt(sg) + host->pio.offset;

	/* This is the space left inside the buffer */
	sg_len = data->sg[host->pio.index].length - host->pio.offset;

	/* Check if we need less than the size of the sg_buffer */
	max = (sg_len > host->pio.len) ? host->pio.len : sg_len;
	if (max > MMCSD_SECTOR_SIZE)
		max = MMCSD_SECTOR_SIZE;

	for (count = 0; count < max/4; count++) {
		u8 write_data[4];
		
		write_data[3] = *sg_ptr++ & 0xff; 
		write_data[2] = *sg_ptr++ & 0xff; 
		write_data[1] = *sg_ptr++ & 0xff; 
		write_data[0] = *sg_ptr++ & 0xff;
		hc_wr32(host->base, HC_BUFF_DATA_PORT0, *(u32*)write_data, slot);
	}
	count=count<<2;

	host->pio.len -= count;
	host->pio.offset += count;

	if (count == sg_len) {
		host->pio.index++;
		host->pio.offset = 0;
	}
}

static void xlpmmc_receive_pio(struct xlpmmc_host *host, int slot)
{
	struct mmc_data *data;
	int max, count, sg_len = 0;
	unsigned char *sg_ptr = NULL;
	struct scatterlist *sg;

	data = host->mrq->data;

	if (!(host->flags & HOST_F_RECV))
		return;

	max = host->pio.len;

	if (host->pio.index < host->dma.len) {
		sg = &data->sg[host->pio.index];
		sg_ptr = sg_virt(sg) + host->pio.offset;

		/* This is the space left inside the buffer */
		sg_len = sg_dma_len(&data->sg[host->pio.index]) - host->pio.offset;

		/* Check if we need less than the size of the sg_buffer */
		if (sg_len < max)
			max = sg_len;
	}
	if (max > MMCSD_SECTOR_SIZE){
		max = MMCSD_SECTOR_SIZE;
	}

	for (count = 0; count < max/4; count++) {
		volatile uint32_t read_data;
		read_data = hc_rd32(host->base, HC_BUFF_DATA_PORT0, slot);
			*sg_ptr++ = (unsigned char)((read_data >>  0) & 0xFF);
			*sg_ptr++ = (unsigned char)((read_data >>  8) & 0xFF);
			*sg_ptr++ = (unsigned char)((read_data >>  16) & 0xFF);
			*sg_ptr++ = (unsigned char)((read_data >>  24) & 0xFF);
	}
	count = count<<2;

	host->pio.len -= count;
	host->pio.offset += count;

	if (sg_len && count == sg_len) {
		host->pio.index++;
		host->pio.offset = 0;
	}
}
#endif /* !HOST_MODE_DMA */

/*
  1)This is called when a command has been completed - grab the response
     and check for errors. Then start the data transfer if it is indicated.
  2) Notify the core about command completion 
*/
static void xlpmmc_cmd_complete(struct xlpmmc_host *host, u32 status, int slot)
{
	struct mmc_request *mrq = host->mrq;
	struct mmc_command *cmd;
	u32 r[4];
	int i;

	if(host->status != HOST_S_CMD) {
		printk("%s entered but host not in CMD state\n", __func__);
		dump_stack();
	}

#if 0
	del_timer(&host->timer);
#endif

	if (host->mrq) {
		cmd = mrq->cmd;
		MMC_DEBUG_PRINTK("  %s: last opcode=CMD%d, arg=0x%08X, err=%d\n", __func__,
				cmd->opcode, cmd->arg, cmd->error);
	} else {
		MMC_DEBUG_PRINTK("  %s: mrq=(null)\n", __func__);
		host->status = HOST_S_IDLE;
		return;
	}

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			r[0] = hc_rd32(host->base, HC_RESPONSE3, slot);
			r[1] = hc_rd32(host->base, HC_RESPONSE2, slot);
			r[2] = hc_rd32(host->base, HC_RESPONSE1, slot);
			r[3] = hc_rd32(host->base, HC_RESPONSE0, slot);
			for (i = 0; i < 3; i++)
				cmd->resp[i] = (r[i] & 0x00FFFFFF) << 8 | (r[i + 1] & 0xFF000000) >> 24;
			cmd->resp[i] = (r[i] & 0x00FFFFFF) << 8;

			MMC_DEBUG_PRINTK("    RESP = 0x%08X 0x%08X 0x%08X 0x%08X\n",
					cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3]);
		} else { 
			cmd->resp[0] = hc_rd32(host->base, HC_RESPONSE0, slot);
			MMC_DEBUG_PRINTK("    RESP = 0x%08X\n", cmd->resp[0]);
		}
	}

	/* If no data to transfer or if there was an error, mark the request finished */
	if (!(host->flags & (HOST_F_XMIT | HOST_F_RECV)) || cmd->error) xlpmmc_finish_request(host); 
	else host->status = HOST_S_DATA;
}

static void xlpmmc_set_clock(struct xlpmmc_host *host, int rate, int slot)
{
	uint32_t divider;
	uint16_t hc_clk_ctl;

	/* Rate (requested by the kernel MMC core driver) and reference clock stored in the host
	 * struct are both in Hz.  Max clock is greater of 52 MHz - spec max for SD card HS mode
	 * or the highest divided reference clock that is <= 52 MHz.  To max out performance, need
	 * to change reference clock to 104 MHz.  100 MHz should be close enough.  Possible in
	 * Firefly, not in Eagle / Storm.  BUT - speed change function not available in 2.2.7 HAL.
	 * Actual clock post divider is ref / (divider value + 1).  PRM (possibly incorrectly) says
	 * ref / (divider value x2).
	 */
	if (rate > (XLP_REF_CLK133MHZ / 3)) {
		printk(KERN_INFO "[%s] Requested clock rate %dMHz exceeds maximum supported clock rate %dMHz\n",
				__func__, rate/1000/1000, XLP_REF_CLK133MHZ/3000/1000);
		divider = 3;
	} else
		divider = XLP_REF_CLK133MHZ / rate;
	if ((rate * divider) < (XLP_REF_CLK133MHZ-1000)) divider++;

	hc_clk_ctl = HCC_VALUE(divider);

//	MMC_DEBUG_PRINTK
	printk(KERN_DEBUG "[%s] Slot %d: ref=%uKHz target=%uKHz divider=%u actual=%uKHz HC_CLK_CTRL=0x%04X\n",
		 __func__, slot, XLP_REF_CLK133MHZ / 1000, rate / 1000, divider,
		(uint32_t)(XLP_REF_CLK133MHZ / divider)/1000, hc_clk_ctl);

	/* Disable SD clock, set new frequency */
	hc_clk_ctl |= HCC_INT_CLK_EN | HCC_10BIT;
	hc_wr16(host->base, HC_CLOCK_CTRL, hc_rd16(host->base, HC_CLOCK_CTRL, slot) & ~HCC_SD_CLK_EN, slot);
	hc_wr16(host->base, HC_CLOCK_CTRL, hc_clk_ctl, slot);

	/* Wait for stable clock */
	while ((hc_rd16(host->base, HC_CLOCK_CTRL, slot) & HCC_INT_CLK_STABLE) == 0) {};

	hc_wr16(host->base, HC_CLOCK_CTRL, hc_clk_ctl | HCC_SD_CLK_EN, slot);

	return;	
}

static int xlpmmc_prepare_data(struct xlpmmc_host *host,
				struct mmc_data *data, int slot)
{
	u32 mmc_blk_ctl;
	struct scatterlist *sg;

	MMC_DEBUG_PRINTK("xlpmmc_prepare_data datablks=%d block sz= 0x%x\n",
			data->blocks, data->blksz);

	if (data->flags & MMC_DATA_READ)
		host->flags |= HOST_F_RECV;
	else
		host->flags |= HOST_F_XMIT;

	if (host->mrq->stop)
		host->flags |= HOST_F_STOP;

#ifdef HOST_MODE_DMA
	host->dma.dir = DMA_BIDIRECTIONAL;
	host->dma.len = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len, host->dma.dir);
	if (host->dma.len == 0)
		return -EINVAL;

	sg = &data->sg[0];

#if 0
	if(sg_phys(sg) >= (4ULL<<30)){
		extern void dump_stack(void);
		printk(">4G ADDR: sg_phys(sg) = %#lx\n",sg_phys(sg));
		dump_stack();
	}
#endif

	hc_wr32(host->base, HC_SDMA_SA_OR_ARG2_LO, (unsigned long)sg_phys(sg), slot);
	host->workaround = (unsigned long)sg_phys(sg);

#else

	host->pio.index = 0;
	host->pio.offset = 0;
	host->pio.len = data->blocks * data->blksz;

#endif

	if(data->blocks > 1)	/*Multiple block transfer?*/
		mmc_blk_ctl = data->blocks<<BLK_CNT_SHT;
	else
		mmc_blk_ctl = 0;
	
	/*Lets slways set 64KB Host bufsize*/
	mmc_blk_ctl |= HOST_BUF_SZ_64;
	
	/*block size bit Xsz[0:11]=[0:11] and Xsz[12]=[15]*/
	mmc_blk_ctl |= (data->blksz<< 0);
	if(data->blksz & BLK_SZ_LOW_BITS)
		mmc_blk_ctl |= BLK_SZ_HGH_BIT;
	else
		mmc_blk_ctl &= ~BLK_SZ_HGH_BIT;

	hc_wr32(host->base, HC_BLOCK_SIZE, mmc_blk_ctl, slot);
	MMC_DEBUG_PRINTK("  xlpmmc_prepare_data datablks=%d block sz= 0x%x\n", data->blocks,  data->blksz);
	
	return 0;
}

/* This actually starts a command or data transaction */
static void xlpmmc_request(struct mmc_host* mmc, struct mmc_request* mrq)
{
	struct xlpmmc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd = mrq->cmd;
	int ret  = 0;
	int slot = host->slot;

	MMC_DEBUG_PRINTK("\n**********\nSD slot %d beginning request, CMD%d, arg=0x%08x data=0x%p\n",
			slot, cmd->opcode, cmd->arg, mrq->data);

	WARN_ON(irqs_disabled());
	WARN_ON(host->status != HOST_S_IDLE);

#if XLPMMC_MAX_SLOTS == 2
	if((xlpmmc_host_data[0]->status != HOST_S_IDLE) || (xlpmmc_host_data[1]->status != HOST_S_IDLE)) {
		printk(KERN_WARNING "Serializing commands to mutliple MMC Slots\n");
		while((xlpmmc_host_data[0]->status != HOST_S_IDLE) || (xlpmmc_host_data[1]->status != HOST_S_IDLE))
		{}
	}
#endif

	host->mrq = mrq;
	host->status = HOST_S_CMD;

	/* fail request immediately if no card is present */
#if 0	
	if (0 == xlpmmc_card_inserted(mmc)) {
		mrq->cmd->error = -ENOMEDIUM;
		xlpmmc_finish_request(host);
		return;
	}
	/* No platform support to know card detection */
#endif
	if (mrq->data) {
		ret = xlpmmc_prepare_data(host, mrq->data, slot);
	}

	if(ret) {
		printk(KERN_WARNING "%s xlpmmc_prepare_data returned error %d\n", __func__, ret);
		mrq->cmd->error = ret;
		xlpmmc_finish_request(host);
		return;
	}

	xlpmmc_send_command(host, 0, cmd, mrq->data, slot);

	/* Work around for slot0, when using two SD/MMCslots */
	hc_wr32(host->base, HC_SDMA_SA_OR_ARG2_LO, host->workaround, slot);
}

static void xlpmmc_reset_controller(struct xlpmmc_host *host, int slot)
{
	/* Enable Interrupts */
	hc_wr32(host->base, HC_NORMAL_INT_STS_EN, 0x37ff7fff, slot);

	/* Enable Interrupt Signals */
	hc_wr32(host->base, HC_NORMAL_INT_SIGNAL_EN, 0x37ff7fff, slot);

	/* Send HW Reset to eMMC-4.4 Card */
	hc_wr16(host->base, HC_PC_HC, 0x1e00, slot);
	hc_wr16(host->base, HC_PC_HC, 0x1f00, slot);

	/* Remove HW Reset to eMMC-4.4 Card */
	hc_wr16(host->base, HC_PC_HC, 0x0f00, slot);

	return;
}

static void xlpmmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct xlpmmc_host *host = mmc_priv(mmc);
	u16 hc_pc_hc;
	int slot = host->slot;

	MMC_DEBUG_PRINTK ("xlpmmc_set_ios host 0x%p ios 0x%p power %u clock %u bus-width %u slot %u\n",
			host, ios, ios->power_mode, ios->clock, ios->bus_width, slot);	

	/*Power */
	if (ios->power_mode == MMC_POWER_OFF)
		xlpmmc_set_power(host, 0, slot);
	else if (ios->power_mode == MMC_POWER_ON) {
		xlpmmc_set_power(host, 1, slot);
	}
	msleep(5);

	/* Clock */
	if (ios->clock && ios->clock != host->clock) {
		xlpmmc_set_clock(host, ios->clock, slot);
		host->clock = ios->clock;
	}
	
	/*BUS width*/
	hc_pc_hc = hc_rd16(host->base, HC_PC_HC, slot);
	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_4:
		hc_pc_hc |= HC_HCR_4BIT_MODE;
		break;
	case MMC_BUS_WIDTH_1:
		hc_pc_hc &= ~HC_HCR_4BIT_MODE;
		break;
	}
	hc_wr16 (host->base, HC_PC_HC, hc_pc_hc, slot);
	msleep(5);

#ifdef XLP_MMC_DEBUG	
	dump_hc_regs(host, slot);
#endif

}

static void xlpmmc_timeout_timer(long unsigned int data)
{  
	struct xlpmmc_host *host = (struct xlpmmc_host *) data;
	int slot = host->slot;

	MMC_DEBUG_PRINTK(KERN_INFO "[%s] slot %d\n", __func__, slot);

	spin_lock(&host->irq_lock);
	if (host->mrq) {
		if(host->mrq->cmd) {
			if (host->mrq->cmd->data && (host->status == HOST_S_DATA)) {
				MMC_DEBUG_PRINTK(KERN_WARNING "SD slot %d software data time-out\n", slot);
				host->mrq->data->error = -ETIMEDOUT; 
				xlpmmc_data_complete(host);
			} else {
				MMC_DEBUG_PRINTK(KERN_WARNING "SD slot %d software command time-out\n", slot);
				host->mrq->cmd->error = -ETIMEDOUT;
				/* Reset the CMD line */
				hc_wr16(host->base, HC_SWRST_TIMEOUT_CTRL, SW_RST_CMD, slot);
				xlpmmc_cmd_complete(host, 0, slot);
			}
		}
	}
	spin_unlock(&host->irq_lock);
}

static irqreturn_t xlpmmc_irq(int irq, void *dev_id)
{
	int slot;
	struct xlpmmc_host **xlpmmc_host_data = dev_id;
	for (slot=0; slot<XLPMMC_MAX_SLOTS; slot ++) {
		struct xlpmmc_host *host = xlpmmc_host_data[slot];
		u16 intstatus = hc_rd16(host->base, HC_NORMAL_INT_STS, slot);
		u16 interrsts = 0;
		u16 last_arg = 0;
		int last_opcode = -1;
		struct mmc_command *cmd = NULL;

		if(!intstatus)
			continue;

		/* 1. Lock the host struct and ack the interrupts */
		spin_lock(&host->irq_lock);
		hc_wr16(host->base, HC_NORMAL_INT_STS, intstatus, slot);

		MMC_DEBUG_PRINTK(KERN_CONT "    SD slot %d xlpmmc_irq interrupt status = 0x%04X, host state = %d host flags = 0x%x",
				slot, intstatus, host->status, host->flags);

		if(host->mrq) {
			cmd = host->mrq->cmd;
			if(cmd) {
				last_opcode = cmd->opcode;
				last_arg = cmd->arg;
				
				MMC_DEBUG_PRINTK(KERN_CONT " last opcode=CMD%d arg=0x%08x", last_opcode, last_arg);
			}
		}
		MMC_DEBUG_PRINTK("\n");

		/* 2. Check for error interrupt and handle any errors */
		if(intstatus & HNIS_ERR) {
			interrsts = hc_rd16(host->base, HC_ERROR_INT_STS, slot);
			hc_wr16(host->base, HC_ERROR_INT_STS, interrsts, slot);
			MMC_DEBUG_PRINTK("    INT ERR STATUS 0x%04X\n", interrsts);

			/* Command time-out or CRC error */
			if(interrsts & CMD_TIMEOUT_ERR) {
				if (interrsts & CMD_CRC_ERR) {
					if(last_opcode > 0) cmd->error = -EILSEQ;
					printk("SD slot %d command line conflict or CRC error"
							", last opcode=CMD%d arg=0x%08x\n", slot, last_opcode, last_arg);
				} else {
					if(last_opcode > 0) cmd->error = -ETIMEDOUT;
					MMC_DEBUG_PRINTK("      Command timeout\n");
				}
				/* Reset the CMD line */
				hc_wr16(host->base, HC_SWRST_TIMEOUT_CTRL, SW_RST_CMD, slot);
#ifdef XLP_MMC_DEBUG
				if(host->status == HOST_S_DATA) printk("      Host state was data, expected command\n");
#endif
				xlpmmc_cmd_complete(host, intstatus, slot);
				spin_unlock(&host->irq_lock);
				continue;
			}

			/* Data time-out error - ignore if also have transfer complete */
			if(interrsts & DATA_TIMEOUT_ERR) {
				if(!(intstatus & HNIS_TC_CMPL)) {
					if(cmd)
						cmd->error = -ETIMEDOUT;
					printk(KERN_CONT "SD slot %d data timeout"
							", last opcode=CMD%d arg=0x%08x\n", slot, last_opcode, last_arg);
					if(host->status == HOST_S_DATA) {
						printk("\n");
						xlpmmc_data_complete(host);
					} else printk(" Host state was not data, expected data\n");
					spin_unlock(&host->irq_lock);
					continue;
				}

			/* Unhandled error */
			} else {
				if(cmd) cmd->error = -EIO;
				printk("SD slot %d unhandled SD interrupt (error status=0x%x), last opcode "
						"= CMD%d arg = 0x%08x\n", interrsts, slot, last_opcode, last_arg);
				if(host->status == HOST_S_DATA) xlpmmc_data_complete(host);
				else xlpmmc_cmd_complete(host, intstatus, slot);
				spin_unlock(&host->irq_lock);
				continue;
			}
		}

		/* 3. If no error or ignored error, first check for command completions */
		if(intstatus & HNIS_CMD_CMPL) {
			if(host->status == HOST_S_CMD) xlpmmc_cmd_complete(host, intstatus, slot);
#ifdef XLP_MMC_DEBUG
			else printk("    Cmd complete irq but host not in cmd state"
					", last opcode=CMD%d arg=0x%08x\n", last_opcode, last_arg);
#endif
		}

#ifndef HOST_MODE_DMA
		/* 4. Check for PIO buffers */
		if(intstatus & HNIS_BUFF_WR_RDY) {
#ifdef XLP_MMC_DEBUG
			if(!host->flags & HOST_F_XMIT) {
				printk("    PIO write buffer ready but host write flag not set\n");
			}
#endif
			xlpmmc_send_pio(host, slot);
		}
		if(intstatus & HNIS_BUFF_RD_RDY) {
#ifdef XLP_MMC_DEBUG
			if(!host->flags & HOST_F_RECV) {
				printk("    PIO read buffer ready but host read flag not set\n");
			}
#endif
			xlpmmc_receive_pio(host, slot);
		}
#endif /* !HOST_MODE_DMA */

		/* 5. Check for transfer completions */
		if(intstatus & HNIS_TC_CMPL) {
			if(host->status == HOST_S_DATA)	xlpmmc_data_complete(host);
#ifdef XLP_MMC_DEBUG
			else printk("    data txfr complete irq but host not in data state\n");
#endif
		}

		/* 6. Card insert or remove */
		if(intstatus & (HNIS_CINS | HNIS_CREM)) tasklet_schedule(&host->card_tasklet);
	
		spin_unlock(&host->irq_lock);
	}
	return IRQ_HANDLED;
}

static void xlpmmc_tasklet_card(long unsigned int param)
{
	struct xlpmmc_host *host;
	host = (struct xlpmmc_host*)param;
	mmc_detect_change(host->mmc, msecs_to_jiffies(0));
}

static const struct mmc_host_ops xlpmmc_ops = {
	.request	= xlpmmc_request,
	.set_ios	= xlpmmc_set_ios,
	.get_ro		= xlpmmc_card_readonly,
	.get_cd		= xlpmmc_card_inserted,
	//.enable_sdio_irq = xlpmmc_enable_sdio_irq,
};

static int __devinit xlpmmc_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc=NULL;
	struct xlpmmc_host *host=NULL;
	struct resource *iomem_r, *irq_r;
	int slot, irq, ret=0;
	void  __iomem *base;

	iomem_r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem_r) {
		dev_err(&pdev->dev, "no mmio defined\n");
		ret = -EINVAL;
		goto out1;
	}

	base = (void *)ioremap_nocache(iomem_r->start, PAGE_SIZE);
	if (!base) {
		dev_err(&pdev->dev, "cannot remap mmio\n");
		ret = -ENOMEM;
		goto out2;
	}

	irq_r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq_r) {
		dev_err(&pdev->dev, "no IRQ defined\n");
		ret = -EINVAL;
		goto out3;
	}

#if XLPMMC_MAX_SLOTS == 1
	/* Enable slot 0, L3 cache alloc */
	hc_wr32(base, HC_SYSCTRL, 0x14, 0);
#else
	/* Enable slots 0 and 1, L3 cache alloc */
	hc_wr32(base, HC_SYSCTRL, 0x1C, 0);
#endif

	msleep(5);

	for (slot=0; slot<XLPMMC_MAX_SLOTS; slot ++) {
		/* malloc memory for the generic mmc host data structure, one per slot */
		/* First argument of mmc_alloc_host is the amount of private (extra) data
		   needed by the host controller data structure. mmc_priv(mmc) will return
		   a pointer to this area. */
		mmc = mmc_alloc_host(sizeof(struct xlpmmc_host), &pdev->dev);
		if (!mmc) {
			dev_err(&pdev->dev, "no memory for mmc_host\n");
			ret = -ENOMEM;
			goto out3;
		}

		host = mmc_priv(mmc);
		xlpmmc_host_data[slot] = host;
		host->mmc = mmc;
		host->base = base;
		spin_lock_init(&host->irq_lock);

		/* Set Data time-out to 2^27 cycles */
		hc_wr16(base, HC_SWRST_TIMEOUT_CTRL, TIMEOUT_VAL, slot);

		/* Clear any set INT Bits */
		hc_wr32(base, HC_ERROR_INT_STS, hc_rd32(base, HC_ERROR_INT_STS, slot), slot);
		hc_wr32(base, HC_NORMAL_INT_STS, hc_rd32(base, HC_NORMAL_INT_STS, slot), slot);
	}

	irq = irq_r->start;
	ret = request_irq(irq, xlpmmc_irq, IRQF_SHARED,
			  "xlp-mmc", xlpmmc_host_data);
	if (ret) {
		dev_err(&pdev->dev, "cannot grab IRQ\n");
		for (slot=0; slot<XLPMMC_MAX_SLOTS; slot ++) {
			host = xlpmmc_host_data[slot];
			if (host->mmc)
				mmc_free_host(host->mmc);
		}
		goto out3;
	}

#if 0
	ret = request_irq(xlp_irt_to_irq(0, XLP_GPIO_INT0_IRT), xlpmmc_det_irq,
			IRQF_SHARED, "mmc-gpio", xlpmmc_host_data);
	if (ret) {
		dev_warn(&pdev->dev, "request MMC detect irq failed\n");
		free_irq(xlp_irt_to_irq(0, XLP_GPIO_INT0_IRT), xlpmmc_host_data);
	}
#endif

	for (slot=0; slot<XLPMMC_MAX_SLOTS; slot ++) {
		host = xlpmmc_host_data[slot];
		mmc  = host->mmc;
		host->platdata = pdev->dev.platform_data;
		host->pdev = pdev;
		host->slot = slot;
		host->irq = irq;
		host->base = base;
		host->ioarea = iomem_r;
		tasklet_init(&host->card_tasklet,
				xlpmmc_tasklet_card, (unsigned long)host);
		setup_timer(&host->timer, xlpmmc_timeout_timer, (unsigned long)host);

		ret = -ENODEV;

#ifdef CONFIG_GPIOLIB
		if (gpio_is_valid(GPIO_MMC_DETECT + slot)) {
			if (gpio_request(GPIO_MMC_DETECT+slot, "mmc_detect")) {
				pr_debug("no detect pin available\n");
			}
		}
#endif

		gpio_regwrite(XLP_GPIO_INTEN00, gpio_regread(XLP_GPIO_INTEN00) | 
				0x1<<(GPIO_MMC_DETECT+slot));
		gpio_regwrite(XLP_8XX_GPIO_INT_POLAR0, gpio_regread(XLP_8XX_GPIO_INT_POLAR0) | 
				0x1<<(GPIO_MMC_DETECT+slot));
		gpio_regwrite(XLP_8XX_GPIO_INT_TYPE0, gpio_regread(XLP_8XX_GPIO_INT_TYPE0) | 
				0x1<<(GPIO_MMC_DETECT+slot));

		mmc->ops = &xlpmmc_ops;

		/* Inform kernel of min and max clock divider values we support - assuming
		 * 10-bit divider and all divider values supported.  Note in 10b mode, divider
		 * value does not appear to be 2x the value in the register (PRM says it IS 2x) */
		mmc->f_min = XLP_REF_CLK133MHZ / 1023;
		mmc->f_max = XLP_REF_CLK133MHZ / 3;  	// 44.33 MHz

		mmc->max_blk_size = 512;
		mmc->max_blk_count = 2048;

		mmc->max_seg_size = XLPMMC_DESCRIPTOR_SIZE;
		mmc->max_segs = XLPMMC_DESCRIPTOR_COUNT;	

		mmc->ocr_avail = OCR_VDD_27_36; /* volt 2.70 ~ 3.60 */
	
		/* High speed mode at single data rate can be supported with 3.3V signaling */
		mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_SD_HIGHSPEED | MMC_CAP_MMC_HIGHSPEED;

		host->status = HOST_S_IDLE;

		xlpmmc_reset_controller(host, slot);
		msleep(5);

		ret = mmc_add_host(mmc);
		if (ret) {
			dev_err(&pdev->dev, "cannot add mmc host\n");
			goto out6;
		}

#ifdef HOST_MODE_DMA
		printk("Initialized SD/MMC Host Controller for slot %d, DMA mode\n", slot);
#else
		printk("Initialized SD/MMC Host Controller for slot %d, PIO mode\n", slot);
#endif

	}
	platform_set_drvdata(pdev, xlpmmc_host_data);

	return 0;	/*Everything is OK */

out6:
	/* Disable the host(s) if init fails */
	/* Disable GPIO interrupt for slot detect pin */
	gpio_regwrite(XLP_GPIO_INTEN00, gpio_regread(XLP_GPIO_INTEN00) & 
			~(0x1<<(GPIO_MMC_DETECT+slot)) );

	/* Normally you would free the pin if using GPIO_LIB, but once initialized
	   the host controller can't be uninitialized, so these pins are "lost" as
	   far as GPIOLIB is concerned */

	/* Disable Interrupts */
	hc_wr32(base, HC_NORMAL_INT_STS_EN, 0, slot);

	/* Disable Interrupt Signals */
	hc_wr32(base, HC_NORMAL_INT_SIGNAL_EN, 0, slot);

	tasklet_kill(&host->card_tasklet);
	del_timer_sync(&host->timer);
	xlpmmc_set_power(host, 0, slot);
	mmc_remove_host(host->mmc);
	mmc_free_host(mmc);

	if(slot) {
		slot--;
		host = xlpmmc_host_data[slot];
		if (host)
			goto out6;
	}

	free_irq(xlp_irt_to_irq(0, XLP_GPIO_INT0_IRT), xlpmmc_host_data);
	free_irq(irq, xlpmmc_host_data);
out3:
	iounmap((void *)base);
out2:
	release_resource(iomem_r);
out1:
	return ret;
}

static int __devexit xlpmmc_remove(struct platform_device *pdev)
{
	void *data = platform_get_drvdata(pdev);
	struct xlpmmc_host **xlpmmc_host_data = data;
	struct xlpmmc_host *host;
	int slot;
	
	for (slot=0; slot<XLPMMC_MAX_SLOTS; slot ++) {
#if 0
		/* Disable GPIO interrupt for slot detect pin */
		gpio_regwrite(XLP_GPIO_INTEN00, gpio_regread(XLP_GPIO_INTEN00) & 
				~(0x1<<(GPIO_MMC_DETECT+slot)) );

		/* Normally you would free the pin if using GPIO_LIB, but once initialized
		   the host controller can't be uninitialized, so these pins are "lost" as
		   far as GPIOLIB is concerned */
#endif

		host = xlpmmc_host_data[slot];
		if (host) {
			/* Disable Interrupts */
			hc_wr32(host->base, HC_NORMAL_INT_STS_EN, 0, slot);

			/* Disable Interrupt Signals */
			hc_wr32(host->base, HC_NORMAL_INT_SIGNAL_EN, 0, slot);

			tasklet_kill(&host->card_tasklet);
			del_timer_sync(&host->timer);
			xlpmmc_set_power(host, 0, slot);
			mmc_remove_host(host->mmc);
			mmc_free_host(host->mmc);
		}
	}

	free_irq(xlp_irt_to_irq(0, XLP_GPIO_INT0_IRT), xlpmmc_host_data);
	free_irq(host->irq, xlpmmc_host_data);
	iounmap((void *)host->base);
	release_resource(host->ioarea);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static int xlpmmc_suspend(struct platform_device *pdev, pm_message_t state)
{
	//TODO: Implement if required. 
	return 0;
}

static int xlpmmc_resume(struct platform_device *pdev)
{
	//TODO: Implement if required. 
	return 0;
}
#else
#define xlpmmc_suspend NULL
#define xlpmmc_resume NULL
#endif

static struct platform_driver xlpmmc_driver = {
	.probe         = xlpmmc_probe,
	.remove        = xlpmmc_remove,
	.suspend       = xlpmmc_suspend,
	.resume        = xlpmmc_resume,
	.driver        = {
		.name  = "mmc-xlp",
		.owner = THIS_MODULE,
	},
};

static int __init xlpmmc_init(void)
{
	return platform_driver_register(&xlpmmc_driver);
}

static void __exit xlpmmc_exit(void)
{
	platform_driver_unregister(&xlpmmc_driver);
}

module_init(xlpmmc_init);
module_exit(xlpmmc_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("XLP SOC SD/MMC Host Controller");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:xlp-mmc");
