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

#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/delay.h>
#include <asm/byteorder.h>
#include <asm/netlogic/hal/nlm_hal.h>
#include <asm/netlogic/xlp_irq.h>
#include <asm/netlogic/hal/nlm_hal_fmn.h>
#include <asm/netlogic/hal/nlm_hal_macros.h>
#include <asm/netlogic/hal/nlm_srio_xlp.h>
#include <asm/netlogic/xlp_srio.h>
#include <asm/netlogic/xlp.h>

#include "libfdt.h"
#include "fdt_helper.h"

typedef struct {
	int mode;
	int rxvc;
	int msgdstid;
	int baud_rate;
	int sys_size;
	int host_devid[4];
}bcm_srio_cfg;

//#define SRIO_DEBUG
#define SRIO_MAX_NODES		1
#define SRIO_MAX_PORTS		4
#define SRIO_CFGMEM_SIZE	4096
#define PCI_MEM_BAR_0   	0x4

#define SWCODE_RX_REQ           0
#define SWCODE_DOORBELL         254
#define SWCODE_DATAMSG          255
#define SWCODE_MAX              256

#define MAX_DATAMSGS            SWCODE_MAX

#define LOW_PRIORITY            0
#define HIGH_PRIORITY           1
#define DBELL_PRI               HIGH_PRIORITY
#define DATAMSG_PRI             HIGH_PRIORITY
#define IOTRANS_PRI             LOW_PRIORITY

#define NLM_QVALID              1

#ifdef SRIO_DEBUG
static int srio_debug_level=3;
#endif
static int rio_devid;
module_param(rio_devid, int, 0x90);

struct bcm_rio_block rio_block;
struct bcm_rio_port *rio_port[SRIO_MAX_PORTS];
bcm_srio_cfg rio_config;

static uint32_t rio_free_rx_bufs[NR_CPUS];

int (*bcm_rio_add_outb_msg)(struct bcm_rio_port *port, int mbox, uint64_t paddr, int size, int dest_id, int letter);
#define SRIO_DEFAULT_RXVC	3
#define SRIO_DEFAULT_MSGDSTID	0
#define SRIO_DEFAULT_SYS_SIZE	1

#ifdef SRIO_DEBUG
#define srio_dbg(level,fmt,args...) { \
        if (level & 1) \
                printk(fmt,##args);  \
}

#define srio_dbg2(level,fmt,args...)  \
{ \
        if (level & 2) \
                printk(fmt,##args);   \
}
#else
#define srio_dbg(...)
#define srio_dbg2(...)
#endif

#define BYTE_ALIGN_ERR(offset)          (0)
#define HWORD_ALIGN_ERR(offset)         (offset & 0x1)
#define WORD_ALIGN_ERR(offset)          (offset & 0x3)
#define DWORD_ALIGN_ERR(offset)         (offset & 0x7)

enum BCM_SRIO_APIERR {
ESRIO_API_INTERNAL = 1,
ESRIO_API_PARAM,
ESRIO_API_NOMEM,
ESRIO_ADDRESS_ALIGN,
ESRIO_TRANS_REQ,
ESRIO_MBOX_OPEN,
ESRIO_QUEUE_FULL,
ESRIO_RESP_TOUT,
};

/**
 * Send free buffer to RX data message Q.
 * @param [in] data Physical address of the 4KB aligned buffer of length 4KB.
 * @retval 0 on success
 * @retval -1 on failure
 * @ingroup srio
*/
int bcm_rio_send_free_rx_buff(uint64_t data)
{
	int retval;
	int retry = 0;

        if (data & (SRIO_BUF_SIZE - 1))
                return -ESRIO_API_INTERNAL;

        srio_dbg2(srio_debug_level,"%s: data 0x%llx \n", __func__, data);

send_retry:
        //4KB aligned address
        retval = nlh_send_msg1(XLP_SRIO_ADDRQ_VC, 0, (uint64_t)(data & 0xFFFFFFF000ULL) | (1ULL << 40));
        if (retval) {
		if (++retry < 200)	
			goto send_retry;
	        nlm_print("send to AddrQ failed 0x%x\n",retval);
	        return -ESRIO_API_INTERNAL;
        }
        return 0;
}



/**
 * bcm_rio_xlp3xxax_doorbell_send - Send a XLP doorbell message
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of target device
 * @data: 16-bit info field of RapidIO doorbell message
 *
 * Sends a XLP doorbell message. Returns %0 on success or
 * %-EINVAL on failure.
 */
static int bcm_rio_xlp3xxax_doorbell_send(struct rio_mport *mport,
                                int index, u16 destid, u16 data)
{
	struct bcm_rio_port *port = (struct bcm_rio_port *)mport->priv;
	uint64_t msg0 = 0, msg1 = 0;
	uint32_t msg_dest_id;
	int retval;
	int cpu = hard_smp_processor_id();

        srio_dbg2(srio_debug_level, "%s: dest_id %d info %d\n",__func__, destid, data);

        msg_dest_id = (cpu << 2) | port->rxvc;
        msg0 = XLP_SRIO_FMBRX_DESC0(port->hw_port_id, FTYPE_DOORBELL, destid, msg_dest_id, DBELL_PRI);
	msg0 |= port->sys_size;
        msg1 = ((uint64_t)data << 48);

        __sync();

        retval = nlh_send_msg2((port->txstn_id + DBELL_PRI),
                        SWCODE_DOORBELL,
                        msg0,
                        msg1);
        if (retval) {
                printk("send failed on cpu %d retval %x \n",cpu, retval);
                return -EINVAL;
        }

        return 0;
}


/*
 * Function to handle received datamessage response
 * @param rio bcm_rio_port structure returned by bcm_rio_port_init port.
 * @param src device id of the source
 * @param msg0  FMN msg0
 * @retval
 *      0 on success
 *      <0 on failure
 */
static int bcm_xlp_rio_datamsg_resp_handler(struct bcm_rio_port *rio, uint32_t src, uint64_t msg0)
{
	struct rio_mport *mport = rio->mport;
        uint32_t mbox, slot;
        int status = 0;
        pdatamsg_transacion_tx_t dmsg_txq;
	unsigned long flags;

        mbox = SRIO_MBOX(msg0);
        if (IS_LINK_RESP_TOUT(msg0))
                status = ESRIO_RESP_TOUT;
        else
                status = SRIO_RESP_STATUS(msg0);

        srio_dbg(srio_debug_level, "%s: src %d mbox %d\n",__func__,src, mbox);

        dmsg_txq = &rio->dmsg_txq[mbox];
        atomic_dec(&dmsg_txq->letter);
        spin_lock_irqsave(&dmsg_txq->dmsg_txlock, flags);
        if (dmsg_txq->used == 0) {
	        //error
                spin_unlock_irqrestore(&dmsg_txq->dmsg_txlock, flags);
		return -1;
        }
	slot = dmsg_txq->tail;
	dmsg_txq->tail = (dmsg_txq->tail + 1) % dmsg_txq->max_entries;
	dmsg_txq->used--;
	spin_unlock_irqrestore(&dmsg_txq->dmsg_txlock, flags);
        mport->outb_msg[mbox].mcback(mport, dmsg_txq->dev_id, -1,
                                                -1);
        return 0;
}


/*
 * Function to handle received request
 * @param rio bcm_rio_port structure returned by bcm_rio_port_init port.
 * @param ftype FTYPE of received request
 * @param src device id of the source
 * @param msg0  FMN msg0
 * @param msg1  FMN msg1
 * @param msg2  FMN msg2
 * @retval
 *      0 on success
 *      <0 on failure
 */

static int bcm_xlp_rio_rxreq_handler(struct bcm_rio_port *rio, uint32_t ftype, int src,
                uint64_t msg0,uint64_t msg1,uint64_t msg2)
{
        int cpu = hard_smp_processor_id();
	unsigned long flags;
        uint64_t paddr = 0;
        int len, mbox, trans, found = 0;
        datamsg_transacion_rx_t *rxmsgq;
        uint16_t dbell_info;
	struct rio_mport *mport = rio->mport;
	struct rio_dbell *dbell;

        switch(ftype) {
                case FTYPE_DOORBELL:
                        srio_dbg(srio_debug_level, "Cpu%d Rcvd FTYPE_DOORBELL\n", cpu);
                        dbell_info = SRIO_DBELL_INFO(msg1);
                        list_for_each_entry(dbell, &mport->dbells, node) {
                                if ((dbell->res->start <= dbell_info) &&
                                    (dbell->res->end >= dbell_info)) {
                                        found = 1;
                                        break;
                                }
                        }
                        if (found) {
                                dbell->dinb(mport, dbell->dev_id, src, -1, dbell_info);
                        }
#if 0 
			else {
                                printk("RIO: spurious doorbell, sid %2.2x info %4.4x\n",
                                       src, dbell_info);
                        }
#endif
                        break;
                case FTYPE_MESSAGE:
                        mbox = SRIO_MBOX(msg0);
                        srio_dbg(srio_debug_level, "Cpu%d Rcvd FTYPE_MESSAGE  mbox %d \n", cpu, mbox);
                        paddr = SRIO_PAYLOAD_ADDR(msg2);
			rxmsgq = &rio->dmsg_rxq[mbox];

			spin_lock_irqsave(&rxmsgq->dmsg_rxlock, flags);
			if (atomic_read(&rio->enabled) == 0) {
                                spin_unlock_irqrestore(&rxmsgq->dmsg_rxlock, flags);
                                return -ESRIO_API_INTERNAL;
                        }

			if ((msg1 & SRIO_DATAMSG_TIMEOUT) || (rxmsgq->status != NLM_QVALID) ||
				(rxmsgq->used == rxmsgq->max_entries)) {
				//FIXME send the buffer back to address Q. should not come here. 
				spin_unlock_irqrestore(&rxmsgq->dmsg_rxlock, flags);
                                if (bcm_rio_send_free_rx_buff(paddr))
                                        return -ESRIO_API_INTERNAL;
                                break;
			}
			else {
                                len = SRIO_PAYLOAD_LENGTH(msg1);
	                        rio_free_rx_bufs[cpu]--;
        	                *(uint64_t *)(rxmsgq->paddr + rxmsgq->head) = (uint64_t)phys_to_virt(paddr);
                	        rxmsgq->head = (rxmsgq->head + 1) % rxmsgq->max_entries;
                        	rxmsgq->used++;
				spin_unlock_irqrestore(&rxmsgq->dmsg_rxlock, flags);
	                        if (mport->inb_msg[mbox].mcback)
        	                        mport->inb_msg[mbox].mcback(mport, rxmsgq->dev_id, mbox, -1);
                	        else
                        	        printk("No rx call back for port %d mbox %d \n",rio->hw_port_id, mbox);
                        }
                        break;
                default:
			trans = SRIO_TRANS_TYPE(msg0);
                        srio_dbg(srio_debug_level, "%s: Rcvd req with ftype %d not handled, desc0 0x%llx trans %d \n", __func__, ftype, msg0, trans);
                        break;
        }
	return 0;
}

/*
 * Function to handle SRIO FMN messages on XLP3xxB0, XLP1xx, XLP2xx
 * @param swcode Received software code
 * @param size Message size
 * @param msg0  FMN msg0
 * @param msg1  FMN msg1
 * @param msg2  FMN msg2
 * @retval
 *      0 on success
 *      <0 on failure
 */

void bcm_xlp_rio_msghandler(uint32_t vc, uint32_t src_id,
                        uint32_t size, uint32_t swcode, uint64_t msg0,uint64_t msg1,
                        uint64_t msg2, uint64_t msg3, void *data)
{
        struct bcm_rio_port *rio;
        uint32_t port_id, src, ftype;

        if ((size == 0) || (src_id <  280) || (src_id > 288))
                return;

        port_id = SRIO_PORT(msg0);
        srio_dbg(srio_debug_level, "%s: port %d swcode %d msg0 0x%llx msg1 0x%llx msg2 0x%llx \n", __func__, port_id, swcode, msg0, msg1, msg2);

        ftype = SRIO_FTYPE(msg0);
        rio = rio_port[port_id];
        if (rio == NULL) {
                srio_dbg(srio_debug_level, "%s: Rcvd msg from uninitialized port %d, desc0 0x%llx \n", __func__,port_id, msg0);
                return;
        }

        if (swcode == SWCODE_RX_REQ) {
                src = SRIO_SRC_ID(msg0);
                bcm_xlp_rio_rxreq_handler(rio, ftype, src, msg0, msg1, msg2);
        }
        else if (swcode == SWCODE_DATAMSG) {
                if ((IS_LINK_RESP_TOUT(msg0)==0) && (ftype != FTYPE_RESPONSE)) {
                        printk("%s: Invalid Ftype in Datamsg resp, desc0 0x%llx \n",__func__, msg0);
                        return;
                }
                bcm_xlp_rio_datamsg_resp_handler(rio, 0, msg0);
        }
        else if (swcode == SWCODE_DOORBELL) {
		//printk("doorbell response\n");
                return;
        }
}

/**
 * This API will configure the mask for data message distribution
 * @param start - Starting byte offset in a data message packet for a 128 bit area of the packet
                        from which 8 bits will be extracted to form an index into the 256 entry data message
                        distribution table.
 * @param [in] mask0 Mask for first 32 bits
 * @param [in] mask1 Mask for second 32 bits
 * @param [in] mask2 Mask for third 32 bits
 * @param [in] mask3 Mask for last 32 bits
 * @retval 0 on success
 * @retval -1 on failure
 * @ingroup srio
 */
int bcm_rio_config_dmsg_lookup(uint32_t start, uint32_t mask0, uint32_t mask1, uint32_t mask2, uint32_t mask3)
{
        unsigned long flags;

        spin_lock_irqsave(&rio_block.dmesg_lock, flags);
        bcm_write_srio_devreg(rio_block.xlp_srio_dev_base, DATAMSGBYTE, start);
        bcm_write_srio_devreg(rio_block.xlp_srio_dev_base, DATAMSGEN3, mask0);
        bcm_write_srio_devreg(rio_block.xlp_srio_dev_base, DATAMSGEN2, mask1);
        bcm_write_srio_devreg(rio_block.xlp_srio_dev_base, DATAMSGEN1, mask2);
        bcm_write_srio_devreg(rio_block.xlp_srio_dev_base, DATAMSGEN0, mask3);
        spin_unlock_irqrestore(&rio_block.dmesg_lock, flags);
        srio_dbg2(srio_debug_level, "DATAMSGBYTE 0x%x mask0 0x%x \n",bcm_read_srio_devreg(rio_block.xlp_srio_dev_base, DATAMSGBYTE), bcm_read_srio_devreg(rio_block.xlp_srio_dev_base, DATAMSGEN3));
        return 0;
}

/**
 * This api will configure the door bell offset in 16 bit info field.
 * @param [in] offset Bit offset of door bell. Valid values: 0-12 on XLP3XX_AX, 0-11 on others
 * @retval 0 on success
 * @retval -1 on failure
 * @ingroup srio
 */
int bcm_rio_config_dbell_offset(uint32_t offset)
{
        unsigned long flags;
        uint32_t srio_comcfg;
        uint32_t max_dbell_off;

        if (is_nlm_xlp3xx_ax()) {
                max_dbell_off = MAX_DBELL_OFFSET_XLP3XX_AX;
        }
        else {
                max_dbell_off = MAX_DBELL_OFFSET;
        }

        if (offset > max_dbell_off)
                return -ESRIO_API_PARAM;

        spin_lock_irqsave(&rio_block.dbell_lock, flags);
        rio_block.dbell_offset = offset;
        srio_comcfg = bcm_read_srio_devreg(rio_block.xlp_srio_dev_base, SRICOMCONFIG);
        srio_comcfg &= DBELLINDEX_MASK | (offset << DBELL_INDX_START_POS);
        bcm_write_srio_devreg(rio_block.xlp_srio_dev_base, SRICOMCONFIG, srio_comcfg);
        spin_unlock_irqrestore(&rio_block.dbell_lock, flags);
        return 0;
}

int fdt_parse_srio_params(void *fdt, int nodeid, bcm_srio_cfg *rio_cfg)
{
	int nodeoffset, plen;
        char srio_fdt_path[80];
        char prop_str[10];
	void *pval;

	sprintf(srio_fdt_path, "/soc/srio@node-%d", nodeid);
        nodeoffset = fdt_path_offset(fdt, srio_fdt_path);
        if(nodeoffset < 0) {
		rio_cfg->mode = SRIO_MODE_x4;
		rio_cfg->rxvc = SRIO_DEFAULT_RXVC;
		rio_cfg->msgdstid = SRIO_DEFAULT_MSGDSTID;
		rio_cfg->baud_rate = SRIO_BAUD_6250M;
		rio_cfg->host_devid[0] = -1;
		rio_cfg->host_devid[1] = rio_cfg->host_devid[2] = rio_cfg->host_devid[3] = -1;
		return 1;
	}
	// TODO	 parse from fdta
	copy_fdt_prop(fdt, srio_fdt_path, "type", PROP_STR, prop_str, 10);
	if (strcmp(prop_str, "master") == 0)
		  rio_cfg->host_devid[0] = 0x90;
        else
          	  rio_cfg->host_devid[0] = -1;
	printk("RapidIo host type from fdt %s\n", prop_str);
	
	pval = (void *)fdt_getprop(fdt, nodeoffset, "mode", &plen);
        if(pval != NULL) {
        	rio_cfg->mode = fdt32_to_cpu(*(unsigned int *)pval);
        }
	else
		rio_cfg->mode = SRIO_MODE_x4;

        pval = (void *)fdt_getprop(fdt, nodeoffset, "rxvc", &plen);
        if(pval != NULL) {
                rio_cfg->rxvc = fdt32_to_cpu(*(unsigned int *)pval);
        }
        else
        	rio_cfg->rxvc = SRIO_DEFAULT_RXVC;

	
	pval = (void *)fdt_getprop(fdt, nodeoffset, "msgdstid", &plen);
        if(pval != NULL) {
                rio_cfg->msgdstid = fdt32_to_cpu(*(unsigned int *)pval);
        }
        else
        	rio_cfg->msgdstid = SRIO_DEFAULT_MSGDSTID;

	pval = (void *)fdt_getprop(fdt, nodeoffset, "baud-rate", &plen);
	if(pval != NULL) {
                rio_cfg->baud_rate = fdt32_to_cpu(*(unsigned int *)pval);
	}
	else
	        rio_cfg->baud_rate = SRIO_BAUD_6250M;

	pval = (void *)fdt_getprop(fdt, nodeoffset, "sys-size", &plen);
        if(pval != NULL) {
                rio_cfg->sys_size = fdt32_to_cpu(*(unsigned int *)pval);
        }
        else
		rio_cfg->sys_size = SRIO_DEFAULT_SYS_SIZE;

	printk("mode %d rxvc %d msgdstid %d baud-rate %d sys-size %d\n", rio_cfg->mode,
				rio_cfg->rxvc, rio_cfg->msgdstid, rio_cfg->baud_rate, rio_cfg->sys_size);
        rio_cfg->host_devid[1] = rio_cfg->host_devid[2] = rio_cfg->host_devid[3] = -1;
	return 1;
}

//Empty macro to fix compilation
#define cpu_to_be8      
#define be8_to_cpu       

#define BCM_RIO_DIRIO_LD(size, type) \
static type bcm_srio_dirio_##size(uint32_t offset, int opcode) \
{ \
        int cpu = hard_smp_processor_id(); \
        srio_dbg2(0, "lw 40bit phys 0x%llx\n",SRIO_DIRIO_40BITPHYS(offset, cpu, opcode)); \
        return (size##_40bit_phys_uncached(SRIO_DIRIO_40BITPHYS(offset, cpu, opcode))); \
}

BCM_RIO_DIRIO_LD(lb, uint8_t)
BCM_RIO_DIRIO_LD(lh, uint16_t)
BCM_RIO_DIRIO_LD(lw, uint32_t)
BCM_RIO_DIRIO_LD(ld, uint64_t)

// Direct io store

#define BCM_RIO_DIRIO_ST(size, type) \
static void bcm_srio_dirio_##size(uint32_t offset, int opcode, type data) \
{ \
        int cpu = hard_smp_processor_id(); \
        srio_dbg2(0, "sw 40bit phys 0x%llx\n",SRIO_DIRIO_40BITPHYS(offset, cpu, opcode)); \
        size##_40bit_phys_uncached(SRIO_DIRIO_40BITPHYS(offset, cpu, opcode), data); \
}

BCM_RIO_DIRIO_ST(sb, uint8_t)
BCM_RIO_DIRIO_ST(sh, uint16_t)
BCM_RIO_DIRIO_ST(sw, uint32_t)
BCM_RIO_DIRIO_ST(sd, uint64_t)


#define BCM_RIO_LOCAL_CFG_LD(size, type, addr) \
int bcm_rio_local_config_read_##size (struct bcm_rio_port *port, type *data, uint32_t offset) \
{ \
        if (addr##_ALIGN_ERR(offset)) \
                return -ESRIO_ADDRESS_ALIGN; \
        *((type *)data) = bcm_read_srio_reg##size(port->xlp_srio_base, offset); \
	/*printk("%s: port %d data 0x%x offset %d \n", __func__, port->hw_port_id, *data, offset);*/ \
        return 0; \
}

/**@{*/
/**
 * Local Config Read.
 * @param port Port on which transaction will be initiated.
 * @param data Physical address to read data into.
 * @param offset Offset in to local configuration space.
 * @retval 0 Transaction completed successfully.
 * @retval <0 Transaction failed.
 * @ingroup srio
 */
BCM_RIO_LOCAL_CFG_LD(8, uint8_t, BYTE)
BCM_RIO_LOCAL_CFG_LD(16, uint16_t, HWORD)
BCM_RIO_LOCAL_CFG_LD(32, uint32_t, WORD)
BCM_RIO_LOCAL_CFG_LD(64, uint64_t, DWORD)
/**@}*/


#define BCM_RIO_LOCAL_CFG_ST(size, type, addr) \
int bcm_rio_local_config_write_##size(struct bcm_rio_port *port,  type data, uint32_t offset) \
{ \
        if (addr##_ALIGN_ERR(offset)) \
                return -ESRIO_ADDRESS_ALIGN; \
/*        printk("%s port %p data 0x%x offset %d \n",__func__,port->xlp_srio_base, data, offset);*/ \
        bcm_write_srio_reg##size(port->xlp_srio_base, offset, ((type)data)); \
        return 0; \
}

/**@{*/
/**
 * Local Config Write.
 * @param port Port on which transaction will be initiated.
 * @param data Physical address of the data to be written.
 * @param offset Offset in to local configuration space.
 * @retval 0 Transaction completed successfully.
 * @retval <0 Transaction failed.
 * @ingroup srio
 */
BCM_RIO_LOCAL_CFG_ST(8, uint8_t, BYTE)
BCM_RIO_LOCAL_CFG_ST(16, uint16_t, HWORD)
BCM_RIO_LOCAL_CFG_ST(32, uint32_t, WORD)
BCM_RIO_LOCAL_CFG_ST(64, uint64_t, DWORD)
/**@}*/


#define BCM_RIO_REMOTE_CFG_LD(size, type, addr, func) \
int bcm_rio_config_read_##size(struct bcm_rio_port *port, type *data,  uint32_t offset,  int dest_id, int hopcount) \
{ \
        uint32_t cmd; \
	uint32_t dioindex = hard_smp_processor_id() & 0x1F; \
	unsigned long flags; \
				\
        if (addr##_ALIGN_ERR(offset)) \
                return -ESRIO_ADDRESS_ALIGN; \
        cmd = SRIO_DIRIO_CMD(dest_id, port->hw_port_id, port->sys_size, 0, 0, 0, hopcount); \
    /*    printk("%s Writing cmd 0x%x in %d offset 0x%x hopcount %d dest 0x%x \n",__func__, cmd, (DIRECTIO_CMD + dioindex), offset, hopcount, dest_id);i*/ \
	spin_lock_irqsave(&rio_block.dirio_lock[dioindex], flags); \
        bcm_write_srio_devreg(port->xlp_srio_dev_base, (DIRECTIO_CMD + dioindex ), cmd); \
        __sync(); \
	spin_unlock_irqrestore(&rio_block.dirio_lock[dioindex], flags); \
        *((type *)data) = be##size##_to_cpu(bcm_srio_dirio_##func(offset, DIRIO_OP_MREAD)); \
        return 0; \
}

/**@{*/
/**
 * Remote Config Read.
 * @param port SRIO port ID.
 * @param data Address to read data into.
 * @param offset Offset in to local configuration space.
 * @param dest_id Remote device ID
 * @param hopcount Should be 0xff if device is not a switch.
 * @retval 0 Transaction completed successfully.
 * @retval <0 Transaction failed.
 * @ingroup srio
 */
BCM_RIO_REMOTE_CFG_LD(8, uint8_t, BYTE, lb)
BCM_RIO_REMOTE_CFG_LD(16, uint16_t, HWORD, lh)
BCM_RIO_REMOTE_CFG_LD(32, uint32_t, WORD, lw)
BCM_RIO_REMOTE_CFG_LD(64, uint64_t, DWORD, ld)
/**@}*/


#define BCM_RIO_REMOTE_CFG_ST(size, type, addr, func) \
int bcm_rio_config_write_##size(struct bcm_rio_port *port, type data,  uint32_t offset,  int dest_id, int hopcount) \
{ \
        uint32_t cmd; \
        int status; \
	uint32_t dioindex = hard_smp_processor_id() & 0x1F; \
	unsigned long flags; \
        if (addr##_ALIGN_ERR(offset)) \
                return -ESRIO_ADDRESS_ALIGN; \
        \
        cmd = SRIO_DIRIO_CMD(dest_id, port->hw_port_id, port->sys_size, 0, 0, 0, hopcount); \
        srio_dbg2(0, "Writing cmd 0x%x in %d \n", cmd, DIRECTIO_CMD + dioindex); \
	spin_lock_irqsave(&rio_block.dirio_lock[dioindex], flags); \
        bcm_write_srio_devreg(port->xlp_srio_dev_base, (DIRECTIO_CMD + dioindex), cmd); \
        __sync(); \
        bcm_srio_dirio_##func(offset, DIRIO_OP_MWRITE, cpu_to_be##size((type )data)); \
        status = bcm_read_srio_devreg(port->xlp_srio_dev_base, DIRECTIO_RESP + dioindex) & DIRIO_RESP_MASK; \
	spin_unlock_irqrestore(&rio_block.dirio_lock[dioindex], flags); \
        if (status != 0) \
                return -ESRIO_TRANS_REQ; \
        return status; \
}

/**@{*/
/**
 * Remote Config Write.
 * @param port SRIO port ID.
 * @param data Address to read data into.
 * @param offset Offset in to local configuration space.
 * @param dest_id Remote device ID
 * @param hopcount Should be 0xff if device is not a switch.
 * @retval 0 Transaction completed successfully.
 * @retval <0 Transaction failed.
 * @ingroup srio
 */
BCM_RIO_REMOTE_CFG_ST(8, uint8_t, BYTE, sb)
BCM_RIO_REMOTE_CFG_ST(16, uint16_t, HWORD, sh)
BCM_RIO_REMOTE_CFG_ST(32, uint32_t, WORD, sw)
BCM_RIO_REMOTE_CFG_ST(64, uint64_t, DWORD, sd)
/**@}*/

/**
 * bcm_local_config_read - Generate a XLP local config space read
 * @mport: RapidIO master port info
 * @index: ID of RapdiIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be read into
 *
 * Generates a XLP local configuration space read. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int bcm_local_config_read(struct rio_mport *mport,
                                int index, u32 offset, int len, u32 *data)
{
	struct bcm_rio_port *port = (struct bcm_rio_port *)mport->priv;
	switch(len) {
		case 1:
			return bcm_rio_local_config_read_8(port, (u8 *)data, offset);
		case 2:
			return bcm_rio_local_config_read_16(port, (u16 *)data, offset);
		default:
			return bcm_rio_local_config_read_32(port, data, offset);
	}
	
	return 0;
}

/**
 * bcm_local_config_write - Generate a XLP local config space write
 * @mport: RapidIO master port info
 * @index: ID of RapdiIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be written
 *
 * Generates a XLP local configuration space write. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int bcm_local_config_write(struct rio_mport *mport,
                                int index, u32 offset, int len, u32 data)
{
	struct bcm_rio_port *port = (struct bcm_rio_port *)mport->priv;

	switch(len) {
		case 1:
			return bcm_rio_local_config_write_8(port, (u8)data, offset);
		case 2:
			return bcm_rio_local_config_write_16(port, (u16)data, offset);
		default:
			return bcm_rio_local_config_write_32(port, data, offset);
	}
	return 0;	
}

/**
 * bcm_rio_config_read - Generate a XLP read maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapdiIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Location to be read into
 *
 * Generates a XLP read maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int
bcm_rio_config_read(struct rio_mport *mport, int index, u16 destid,
                        u8 hopcount, u32 offset, int len, u32 *val)
{
	struct bcm_rio_port *port = (struct bcm_rio_port *)mport->priv;
	
	switch(len) {
		case 1:
			return bcm_rio_config_read_8(port, (u8 *)val, offset, destid, hopcount);
		case 2:
			return bcm_rio_config_read_16(port, (u16 *)val, offset, destid, hopcount);
		default:
			return bcm_rio_config_read_32(port, val, offset, destid, hopcount);
	}
}

/**
 * bcm_rio_config_write - Generate a XLP write maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapdiIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Value to be written
 *
 * Generates an XLP write maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int
bcm_rio_config_write(struct rio_mport *mport, int index, u16 destid,
                        u8 hopcount, u32 offset, int len, u32 val)
{
	struct bcm_rio_port *port = (struct bcm_rio_port *)mport->priv;

	switch(len) {
		case 1:
			return bcm_rio_config_write_8(port, val, offset, destid, hopcount);
		case 2:
			return bcm_rio_config_write_16(port, val, offset, destid, hopcount);
		default:
			return bcm_rio_config_write_32(port, val, offset, destid, hopcount);
	}
	return 0;
}

/**
 * bcm_rio_doorbell_send - Send a XLP doorbell message
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of target device
 * @data: 16-bit info field of RapidIO doorbell message
 *
 * Sends a XLP doorbell message. Returns %0 on success or
 * %-EINVAL on failure.
 */
static int bcm_rio_doorbell_send(struct rio_mport *mport,
                                int index, u16 destid, u16 data)
{
	struct bcm_rio_port *port = (struct bcm_rio_port *)mport->priv;
        uint64_t msg0, msg1;
        uint32_t msg_dest_id;
        int retval;
	int cpu = hard_smp_processor_id();

        srio_dbg2(srio_debug_level, "%s: dest_id %d info %d\n",__func__, destid, data);

        msg_dest_id = (cpu << 2) | port->rxvc;
        msg0 = xlp_dbell_fmbrx_desc0(SRIO_P2D_LAST, SRIO_SEND_RESP, destid, msg_dest_id, data);
	msg0 |= ((uint64_t)port->sys_size << 52ULL);
        msg1 = 0ULL;

        __sync();

        retval = nlh_send_msg2((port->txstn_id + DBELL_PRI),
                        SWCODE_DOORBELL,
                        msg0,
                        msg1);

        if (retval) {
                printk("send failed on cpu %d retval %x \n",cpu, retval);
                return -ESRIO_API_INTERNAL;
        }

        return 0;
}


static int xlp3xx_add_rio_outb_msg(struct bcm_rio_port *port, int mbox, uint64_t paddr, int len, int dest_id, int letter)
{
        uint64_t msg0, msg1;
	uint32_t ssize, msglen, pktsize = 14;
	int cpu = hard_smp_processor_id(), msg_dest_id, retval;

	msg_dest_id = (cpu << 2) | port->rxvc;

	msg0 = XLP_SRIO_FMBRX_DESC0(port->hw_port_id, FTYPE_MESSAGE, dest_id, msg_dest_id, DATAMSG_PRI);

        ssize = SRIO_MAX_SEGMENT_SIZE;
        do {
                if ((ssize > len) && (ssize > SRIO_MIN_SEGMENT_SIZE)) {
                        pktsize--;
                        ssize >>= 1;
                }
                else
                        break;
        }while(1);

        msglen = ((len - 1) / ssize);

        msg0 |= ((uint64_t)port->sys_size << 52ULL) | (msglen << 12) | (pktsize << 8) | (mbox << 4) | (letter << 6);
        msg1 = (((uint64_t)len << 48ULL) | (paddr & 0xffffffffffULL));

        srio_dbg2(srio_debug_level,"%s msg0 0x%llx msg1 %llx \n",__func__, msg0, msg1);
        __sync();

        retval = nlh_send_msg2((port->txstn_id + DATAMSG_PRI),
                        SWCODE_DATAMSG,
                        msg0,
                        msg1);

	return 0;
}



static int xlp_add_rio_outb_msg(struct bcm_rio_port *port, int mbox, uint64_t paddr, int size, int dest_id, int letter)
{
	uint64_t msg0, msg1;
	int cpu = hard_smp_processor_id(), msg_dest_id, retval;

	msg_dest_id = (cpu << 2) | port->rxvc;

        msg0 = xlp_dmesg_fmbrx_desc0(SRIO_P2D_LAST, SRIO_SEND_RESP, dest_id, msg_dest_id, mbox);
	msg0 |= ((uint64_t)port->sys_size << 52ULL) | (letter << 6);
        msg1 = xlp_dmesg_fmbrx_desc1(size, size, paddr);

        srio_dbg2(srio_debug_level,"%s txstn %d size %d msg0 0x%llx msg1 %llx \n",__func__, port->txstn_id, size, msg0, msg1);
        __sync();

        retval = nlh_send_msg2((port->txstn_id + DATAMSG_PRI),
                        SWCODE_DATAMSG,
                        msg0,
                        msg1);

	return 0;
}


/**
 * rio_hw_add_outb_message - Add message to the XLP outbound message queue
 * @mport: Master port with outbound message queue
 * @rdev: Target of outbound message
 * @mbox: Outbound mailbox
 * @buffer: Message to add to outbound queue
 * @len: Length of message
 *
 * Adds the @buffer message to the XLP outbound message queue. Returns
 * %0 on success or %-EINVAL on failure.
 */
int
rio_hw_add_outb_message(struct rio_mport *mport, struct rio_dev *rdev, int mbox,
                        void *buffer, size_t len)
{
	struct bcm_rio_port *port = (struct bcm_rio_port *)mport->priv;
	pdatamsg_transacion_tx_t txq;
	int dest_id = rdev->destid, letter;
	unsigned long flags;
	
	if ((len > rio_block.max_pkt_len) || (mbox >= MAX_MBOX))
		return -EINVAL;

	srio_dbg2(srio_debug_level, "%s: dest_id %d  mbox %d buffer 0x%p len %zd\n",__func__, dest_id, mbox, buffer,len);	
	txq = &port->dmsg_txq[mbox];
	if (txq->status != NLM_QVALID)
		return -EINVAL;

        if (atomic_read(&txq->letter) == 4)
            return -EINVAL;

	spin_lock_irqsave(&txq->dmsg_txlock, flags);
	letter = txq->used & 0x3;
	*(uint64_t *)(txq->paddr + txq->head) = (uint64_t)buffer;
	txq->used++;
	if (++txq->head == txq->max_entries)
		txq->head = 0;
        atomic_inc(&txq->letter);
	spin_unlock_irqrestore(&txq->dmsg_txlock, flags);

	bcm_rio_add_outb_msg(port, mbox, virt_to_phys(buffer), len, dest_id, letter);
	return 0;	
}


/**
 * rio_open_outb_mbox - Initialize XLP outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the outbound mailbox ring
 *
 * Initializes buffer ring, request the outbound message interrupt,
 * and enables the outbound message unit. Returns %0 on success and
 * %-EINVAL or %-ENOMEM on failure.
 */
int rio_open_outb_mbox(struct rio_mport *mport, void *dev_id, int mbox, int entries)
{
	struct bcm_rio_port *port=(struct bcm_rio_port *) mport->priv;
	pdatamsg_transacion_tx_t txq;
	unsigned long flags;

	srio_dbg2(srio_debug_level,"%s: dev %p mbox %d entries %d\n",__func__, dev_id, mbox, entries);
	if (entries < 4) {	//FIXME
                return -EINVAL;
        }

        if (mbox >= MAX_MBOX)
                return -EINVAL;

	txq = &port->dmsg_txq[mbox];
	spin_lock_irqsave(&txq->dmsg_txlock, flags);
	txq->head = txq->tail = txq->used  = 0;
	txq->max_entries = entries;		
	txq->dev_id = dev_id;
        atomic_set(&txq->letter, 0);
	txq->paddr = kmalloc(sizeof(uint64_t) * entries, GFP_KERNEL | GFP_DMA);
	if (txq->paddr == NULL) {
		spin_unlock_irqrestore(&txq->dmsg_txlock, flags);
		return -ENOMEM;
	}
	txq->status = NLM_QVALID;
	spin_unlock_irqrestore(&txq->dmsg_txlock, flags);

	return 0;
}

/**
 * rio_open_inb_mbox - Initialize inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the inbound mailbox ring
 *
 * Initializes buffer ring, request the inbound message interrupt,
 * and enables the inbound message unit. Returns %0 on success
 * and %-EINVAL or %-ENOMEM on failure.
 */
int rio_open_inb_mbox(struct rio_mport *mport, void *dev_id, int mbox,
                      int entries)
{
	struct bcm_rio_port *port=(struct bcm_rio_port *) mport->priv;
	pdatamsg_transacion_rx_t rxq;
	unsigned long flags;

	if (mbox >= MAX_MBOX)
		return -EINVAL;

	srio_dbg2(srio_debug_level,"%s: dev_id %p mbox %d entries %d\n", __func__, dev_id, mbox, entries);
	rxq = &port->dmsg_rxq[mbox];
	spin_lock_irqsave(&rxq->dmsg_rxlock, flags);
	rxq->head = rxq->tail = rxq->used = 0;
	rxq->max_entries = entries;
	rxq->dev_id = dev_id;
	rxq->paddr = kmalloc(sizeof(uint64_t) * entries, GFP_KERNEL | GFP_DMA);
	if (rxq->paddr == NULL) {
		printk("%s kmalloc failed\n",__func__);
		spin_unlock_irqrestore(&rxq->dmsg_rxlock, flags);
		return -ENOMEM;
	}
	rxq->status = NLM_QVALID;
	atomic_set(&port->enabled, 1);
	spin_unlock_irqrestore(&rxq->dmsg_rxlock, flags);
	return 0;
}

/**
 * rio_close_outb_mbox - Shut down outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, free all buffers, and
 * frees the outbound message interrupt.
 */
void rio_close_outb_mbox(struct rio_mport *mport, int mbox)
{
        struct bcm_rio_port *port=(struct bcm_rio_port *) mport->priv;
        pdatamsg_transacion_tx_t txq;
	unsigned long flags;

        srio_dbg2(srio_debug_level,"%s: mbox %d \n",__func__, mbox);
	if (mbox >= MAX_MBOX)
                return;

	txq = &port->dmsg_txq[mbox];
	spin_lock_irqsave(&txq->dmsg_txlock, flags);	
	txq->head = txq->tail = txq->used  = 0;
	txq->max_entries = 0;
	txq->dev_id = NULL;
	txq->status = 0;
	if (txq->paddr != NULL) {
		kfree(txq->paddr);
		txq->paddr = NULL;
	}
	spin_unlock_irqrestore(&txq->dmsg_txlock, flags);	
}

/**
 * rio_hw_get_inb_message - Fetch inbound message from the message unit
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 *
 * Gets the next available inbound message from the inbound message queue.
 * A pointer to the message is returned on success or NULL on failure.
 */
void *rio_hw_get_inb_message(struct rio_mport *mport, int mbox)
{
	struct bcm_rio_port *port=(struct bcm_rio_port *) mport->priv;
	pdatamsg_transacion_rx_t rxmsgq;
	unsigned long flags;
	void *buf = NULL;

	if (mbox >= MAX_MBOX)
                return NULL;

	rxmsgq = &port->dmsg_rxq[mbox];
	srio_dbg2(srio_debug_level,"%s from mbox %d : %p\n", __func__, mbox, &rxmsgq->dmsg_rxlock);
	spin_lock_irqsave(&rxmsgq->dmsg_rxlock, flags);
	if ((rxmsgq->status == NLM_QVALID) && (rxmsgq->used)) {
		buf =(void *)(*(uint64_t *)(rxmsgq->paddr + rxmsgq->tail));
		rxmsgq->tail = (rxmsgq->tail + 1) % rxmsgq->max_entries;
		rxmsgq->used--;
	}
	spin_unlock_irqrestore(&rxmsgq->dmsg_rxlock, flags);
	return buf;
}

/**
 * rio_hw_add_inb_buffer - Add buffer to the inbound message queue
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 * @buf: Buffer to add to inbound queue
 *
 * Assumes buffer size 4KB
 * Adds the @buf buffer to the inbound message queue. Returns
 * %0 on success or %-EINVAL on failure.
 */
int rio_hw_add_inb_buffer(struct rio_mport *mport, int mbox, void *buf)
{
	//struct bcm_rio_port *port=(struct bcm_rio_port *) mport->priv;
	int retval, cpu = hard_smp_processor_id();

	retval = bcm_rio_send_free_rx_buff((uint64_t)virt_to_phys(buf));

	if (retval) {
		printk("RIO Send rx buf failed %d\n", retval);
		return -EINVAL;
	}
	rio_free_rx_bufs[cpu]++;
	return 0;
}

static int xlp_rio_pop_msg(struct bcm_rio_port *port)
{
	unsigned long base = port->xlp_srio_dev_base;
	volatile uint32_t val;
	int cpu = 0, i;
	void *buffer;

	val = bcm_read_srio_devreg(base, SRIOIPCTRL);
	val |= (PORT0_LOOPBACK << port->hw_port_id);
	bcm_write_srio_devreg(base, SRIOIPCTRL, val);

	buffer = kmalloc(256, GFP_KERNEL|GFP_DMA);
	if (buffer == NULL) {
		printk("kmalloc failed. address Q is not flushed \n");
		return -1;
	}
	for(cpu = 0; cpu < NR_CPUS; cpu++) {
		if(!rio_free_rx_bufs[cpu])
			continue;
		for(i=0; i < rio_free_rx_bufs[cpu]; i++)
			bcm_rio_add_outb_msg(port, 0, virt_to_phys(buffer), 256, 0, 0);
		rio_free_rx_bufs[cpu] = 0;
	}	
	kfree(buffer);

	for(i=0;i<10000000;i++);
        val = bcm_read_srio_devreg(port->xlp_srio_dev_base, SRIOIPCTRL);
        val &= ~(PORT0_LOOPBACK << port->hw_port_id);
        bcm_write_srio_devreg(port->xlp_srio_dev_base, SRIOIPCTRL, val);
	for(i=0;i<100000;i++);
	return 0;
}


/**
 * rio_close_inb_mbox - Shut down MPC85xx inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the inbound message unit, free all buffers, and
 * frees the inbound message interrupt.
 */
void rio_close_inb_mbox(struct rio_mport *mport, int mbox)
{
	struct bcm_rio_port *port=(struct bcm_rio_port *) mport->priv;
	pdatamsg_transacion_rx_t rxq;
	unsigned long flags;

	srio_dbg2(srio_debug_level,"%s: mbox %d \n",__func__, mbox);
        if (mbox >= MAX_MBOX)
                return;

	rxq = &port->dmsg_rxq[mbox];
	spin_lock_irqsave(&rxq->dmsg_rxlock, flags);
	if ((rxq->status == NLM_QVALID) && (rxq->used)) {
		printk("Close mbox called when the Q is not empty \n");
	}
	rxq->head = rxq->tail = rxq->used  = 0;
	rxq->max_entries = 0;
	rxq->dev_id = NULL;
	rxq->status = 0;
        if (rxq->paddr != NULL) {
                kfree(rxq->paddr);
                rxq->paddr = NULL;
        }
	atomic_set(&port->enabled, 0);
	spin_unlock_irqrestore(&rxq->dmsg_rxlock, flags);

	xlp_rio_pop_msg(port);
}


static int nlm_xlp3xx_ax_rio_handle_rxreq(struct bcm_rio_port *rio, uint32_t ftype, int src,
                uint64_t msg0,uint64_t msg1,uint64_t msg2)
{
        int cpu = hard_smp_processor_id();
	unsigned long flags;
	uint64_t paddr = 0;
        int len, mbox, found = 0;
        datamsg_transacion_rx_t *rxmsgq;
        uint16_t dbell_info;
	struct rio_mport *mport = rio->mport;
	struct rio_dbell *dbell;

        switch(ftype) {
                case FTYPE_DOORBELL:
                        dbell_info = SRIO_DBELL_INFO(msg1);
			srio_dbg(srio_debug_level, "Cpu%d Rcvd FTYPE_DOORBELL\n", cpu);
	                list_for_each_entry(dbell, &mport->dbells, node) {
	                        if ((dbell->res->start <= dbell_info) &&
        	                    (dbell->res->end >= dbell_info)) {
                	                found = 1;
                        	        break;
                        	}
                	}
	                if (found) {
        	                dbell->dinb(mport, dbell->dev_id, src, -1, dbell_info);
                	}
#if 0 
			else {
                        	printk("RIO: spurious doorbell, sid %2.2x info %4.4x\n",
	                               src, dbell_info);
        	        }
#endif
                        break;
                case FTYPE_MESSAGE:
                        mbox = SRIO_MBOX(msg0);
                        srio_dbg(srio_debug_level, "Cpu%d Rcvd FTYPE_MESSAGE  mbox %d \n", cpu, mbox);
                        len = SRIO_PAYLOAD_LENGTH_XLP3XX_AX(msg1);
                        paddr = SRIO_PAYLOAD_ADDR(msg2);
			
			rxmsgq = &rio->dmsg_rxq[mbox];
			spin_lock_irqsave(&rxmsgq->dmsg_rxlock, flags);
			if (atomic_read(&rio->enabled) == 0) {
				spin_unlock_irqrestore(&rxmsgq->dmsg_rxlock, flags);
				return -ESRIO_API_INTERNAL;
			}
			if ((rxmsgq->status != NLM_QVALID) || 
			    (rxmsgq->used == rxmsgq->max_entries)) {				
				//FIXME send the buffer back to address Q. should not come here.
				spin_unlock_irqrestore(&rxmsgq->dmsg_rxlock, flags);
				if (bcm_rio_send_free_rx_buff(paddr))
                                        return -ESRIO_API_INTERNAL;
				break;
			}
			rio_free_rx_bufs[cpu]--;
			*(uint64_t *)(rxmsgq->paddr + rxmsgq->head) = (uint64_t)phys_to_virt(paddr);
			rxmsgq->head = (rxmsgq->head + 1) % rxmsgq->max_entries;
			rxmsgq->used++;
			spin_unlock_irqrestore(&rxmsgq->dmsg_rxlock, flags);
			if (mport->inb_msg[mbox].mcback)
                	        mport->inb_msg[mbox].mcback(mport, rxmsgq->dev_id, mbox, -1);
	                else
				printk("No rx call back for port %d mbox %d \n",rio->hw_port_id, mbox);
                        break;
                default:
                        srio_dbg(srio_debug_level, "%s: Rcvd req with ftype %d not handled, desc0 0x%llx \n", __func__, ftype, msg0);
                        break;
        }
        return 0;
}

void bcm_xlp3xx_ax_rio_msghandler(uint32_t vc, uint32_t src_id,
                        uint32_t size, uint32_t swcode, uint64_t msg0,uint64_t msg1,
                        uint64_t msg2, uint64_t msg3, void* data)
{
        struct bcm_rio_port *rio;
	struct rio_mport *mport;
        pdatamsg_transacion_tx_t dmsg_txq;
        uint32_t port_id, src, ftype;
#ifdef SRIO_DEBUG
        int cpu = hard_smp_processor_id();
#endif
        uint32_t status;
        uint32_t mbox, slot;
	unsigned long flags;

        if ((size == 0) || (src_id <  XLP_3XX_SRIO_VC_BASE) || (src_id > XLP_3XX_SRIO_VC_LIMIT))
                return;
        
	srio_dbg(srio_debug_level, "rcvd message cpu %d  vc %d size %d \n",cpu, vc, size);

        port_id = SRIO_PORT(msg0);
        srio_dbg(srio_debug_level, "%s: port %d swcode %d msg0 0x%llx msg1 0x%llx msg2 0x%llx \n", __func__, port_id, swcode, msg0, msg1, msg2);

        ftype = SRIO_FTYPE(msg0);
        src = SRIO_SRC_ID(msg0);
        rio = rio_port[port_id];
	if (rio == NULL) {
                printk("%s: Rcvd msg from uninitialized port %d, desc0 0x%llx \n", __func__,port_id, msg0);
                return;
        }
	mport = rio->mport;
	if (mport == NULL) {
		printk("mport not registered yet for srio physical port %d\n", port_id);
		return;
	}

        switch(swcode) {
               case SWCODE_RX_REQ:
                        nlm_xlp3xx_ax_rio_handle_rxreq(rio, ftype, src, msg0, msg1, msg2);
                        break;
               case SWCODE_DATAMSG:
	
			if (atomic_read(&rio->enabled) == 0) {
		                return;
		        }
		
                        if ((IS_LINK_RESP_TOUT(msg0)==0) && (ftype != FTYPE_RESPONSE)) {
                                srio_dbg(srio_debug_level, "%s: Invalid Ftype in Datamsg resp, desc0 0x%llx \n",__func__, msg0);
                                return;
                        }
                        mbox = SRIO_MBOX(msg0);
                        status = SRIO_RESP_STATUS(msg0);
			dmsg_txq = &rio->dmsg_txq[mbox];
                        atomic_dec(&dmsg_txq->letter); 
			srio_dbg(srio_debug_level,"FTYPE_RESPONSE mbox %d status 0x%x\n", mbox, status);
			spin_lock_irqsave(&dmsg_txq->dmsg_txlock, flags);
			if (dmsg_txq->used == 0) {
				//error
				spin_unlock_irqrestore(&dmsg_txq->dmsg_txlock, flags);
				break;
			}
			slot = dmsg_txq->tail;	
			dmsg_txq->tail = (dmsg_txq->tail + 1) % dmsg_txq->max_entries;
			dmsg_txq->used--;
			spin_unlock_irqrestore(&dmsg_txq->dmsg_txlock, flags);
			mport->outb_msg[mbox].mcback(mport, dmsg_txq->dev_id, -1,
                                                slot);
                        break;
		case SWCODE_DOORBELL:
                        if (ftype != FTYPE_RESPONSE) {
                                srio_dbg(srio_debug_level, "%s: Invalid Ftype in resp msg, desc0 0x%llx swcode %d \n",__func__, msg0, swcode);
                                return;
                        }
                        break;
		default:
			break;
	}
	return;	
}


void dump_sriodev_cfg(uint64_t base)
{
        printk("complex configuration 0x%x \n", bcm_read_srio_devreg(base, SRICOMCONFIG));
        printk("control 0x%x \n", bcm_read_srio_devreg(base, SRIOIPCTRL));
        printk("IP config 0x%x \n", bcm_read_srio_devreg(base, SRIOIPCONFIG));
        printk("IP status 0x%x \n", bcm_read_srio_devreg(base, SRIOIPSTATUS));
        printk("Device ID 0x%x \n", bcm_read_srio_devreg(base, SRIODEVICEID));
}

int bcm_srio_configure_phy(uint64_t cfgbase, bcm_srio_cfg *rio_cfg)
{
	switch(rio_cfg->mode) {
		case SRIO_MODE_x1:
		        // ipconfig FIXME port baud rate
        		bcm_write_srio_devreg(cfgbase, SRIOIPCONFIG, (PATH_MODE_SEL_VAL_0 << PATH_MODE_SEL_POS) |
                	        (1<<PATH_BAUD_RATE_POS) | (rio_cfg->baud_rate << PORT0_BAUD_RATE_POS) |
                        	(rio_cfg->baud_rate << PORT1_BAUD_RATE_POS) | (rio_cfg->baud_rate << PORT2_BAUD_RATE_POS) 				  | (rio_cfg->baud_rate << PORT3_BAUD_RATE_POS));
			break;
		case SRIO_MODE_x4:
			bcm_write_srio_devreg(cfgbase, SRIOIPCONFIG, (PATH_MODE_SEL_VAL_4 << PATH_MODE_SEL_POS) |
                                (1<<PATH_BAUD_RATE_POS) | (rio_cfg->baud_rate << PORT0_BAUD_RATE_POS) |
                                (rio_cfg->baud_rate << PORT1_BAUD_RATE_POS) | (rio_cfg->baud_rate << PORT2_BAUD_RATE_POS) 				  |(rio_cfg->baud_rate << PORT3_BAUD_RATE_POS));
			break;
	}
        udelay(256);
	return 0;
}

void bcm_reset_srio(void)
{
	uint64_t cfgbase = rio_block.xlp_srio_dev_base, membase = rio_block.xlp_srio_base;
	volatile uint32_t val;

        // Bring the PHY and register interface of PHY out of reset first
        val = bcm_read_srio_devreg(cfgbase, SRICOMCONFIG);
        val &= 0xCFFFFFFF;
        bcm_write_srio_devreg(cfgbase, SRICOMCONFIG, val);

        udelay(256);

        // Bring the controller out of reset
        val = bcm_read_srio_devreg(cfgbase, SRICOMCONFIG);
        val &= 0xC7FFFFFF;
        bcm_write_srio_devreg(cfgbase, SRICOMCONFIG, val);

        bcm_write_srio_reg32(membase, RIO_PORTGEN_CSR, 0xC0000000); // Master enable
        udelay(256);

        __sync();

}

static void print_rio_lane_speed(uint64_t cfgbase)
{
        uint32_t val, m_val, n_val, lane;

        for(lane = 0 ; lane < 4; lane++) {
                bcm_write_srio_devreg(cfgbase, PHYRDWRCTRL, (4 << PHY_REGADDR)| (1<<lane));
                val = bcm_read_srio_devreg(cfgbase, PHYRDWRACCS) & 0xFF;
                printk("Lane%d F val %d\n",lane, (val & 0xF) + 1);
                bcm_write_srio_devreg(cfgbase, PHYRDWRCTRL, (5 << PHY_REGADDR)| (1<<lane));
                val = bcm_read_srio_devreg(cfgbase, PHYRDWRACCS) & 0xFF;
                m_val = ((val >> 5) & 0x3) + 1;
                n_val = (val & 0x1F) + 1;
                printk("Lane%d N val %d M val %d \n",lane, n_val, m_val);
        }
}

static int bcm_poll_sriolane_status(void)
{
	volatile uint32_t val;
	uint64_t membase = rio_block.xlp_srio_base;
	int retry_count = 0, port;

        print_rio_lane_speed(rio_block.xlp_srio_dev_base);

        printk("Check port 0 OK 0x%x \n",bcm_read_srio_reg32(membase, 0x158));
        while(!((val = bcm_read_srio_reg32(membase, 0x158)) & 0x2)) {
		if (++retry_count == 10000000) {
			printk("Port 0 link is down\n");
			break;
		}	
	}

        if (rio_config.mode == SRIO_MODE_x1) {
		for(port=1; port < 4; port++) {
			retry_count = 0;
	                printk("Check port %d OK 0x%x \n", port, bcm_read_srio_reg32(membase, (0x158 + (0x20 * port))));
        	        while(!((val = bcm_read_srio_reg32(membase, (0x158 + (0x20 * port)))) & 0x2)) {
				if (++retry_count == 1000) {
					printk("Port %d link is down\n", port);
					break;
				}
			}
		}
        }
	return 0;
}

static int update_srio_portmask(uint32_t *port_mask)
{
	int i;

	*port_mask = 0;
	for(i=0; i<SRIO_MAX_PORTS; i++) {
		if ((bcm_read_srio_reg32(rio_block.xlp_srio_base, (0x158 + (i * 0x20)))) & 0x2)
			*port_mask |= (1<<i);
	}
	return 0;
}


int bcm_rio_setup(struct bcm_rio_port *rio_port, int index, struct rio_mport *mport) 
{
	int mbox = 0;

	rio_port->mport = mport;
	rio_port->xlp_srio_base = rio_block.xlp_srio_base;
	rio_port->xlp_srio_dev_base = rio_block.xlp_srio_dev_base;

	rio_port->node = 0;
        rio_port->msgdst_id = rio_config.msgdstid;
        rio_port->rxvc = rio_config.rxvc;
        rio_port->hw_port_id = index;
        rio_port->txstn_id = XLP_SRIO_TXVC_BASE + (rio_port->hw_port_id * 2);
        atomic_set(&rio_port->enabled, 0);
        rio_port->ref_cnt = 1;
	rio_port->device_id = rio_config.host_devid[index];
	rio_port->sys_size = rio_config.sys_size;

	for(mbox = 0; mbox < MAX_MBOX; mbox++) {
		spin_lock_init(&rio_port->dmsg_txq[mbox].dmsg_txlock);
		spin_lock_init(&rio_port->dmsg_rxq[mbox].dmsg_rxlock);
	}
	printk("RIO port %d msgdst_id %d rxvc %d hw_port_id %d txstn_id %d hostdevice_id %d\n", index, rio_port->msgdst_id, rio_port->rxvc,
			rio_port->hw_port_id, rio_port->txstn_id, rio_port->device_id);
	return 0;
}



/**
 * Initialize SRIO glue logic.
 * @return Number of SRIO ports.
 * @retval <0 Failure.
 * @ingroup srio
 */

int bcm_init_rio(void)
{
	uint64_t cfgbase, membase;
	int i = 0, regindex, cpu;
	volatile uint32_t val = 0;
	
	rio_block.xlp_srio_dev_base = NLM_SRIO_CFG_BASE;
	cfgbase = rio_block.xlp_srio_dev_base;

#ifdef HW_EMULATOR
        bcm_write_srio_devreg(cfgbase, 4, 0xd0060000);
       	udelay(4);
        bcm_write_srio_devreg(cfgbase, 5, 0x00000001);
       	udelay(4);
#endif

        if (is_nlm_xlp3xx_ax()) {
                rio_block.max_dbell_off = MAX_DBELL_OFFSET_XLP3XX_AX;
		rio_block.max_dbell = MAX_DOORBELL_XLP3XX_AX;
        }
        else {
                rio_block.max_dbell_off = MAX_DBELL_OFFSET;
		rio_block.max_dbell = MAX_DOORBELL;
        }

	rio_block.max_pkt_len = MAX_PACKET_LEN;
        nlm_hal_write_32bit_reg(0x180002bc, 0 , 0x80000000);	//FIXME
        nlm_hal_write_32bit_reg(0x180002c0, 0, 0xfffff000);

	rio_block.xlp_srio_base = (bcm_read_srio_devreg(cfgbase, PCI_MEM_BAR_0) & (~0xfULL));
	membase = rio_block.xlp_srio_base;

	bcm_srio_configure_phy(cfgbase, &rio_config);

        if (is_nlm_xlp3xx_ax()) {
                val = bcm_read_srio_devreg(cfgbase, SRICOMCONFIG);
                val |= (1<<10) | (1<<9) | (1<<3);
                bcm_write_srio_devreg(cfgbase, SRICOMCONFIG, val);
                srio_dbg(srio_debug_level,"SRICOMCONFIG %0x\n",bcm_read_srio_devreg(cfgbase, SRICOMCONFIG));
        }

	bcm_reset_srio();

        bcm_write_srio_reg32(membase, RIO_PORT0CTRL_CSR, 0x40600001);
        bcm_write_srio_reg32(membase, RIO_PORT0CTRL2_CSR, (10<<28) | (1<<27) | (3<<16) );

	bcm_write_srio_reg32(membase, RIO_SRC_OP, (1<<10) | (1<<11) | (0xf << 12) ); //| (0xf<<16));
        udelay(256);

        srio_dbg(srio_debug_level,"RIO_PORT0CTRL_CSR %x\n",bcm_read_srio_reg32(membase, RIO_PORT0CTRL_CSR));

        if (rio_config.mode == SRIO_MODE_x4) {
                bcm_write_srio_reg32(membase, RIO_PORT1CTRL_CSR, 0x40600001);
                udelay(256);

                srio_dbg(srio_debug_level,"RIO_PORT1CTRL_CSR %x\n",bcm_read_srio_reg32(membase, RIO_PORT1CTRL_CSR));

                bcm_write_srio_reg32(membase, RIO_PORT2CTRL_CSR, 0x40600001);
                udelay(256);

                srio_dbg(srio_debug_level,"RIO_PORT2CTRL_CSR %x\n",bcm_read_srio_reg32(membase, RIO_PORT2CTRL_CSR));

                bcm_write_srio_reg32(membase, RIO_PORT3CTRL_CSR, 0x40600001);
                udelay(256);

                srio_dbg(srio_debug_level,"RIO_PORT3CTRL_CSR %x\n",bcm_read_srio_reg32(membase, RIO_PORT3CTRL_CSR));
        }

        if (is_nlm_xlp3xx_ax()) {
                bcm_write_srio_devreg(cfgbase, RDEXC_BASE, 0xFFFFFFFF);
                bcm_write_srio_devreg(cfgbase, RDEXC_LIMIT, 0);
                srio_dbg(srio_debug_level,"RdExcl Base 0x%x \n", bcm_read_srio_devreg(cfgbase, RDEXC_BASE));

                regindex = DOORBELL_DESTID_XLP3XX_AX;
        }
        else {
                bcm_write_srio_devreg(cfgbase, SRIO_DATAMSG_SIZE, 0xE);  // 0xE - 256 Bytes is maximum segment size
                regindex = DOORBELL_DESTID;
                if (!is_nlm_xlp3xx_b0()) {
                        bcm_write_srio_reg32(membase, RIO_RESPTOUT_CSR, (100000 | (0x20 <<24)));
			for(i = 0 ; i<SRIO_MAX_PORTS; i++) {
                        	bcm_write_srio_devreg(cfgbase, FLUSHADDRQ + i, (0x3FFF << 2)); //lower 2 bits = 0
			}
                }
        }

        bcm_write_srio_devreg(cfgbase, SRIOIPCTRL, (4<<21 )| (1 <<20) | (0xF << 16) | (0xF << 12) |  (0xF << 4));
        udelay(256);

        for(i=0; i<rio_block.max_dbell; i++) {
                bcm_write_srio_devreg(cfgbase, regindex+i, rio_config.msgdstid);
        }

	for (cpu = 0, i = 0; i<MAX_MSGDESTINATION ; i++, cpu++) {
		if (cpu == 16)
			cpu = 0;
		if(!cpu_isset(cpu, phys_cpu_present_map)) {
	                bcm_write_srio_devreg(cfgbase, DATAMSG_DESTID+i, rio_config.msgdstid );
		}
		else{
			bcm_write_srio_devreg(cfgbase, DATAMSG_DESTID+i, ((cpu*4)+rio_config.msgdstid) );
		}
        }

        for(i=0; i<MAX_SWITCH_ENTRY; i++) {
                bcm_write_srio_devreg(cfgbase, SWITCH_ENTRY+i, 0);
        }

        bcm_write_srio_devreg(cfgbase, SWITCHDEFAULT, 0);

        for(i=0; i<NLM_MAX_SRIO_PORTS; i++) {
                bcm_write_srio_devreg(cfgbase, MSG_DESTID + i, rio_config.msgdstid);
        }

        bcm_rio_config_dmsg_lookup(16, 0xFF000000, 0, 0, 00);

	bcm_poll_sriolane_status();
	return 0;
}

#if 0
static irqreturn_t xlp_srio_irq_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}
#endif

extern void *fdt;

int bcm_rio_module_init(void)
{	
	uint32_t max_ports, ports_available, val;
	int i=0, retval = 0;
	struct rio_mport *mport;
	struct rio_ops *ops;

	if (is_nlm_xlp3xx() == 0) {
		printk("RapidIO not supported\n");
		return -1;
	}
#ifdef HW_EMULATOR
	enable_ELPA();
#endif

	// parse fdt
	max_ports = fdt_parse_srio_params(fdt, 0, &rio_config);
	// configure srio 
	bcm_init_rio();

	update_srio_portmask(&ports_available);
	printk("Available SRIO ports 0x%x\n",ports_available);

        ops = kmalloc(sizeof(struct rio_ops), GFP_KERNEL);
        if (!ops) { 
                return -ENOMEM;
        }
        ops->lcread = bcm_local_config_read;
        ops->lcwrite = bcm_local_config_write;
        ops->cread = bcm_rio_config_read;
        ops->cwrite = bcm_rio_config_write;
	if (is_nlm_xlp3xx_ax()) {
	        ops->dsend = bcm_rio_xlp3xxax_doorbell_send;
		bcm_rio_add_outb_msg = xlp3xx_add_rio_outb_msg;
	}
	else {
		ops->dsend = bcm_rio_doorbell_send;
		bcm_rio_add_outb_msg = xlp_add_rio_outb_msg;
	}

	// 	
	for(i=0; i < max_ports; i++) {
		if ((ports_available & (1<<i)) == 0) {
			rio_port[i] = NULL;
			continue;
		}
	
		rio_port[i] = kmalloc(sizeof(struct bcm_rio_port), GFP_KERNEL | GFP_DMA);
		if (rio_port[i] == NULL) {
			retval = -ENOMEM;
			goto errout;
		}
			
		rio_port[i]->mport = NULL;
		mport = kmalloc(sizeof(struct rio_mport), GFP_KERNEL | GFP_DMA);
		if (mport == NULL) {
			retval = -ENOMEM;
			goto errout;
		}			
		
		bcm_rio_setup(rio_port[i], i, mport);

		mport->priv = rio_port[i];
		mport->id = 0;
		mport->index = 0;

		INIT_LIST_HEAD(&mport->dbells);
		mport->phy_type = RIO_PHY_SERIAL;
                mport->sys_size = rio_port[i]->sys_size;   
                mport->host_deviceid = rio_port[i]->device_id;

		mport->iores.start = rio_block.xlp_srio_dev_base;
	        mport->iores.end = rio_block.xlp_srio_dev_base + SRIO_CFGMEM_SIZE - 1;
        	mport->iores.flags = IORESOURCE_MEM;
        	mport->iores.name = "rio_io_win";
		
	        rio_init_dbell_res(&mport->riores[RIO_DOORBELL_RESOURCE], 0, 0xffff);
	        rio_init_mbox_res(&mport->riores[RIO_INB_MBOX_RESOURCE], 0, 0);
        	rio_init_mbox_res(&mport->riores[RIO_OUTB_MBOX_RESOURCE], 0, 0);
		sprintf(mport->name, "RIO%d mport",i);

		mport->ops = ops;	
		if (rio_port[i]->device_id < 0) {
			bcm_rio_local_config_write_32(rio_port[i], 0, 0x60);
			udelay(256);
		}
		else {
			bcm_rio_local_config_write_32(rio_port[i], rio_port[i]->device_id, 0x60);
		}

		rio_register_mport(mport);
		atomic_set(&rio_port[i]->enabled, 1);

		if ((is_nlm_xlp3xx_ax()) && (rio_port[i]->device_id > 0)) {
			bcm_rio_local_config_write_32(rio_port[i], rio_port[i]->device_id, 0x68);
			bcm_rio_local_config_read_32(rio_port[i], &val, 0x68 );
		}
	}

#if 0
	retval = request_irq(XLP_SRIO_IRQ(0,0), xlp_srio_irq_handler,
                IRQF_SHARED, "srio_irq", &rio_block);
        if (retval) {
                printk("RIO request irq failed %d \n", retval);
        }
#endif	
	if (is_nlm_xlp3xx_ax()) {
		if(register_xlp_msgring_handler(XLP_MSG_HANDLE_SRIO, bcm_xlp3xx_ax_rio_msghandler, NULL)) 
			printk("register_xlp_msgring_handler failed for SRIO\n");
	}
	else {
		if(register_xlp_msgring_handler(XLP_MSG_HANDLE_SRIO, bcm_xlp_rio_msghandler, NULL)) {
                        printk("register_xlp_msgring_handler failed for SRIO\n");
                }
	}
	return retval;

errout:	
	while(i > 0) {
		if (rio_port[i] != NULL) {
			if (rio_port[i]->mport != NULL)
				kfree(rio_port[i]->mport);
			kfree(rio_port[i]);
		}
		i--;
	}
	return retval;	
}
