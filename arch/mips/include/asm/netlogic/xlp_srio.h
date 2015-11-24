/*-
 * Copyright (c) 2003-2012 Broadcom Corporation
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


/**
 * @brief This is a substructure of nlm_rio_port_resource.
 * @ingroup srio
 */
typedef struct {
	spinlock_t dmsg_txlock;
	uint64_t *paddr;	
	void *dev_id;
        int status;
        int max_entries;
	int used;
        int head;
        int tail;
        atomic_t letter;
}datamsg_transacion_tx_t, *pdatamsg_transacion_tx_t;

/**
 * @brief This is a substructure of nlm_rio_port_resource.
 * @ingroup srio
 */
typedef struct {
	spinlock_t dmsg_rxlock;
	uint64_t *paddr;	
	void *dev_id;
	int status;	
        int max_entries;        
	int used;
        int head;	
        int tail;
}datamsg_transacion_rx_t, *pdatamsg_transacion_rx_t;


struct bcm_rio_port {
	datamsg_transacion_tx_t dmsg_txq[MAX_MBOX];
	datamsg_transacion_rx_t dmsg_rxq[MAX_MBOX];
	
	struct rio_mport *mport;
	uint64_t xlp_srio_base;
	uint64_t xlp_srio_dev_base;

        // port
        atomic_t enabled;
        int device_id;
        uint32_t hw_port_id;
	uint32_t node;

        // FMN
        uint32_t msgdst_id;
        uint32_t rxvc;
 	uint32_t txstn_id;
	uint32_t sys_size;

	// SW
        uint32_t flags; // loopback, port write
        uint32_t ref_cnt;
};

typedef struct bcm_rio_port bcm_rio_port_t, *pbcm_rio_port_t;

struct bcm_rio_block {
	uint64_t xlp_srio_base;
	uint64_t xlp_srio_dev_base;

        int node;	
        int num_ports;
        int max_rx_bufs;
        int rxbuf_threshold;
        int dbell_offset;
	int max_dbell;
	int max_dbell_off;
	int max_pkt_len;
	int max_seg_size;

        spinlock_t dirio_lock[32];
        spinlock_t dbell_lock;
        spinlock_t dmesg_lock;
        spinlock_t rio_lock;
        atomic_t rx_bufs_used[NR_CPUS];

#ifdef INCLUDE_DATASTREAMING
        int max_dstrm_bufs;
        int dstrmbuf_threshold;
	spinlock_t dstrm_lock;
        atomic_t dstrm_bufs_used[NR_CPUS];
#endif
};

static __inline__ uint8_t bcm_read_srio_reg8(uint64_t srio_base, uint32_t offset)
{       uint32_t val;
        val = lw_40bit_phys_uncached(srio_base + (offset & ~0x3));
        switch(offset & 3) {
                case 0:
                        return  (uint8_t) (val >> 24);
                case 1:
                        return  (uint8_t) (val >> 16);
                case 2:
                        return  (uint8_t) (val >> 8);
                case 3:
                        return  (uint8_t) val;
        }
        return 0;
}

static __inline__ void bcm_write_srio_reg8(uint64_t srio_base, uint32_t offset, uint8_t data)
{       uint32_t val;
        val = lw_40bit_phys_uncached(srio_base + (offset & ~0x3));
        switch(offset & 3) {
                case 0:
                        val = (val & 0x00ffffff) | (data<<24);
                        break;
                case 1:
                        val = (val & 0xff00ffff) | (data<<16);
                        break;
                case 2:
                        val = (val & 0xffff00ff) | (data<<8);
                        break;
                case 3:
                        val = (val & 0xffffff00) | data;
                        break;
        }
        sw_40bit_phys_uncached(srio_base + (offset & ~0x3), val);
}



static __inline__ uint16_t bcm_read_srio_reg16(uint64_t srio_base, uint32_t offset)
{       uint32_t val;
        val = lw_40bit_phys_uncached(srio_base + (offset & ~0x3));
        switch(offset & 3) {
                case 0:
                        return  (uint16_t) (val >> 16);
                case 2:
                        return  (uint16_t) val;
        }
        return 0;
}

static __inline__ void bcm_write_srio_reg16(uint64_t srio_base, uint32_t offset, uint16_t data)
{       uint32_t val;
        val = lw_40bit_phys_uncached(srio_base + (offset & ~0x3));
        switch(offset & 3) {
                case 0:
                        val = (val & 0x0000ffff) | (data<<16);
                        break;
                case 2:
                        val = (val & 0xffff0000) | data;
                        break;
        }
        sw_40bit_phys_uncached(srio_base + (offset & ~0x3), val);
}

static __inline__ uint64_t bcm_read_srio_reg64(uint64_t srio_base, uint32_t offset)
{       uint64_t val;
        val = lw_40bit_phys_uncached(srio_base + offset);
        val = (val << 32) | lw_40bit_phys_uncached(srio_base + offset + 4);
        return val;
}

static __inline__ void bcm_write_srio_reg64(uint64_t srio_base, uint32_t offset, uint64_t data)
{
        sw_40bit_phys_uncached(srio_base + offset , (uint32_t) (data >> 32));
        sw_40bit_phys_uncached(srio_base + offset + 4, (uint32_t) data);
}

#define bcm_read_srio_reg32(srio_base, offset)     lw_40bit_phys_uncached((uint64_t)srio_base + offset)
#define bcm_read_srio_reg(srio_base, index)        bcm_read_srio_reg32(srio_base, (index<<2))
#define bcm_write_srio_reg32(srio_base, offset, val)       sw_40bit_phys_uncached((uint64_t)srio_base + offset, val)
#define bcm_write_srio_reg(srio_base, index, val)  bcm_write_srio_reg32(srio_base, (index<<2), val)


#define bcm_read_srio_devreg(srio_cfg_base, index) 		nlm_hal_read_32bit_reg((uint64_t)srio_cfg_base, index)
#define bcm_write_srio_devreg(srio_cfg_base, index, val)	nlm_hal_write_32bit_reg((uint64_t)srio_cfg_base, index, val)


