/*-
 * Copyright (c) 2003-2015 Broadcom Corporation
 * All Rights Reserved
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * http://www.gnu.org/licenses/gpl-2.0.txt
 * or the Broadcom license below:

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
 * #BRCM_4# */

#ifndef __NLM_HAL_NAE_H__
#define __NLM_HAL_NAE_H__
#ifdef NLM_LINUX_KERNEL
#include <linux/netdevice.h>
#endif
#include "nlm_hal.h"

#define NUM_DIST_VEC 				16
#define NUM_WORDS_PER_DV 			16
#define MAX_DV_TBL_ENTRIES			(NUM_DIST_VEC * NUM_WORDS_PER_DV)

#define XLP_3XX_MAX_PORTS			8
#define XLP_2XX_MAX_PORTS			8
#define XLP_MAX_PORTS				18

#define NAE_RECV_NONE				0x00000000
#define NAE_RECV_RX					0x00000001
#define NAE_RECV_TXC			 	0x00000002
#define NAE_RECV_UNKNOWN		 	0x80000000
#define NULL_VFBID				 	127
#define MAX_NAE_CONTEXTS		 	524
#define XLP8XX_MAX_NAE_COMPLEX	 	5
#define MAX_CAL_SLOTS		 		64
#define MAX_VFBID_ENTRIES			128

#define XLP_MAX_FLOWS				(64 << 10)

#define SGMII_CAL_SLOTS				3
#define XAUI_CAL_SLOTS				13
#define ILK_CAL_SLOTS				26

#define MAX_PORTS_PERBLOCK			4
#define XLP_MAX_INTERLAKEN_IF		2

#define XLP3XX_MAX_NAE_COMPLEX		2
#define XLP3XX_MAX_NAE_CONTEXTS		64
#define MAX_POE_CLASSES		 		8
#define MAX_POE_CLASS_CTXT_TBL_SZ	((MAX_NAE_CONTEXTS / MAX_POE_CLASSES) + 1)
/* Fixed wrong math for 3xx/2xx which has only 64 contexts */
#define XLP3XX_MAX_POE_CLASS_CTXT_TBL_SZ	((XLP3XX_MAX_NAE_CONTEXTS / MAX_POE_CLASSES))
#define XLP3XX_SGMII_PARSERSEQ_FIFO_MAX		30
/*################################*/
#define XLP3XX_STG2_FIFO_SZ			512
#define XLP3XX_EH_FIFO_SZ			512
#define XLP3XX_FROUT_FIFO_SZ		512
#define XLP3XX_MS_FIFO_SZ			512
#define XLP3XX_PKT_FIFO_SZ			8192
#define XLP3XX_PKTLEN_FIFO_SZ		512

#define XLP3XX_MAX_STG2_OFFSET		0x7F
#define XLP3XX_MAX_EH_OFFSET		0x1f
#define XLP3XX_MAX_FREE_OUT_OFFSET	0x1f
#define XLP3XX_MAX_MS_OFFSET		0xF
#define XLP3XX_MAX_PMEM_OFFSET		0x7FE


#define XLP3XX_STG1_2_CREDIT		XLP3XX_STG2_FIFO_SZ
#define XLP3XX_STG2_EH_CREDIT		XLP3XX_EH_FIFO_SZ
#define XLP3XX_STG2_FROUT_CREDIT	XLP3XX_FROUT_FIFO_SZ
#define XLP3XX_STG2_MS_CREDIT		XLP3XX_MS_FIFO_SZ

/*################################*/

/*################################*/
#define XLP8XX_STG2_FIFO_SZ			2048
#define XLP8XX_EH_FIFO_SZ			4096
#define XLP8XX_FROUT_FIFO_SZ		4096
#define XLP8XX_MS_FIFO_SZ			2048
#define XLP8XX_PKT_FIFO_SZ			16384
#define XLP8XX_PKTLEN_FIFO_SZ		2048

#define XLP8XX_MAX_STG2_OFFSET		0x7F
#define XLP8XX_MAX_EH_OFFSET		0x7F
#define XLP8XX_MAX_FREE_OUT_OFFSET	0x7F
#define XLP8XX_MAX_MS_OFFSET		0x14
#define XLP8XX_MAX_PMEM_OFFSET		0x7FE

#define XLP8XX_STG1_2_CREDIT		XLP8XX_STG2_FIFO_SZ
#define XLP8XX_STG2_EH_CREDIT		XLP8XX_EH_FIFO_SZ
#define XLP8XX_STG2_FROUT_CREDIT	XLP8XX_FROUT_FIFO_SZ
#define XLP8XX_STG2_MS_CREDIT		XLP8XX_MS_FIFO_SZ

#define XLP_FREEIN_SPILL_DEFAULT_MEM_ADDR	(252ULL << 20)
#define XLP_FREEIN_SPILL_DEFAULT_MEM_SIZE	(4ULL << 20)

/*################################*/

#define NETIOR_CMPLX_0_INIT_CREDIT	0
#define NETIOR_CMPLX_1_INIT_CREDIT	8
#define NETIOR_CMPLX_2_INIT_CREDIT	16
#define NETIOR_CMPLX_3_INIT_CREDIT	24
#define NETIOR_CMPLX_4_INIT_CREDIT	18

struct nae_complex_config {
	uint16_t num_free_desc[MAX_PORTS_PERBLOCK];
	uint16_t free_desc_size[MAX_PORTS_PERBLOCK];
	uint16_t intf_fifo_size[MAX_PORTS_PERBLOCK];
	uint16_t prsr_seq_fifo_size[MAX_PORTS_PERBLOCK];
	uint16_t rx_buf_size[MAX_PORTS_PERBLOCK];
	uint16_t ucore_mask[MAX_PORTS_PERBLOCK];
	uint16_t ext_phy_mode[MAX_PORTS_PERBLOCK];
	uint16_t ext_phy_addr[MAX_PORTS_PERBLOCK];
	uint16_t ext_phy_bus[MAX_PORTS_PERBLOCK];
	uint16_t mgmt[MAX_PORTS_PERBLOCK];
	uint16_t disable[MAX_PORTS_PERBLOCK];			// Allow disable of a port in DTS
	uint16_t rx_ctxt_base[MAX_PORTS_PERBLOCK];		// Allow arbitrary selection of context
	uint16_t tx_ctxt_base[MAX_PORTS_PERBLOCK];
	uint16_t loopback[MAX_PORTS_PERBLOCK];
	uint16_t num_channels[MAX_PORTS_PERBLOCK];
	uint16_t num_rx_channels[MAX_PORTS_PERBLOCK];	// From 2.3.1
	uint16_t num_tx_channels[MAX_PORTS_PERBLOCK];	// From 2.3.1
	uint16_t num_lanes;
	uint16_t lane_rate;
	uint16_t higig_mode;
	uint16_t xgmii_speed;
	uint16_t vlan_pri_en;
	uint16_t msec_port_enable;
};

struct poe_statistics {
	uint64_t ooo_msg_count;
	uint64_t inorder_msg_count;
	uint64_t loc_stor_access_count;
	uint64_t ext_stor_access_count;
	uint64_t loc_stor_alloc_count;
	uint64_t ext_stor_alloc_count;
};


/* Temporarily specifying these sizes here.
   These will be moved to FDT soon
*/

static inline uint32_t nlm_stg2_fifo_sz(void)
{
	if(is_nlm_xlp3xx()||is_nlm_xlp2xx()){
		return XLP3XX_STG2_FIFO_SZ;
	}else{
		return XLP8XX_STG2_FIFO_SZ;
	}
}

static inline uint32_t nlm_eh_fifo_sz(void)
{
	if(is_nlm_xlp3xx()||is_nlm_xlp2xx()){
		return XLP3XX_EH_FIFO_SZ;
	}else{
		return XLP8XX_EH_FIFO_SZ;
	}
}

static inline uint32_t nlm_frout_fifo_sz(void)
{
	if(is_nlm_xlp3xx()||is_nlm_xlp2xx()){
		return XLP3XX_FROUT_FIFO_SZ;
	}else{
		return XLP8XX_FROUT_FIFO_SZ;
	}
}

static inline uint32_t nlm_ms_fifo_sz(void)
{
	if(is_nlm_xlp3xx()||is_nlm_xlp2xx()){
		return XLP3XX_MS_FIFO_SZ;
	}else{
		return XLP8XX_MS_FIFO_SZ;
	}
}

static inline uint32_t nlm_pkt_fifo_sz(void)
{
	if(is_nlm_xlp3xx()||is_nlm_xlp2xx()){
		return XLP3XX_PKT_FIFO_SZ;
	}else{
		return XLP8XX_PKT_FIFO_SZ;
	}
}

static inline uint32_t nlm_pktlen_fifo_sz(void)
{
	if(is_nlm_xlp3xx()||is_nlm_xlp2xx()){
		return XLP3XX_PKTLEN_FIFO_SZ;
	}else{
		return XLP8XX_PKTLEN_FIFO_SZ;
	}
}

static inline uint32_t max_stg2_offset(void)
{
	if(is_nlm_xlp3xx()||is_nlm_xlp2xx()){
		return XLP3XX_MAX_STG2_OFFSET;
	}else{
		return XLP8XX_MAX_STG2_OFFSET;
	}
}

static inline uint32_t max_eh_offset(void)
{
	if(is_nlm_xlp3xx()||is_nlm_xlp2xx()){
		return XLP3XX_MAX_EH_OFFSET;
	}else{
		return XLP8XX_MAX_EH_OFFSET;
	}
}

static inline uint32_t max_free_out_offset(void)
{
	if(is_nlm_xlp3xx()||is_nlm_xlp2xx()){
		return XLP3XX_MAX_FREE_OUT_OFFSET;
	}else{
		return XLP8XX_MAX_FREE_OUT_OFFSET;
	}
}

static inline uint32_t max_ms_offset(void)
{
	if(is_nlm_xlp3xx()||is_nlm_xlp2xx()){
		return XLP3XX_MAX_MS_OFFSET;
	}else{
		return XLP8XX_MAX_MS_OFFSET;
	}
}

static inline uint32_t max_pmem_offset(void)
{
	if(is_nlm_xlp3xx()||is_nlm_xlp2xx()){
		return XLP3XX_MAX_PMEM_OFFSET;
	}else{
		return XLP8XX_MAX_PMEM_OFFSET;
	}
}

static inline uint32_t stg1_2_credit(void)
{
	if(is_nlm_xlp3xx()||is_nlm_xlp2xx()){
		return XLP3XX_STG1_2_CREDIT;
	}else{
		return XLP8XX_STG1_2_CREDIT;
	}
}

static inline uint32_t stg2_eh_credit(void)
{
	if(is_nlm_xlp3xx()||is_nlm_xlp2xx()){
		return XLP3XX_STG2_EH_CREDIT;
	}else{
		return XLP8XX_STG2_EH_CREDIT;
	}
}

static inline uint32_t stg2_frout_credit(void)
{
	if(is_nlm_xlp3xx()||is_nlm_xlp2xx()){
		return XLP3XX_STG2_FROUT_CREDIT;
	}else{
		return XLP8XX_STG2_FROUT_CREDIT;
	}
}

static inline uint32_t stg2_ms_credit(void)
{
	if(is_nlm_xlp3xx()||is_nlm_xlp2xx()){
		return XLP3XX_STG2_MS_CREDIT;
	}else{
		return XLP8XX_STG2_MS_CREDIT;
	}
}


/*###################################################*/
/* To access Interface specific regs in NAE block */
/* xlp_nae_base is set to xlp_mac_base + 0xE000 (BLOCK_7 * XLP_NA_REG_BLOCK_SIZE) */
#define XLP_NAE_OFFSET(node, iface) \
	(xlp_nae_base[node] | (((iface) & 0xf) << 9))

/* To access individual gmac regs */
#define XLP_MAC_OFFSET(node, blk, iface) \
	(xlp_mac_base[node] + (((blk) * XLP_NA_REG_BLOCK_SIZE)) + ((iface) * XLP_NA_REG_IFACE_SIZE))

#ifndef __ASSEMBLY__
/* To access POE regs based in PCI Memory */

#define nlm_hal_write_poe_pcim_reg(node, reg, val) nlm_hal_write_32bit_reg(xlp_poe_base_pcim[node], (reg), (val))
#define nlm_hal_read_poe_pcim_reg(node, reg) nlm_hal_read_32bit_reg(xlp_poe_base_pcim[node], (reg))

/* To access POE regs based in PCIE config space */

#define nlm_hal_write_poe_pcie_reg(node, reg, val) nlm_hal_write_32bit_reg(xlp_poe_base_pcie[node], (reg), (val))
#define nlm_hal_read_poe_pcie_reg(node, reg) nlm_hal_read_32bit_reg(xlp_poe_base_pcie[node], (reg))

/* NAE */
#define nlm_hal_write_nae_reg(node, reg, val) nlm_hal_write_32bit_reg(xlp_nae_base[node], (reg), (val))
#define nlm_hal_read_nae_reg(node, reg) nlm_hal_read_32bit_reg(xlp_nae_base[node], (reg))

#define nlm_hal_write_nae_iface_reg(node, iface, reg, val) nlm_hal_write_32bit_reg(XLP_NAE_OFFSET(node, iface), (reg), (val))
#define nlm_hal_read_nae_iface_reg(node, iface, reg) nlm_hal_read_32bit_reg(XLP_NAE_OFFSET(node, iface), (reg))

#define nlm_hal_write_ucode(node, ucore, offset, val) \
  nlh_write_cfg_reg32((xlp_mac_base[node] + 0x10000 + (ucore * CODE_SIZE_PER_UCORE) + offset), (val))

#define nlm_hal_read_ucode(node, ucore, offset) \
  nlh_read_cfg_reg32((xlp_mac_base[node] + 0x10000 + (ucore * CODE_SIZE_PER_UCORE) + offset))

#define nlm_hal_write_mac_reg(node, blk, iface, reg, val) nlm_hal_write_32bit_reg(XLP_MAC_OFFSET(node, blk, iface), (reg), (val))

#define nlm_hal_read_mac_reg(node, blk, iface, reg) nlm_hal_read_32bit_reg(XLP_MAC_OFFSET(node, blk, iface), (reg))

#define read_gmac_reg(node, idx, reg) nlm_hal_read_mac_reg(node, (((idx) & 0xff)>>2), ((idx) & 0x3), reg)
#define write_gmac_reg(node, idx, reg, val) nlm_hal_write_mac_reg(node, (((idx) & 0xff)>>2), ((idx) & 0x3), (reg), (val))

static __inline__ uint32_t vfbid_to_dest_map(unsigned int vfbid, unsigned int dest, int cmd) {
	return ((dest & 0x3fff) << 16) | ((vfbid & 0x7f) << 4) | (cmd & 0x1);
}

static __inline__ uint32_t ucore_spray_config(unsigned int interface, unsigned int ucore_mask, int cmd) {
	return ((cmd & 0x1) << 31) | ((ucore_mask & 0xffff) << 8) | (interface & 0x1f);
}

static __inline__ uint32_t poe_class_config(unsigned int table_index, unsigned int poe_class, int cmd) {
	return ((poe_class & 0xffffff) << 8) | ((cmd & 0x1) << 7) | (table_index & 0x7f);
}

static __inline__ uint32_t flow_base_mask_config(unsigned int interface, unsigned int base, unsigned int mask, int cmd) {
	return ((base & 0xffff) << 16) | ((cmd & 0x1) << 15) | ((mask & 0x1f) << 8) | (interface & 0x1f);
}

int nlm_hal_get_frin_total_queue(int node);
int nlm_hal_get_frin_queue_base(int node);
extern int nlm_hal_init_poe_distvec(int node, int vec, uint32_t cm0, uint32_t cm1, uint32_t cm2, uint32_t cm3, uint32_t vcmask);
extern void nlm_hal_init_poe_ext_storage(int node,
					 uint64_t fbp_base_phys,
					 uint64_t fbp_base_virt,
					 uint64_t msg_base_phys,
					 uint64_t msg_base_virt);

extern int nlm_hal_load_ucore(int node, int ucore_mask, unsigned int *opcodes, int num_opcodes);

extern int nlm_hal_init_if(int node, int port);
extern void nlm_hal_init_ingress(int node, unsigned int desc_size);
extern void nlm_hal_init_egress(int node);

extern struct nlm_hal_ext_phy *get_phy_info(int inf);
extern int  nlm_hal_init_ext_phy(int node, int intf);
extern int  nlm_hal_ext_phy_an(int node, int intf);
extern int  nlm_hal_status_ext_phy(int node, int intf, struct nlm_hal_mii_info *mii_info);
extern void nlm_hal_restart_an(int node, int intf, struct nlm_hal_mii_info *mii_info);

extern int nlm_enable_poe_statistics(int node);
extern int nlm_disable_poe_statistics(int node);
extern int nlm_read_poe_statistics(int node, struct poe_statistics *stats);

extern void nlm_hal_prepad_enable(int node, int size);
extern void nlm_hal_reset_1588_accum(int node);
extern void nlm_hal_1588_ld_freq_mul(int node, uint32_t ptp_inc_den, uint32_t ptp_inc_num,
					uint32_t ptp_inc_intg);
extern void nlm_hal_1588_ld_offs(int node, uint32_t ptp_off_hi, uint32_t ptp_off_lo);
extern void nlm_hal_1588_ld_user_val(int node, uint32_t user_val_hi, uint32_t user_val_lo);
extern void nlm_hal_1588_ptp_clk_sel(int node, int clk_type);
extern uint32_t nlm_hal_get_int_sts(int node);
extern uint64_t  nlm_hal_1588_ptp_get_counter(int node, int counter);
extern void nlm_hal_1588_ptp_set_counter(int node, int counter, uint64_t cnt_val);
extern int nlm_hal_is_intr_1588(int node);
extern void nlm_hal_enable_1588_intr(int node,int mask);
extern void nlm_hal_clear_1588_intr(int node, int timer);

extern void nlm_hal_set_context_xon_xoff_threshold(int node, int mtu_len);

enum NAE_REG_CMD {
	CMD_READ = 0,
	CMD_WRITE
};

enum if_link {
	LINK_DOWN = 0,
	LINK_UP
};

/* DO NOT CHANGE ENUM VALUES */
enum if_speed {
	SPEED_UNKNOWN = -1,
	SPEED_10M = 0,
	SPEED_100M = 1,
	SPEED_1000M = 2
};


/* NETWORK INF CTRL REG */
#define SOFTRESET			(1 << 11)
#define STATS_EN			(1 << 16)
#define STATS_RESET			(1 << 15)
#define TX_EN				(1 << 2)
#define SPEED(x)			((x) & 0x3)

/* MAC_CONF1 */
#define INF_SOFTRESET		(1 << 31)
#define INF_LOOP_BACK		(1 << 8)
#define INF_RX_FLOW_CTL		(1 << 5)
#define INF_TX_FLOW_CTL		(1 << 4)
#define INF_RX_ENABLE		(1 << 2)
#define INF_TX_ENABLE		(1)

/* MAC_CONF2 */
#define INF_PREMBL_LEN(x)	(((x) & 0xf) << 12)
#define INF_IFMODE(x)		(((x) & 0x3) << 8)
#define INF_LENCHK(x)		((((x) & 0x1)) << 4)
#define INF_PADCRCEN(x)		(((x) & 0x1) << 2)
#define INF_PADCRC(x)		(((x) & 0x1) << 1)
#define INF_FULLDUP(x)		((x) & 0x1)
#define TXINITIORCR(x)		((x) & 0x7ffff) << 8

#define NAE_RX_ENABLE		0x1
#define NAE_TX_ENABLE		0x1
#define NAE_TX_ACE			0x2
#define NAE_TX_COMPATIBLE	0x4

#define INF_BYTE_MODE		0x2
#define INF_NIBBLE_MODE		0x1

/* PHY Access routines
 * Internal MDIO: 0x799
 *    -- support clause 22
 *    -- support clause 45 with devType=0x5 only
 * External MDIO: EXT_G0:0x79D EXT_G1:0x7A1
 *    -- support clause 22 only
 *    -- used for 1GE interface
 * External MDIO: EXT_XG0:0x7A5 EXT_XG1:0x7A9
 *    -- support clause 22
 *    -- support clause 45 with devType as argument
 *    -- used for 10GE interface
 */
enum {
	NLM_HAL_INT_MDIO		= 0, /* Internal MDIO Clause 22 */
	NLM_HAL_EXT_MDIO		= 1, /* EXT_G<0,1>: MDIO Clause 22 */
	NLM_HAL_INT_MDIO_C45	= 2, /* Internal MDIO: Clause 45 */
	NLM_HAL_EXT_MDIO_C45	= 3  /* EXT_XG<0,1>: External MDIO: Clause 45 */
};

/* MDIO reset/read/write
 * Note: block, intf_type are going to be removed.
 * block     = BLOCK_7  (NAE Block)
 * intf_type = LINE_CFG (0xF)
 */
extern int nlm_hal_mdio_reset(int node, int type, int bus);
extern int nlm_hal_mdio_wr(int node, int type, int bus, int phyaddr, int regidx, uint16_t val);
extern int nlm_hal_mdio_rd(int node, int type, int bus, int phyaddr, int regidx);
extern int nlm_hal_mdio_read(int node, int type, int bus, int block, int intf_type, int phyaddr,
		int regidx);
extern int nlm_hal_mdio_write(int node, int type, int bus, int block, int intf_type, int phyaddr,
		int regidx, uint16_t val);

/* External MDIO: C45 read/write */
extern int nlm_hal_xgmac_mdio_read(int node, int bus, int phyaddr, int dev_addr, int regidx);
extern int nlm_hal_xgmac_mdio_write(int node, int bus, int phyaddr, int dev_addr, int regidx,
		uint16_t val);

extern int nlm_hal_c45_mdio_indirect_write_external(int node, int bus,
			int phyaddr, int dev_addr, uint32_t reg_addr, uint32_t write_data);
extern int nlm_hal_c45_mdio_indirect_read_external (int node, int bus,
			int phyaddr, int dev_addr, uint32_t reg_addr);

#define NLM_C45_WRITE(node, bus, phyaddr, dev_addr, reg_addr, wdata)	\
	nlm_hal_c45_mdio_indirect_write_external(node, bus, phyaddr, dev_addr, reg_addr, wdata)

#define NLM_C45_READ(node, bus, phyaddr, dev_addr, reg_addr)	\
	nlm_hal_c45_mdio_indirect_read_external(node, bus, phyaddr, dev_addr, reg_addr)

extern int xlp3xx_8xxb0_nae_lane_reset_txpll(int node, int block, int lane_ctrl, int phymode);

/* PCS initialization
 */
extern void nlm_hal_mdio_init(int node);
extern void nlm_hal_sgmii_pcs_init(int node, int sgmii_cplx_mask);

extern void nlm_hal_sgmii_phy_init(int node);

extern void nlm_hal_reset_nae_ownership(void *fdt, int dom_id);
extern void reset_nae_mgmt(int node);
extern int nlm_hal_write_ucore_shared_mem(int node, unsigned int *data, int words);

extern uint16_t nlm_hal_get_hwport(int node, uint32_t context);

#ifdef NLM_HAL_LINUX_KERNEL
extern int nlm_hal_set_ilk_framesize(int node, int intf, int port, uint32_t size);
extern int nlm_hal_get_ilk_mac_stats(int node, int intf, int port, void *data);
#endif

extern void nlm_hal_msec_tx_default_config(int node, unsigned int port_enable, unsigned int preamble_len, unsigned int packet_num, unsigned int pn_thrshld);
extern void nlm_hal_msec_rx_default_config(int node, unsigned int port_enable, unsigned int preamble_len, unsigned int packet_num, unsigned int replay_win_size);
extern void nlm_hal_msec_rx_mem_config(int node, int port, int index, uint64_t sci, unsigned char *key, uint64_t sci_mask);
extern void nlm_hal_msec_rx_config(int node, unsigned int port_enable, unsigned int preamble_len, unsigned int packet_num, unsigned int replay_win_size);
extern void nlm_hal_msec_tx_mem_config(int node, int context, int tci, uint64_t sci, unsigned char *key);
extern void nlm_hal_msec_tx_config(int node, unsigned int port_enable, unsigned int preamble_len, unsigned int packet_num, unsigned int pn_thrshld);
extern int nlm_hal_retrieve_shared_freein_fifo_info(void *fdt,
		int shared_dom_id, int *owner_replenish, char **paddr_info, int *paddr_info_len,
		char **desc_info, int *desc_info_len);
extern unsigned int nlm_hal_retrieve_freein_fifo_mask(void *fdt, int node, int dom_id);
#endif /*__ASSEMBLY__ */


/* POE APIS */

static inline void nlm_write_enqspill_threshold(int node, uint32_t threshold)
{
	nlm_hal_write_poe_pcie_reg(node, POE_ENQ_SPILL_THOLD, (threshold & 0xFF));
}

static inline void nlm_write_deqspill_threshold(int node, uint32_t threshold)
{
	nlm_hal_write_poe_pcie_reg(node, POE_DEQ_SPILL_THOLD, (threshold & 0xFF));
}

static inline void nlm_write_deqspill_timer(int node, uint32_t timer)
{
	nlm_hal_write_poe_pcie_reg(node, POE_DEQ_SPILL_TIMER, (timer & 0x3FF));
}

static inline void nlm_enable_distribution_class_drop(int node, uint32_t class_mask)
{
	uint32_t val = nlm_hal_read_poe_pcie_reg(node, POE_DISTR_CLASS_DROP_EN);
	val |= (class_mask & 0xFF);
	nlm_hal_write_poe_pcie_reg(node, POE_DISTR_CLASS_DROP_EN, val);
}

static inline void nlm_enable_distribution_vector_drop(int node, uint32_t vector_mask)
{
	uint32_t val = nlm_hal_read_poe_pcie_reg(node, POE_DISTR_VEC_DROP_EN);
	val |= (vector_mask & 0xFFFF);
	nlm_hal_write_poe_pcie_reg(node, POE_DISTR_VEC_DROP_EN, val);
}

static inline void nlm_disable_distribution_class_drop(int node, uint32_t class_mask)
{
	uint32_t val = nlm_hal_read_poe_pcie_reg(node, POE_DISTR_CLASS_DROP_EN);
	val &= ~(class_mask & 0xFF);
	nlm_hal_write_poe_pcie_reg(node, POE_DISTR_CLASS_DROP_EN, val);
}

static inline void nlm_disable_distribution_vector_drop(int node, uint32_t vector_mask)
{
	uint32_t val = nlm_hal_read_poe_pcie_reg(node, POE_DISTR_VEC_DROP_EN);
	val &= ~(vector_mask & 0xFFFF);
	nlm_hal_write_poe_pcie_reg(node, POE_DISTR_VEC_DROP_EN, val);
}

static inline void nlm_write_distvec_drop_timer(int node, uint32_t timer)
{
	nlm_hal_write_poe_pcie_reg(node, POE_DISTRVEC_DROP_TIMER, (timer & 0xFFFF));
}

static inline void nlm_enable_distribution(int node)
{
	nlm_hal_write_poe_pcie_reg(node, POE_DIST_ENABLE, 1);
}

static inline void nlm_disable_distribution(int node)
{
	nlm_hal_write_poe_pcie_reg(node, POE_DIST_ENABLE, 0);
}

static inline void nlm_write_poe_dest_threshold(int node, uint32_t threshold)
{
	nlm_hal_write_poe_pcie_reg(node, POE_DEST_THRESHOLD, (threshold & 0xFFFF));
}

static inline void nlm_write_poe_distr_threshold(int node, uint32_t threshold0, uint32_t threshold1, uint32_t threshold2, uint32_t threshold3)
{
	nlm_hal_write_poe_pcie_reg(node, POE_DIST_THRESHOLD_0, threshold0);
	nlm_hal_write_poe_pcie_reg(node, POE_DIST_THRESHOLD_0+1, threshold1);
	nlm_hal_write_poe_pcie_reg(node, POE_DIST_THRESHOLD_0+2, threshold2);
	nlm_hal_write_poe_pcie_reg(node, POE_DIST_THRESHOLD_0+3, threshold3);
}

static inline uint32_t nlm_hal_ptp_timer_hi(int node, int inf_num)
{
	return nlm_hal_read_nae_reg(node, IF_1588_TMSMP_HI+(2*inf_num));
}

static inline uint32_t nlm_hal_ptp_timer_lo(int node, int inf_num)
{
	return nlm_hal_read_nae_reg(node, IF_1588_TMSMP_LO+(2*inf_num));
}


extern uint32_t nlm_hal_get_rtc(int node, uint32_t* p_val_hi, uint32_t* p_val_lo);

#endif /* __NLM_HAL_NAE_H__ */

