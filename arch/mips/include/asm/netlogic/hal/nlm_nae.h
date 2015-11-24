/*-
 * Copyright (c) 2003-2014 Broadcom Corporation
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

#ifndef __NLM_NAE_H__
#define __NLM_NAE_H__

#ifndef __ASSEMBLY__
#define MAX_NAE_CONTEXTS_PERNODE	524
#define MAX_NAE_PORTS_PERNODE		18
#define MAX_NAE_FREEIN_DESCS_QUEUE	19		// +1 for GDX port
#define XLP_MAX_INTERLAKEN_IF		2

#ifndef NLM_MAX_NODES
#define NLM_MAX_NODES				4
#endif

#define MAX_NAE_CPLX_PER_NODE		8

#define FREEBACK_TO_NAE				0x01
#define VFBID_FROM_FDT				0x02
#define FREEIN_SPILL_DYNAMIC		0x04
#define POE_ENQSPILL_DYNAMIC		0x08
#define POE_DEQSPILL_DYNAMIC		0x10
#define NAE_RESET_DONE				0x20
#define NAE_INIT_VALID				0x40

#ifndef NLM_NCPUS_PER_NODE
#define NLM_NCPUS_PER_NODE			32
#endif

enum higig_mode{
	NO_HIGIG,
	HIGIG,
	HIGIG2
};

#ifndef NLM_NUM_THREADS_PER_CORE
#define NLM_NUM_THREADS_PER_CORE	4
#endif

/* Only 3 domains can share one nae node including the owner */
#define NLM_NAE_MAX_SHARED_DOMS		2

#define NLM_NAE_MAX_FREEIN_FIFOS_PER_NODE 20
/* XAUI Card only support XAUI mode */
#define NLM_NAE_XAUI_MODE_XAUI		0

/* RXAUI Card support 3 different modes */
#define NLM_NAE_RXAUI_MODE_XAUI		1
#define NLM_NAE_RXAUI_MODE_BROADCOM	2
#define NLM_NAE_RXAUI_MODE_MARVELL	3

/* SGMII MDIO Phy address to use when the SGMII port is directly connected */
#define SGMII_DIRECT_ATTACH			99

struct nlm_hal_nae_port {
	int      valid;
	int      mgmt;
//	int      num_free_desc;
	int      txq_base;
	int      rxq;
	int      hw_port_id;
	int      vlan_pri_en;
	int      iftype;
	uint32_t rx_ctxt_base;		// Allow arbitrary specification of base context per-port
	uint32_t tx_ctxt_base;
	uint32_t num_rx_channels;	// From 2.3.1
	uint32_t num_tx_channels;
	uint32_t rx_buf_size;
	uint32_t intf_fifo_size;
//	uint32_t free_desc_size;
	uint32_t prsr_seq_fifo_size;
	uint32_t rx_slots_reqd;
	uint32_t tx_slots_reqd;
	uint32_t ucore_mask;
	uint32_t ext_phy_addr;
	uint32_t ext_phy_bus;
	uint32_t rxaui_scrambler;	/* 0: disable scrambler ; 1: enable scrambler */
	uint32_t rxaui_mode;		/* 0: broadcom mode; 1: marvell */
	uint32_t xaui_mode;			/* 0: XPACK PHY; 1: NLP1042 PHY */
	int      loopback;
	int      autoneg_done;		// Added for u-boot - only do auto-negotiation once when port is first opened
};

struct nlm_hal_nae_config {
	int fb_vc;
	int rx_vc;
	int frin_queue_base;
	int frin_total_queue;
	int num_ports;
	uint32_t flags;
	int rx_cal_slots;
	int tx_cal_slots;
	/* onchip descs per queue: value is taken from array for all 
	   queues upto 0-17 */
	int freein_fifo_onchip_num_descs[MAX_NAE_FREEIN_DESCS_QUEUE];
	/* spill descs per queue, it will be added with the onchip size  */
	int freein_fifo_spill_num_descs; 
	uint64_t freein_spill_base;
	uint64_t freein_spill_size;
	struct nlm_hal_nae_port ports[MAX_NAE_PORTS_PERNODE];
	uint32_t cntx2port[MAX_NAE_CONTEXTS_PERNODE];
	uint32_t num_lanes[XLP_MAX_INTERLAKEN_IF];
	uint32_t lane_rate[XLP_MAX_INTERLAKEN_IF];
	/*egress fifo  */
	uint32_t stg2fifo_base;
	uint32_t ehfifo_base;
	uint32_t froutfifo_base;
	uint32_t msfifo_base;
	uint32_t pktfifo_base;
	uint32_t pktlenfifo_base;
	/* NAE complex map */
	uint32_t sgmii_complex_map;
	uint32_t xaui_complex_map;
	uint32_t higig_mode[MAX_NAE_CPLX_PER_NODE];
	uint32_t xgmii_speed[MAX_NAE_CPLX_PER_NODE];
	uint32_t rxaui_complex_map;
	uint32_t ilk_complex_map;
	/* total queues used = num_contexts */
	uint32_t num_tx_contexts;		// From 2.3.1
	uint32_t num_rx_contexts;

	/* I am the owner or not, who initialize the node */
	int owned;
	/* Freein fifo mask. Out of the max rx fifos, domain ownership
	of rx-fifos. */
	uint32_t freein_fifo_dom_mask;

	/* vfbtable id offset, software freeback and hardware freebaack */
	uint32_t vfbtbl_sw_offset;
	uint32_t vfbtbl_sw_nentries;
	uint32_t vfbtbl_hw_offset;
	uint32_t vfbtbl_hw_nentries;

	/* port fifo mode enabled/disabled */
	unsigned int port_fifo_en;
	
	uint32_t msec_port_enable;
	unsigned char sectag_offset[MAX_NAE_PORTS_PERNODE];
	unsigned char sectag_len[MAX_NAE_PORTS_PERNODE];
	unsigned char icv_len[MAX_NAE_PORTS_PERNODE];
};

typedef struct nlm_hal_nae_config * nlm_nae_config_ptr;

struct nlm_node_config
{
	int valid;
	int num_nodes;  /* Number of nodes */
	struct nlm_hal_nae_config *nae_cfg[NLM_MAX_NODES];      /* NAE configuration */
	struct fmn_cfg *fmn_cfg[NLM_MAX_NODES];
};

enum freq_config {
	NLM_DEFAULT	= 0,
	NLM_NAE,
	NLM_RSA,
	NLM_SAE,
	NLM_DTRE,
	NLM_CDE,
};

extern struct nlm_node_config nlm_node_cfg;

enum if_type {
	UNKNOWN_IF    = 0,
	SGMII_IF      = 1,
	XAUI_IF       = 2,
	INTERLAKEN_IF = 3,
	RXAUI_IF      = 6,
};

extern int nlm_hal_write_ucore_shared_mem(int node, unsigned int *data, int words);
extern int nlm_config_vfbid_table(int node, uint32_t start, uint32_t num_entries, uint32_t *vfbid_tbl);
extern uint32_t *cntx2port[];
extern int nlm_hal_restart_ucore(int node, void *fdt);
extern void nlm_hal_derive_cpu_to_freein_fifo_map(int node, unsigned int phys_cpu_map,
		unsigned int freein_fifo_mask, unsigned int *cpu_2_freein_fifo_map);
extern void nlm_hal_modify_nae_ucore_sram_mem(int node, int ucoreid, unsigned int *data, 
		int off, int words);
extern void nlm_hal_read_nae_ucore_sram_mem(int node, int ucoreid, unsigned int *data, 
		int off, int words);

extern void nlm_hal_disable_xaui_flow_control(int node, int block);

#endif
#endif
