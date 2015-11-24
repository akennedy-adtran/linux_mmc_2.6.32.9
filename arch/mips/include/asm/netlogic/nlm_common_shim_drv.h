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


#ifndef _SHIM_H_
#define _SHIM_H_

#define DEBUG   1

extern int debug_var;

#ifdef DEBUG
#define debug(level, fmt, args...) \
	if (debug_var&level) printk(KERN_INFO fmt, ## args)
#else
#define debug(fmt, args...)
#endif
#define DEBUG_TRACE     0x8000
#define DEBUG_INFO      0x4000
#define DEBUG_TX_FSM    0x2000
#define DEBUG_RX_FSM    0x1000
#define DEBUG_REG_WRITE 0x0800
#define DEBUG_ENTRY     0x0400
#define DEBUG_EXIT      0x0200
#define DEBUG_DSCR      0x0100
#define DEBUG_ERROR     0x0080

#define ENTRY_LO(v) ((uint32_t)(v&0xffffffff))
#define ENTRY_HI(v) ((uint32_t)((v>>32)&0xffffffff))

#define SHIM_OK     0
#define SHIM_ERROR  -1

/**********************************************************************
 *  DMA Descriptor structure
 ********************************************************************* */

typedef struct shimdscr_s {
	uint64_t  dscr_a;
	uint64_t  dscr_b;
} shimdscr_t;

typedef unsigned long paddr_t;
typedef unsigned long vaddr_t;
typedef unsigned long shim_port_t;

/**********************************************************************
 *  DMA Controller structure
 ********************************************************************* */

typedef struct shim_cb_s {
    int             shim_macnum;
	int             shim_channel;	/* channel number */
	int		        shim_txdir;     /* direction (1=transmit) */
	int		        shim_maxdescr;	/* total # of descriptors in ring */
	shim_port_t     shim_config0;	/* DMA config register 0 */
	shim_port_t     shim_config1;	/* DMA config register 1 */
	shim_port_t     shim_dscrbase;	/* Descriptor base address */
	shim_port_t     shim_dscrcnt;   /* Descriptor count register */
	shim_port_t     shim_curdscr;	/* current descriptor address */
	shim_port_t     shim_dscraddcnt;/* Descriptor add count register */

    /* This stuff is for maintenance of the ring */
	
	shimdscr_t     *shim_dscrtable;	/* base of descriptor table */
	shimdscr_t     *shim_dscrtable_end; /* end of descriptor table */

    struct packet **shim_ctxtable;

    paddr_t         shim_dscrtable_phys; /* and also the phys addr */
    shimdscr_t     *shim_addptr;  /* next dscr for sw to add */
    shimdscr_t     *shim_remptr;  /* next dscr for sw to remove */
    int             fsm_state;
    int             shim_ready;    /* Activated by driver */
    int             rcv_buf_full;   /* When set do not issue msgld */
} shim_cb_t;

#define SHIM_READCSR(t)    (*(uint64_t*)(t))
#define SHIM_WRITECSR(t,v) (*(uint64_t*)(t) = v)
#define SHIM_NEXTBUF(d,f) ((((d)->f+1) == (d)->shim_dscrtable_end) ? \
              (d)->shim_dscrtable : (d)->f+1)
#define SHIM_MAC_TYPE(macnum) ((macnum < MAC_MAX_GMAC) ? GMAC : XGMAC)
#define SHIM_MAC_ID(macnum)   ((macnum < MAC_MAX_GMAC) ? macnum :\
                             (macnum - MAC_MAX_GMAC))
#define SHIM_GET_CB(macnum, txrx, chan) &shim_cb[macnum][txrx][chan]
#define SHIM_CURDSCR(p) ((shimdscr_t *)(uint32_t)\
                SHIM_READCSR(p->shim_curdscr))



#define SHIM_ST_INVALID     -1
#define SHIM_TR_INVALID     -1
#define SHIM_EV_INVALID     -1

/*
 *  Finite State machine States
 *
 *  Count of non-terminal states.  The generated states INVALID and DONE
 *  are terminal, but INIT is not  :-).
 */
#define SHIM_TX_STATE_CT 3 
#define SHIM_TX_EVENT_CT 5
typedef enum {
    SHIM_ST_TX_INIT,
    SHIM_ST_TX_ACTIVE,
    SHIM_ST_TX_PAUSE,  
} te_shim_tx_state;

typedef enum {
    SHIM_EV_TX_RESET,     
    SHIM_EV_TX_CPU_PAUSE_EN,    
    SHIM_EV_TX_CPU_RESUME,
    SHIM_EV_TX_DSCR_CNT,
    SHIM_EV_TX_ADD_DSCR_CNT,
} te_shim_tx_event;

#define SHIM_RX_STATE_CT 2
#define SHIM_RX_EVENT_CT 4

typedef enum {
    SHIM_ST_RX_INIT,
    SHIM_ST_RX_ACTIVE,
} te_shim_rx_state;

typedef enum {
    SHIM_EV_RX_RESET,     
    SHIM_EV_RX_DSCR_CNT,
    SHIM_EV_RX_DSCR_ADD_CNT,
    SHIM_EV_RX_PKT_FROM_MSGRNG
} te_shim_rx_event;


/*
 *  Run the FSM.  Will return SHIM_ST_DONE or SHIM_ST_INVALID
 */
int shim_run_rx_fsm(shim_cb_t *shim_cb_p, int trans_evt);
int shim_run_tx_fsm(shim_cb_t *shim_cb_p, int trans_evt);

        
/*
 * Cast to 64-bit number.  Presumably the syntax is different in 
 * assembly language.
 *
 * Note: you'll need to define uint32_t and uint64_t in your headers.
 */

#if !defined(__ASSEMBLER__)
#define _SHIM_MAKE64(x) ((uint64_t)(x))
#define _SHIM_MAKE32(x) ((uint32_t)(x))
#else
#define _SHIM_MAKE64(x) (x)
#define _SHIM_MAKE32(x) (x)
#endif


/*
 * Make a mask for 1 bit at position 'n'
 */

#define _SHIM_MAKEMASK1(n) (_SHIM_MAKE64(1) << _SHIM_MAKE64(n))
#define _SHIM_MAKEMASK1_32(n) (_SHIM_MAKE32(1) << _SHIM_MAKE32(n))

/*
 * Make a mask for 'v' bits at position 'n'
 */

#define _SHIM_MAKEMASK(v,n) (_SHIM_MAKE64((_SHIM_MAKE64(1)<<(v))-1) << _SHIM_MAKE64(n))
#define _SHIM_MAKEMASK_32(v,n) (_SHIM_MAKE32((_SHIM_MAKE32(1)<<(v))-1) << _SHIM_MAKE32(n))

/*
 * Make a value at 'v' at bit position 'n'
 */

#define _SHIM_MAKEVALUE(v,n) (_SHIM_MAKE64(v) << _SHIM_MAKE64(n))
#define _SHIM_MAKEVALUE_32(v,n) (_SHIM_MAKE32(v) << _SHIM_MAKE32(n))

#define _SHIM_GETVALUE(v,n,m) ((_SHIM_MAKE64(v) & _SHIM_MAKE64(m)) >> _SHIM_MAKE64(n))
#define _SHIM_GETVALUE_32(v,n,m) ((_SHIM_MAKE32(v) & _SHIM_MAKE32(m)) >> _SHIM_MAKE32(n))

/*
 * Macros to read/write on-chip registers
 * XXX should we do the PHYS_TO_K1 here?
 */


#if !defined(__ASSEMBLER__)
#define SHIMWRITECSR(csr,val) *((volatile uint64_t *) PHYS_TO_K1(csr)) = (val)
#define SHIMREADCSR(csr) (*((volatile uint64_t *) PHYS_TO_K1(csr)))
#endif /* __ASSEMBLER__ */


/*  *********************************************************************
    *  SHIM Registers
    ********************************************************************* */

/* 
 * Ethernet Configuration Register 0  
 * Registers: DMA_CONFIG0_MAC_x_RX_CH_0 
 * Registers: DMA_CONFIG0_MAC_x_TX_CH_0
 */


#define M_SHIM_DROP                  _SHIM_MAKEMASK1(0)
#define M_SHIM_CHAIN_SEL             _SHIM_MAKEMASK1(1)
#define M_SHIM_RESERVED1             _SHIM_MAKEMASK1(2)
#define M_SHIM_EOP_INT_EN            _SHIM_MAKEMASK1(3)
#define M_SHIM_HWM_INT_EN            _SHIM_MAKEMASK1(4)
#define M_SHIM_LWM_INT_EN            _SHIM_MAKEMASK1(5)
#define M_SHIM_TBX_EN                _SHIM_MAKEMASK1(6)
#define M_SHIM_TDX_EN                _SHIM_MAKEMASK1(7)

#define S_SHIM_INT_PKTCNT            _SHIM_MAKE64(8)
#define M_SHIM_INT_PKTCNT            _SHIM_MAKEMASK(8,S_SHIM_INT_PKTCNT)
#define V_SHIM_INT_PKTCNT(x)         _SHIM_MAKEVALUE(x,S_SHIM_INT_PKTCNT)
#define G_SHIM_INT_PKTCNT(x)         _SHIM_GETVALUE(x,S_SHIM_INT_PKTCNT,M_SHIM_INT_PKTCNT)

#define S_SHIM_RINGSZ                _SHIM_MAKE64(16)
#define M_SHIM_RINGSZ                _SHIM_MAKEMASK(16,S_SHIM_RINGSZ)
#define V_SHIM_RINGSZ(x)             _SHIM_MAKEVALUE(x,S_SHIM_RINGSZ)
#define G_SHIM_RINGSZ(x)             _SHIM_GETVALUE(x,S_SHIM_RINGSZ,M_SHIM_RINGSZ)

#define S_SHIM_HIGH_WATERMARK        _SHIM_MAKE64(32)
#define M_SHIM_HIGH_WATERMARK        _SHIM_MAKEMASK(16,S_SHIM_HIGH_WATERMARK)
#define V_SHIM_HIGH_WATERMARK(x)     _SHIM_MAKEVALUE(x,S_SHIM_HIGH_WATERMARK)
#define G_SHIM_HIGH_WATERMARK(x)     _SHIM_GETVALUE(x,S_SHIM_HIGH_WATERMARK,M_SHIM_HIGH_WATERMARK)

#define S_SHIM_LOW_WATERMARK         _SHIM_MAKE64(48)
#define M_SHIM_LOW_WATERMARK         _SHIM_MAKEMASK(16,S_SHIM_LOW_WATERMARK)
#define V_SHIM_LOW_WATERMARK(x)      _SHIM_MAKEVALUE(x,S_SHIM_LOW_WATERMARK)
#define G_SHIM_LOW_WATERMARK(x)      _SHIM_GETVALUE(x,S_SHIM_LOW_WATERMARK,M_SHIM_LOW_WATERMARK)

/*
 * Ethernet Configuration Register 1 
 * Registers: DMA_CONFIG1_MAC_x_RX_CH_0 
 * Registers: DMA_CONFIG1_SHIM_x_TX_CH_0
 */

#define M_SHIM_HDR_CF_EN             _SHIM_MAKEMASK1(0)
#define M_SHIM_ASIC_XFR_EN           _SHIM_MAKEMASK1(1)
#define M_SHIM_PRE_ADDR_EN           _SHIM_MAKEMASK1(2)
#define M_SHIM_FLOW_CTL_EN           _SHIM_MAKEMASK1(3)
#define M_SHIM_NO_DSCR_UPDT          _SHIM_MAKEMASK1(4)
#define M_SHIM_L2CA		    _SHIM_MAKEMASK1(5)
#define M_SHIM_CPU_PAUSE_EN		    _SHIM_MAKEMASK1(6)

#define M_SHIM_MBZ1                  _SHIM_MAKEMASK(6,15)

#define S_SHIM_HDR_SIZE              _SHIM_MAKE64(21)
#define M_SHIM_HDR_SIZE              _SHIM_MAKEMASK(9,S_SHIM_HDR_SIZE)
#define V_SHIM_HDR_SIZE(x)           _SHIM_MAKEVALUE(x,S_SHIM_HDR_SIZE)
#define G_SHIM_HDR_SIZE(x)           _SHIM_GETVALUE(x,S_SHIM_HDR_SIZE,M_SHIM_HDR_SIZE)

#define M_SHIM_MBZ2                  _SHIM_MAKEMASK(5,32)

#define S_SHIM_ASICXFR_SIZE          _SHIM_MAKE64(37)
#define M_SHIM_ASICXFR_SIZE          _SHIM_MAKEMASK(9,S_SHIM_ASICXFR_SIZE)
#define V_SHIM_ASICXFR_SIZE(x)       _SHIM_MAKEVALUE(x,S_SHIM_ASICXFR_SIZE)
#define G_SHIM_ASICXFR_SIZE(x)       _SHIM_GETVALUE(x,S_SHIM_ASICXFR_SIZE,M_SHIM_ASICXFR_SIZE)

#define S_SHIM_INT_TIMEOUT           _SHIM_MAKE64(48)
#define M_SHIM_INT_TIMEOUT           _SHIM_MAKEMASK(16,S_SHIM_INT_TIMEOUT)
#define V_SHIM_INT_TIMEOUT(x)        _SHIM_MAKEVALUE(x,S_SHIM_INT_TIMEOUT)
#define G_SHIM_INT_TIMEOUT(x)        _SHIM_GETVALUE(x,S_SHIM_INT_TIMEOUT,M_SHIM_INT_TIMEOUT)

/*
 * Ethernet Descriptor base address 
 */

#define M_SHIM_DSCRBASE_MBZ          _SHIM_MAKEMASK(4,0)
#define M_SHIM_DSCRBASE              _SHIM_MAKEMASK(36,4)


/* 
 * Current Descriptor Address Register 
 */

#define S_SHIM_CURDSCR_ADDR          _SHIM_MAKE64(0)
#define M_SHIM_CURDSCR_ADDR          _SHIM_MAKEMASK(40,S_SHIM_CURDSCR_ADDR)
#define S_SHIM_CURDSCR_COUNT         _SHIM_MAKE64(0)
#define M_SHIM_CURDSCR_COUNT         _SHIM_MAKEMASK(16,S_SHIM_CURDSCR_COUNT)

#define M_SHIM_CURDSCR_ADD_COUNT     M_SHIM_CURDSCR_COUNT

/*  ********************************************************************* 
    * Ethernet, MAC is not used
    ********************************************************************* */

#define A_MAC_BASE_0               0 

#define MAC_SPACING                 0x1000
#define SHIM_TXRX_SPACING        0x0400
#define SHIM_CHANNEL_SPACING     0x0100
#define SHIM_RX                      0
#define SHIM_TX                      1
#define MAX_SHIM_TXRX                2
#define MAC_NUM_SHIMCHAN         2           /* channels per direction */

#define MAC_NUM_PORTS               6
#define MAC_MAX_GMAC                4

#define A_MAC_CHANNEL_BASE(macnum)                  \
            (A_MAC_BASE_0 +                         \
             MAC_SPACING*(macnum))

#define R_SHIM_CHANNELS      0x800 /* Relative to A_MAC_CHANNEL_BASE */

#define A_SHIM_CHANNEL_BASE(macnum,txrx,chan)    \
             ((A_MAC_CHANNEL_BASE(macnum)) +        \
             R_SHIM_CHANNELS +                   \
             (SHIM_TXRX_SPACING*(txrx)) +        \
             (SHIM_CHANNEL_SPACING*(chan)))

#define R_SHIM_CHANNEL_BASE(txrx,chan)    \
             (R_SHIM_CHANNELS +                   \
             (SHIM_TXRX_SPACING*(txrx)) +        \
             (SHIM_CHANNEL_SPACING*(chan)))

#define A_SHIM_REGISTER(macnum,txrx,chan,reg)           \
            (A_SHIM_CHANNEL_BASE(macnum,txrx,chan) +    \
            (reg))

#define R_SHIM_REGISTER(txrx,chan,reg)           \
            (R_SHIM_CHANNEL_BASE(txrx,chan) +    \
            (reg))

#define I_SHIM_REGISTER(macnum,txrx,chan,reg)\
            ((A_SHIM_CHANNEL_BASE(macnum,txrx,chan) +    \
            (reg))/sizeof(unsigned long long int))
/* 
 * SHIM channel registers, relative to A_SHIM_CHANNEL_BASE
 */

#define R_SHIM_CONFIG0               0x00000000
#define R_SHIM_CONFIG1               0x00000008
#define R_SHIM_DSCR_BASE             0x00000010
#define R_SHIM_DSCR_CNT              0x00000018
#define R_SHIM_CUR_DSCRA             0x00000020
#define R_SHIM_CUR_DSCRB             0x00000028
#define R_SHIM_CUR_DSCRADDR          0x00000030
#define R_SHIM_DSCR_ADD_CNT          0x00000038


/*  *********************************************************************
    *  DMA Descriptors
    ********************************************************************* */

/*
 * Descriptor doubleword "A"  (Table 7-12)
 */

#define S_SHIM_DSCRA_OFFSET          _SHIM_MAKE64(0)
#define M_SHIM_DSCRA_OFFSET          _SHIM_MAKEMASK(5,S_SHIM_DSCRA_OFFSET)

/* Note: Don't shift the address over, just mask it with the mask below */
#define S_SHIM_DSCRA_A_ADDR          _SHIM_MAKE64(5)
#define M_SHIM_DSCRA_A_ADDR          _SHIM_MAKEMASK(35,S_SHIM_DSCRA_A_ADDR)

#define M_SHIM_DSCRA_A_ADDR_OFFSET   (M_SHIM_DSCRA_OFFSET | M_SHIM_DSCRA_A_ADDR)

#define S_SHIM_DSCRA_A_ADDR_UA        _SHIM_MAKE64(0)
#define M_SHIM_DSCRA_A_ADDR_UA        _SHIM_MAKEMASK(40,S_SHIM_DSCRA_A_ADDR_UA)

#define S_SHIM_DSCRA_A_SIZE          _SHIM_MAKE64(40)
#define M_SHIM_DSCRA_A_SIZE          _SHIM_MAKEMASK(9,S_SHIM_DSCRA_A_SIZE)
#define V_SHIM_DSCRA_A_SIZE(x)       _SHIM_MAKEVALUE(x,S_SHIM_DSCRA_A_SIZE)
#define G_SHIM_DSCRA_A_SIZE(x)       _SHIM_GETVALUE(x,S_SHIM_DSCRA_A_SIZE,M_SHIM_DSCRA_A_SIZE)

#define M_SHIM_DSCRA_INTERRUPT       _SHIM_MAKEMASK1(49)
#define M_SHIM_DSCRA_OFFSETB	    _SHIM_MAKEMASK1(50)

#define S_SHIM_DSCRA_STATUS          _SHIM_MAKE64(51)
#define M_SHIM_DSCRA_STATUS          _SHIM_MAKEMASK(13,S_SHIM_DSCRA_STATUS)
#define V_SHIM_DSCRA_STATUS(x)       _SHIM_MAKEVALUE(x,S_SHIM_DSCRA_STATUS)
#define G_SHIM_DSCRA_STATUS(x)       _SHIM_GETVALUE(x,S_SHIM_DSCRA_STATUS,M_SHIM_DSCRA_STATUS)

/*
 * Descriptor doubleword "B"  (Table 7-13)
 */


#define S_SHIM_DSCRB_OPTIONS         _SHIM_MAKE64(0)
#define M_SHIM_DSCRB_OPTIONS         _SHIM_MAKEMASK(4,S_SHIM_DSCRB_OPTIONS)
#define V_SHIM_DSCRB_OPTIONS(x)      _SHIM_MAKEVALUE(x,S_SHIM_DSCRB_OPTIONS)
#define G_SHIM_DSCRB_OPTIONS(x)      _SHIM_GETVALUE(x,S_SHIM_DSCRB_OPTIONS,M_SHIM_DSCRB_OPTIONS)

#define R_SHIM_DSCRB_ADDR            _SHIM_MAKE64(0x10)

/* Note: Don't shift the address over, just mask it with the mask below */
#define S_SHIM_DSCRB_B_ADDR          _SHIM_MAKE64(5)
#define M_SHIM_DSCRB_B_ADDR          _SHIM_MAKEMASK(35,S_SHIM_DSCRB_B_ADDR)

#define S_SHIM_DSCRB_B_SIZE          _SHIM_MAKE64(40)
#define M_SHIM_DSCRB_B_SIZE          _SHIM_MAKEMASK(9,S_SHIM_DSCRB_B_SIZE)
#define V_SHIM_DSCRB_B_SIZE(x)       _SHIM_MAKEVALUE(x,S_SHIM_DSCRB_B_SIZE)
#define G_SHIM_DSCRB_B_SIZE(x)       _SHIM_GETVALUE(x,S_SHIM_DSCRB_B_SIZE,M_SHIM_DSCRB_B_SIZE)

#define M_SHIM_DSCRB_B_VALID         _SHIM_MAKEMASK1(49)

#define S_SHIM_DSCRB_PKT_SIZE        _SHIM_MAKE64(50)
#define M_SHIM_DSCRB_PKT_SIZE        _SHIM_MAKEMASK(14,S_SHIM_DSCRB_PKT_SIZE)
#define V_SHIM_DSCRB_PKT_SIZE(x)     _SHIM_MAKEVALUE(x,S_SHIM_DSCRB_PKT_SIZE)
#define G_SHIM_DSCRB_PKT_SIZE(x)     _SHIM_GETVALUE(x,S_SHIM_DSCRB_PKT_SIZE,M_SHIM_DSCRB_PKT_SIZE)

/* 
 * Ethernet Descriptor Status Bits (Table 7-15)
 */

#define M_SHIM_ETHRX_BADIP4CS        _SHIM_MAKEMASK1(51)
#define M_SHIM_ETHRX_DSCRERR	    _SHIM_MAKEMASK1(52)

#define S_SHIM_ETHRX_RXCH            53
#define M_SHIM_ETHRX_RXCH            _SHIM_MAKEMASK(2,S_SHIM_ETHRX_RXCH)
#define V_SHIM_ETHRX_RXCH(x)         _SHIM_MAKEVALUE(x,S_SHIM_ETHRX_RXCH)
#define G_SHIM_ETHRX_RXCH(x)         _SHIM_GETVALUE(x,S_SHIM_ETHRX_RXCH,M_SHIM_ETHRX_RXCH)

#define S_SHIM_ETHRX_PKTTYPE         55
#define M_SHIM_ETHRX_PKTTYPE         _SHIM_MAKEMASK(3,S_SHIM_ETHRX_PKTTYPE)
#define V_SHIM_ETHRX_PKTTYPE(x)      _SHIM_MAKEVALUE(x,S_SHIM_ETHRX_PKTTYPE)
#define G_SHIM_ETHRX_PKTTYPE(x)      _SHIM_GETVALUE(x,S_SHIM_ETHRX_PKTTYPE,M_SHIM_ETHRX_PKTTYPE)

#define K_SHIM_ETHRX_PKTTYPE_IPV4    0
#define K_SHIM_ETHRX_PKTTYPE_ARPV4   1
#define K_SHIM_ETHRX_PKTTYPE_802     2
#define K_SHIM_ETHRX_PKTTYPE_OTHER   3
#define K_SHIM_ETHRX_PKTTYPE_USER0   4
#define K_SHIM_ETHRX_PKTTYPE_USER1   5
#define K_SHIM_ETHRX_PKTTYPE_USER2   6
#define K_SHIM_ETHRX_PKTTYPE_USER3   7

#define M_SHIM_ETHRX_MATCH_EXACT     _SHIM_MAKEMASK1(58)
#define M_SHIM_ETHRX_MATCH_HASH      _SHIM_MAKEMASK1(59)
#define M_SHIM_ETHRX_BCAST           _SHIM_MAKEMASK1(60)
#define M_SHIM_ETHRX_MCAST           _SHIM_MAKEMASK1(61)
#define M_SHIM_ETHRX_BAD	            _SHIM_MAKEMASK1(62)
#define M_SHIM_ETHRX_SOP             _SHIM_MAKEMASK1(63)

/*
 * Ethernet Transmit Status Bits (Table 7-16)
 */

#define M_SHIM_ETHTX_SOP	    	    _SHIM_MAKEMASK1(63)

/* 
 * Ethernet Transmit Options (Table 7-17)
 */

#define K_SHIM_ETHTX_NOTSOP          _SHIM_MAKE64(0x00)
#define K_SHIM_ETHTX_APPENDCRC       _SHIM_MAKE64(0x01)
#define K_SHIM_ETHTX_REPLACECRC      _SHIM_MAKE64(0x02)
#define K_SHIM_ETHTX_APPENDCRC_APPENDPAD _SHIM_MAKE64(0x03)
#define K_SHIM_ETHTX_APPENDVLAN_REPLACECRC _SHIM_MAKE64(0x04)
#define K_SHIM_ETHTX_REMOVEVLAN_REPLACECRC _SHIM_MAKE64(0x05)
#define K_SHIM_ETHTX_REPLACEVLAN_REPLACECRC _SHIM_MAKE64(0x6)
#define K_SHIM_ETHTX_NOMODS          _SHIM_MAKE64(0x07)
#define K_SHIM_ETHTX_RESERVED1       _SHIM_MAKE64(0x08)
#define K_SHIM_ETHTX_REPLACESADDR_APPENDCRC _SHIM_MAKE64(0x09)
#define K_SHIM_ETHTX_REPLACESADDR_REPLACECRC _SHIM_MAKE64(0x0A)
#define K_SHIM_ETHTX_REPLACESADDR_APPENDCRC_APPENDPAD _SHIM_MAKE64(0x0B)
#define K_SHIM_ETHTX_REPLACESADDR_APPENDVLAN_REPLACECRC _SHIM_MAKE64(0x0C)
#define K_SHIM_ETHTX_REPLACESADDR_REMOVEVLAN_REPLACECRC _SHIM_MAKE64(0x0D)
#define K_SHIM_ETHTX_REPLACESADDR_REPLACEVLAN_REPLACECRC _SHIM_MAKE64(0x0E)
#define K_SHIM_ETHTX_RESERVED2       _SHIM_MAKE64(0x0F)

#define S_MAC_RX_CH0                _SHIM_MAKE64(0)
#define S_MAC_RX_CH1                _SHIM_MAKE64(8)
#define S_MAC_TX_CH0                _SHIM_MAKE64(16)
#define S_MAC_TX_CH1                _SHIM_MAKE64(24)

#define M_MAC_INT_CHANNEL           _SHIM_MAKEMASK(8,0)
#define R_MAC_ENABLE                    0x00000400
#define R_MAC_STATUS                    0x00000408
#define R_MAC_ETHERNET_ADDR             0x00000208
        
extern shim_cb_t shim_cb[MAC_NUM_PORTS][MAX_SHIM_TXRX][MAC_NUM_SHIMCHAN];
/* Register file holding all register values, in a fixed location */
extern uint64_t reg_file[MAC_SPACING * MAC_NUM_PORTS];
int shim_handle_rx_pkt_from_msgrng(shim_cb_t *shim_cb_p);
int shim_handle_reset(shim_cb_t *shim_cb_p);
int shim_handle_tx_dscr_cnt(shim_cb_t *shim_cb_p);
int shim_handle_tx_cpu_pause_en(shim_cb_t *shim_cb_p);
int shim_handle_tx_resume(shim_cb_t *shim_cb_p);


#endif /* _SHIM_H_ */
