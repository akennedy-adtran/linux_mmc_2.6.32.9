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


#ifndef _ASM_NLM_MSG_RING_H
#define _ASM_NLM_MSG_RING_H

#include <linux/types.h>

#include <asm/asm.h>
#include <asm/netlogic/debug.h>
#include <asm/netlogic/mips-exts.h>

#ifndef __STR
#define __STR(x) #x
#endif
#ifndef STR
#define STR(x) __STR(x)
#endif

#define find_msb_one_bit(source)                                \
({ uint64_t __res;                                              \
        __asm__ __volatile__(                                   \
	".set\tpush\n\t"					\
	".set\tnoreorder\n\t"					\
        "dlco\t$8, %1\n\t"                                      \
	".set\tpop"						\
        : "=r" (__res): "r" (source): "$8"                      \
        );                                                      \
        __res;})

#define read_32bit_cp2_register(source)                         \
({ int __res;                                                   \
        __asm__ __volatile__(                                   \
	".set\tpush\n\t"					\
	".set\treorder\n\t"					\
        "mfc2\t%0,"STR(source)"\n\t"                            \
	".set\tpop"						\
        : "=r" (__res));                                        \
        __res;})

#define write_32bit_cp2_register(register,value)                \
        __asm__ __volatile__(                                   \
        "mtc2\t%0,"STR(register)"\n\t"				\
	"nop"							\
        : : "r" (value));

#define read_32bit_cp2_register_sel(source, sel)                \
({ int __res;                                                   \
        __asm__ __volatile__(                                   \
	".set\tpush\n\t"					\
        ".set mips32\n\t"                                       \
        "mfc2\t%0,"STR(source)", %1\n\t"                        \
	".set\tpop"						\
        : "=r" (__res) : "i" (sel) );                           \
        __res;})

#define write_32bit_cp2_register_sel(reg, value, sel)           \
        __asm__ __volatile__(                                   \
	".set\tpush\n\t"					\
        ".set mips32\n\t"                                       \
        "mtc2\t%0,"STR(reg)", %1\n\t"                           \
	".set\tpop"						\
        : : "r" (value), "i" (sel) );

#define MSGRNG_TX_BUF_REG $0
#define MSGRNG_RX_BUF_REG $1

#define MSGRNG_MSG_STATUS_REG $2
#define MSGRNG_MSG_CONFIG_REG $3

#define MSGRNG_MSG_BUCKSIZE_REG $4

#define MSGRNG_CC_0_REG  $16
#define MSGRNG_CC_1_REG  $17
#define MSGRNG_CC_2_REG  $18
#define MSGRNG_CC_3_REG  $19
#define MSGRNG_CC_4_REG  $20
#define MSGRNG_CC_5_REG  $21
#define MSGRNG_CC_6_REG  $22
#define MSGRNG_CC_7_REG  $23
#define MSGRNG_CC_8_REG  $24
#define MSGRNG_CC_9_REG  $25
#define MSGRNG_CC_10_REG $26
#define MSGRNG_CC_11_REG $27
#define MSGRNG_CC_12_REG $28
#define MSGRNG_CC_13_REG $29
#define MSGRNG_CC_14_REG $30
#define MSGRNG_CC_15_REG $31

#define msgrng_read_status() read_32bit_cp2_register(MSGRNG_MSG_STATUS_REG)

#define msgrng_read_config() read_32bit_cp2_register(MSGRNG_MSG_CONFIG_REG)
#define msgrng_write_config(value) write_32bit_cp2_register(MSGRNG_MSG_CONFIG_REG, value)

#define msgrng_read_bucksize(bucket) read_32bit_cp2_register_sel(MSGRNG_MSG_BUCKSIZE_REG, bucket)
#define msgrng_write_bucksize(bucket, value) write_32bit_cp2_register_sel(MSGRNG_MSG_BUCKSIZE_REG, value, bucket)

#define msgrng_read_cc(reg, pri) read_32bit_cp2_register_sel(reg, pri)
#define msgrng_write_cc(reg, value, pri) write_32bit_cp2_register_sel(reg, value, pri)

#ifndef _ABI64
#define read_64bit_cp2_register_sel(source, sel)			\
({									\
	unsigned int high, low;						\
									\
		__asm__ __volatile__(					\
			".set\tmips64\n\t"				\
			"dmfc2\t$8, "STR(source)","STR(sel)"\n\t"		\
			"dsrl32\t%0, $8, 0\n\t"			        \
                        "dsll32\t$8, $8, 0\n\t"                         \
                        "dsrl32\t%1, $8, 0\n\t"                         \
			".set\tmips0"					\
			: "=r" (high), "=r"(low): "i"(sel) : "$8");	\
	( (((unsigned long long)high)<<32) | low);					\
})

#define write_64bit_cp2_register_sel(source, val, sel)			\
do {									\
     unsigned int high = val>>32;                                       \
     unsigned int low  = val & 0xffffffff;                              \
		__asm__ __volatile__(					\
			".set\tmips64\n\t"				\
                        "dsll32 $8, %1, 0\n"                            \
                        "dsll32 $9, %0, 0\n"                            \
                        "dsrl32 $8, $8, 0\n"                            \
                        "or     $8, $8, $9\n"				\
			"dmtc2\t$8, "STR(source)", %2\n\t"		\
			".set\tmips0"					\
			: : "r" (high), "r" (low), "i"(sel): "$8", "$9");		\
} while (0)

#else
#define read_64bit_cp2_register(source)                         \
({ unsigned long long __res;                                    \
        __asm__ __volatile__(                                   \
	".set\tpush\n\t"					\
	".set\treorder\n\t"					\
        ".set\tmips64\n\t"                                      \
        "dmfc2\t%0,"STR(source)"\n\t"                            \
	".set\tpop"						\
        : "=r" (__res));                                        \
        __res;})

#define write_64bit_cp2_register(register,value)                \
        __asm__ __volatile__(                                   \
	".set\tpush\n\t"					\
	".set\treorder\n\t"					\
        "dmtc2\t%0,"STR(register)"\n\t"				\
	"nop"							\
	".set\tpop"						\
        : : "r" (value));

#define read_64bit_cp2_register_sel(source, sel)                \
({ unsigned long long __res;                                    \
        __asm__ __volatile__(                                   \
	".set\tpush\n\t"					\
        ".set mips64\n\t"                                       \
        "dmfc2\t%0,"STR(source)", %1\n\t"                        \
	".set\tpop"						\
        : "=r" (__res) : "i" (sel) );                           \
        __res;})

#define write_64bit_cp2_register_sel(reg, value, sel)           \
        __asm__ __volatile__(                                   \
	".set\tpush\n\t"					\
        ".set mips64\n\t"                                       \
        "dmtc2\t%0,"STR(reg)", %1\n\t"                           \
	".set\tpop"						\
        : : "r" (value), "i" (sel) );
#endif

#define msgrng_load_rx_msg0() read_64bit_cp2_register_sel(MSGRNG_RX_BUF_REG, 0)
#define msgrng_load_rx_msg1() read_64bit_cp2_register_sel(MSGRNG_RX_BUF_REG, 1)
#define msgrng_load_rx_msg2() read_64bit_cp2_register_sel(MSGRNG_RX_BUF_REG, 2)
#define msgrng_load_rx_msg3() read_64bit_cp2_register_sel(MSGRNG_RX_BUF_REG, 3)

#define msgrng_load_tx_msg0(value) write_64bit_cp2_register_sel(MSGRNG_TX_BUF_REG, value, 0)
#define msgrng_load_tx_msg1(value) write_64bit_cp2_register_sel(MSGRNG_TX_BUF_REG, value, 1)
#define msgrng_load_tx_msg2(value) write_64bit_cp2_register_sel(MSGRNG_TX_BUF_REG, value, 2)
#define msgrng_load_tx_msg3(value) write_64bit_cp2_register_sel(MSGRNG_TX_BUF_REG, value, 3)

/* Station IDs */
#define MSGRNG_STNID_CPU0  0x00
#define MSGRNG_STNID_CPU1  0x08
#define MSGRNG_STNID_CPU2  0x10
#define MSGRNG_STNID_CPU3  0x18
#define MSGRNG_STNID_CPU4  0x20
#define MSGRNG_STNID_CPU5  0x28
#define MSGRNG_STNID_CPU6  0x30
#define MSGRNG_STNID_CPU7  0x38

#define MSGRING_STNID_DEVICES 64
#define MSGRNG_STNID_XGS0_TX 64
#define MSGRNG_STNID_XMAC0_00_TX 64
#define MSGRNG_STNID_XMAC0_01_TX 65
#define MSGRNG_STNID_XMAC0_02_TX 66
#define MSGRNG_STNID_XMAC0_03_TX 67
#define MSGRNG_STNID_XMAC0_04_TX 68
#define MSGRNG_STNID_XMAC0_05_TX 69
#define MSGRNG_STNID_XMAC0_06_TX 70
#define MSGRNG_STNID_XMAC0_07_TX 71
#define MSGRNG_STNID_XMAC0_08_TX 72
#define MSGRNG_STNID_XMAC0_09_TX 73
#define MSGRNG_STNID_XMAC0_10_TX 74
#define MSGRNG_STNID_XMAC0_11_TX 75
#define MSGRNG_STNID_XMAC0_12_TX 76
#define MSGRNG_STNID_XMAC0_13_TX 77
#define MSGRNG_STNID_XMAC0_14_TX 78
#define MSGRNG_STNID_XMAC0_15_TX 79

#define MSGRNG_STNID_XGS1_TX 80
#define MSGRNG_STNID_XMAC1_00_TX 80
#define MSGRNG_STNID_XMAC1_01_TX 81
#define MSGRNG_STNID_XMAC1_02_TX 82
#define MSGRNG_STNID_XMAC1_03_TX 83
#define MSGRNG_STNID_XMAC1_04_TX 84
#define MSGRNG_STNID_XMAC1_05_TX 85
#define MSGRNG_STNID_XMAC1_06_TX 86
#define MSGRNG_STNID_XMAC1_07_TX 87
#define MSGRNG_STNID_XMAC1_08_TX 88
#define MSGRNG_STNID_XMAC1_09_TX 89
#define MSGRNG_STNID_XMAC1_10_TX 90
#define MSGRNG_STNID_XMAC1_11_TX 91
#define MSGRNG_STNID_XMAC1_12_TX 92
#define MSGRNG_STNID_XMAC1_13_TX 93
#define MSGRNG_STNID_XMAC1_14_TX 94
#define MSGRNG_STNID_XMAC1_15_TX 95

#define MSGRNG_STNID_GMAC 96
#define MSGRNG_STNID_GMACRFR_0  97
#define MSGRNG_STNID_GMACTX0  98
#define MSGRNG_STNID_GMACTX1  99
#define MSGRNG_STNID_GMACTX2  100
#define MSGRNG_STNID_GMACTX3  101
#define MSGRNG_STNID_GMACRFR_1  103

#define MSGRNG_STNID_DMA      104
#define MSGRNG_STNID_DMA_0    104
#define MSGRNG_STNID_DMA_1    105
#define MSGRNG_STNID_DMA_2    106
#define MSGRNG_STNID_DMA_3    107

#define MSGRNG_STNID_XGS0FR 112
#define MSGRNG_STNID_XMAC0RFR 113

#define MSGRNG_STNID_XGS1FR 114
#define MSGRNG_STNID_XMAC1RFR 115

#define MSGRNG_STNID_SEC 120
#define MSGRNG_STNID_SEC0 120
#define MSGRNG_STNID_SEC1 121
#define MSGRNG_STNID_SEC2 122
#define MSGRNG_STNID_SEC3 123
#define MSGRNG_STNID_PK0  124

#define MSGRNG_STNID_GMAC1      80
#define MSGRNG_STNID_GMAC1_FR   81
#define MSGRNG_STNID_GMAC1_TX0  82
#define MSGRNG_STNID_GMAC1_TX1  83
#define MSGRNG_STNID_GMAC1_TX2  84
#define MSGRNG_STNID_GMAC1_TX3  85
#define MSGRNG_STNID_GMAC0      96
#define MSGRNG_STNID_GMAC0_FR   97
#define MSGRNG_STNID_GMAC0_TX0  98
#define MSGRNG_STNID_GMAC0_TX1  99
#define MSGRNG_STNID_GMAC0_TX2  100
#define MSGRNG_STNID_GMAC0_TX3  101
#define MSGRNG_STNID_CMP_0      108
#define MSGRNG_STNID_CMP_1      109
#define MSGRNG_STNID_CMP_2      110
#define MSGRNG_STNID_CMP_3      111
#define MSGRNG_STNID_PCIE_0     116
#define MSGRNG_STNID_PCIE_1     117
#define MSGRNG_STNID_PCIE_2     118
#define MSGRNG_STNID_PCIE_3     119
#define MSGRNG_STNID_XLS_PK0    121

#define MSGRNG_CODE_DEVICE         0
#define MSGRNG_CODE_MAC            MSGRNG_CODE_DEVICE
#define MSGRNG_CODE_XGMAC          MSGRNG_CODE_DEVICE
#define MSGRNG_CODE_SPI4           MSGRNG_CODE_DEVICE
#define MSGRNG_CODE_SEC            MSGRNG_CODE_DEVICE
#define MSGRNG_CODE_BOOT_WAKEUP    200

static inline int msgrng_xgmac_stid_rfr(int id)
{
  return !id ? MSGRNG_STNID_XMAC0RFR : MSGRNG_STNID_XMAC1RFR;
}

static inline int msgrng_xgmac_stid_tx(int id)
{
  return !id ? MSGRNG_STNID_XMAC0_00_TX : MSGRNG_STNID_XMAC1_00_TX;
}

static inline int msgrng_gmac_stid_rfr(int id)
{
  if (id & 0x4)
      return (MSGRNG_STNID_GMAC1_FR);
  return (MSGRNG_STNID_GMACRFR_0);
}

static inline int msgrng_gmac_stid_rfr_split_mode(int id)
{
  return ((id>>1)?MSGRNG_STNID_GMACRFR_1:MSGRNG_STNID_GMACRFR_0);
}

static inline int msgrng_gmac_stid_tx(int id)
{
  if (id & 0x4)
      return (MSGRNG_STNID_GMAC1_TX0 + (id & 0x3));
  return (MSGRNG_STNID_GMACTX0 + id);
}

static inline int msgrng_gmac0_stid_rfr(int id)
{
  return (MSGRNG_STNID_GMAC0_FR);
}
static inline int msgrng_gmac0_stid_tx(int id)
{
  return (MSGRNG_STNID_GMAC0_TX0 + id);
}
static inline int msgrng_gmac1_stid_rfr(int id)
{
  return (MSGRNG_STNID_GMAC1_FR);
}
static inline int msgrng_gmac1_stid_tx(int id)
{
  return (MSGRNG_STNID_GMAC1_TX0 + (id & 0x3));
}

static inline void msgrng_send(unsigned int stid)
{
  __asm__ volatile (
		    ".set push\n"
		    ".set noreorder\n"
		    "sync\n"
		    //		    "msgsnd %0\n"
		    "move  $8, %0\n"
		    "c2    0x80001\n"
		    ".set pop\n"
		    : : "r" (stid) : "$8"
		    );
}

static inline void msgrng_receive(unsigned int pri)
{
  __asm__ volatile (
		    ".set push\n"
		    ".set noreorder\n"
		    //		    "msgld %0\n"
		    "move $8, %0\n"
		    "c2   0x80002\n"
		    ".set pop\n"
		    : : "r" (pri) : "$8"
		    );
}

static inline void msgrng_wait(unsigned int mask)
{
  __asm__ volatile (
		    ".set push\n"
		    ".set noreorder\n"
		    //		    "msgwait %0\n"
		    "move $8, %0\n"
		    /*to ensure msgwait picks up the right bucket */
		    ""STR(PTR_ADDU)" $8, $8, $0\n"
		    "c2   0x80003\n"
		    ".set pop\n"
		    : :"r" (mask) : "$8"
		    );
}

#ifdef CONFIG_NLM_ENABLE_COP2

#define msgrng_enable(flags) 
#define msgrng_disable(flags) 

#else

#define msgrng_enable(flags)                \
do {                                        \
  preempt_disable(); \
  __asm__ volatile (                        \
		    ".set push\n\t"                 \
		    ".set reorder\n\t"              \
		    ".set noat\n\t"                 \
		    "mfc0 %0, $12\n\t"              \
		    "li  $8, 0x40000001\n\t"        \
		    "or  $1, %0, $8\n\t"            \
		    "xori $1, 1\n\t"                \
		    ".set noreorder\n\t"            \
		    "mtc0 $1, $12\n\t"              \
		    ".set\tpop\n\t"                 \
		    : "=r" (flags)                  \
		    :                               \
		    : "$8"                          \
		    );                              \
  preempt_enable(); \
} while (0)
#define msgrng_disable(flags) __asm__ volatile (    \
                 "mtc0 %0, $12" : : "r" (flags))

#endif

#define msgrng_flags_save(flags) msgrng_enable(flags)
#define msgrng_flags_restore(flags) msgrng_disable(flags)

struct msgrng_msg {
  __u64 msg0;
  __u64 msg1;
  __u64 msg2;
  __u64 msg3;
};

static inline void message_send_block_fast(int size, unsigned int code, unsigned int stid,
                                         unsigned long long msg0, unsigned long long msg1,
					 unsigned long long msg2, unsigned long long msg3)
{
  __asm__ __volatile__ (".set push\n"
                        ".set noreorder\n"
                        ".set mips64\n"
                        "dmtc2 %1, "STR(MSGRNG_TX_BUF_REG)", 0\n"
                        "dmtc2 %2, "STR(MSGRNG_TX_BUF_REG)", 1\n"
                        "dmtc2 %3, "STR(MSGRNG_TX_BUF_REG)", 2\n"
                        "dmtc2 %4, "STR(MSGRNG_TX_BUF_REG)", 3\n"
		        "sync\n"
                        "move $8, %0\n"
                        "1: c2 0x80001\n"
                        "mfc2 $8, "STR(MSGRNG_MSG_STATUS_REG)"\n"
                        "andi $8, $8, 0x6\n"
                        "bnez $8, 1b\n"
                        "move $8, %0\n"
                        ".set pop\n"
                        :
                        : "r"(((size-1)<<16)|(code<<8)|stid), "r" (msg0), "r" (msg1), "r"(msg2), "r"(msg3)
                        : "$8"
                        );
}

#define message_receive_fast(bucket, size, code, stid, msg0, msg1, msg2, msg3)      \
        ( { unsigned int _status=0, _tmp=0;                     \
           msgrng_receive(bucket);                              \
           while ( (_status=msgrng_read_status()) & 0x08) ;     \
           _tmp = _status & 0x30;                               \
           if (likely(!_tmp)) {                                 \
                 (size)=((_status & 0xc0)>>6)+1;                \
                 (code)=(_status & 0xff00)>>8;                  \
                 (stid)=(_status & 0x7f0000)>>16;               \
                 (msg0)=msgrng_load_rx_msg0();                  \
                 (msg1)=msgrng_load_rx_msg1();                  \
                 (msg2)=msgrng_load_rx_msg2();                  \
                 (msg3)=msgrng_load_rx_msg3();                  \
                 _tmp=0;                                        \
                }                                               \
           _tmp;                                                \
        } )

static __inline__ int message_send(unsigned int size, unsigned int code,
				   unsigned int stid, struct msgrng_msg *msg)
{
  unsigned int dest = 0;
  unsigned long long status=0;
  int i=0;

  msgrng_load_tx_msg0(msg->msg0);
  msgrng_load_tx_msg1(msg->msg1);
  msgrng_load_tx_msg2(msg->msg2);
  msgrng_load_tx_msg3(msg->msg3);

  dest = ((size-1)<<16)|(code<<8)|(stid);

  //dbg_msg("Sending msg<%Lx,%Lx,%Lx,%Lx> to dest = %x\n",
    //msg->msg0, msg->msg1, msg->msg2, msg->msg3, dest);


  for(i=0;i<16;i++) {
  	msgrng_send(dest);
	status = msgrng_read_status();
//	dbg_msg("status = %Lx\n", status);

	if (status & 0x6) {
	  continue;
	}
	else break;
	}
    if (i==16) {
	  if (dest == 0x61)
		  //dbg_msg("Processor %x: Unable to send msg to %llx\n", processor_id(), dest);
	  return status & 0x6;
	}
  return msgrng_read_status() & 0x06;
}

static __inline__ int message_send_retry(unsigned int size, unsigned int code,
					 unsigned int stid,
					 struct msgrng_msg *msg)
{
  int res = 0;
  int retry = 0;

  for(;;) {
    res = message_send(size, code, stid, msg);
    /* retry a pending fail */
    if (res & 0x02) continue;
    /* credit fail */
    if (res & 0x04) retry++;
    else break;
    if (retry == 4) return res & 0x06;
  }

  return 0;
}

static __inline__ int message_receive(int pri, int *size, int *code, int *src_id,
				      struct msgrng_msg *msg)
{
  int res = message_receive_fast(pri, *size, *code, *src_id, msg->msg0, msg->msg1, msg->msg2, msg->msg3);

#ifdef MSGRING_DUMP_MESSAGES
  if (!res) {
    dbg_msg("Received msg <%llx, %llx, %llx, %llx> <%d,%d,%d>\n",
	    msg->msg0, msg->msg1, msg->msg2, msg->msg3,
	    *size, *code, *src_id);
  }
#endif

  return res;
}

#define MSGRNG_STN_RX_QSIZE 256

typedef unsigned short bucket_t;
#define MAX_NUM_MSGRNG_STN_CC   128
#define MAX_NUM_GMAC_STNS 8
#define MAX_NUM_XGMAC_STNS 18
#define NR_STNS_PER_CORE 8

struct stn_cc {
	bucket_t counters[16][8];
};

struct bucket_size {
	bucket_t bucket[MAX_NUM_MSGRNG_STN_CC];
};

extern struct bucket_size bucket_sizes;

extern struct stn_cc cc_table_cpu_0;
extern struct stn_cc cc_table_cpu_1;
extern struct stn_cc cc_table_cpu_2;
extern struct stn_cc cc_table_cpu_3;
extern struct stn_cc cc_table_cpu_4;
extern struct stn_cc cc_table_cpu_5;
extern struct stn_cc cc_table_cpu_6;
extern struct stn_cc cc_table_cpu_7;
extern struct stn_cc cc_table_xgs_0;
extern struct stn_cc cc_table_xgs_1;
extern struct stn_cc cc_table_gmac;
extern struct stn_cc cc_table_dma;
extern struct stn_cc cc_table_sec;

extern struct bucket_size xls_bucket_sizes;
extern struct stn_cc xls_cc_table_cpu_0;
extern struct stn_cc xls_cc_table_cpu_1;
extern struct stn_cc xls_cc_table_cpu_2;
extern struct stn_cc xls_cc_table_cpu_3;
extern struct stn_cc xls_cc_table_gmac0;
extern struct stn_cc xls_cc_table_gmac1;
extern struct stn_cc xls_cc_table_cmp;
extern struct stn_cc xls_cc_table_pcie;
extern struct stn_cc xls_cc_table_dma;
extern struct stn_cc xls_cc_table_sec;

extern struct bucket_size shared_bucket_sizes;

extern struct stn_cc shared_cc_table_cpu_0;
extern struct stn_cc shared_cc_table_cpu_1;
extern struct stn_cc shared_cc_table_cpu_2;
extern struct stn_cc shared_cc_table_cpu_3;
extern struct stn_cc shared_cc_table_cpu_4;
extern struct stn_cc shared_cc_table_cpu_5;
extern struct stn_cc shared_cc_table_cpu_6;
extern struct stn_cc shared_cc_table_cpu_7;
extern struct stn_cc shared_cc_table_gmac;
extern struct stn_cc shared_cc_table_dma;

#ifdef CONFIG_NLM_ENABLE_COP2

#define msgrng_access_save(lock, iflags, mflags)
#define msgrng_access_restore(lock, iflags, mflags)

#define msgrng_access_enable(mflags) 
#define msgrng_access_disable(mflags) 

#else

#define msgrng_access_save(lock, iflags, mflags) do {        \
  spin_lock_irqsave(lock, iflags);                           \
  msgrng_flags_save(mflags);                                 \
 }while(0)

#define msgrng_access_restore(lock, iflags, mflags) do {     \
  msgrng_flags_restore(mflags);                              \
  spin_unlock_irqrestore(lock, iflags);                      \
 }while(0)

#define msgrng_access_enable(mflags) do {   \
  preempt_disable();                        \
  msgrng_flags_save(mflags);                \
} while(0)
#define msgrng_access_disable(mflags) do {   \
  msgrng_flags_restore(mflags);              \
  preempt_enable();                          \
} while(0)

#endif

enum {
  TX_STN_CPU_0,
  TX_STN_CPU_1,
  TX_STN_CPU_2,
  TX_STN_CPU_3,
  TX_STN_CPU_4,
  TX_STN_CPU_5,
  TX_STN_CPU_6,
  TX_STN_CPU_7,
  TX_STN_GMAC,
  TX_STN_DMA,
  TX_STN_XGS_0,
  TX_STN_XGS_1,
  TX_STN_SEC,
  TX_STN_GMAC0,
  TX_STN_GMAC1,
  TX_STN_CMP,
  TX_STN_PCIE,
  TX_STN_INVALID,
  MAX_TX_STNS
};

#ifdef CONFIG_NLM_XLP
extern int register_xlp_msgring_handler(int major,
                             void (*action) (uint32_t, uint32_t, uint32_t, uint32_t,
                                             uint64_t, uint64_t, uint64_t, uint64_t, void *),
                             void *dev_id);
extern int unregister_xlp_msgring_handler(int, void *);
#else
extern int register_msgring_handler(int major,
				    void (*action)(int, int,int,int,struct msgrng_msg *, void *),
				    void *dev_ctx);

#endif

extern void nlm_common_msgring_cpu_init(void);

extern void nlm_common_msgring_config(void);

#define cpu_to_msgring_bucket(cpu) ((((cpu) >> 2)<<3)|((cpu) & 0x03))


/* PR: We need to make the following entrities visible across the kernel */
#define CPU_BASE_BUCKET(x)   (((x)>>2)<<3)

#define THR_LO_BUCKETID (netlogic_thr_id() & 3)
#define THR_HI_BUCKETID (THR_LO_BUCKETID + 4)

#define THIS_THR_LO_BUCKET cpu_to_msgring_bucket(hard_smp_processor_id())
#define THIS_THR_HI_BUCKET (THIS_THR_LO_BUCKET)

#define THR_LO_BKT_STATUS_MASK (1U << THR_LO_BUCKETID)
#define THR_HI_BKT_STATUS_MASK (1U << THR_HI_BUCKETID)

struct msgrng_msg;

struct tx_stn_handler {
	void (*action)(int, int, int, int, struct msgrng_msg *, void *);
	void *dev_id;
};

struct tx_stn {
	struct tx_stn_handler handler;
};

extern struct tx_stn tx_stns[];
extern int rxstn_to_txstn_map[];
extern int xls_rxstn_to_txstn_map[];

extern int rmik_queue_pkt_mem(uint32_t fbstid, uint64_t physaddr);
extern void rmik_init_replenish_work(int);
extern void nlm_nlm_common_drop_message_unowned(int fbid, uint64_t physaddr, int cop_en);

#endif
