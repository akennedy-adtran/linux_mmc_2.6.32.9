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

/*
 * XLS RapidIO definitions
 */

#ifndef __XLS_SRIO_H__
#define __XLS_SRIO_H__

#include <asm/netlogic/sim.h>

static inline int is_xlsb0_srio(void) 
{
    unsigned int gpio_srio = 0;
    uint32_t *gpio_mmio = (uint32_t *)
                            (DEFAULT_NETLOGIC_IO_BASE + NETLOGIC_IO_GPIO_OFFSET);

    if (xlr_board_atx_xi() || xlr_board_atx_xii()) {
        gpio_srio = ((netlogic_read_reg(gpio_mmio,21) >> 26) & 0x3);
        if (gpio_srio) {
            return 1;
        }
        else {
            return 0;
        }
    }
    return 0;
}

#define RIO_OPS_LOCAL_CONFIG_READ(i) \
        static int nlm_local_config_read_##i(struct rio_mport *mport, int pindex, u32 offset,int len, \
                    u32 *data) \
        { \
            return nlm_local_config_read(rio_controller[i], pindex, offset, len, data);\
        }

#define RIO_OPS_LOCAL_CONFIG_WRITE(i) \
static int nlm_local_config_write_##i(struct rio_mport *mport, int pindex, u32 offset,int len, u32 data)\
{ \
  return nlm_local_config_write(rio_controller[i], pindex, offset, len, data);\
}

#define RIO_OPS_CONFIG_READ(i) \
static int nlm_rio_config_read_##i(struct rio_mport *mport, int index, u16 destid, u8 hopcount,\
                                     u32 offset,int len, u32 *val) \
{ \
    return nlm_rio_config_read(rio_controller[i], index, destid, hopcount,\
                        offset, len, val);\
}

#define RIO_OPS_CONFIG_WRITE(i) \
static int nlm_rio_config_write_##i(struct rio_mport *mport, int index, u16 destid, u8 hopcount, \
                                    u32 offset,int len, u32 val)\
{\
    return nlm_rio_config_write(rio_controller[i], index, destid, \
                            hopcount, offset, len, val);\
}

#define RIO_OPS_DOORBELL_SEND(i)\
static int nlm_rio_doorbell_send_##i(struct rio_mport *mport, int index, u16 destid, u16 data)\
{\
    return nlm_rio_doorbell_send(rio_controller[i], index, destid, data);\
}

void nlm_rio_setup(void);

#define SRIO_IRQ(irq) (PIC_SRIO_LINK0_IRQ+irq)
#define SRIO_CFG_BIT 21

#define SRIO_X1_MODE 1
#define SRIO_X4_MODE 2

#define MAX_SRIO_PORTS 4
#define MIN_SRIO_PORTS 1

#define NLM_SRIO_MEM_SIZE (16<<20)

/*BE space starts @ 0x14000000*/
#define NLM_SRIO_MEM_0 ((256<<20) + (64<<20))
#define NLM_SRIO_MEM_1 (NLM_SRIO_MEM_0 + NLM_SRIO_MEM_SIZE) 
#define NLM_SRIO_MEM_2 (NLM_SRIO_MEM_1 + NLM_SRIO_MEM_SIZE) 
#define NLM_SRIO_MEM_3 (NLM_SRIO_MEM_2 + NLM_SRIO_MEM_SIZE) 

#define MAX_TQ_ENTRY 256
#define MAX_SQ_ENTRY MAX_TQ_ENTRY 

#define SIZE_OF_TQ_ENTRY 24
#define SIZE_OF_SQ_ENTRY (16<<1)

#define MAX_TRANSACTION_Q   2
#define MAX_STATUS_Q        MAX_TRANSACTION_Q 

#define MAX_MAILBOX_Q       4
#define MAX_FREEL_Q         MAX_MAILBOX_Q 
#define MAX_MAILBOX_ENTRY   128
#define MAX_FREEL_ENTRY     MAX_MAILBOX_ENTRY   
#define SIZE_OF_MQ_ENTRY    (16<<1)
#define SIZE_OF_FQ_ENTRY    (8)

/***************************************/
/**Glue logic Register Goes here**/
/***************************************/
#define SRIO_CTRL 0x0
#define SRIO_PHY_CTRL0  0x1
#define SRIO_PHY_CTRL1  0x2
#define SRIO_PHY0_CTRL  0x3
#define SRIO_PHY1_CTRL  0x4
#define SRIO_PHY2_CTRL  0x5
#define SRIO_PHY3_CTRL  0x6
#define SRIO_COHERENT_MEM_BASE 0x8
#define SRIO_COHERENT_MEM_LIMIT 0x9
#define SRIO_REG_L2ALLOC_MEM_BASE 0x10
#define SRIO_REG_L2ALLOC_MEM_LIMIT 0x11
#define SRIO_REG_READEX_MEM_BASE 0x12
#define SRIO_REG_READEX_MEM_LIMIT 0x13
#define SRIO_REG_PHY_CR_CMD 0x16
#define SRIO_REG_PHY_CR_WR_DATA 0x17
#define SRIO_REG_PHY_CR_RESP 0x18
#define SRIO_REG_PHY_CR_RD_DATA 0x19
/***************************************/
/**Glue logic Register Ends here**/
/***************************************/

/***************************************/
/**Extended Feature Register Goes here**/
/***************************************/
#define P0_EAS_CSR 0x158

#define P0_CTRL_CSR 0x15c
    /*BIT Fields*/
    #define OUTPUT_PORT_EN  22
    #define INPUT_PORT_EN   21
    #define MULTI_EVENT_EN  12

/***************************************/
/**Extended Feature Register Ends here**/
/***************************************/

/***************************************/
/*Jennic Controller registers goes here*/
/***************************************/

/*Transaction Types goes here*/
#define TYPE_MAINTAIN_READ      13
#define TYPE_MAINTAIN_WRITE     14
#define TYPE_MESSAGE            16
#define TYPE_DOORBELL           17

/*Transaction Types ends here*/

/*Implementation defined registers starts*/
#define MASTER_INTR_STATUS_REG 0x10000
    #define MISR_DF     (1<<3)
    #define MISR_MQ3    (1<<6)
    #define MISR_SQ     (1<<9)
    #define MISR_GEN    (1<<11)
#define MASTER_INTR_ENABLE_REG 0x10004
    /*BIT Fields*/
    #define MIER_DF     3
    #define MIER_MQ3    6
    #define MIER_SQ     9
    #define MIER_GEN    11
#define GENERAL_INTR_STATUS_REG 0x10010
    /*BIT Fields*/
    #define GISR_PERR   (1<<2)
    #define GISR_DEC    (1<<5)
    #define GISR_MQWE   (1<<7)
    #define GISR_SQWE   (1<<8)
#define GENERAL_INTR_ENABLE_REG 0x10014
    /*BIT Fields*/
    #define GIER_PERR   2
    #define GIER_DEC    5    
    #define GIER_MQWE   7
    #define GIER_SQWE   8
#define DMA_ERR_CAP_HIGH    0x10030
#define DMA_ERR_CAP_LOW     0x10034
#define DMA_ERR_CAP_INFO    0x10038
/*Transaction Q Registers*/
#define TRANSACTION_QUEUE_START(n)  (0x20000+0x20*(n))
#define TRANSACTION_QUEUE_END(n)    (0x20004+0x20*(n))
#define TRANSACTION_QUEUE_HEAD(n)   (0x20008+0x20*(n))
#define TRANSACTION_QUEUE_TAIL(n)   (0x2000c+0x20*(n))
    /*BIT Fields*/
    #define TQ_LOCK 0
    #define TQ_FULL 1
#define TRANSACTION_QUEUE_UPTR(n)   (0x20010+0x20*(n))
#define TRANSACTION_QUEUE_CTRL_1    (0x20200)
    /*BIT Fields*/
    #define CONFIGURE_TQUEUE(n)      (16+n)
    #define ENABLE_QUEUE            11
#define TRANSACTION_QUEUE_CTRL_2    (0x20204)
#define TRANSACTION_QUEUE_STAT      (0x20208)
#define TRANSACTION_QUEUE_IER       (0x2020c)
    /*BIT Fields*/
    #define IN_ENABLE(n)            (16+n)

/*Maintenance Transactions Fields starts*/
/*word-0*/
#define MAINT_DEST_ID 16
#define MAINT_DID_SIZE 7
/*word-1*/
#define MAINT_TRANS_SIZE 2
#define MAINT_TRANS_DEFAULT_BITS 0
/*word-2*/
/*word-3*/
#define MAINT_HOP_COUNT     22
#define MAINT_REG_OFFSET    0
/*word-4*/
/*word-5*/
/*word-6*/

/*Maintenance Transactions Fields ends*/

/*Message Transactions Fields starts*/
/*word-0*/
#define MSG_DEST_ID 16
#define MSG_DID_SIZE 7
/*word-1*/
#define MSG_TRAN_SIZE       3
#define MSG_MAILBOX_NUMBER  24
#define MSG_SEG_SIZE        16
/*Message Transactions Fields ends*/

/*Door bell Transactions Fields starts*/
/*word-0*/
#define DBELL_DEST_ID 16
#define DBELL_DID_SIZE 7
/*word-1*/
#define DBELL_INFO  16
/*Door bell Transactions Fields ends*/

/*Status Q Registers Starts*/
#define STATUS_QUEUE_START(n)   (0x20400+0x20*(n))
#define STATUS_QUEUE_END(n)     (0x20404+0x20*(n))
#define STATUS_QUEUE_HEAD(n)    (0x20408+0x20*(n))
    /*Bit Fields */
    #define  SQ_EMPTY 1
#define STATUS_QUEUE_TAIL(n)    (0x2040c+0x20*(n))
#define STATUS_QUEUE_UPTR(n)    (0x20410+0x20*(n))
#define STATUS_QUEUE_CTRL       (0x20600)
    /*BIT Fields*/
    #define CONFIGURE_SQUEUE(n)      (16+n)
#define STATUS_QUEUE_STAT       (0x20608)
#define STATUS_QUEUE_IER        (0x2060c)
    /*BIT Fields*/
    #define NNE_INTR(n)         (n)
/*Status Q Registers Ends*/

/*Free list Q Registers Starts*/
#define FREEL_QUEUE_START(n)       (0x20a00+0x20*(n))
#define FREEL_QUEUE_END(n)         (0x20a04+0x20*(n))
#define FREEL_QUEUE_HEAD(n)        (0x20a08+0x20*(n))
#define FREEL_QUEUE_TAIL(n)        (0x20a0c+0x20*(n))
#define FREEL_QUEUE_UPTR(n)        (0x20a10+0x20*(n))
    /*Bit fields*/
    #define FQ_LOCK     0
    #define FQ_FULL     1
#define FREEL_BUF_SIZE(n)          (0x20a14+0x20*(n))
    /*BIT fields*/
    #define FL_BUF_SIZE            3
#define FREEL_CONTROL_REG          (0x20c00)
    /*Bit fields*/
    #define CONFIGURE_FLQUEUE(n)    (n+16)
#define FREEL_STATUS_REG           (0x20c08)
#define FREEL_INT_EN               (0x20c0c)
/*Free list Q Registers Ends*/

/*Mailbox Q Registers Starts*/
#define MAILBOX_QUEUE_START(n)  (0x21000+0x20*(n))
#define MAILBOX_QUEUE_END(n)    (0x21004+0x20*(n))
#define MAILBOX_QUEUE_HEAD(n)  (0x21008+0x20*(n))
    /*Bit fields*/
    #define MQ_EMPTY    1
#define MAILBOX_QUEUE_TAIL(n)    (0x2100c+0x20*(n))
#define MAILBOX_QUEUE_UPTR(n)    (0x21010+0x20*(n))

#define MAILBOX_CONTROL_1   0x21800
    /*BIT Fields*/
    #define CONFIGURE_MQUEUE(n) (n)
#define MAILBOX_CONTROL_2   0x21804
#define MAILBOX_CONTROL_3   0x21808
    /*BIT Fields*/
    #define HIGH_MAILB_NO   (6)

#define MAILBOX_STATUS_1    0x21820
#define MAILBOX_STATUS_2    0x21824
#define MAILBOX_STATUS_3    0x21828
#define MAILBOX_STATUS_4    0x2182c

#define MAILBOX_INT_EN_1    0x21830
#define MAILBOX_INT_EN_2    0x21834
#define MAILBOX_INT_EN_3    0x21838
#define MAILBOX_INT_EN_4    0x2183c

/*Mailbox Q Registers Ends*/

/*Door Bell Registers Starts*/
#define DOORBELL_INFO       0x20804
    /*Bit fields*/
    #define DFIFO_NUM       16 
#define DOORBELL_INTR       0x2080c
    /*Bit fields*/
    #define DB_NNE          0
#define DOORBELL_READ       0x20810
    /*Bit fields*/
    #define DB_SRCID        16
/*Door Bell Registers Ends*/

/*Implementation defined registers ends*/


/***************************************/
/*Jennic Controller registers ends here*/
/***************************************/

#endif				/* __XLS_SRIO_H__ */
