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


#ifndef _NLM_CDE_H
#define _NLM_CDE_H

#include <asm/netlogic/msgring.h>
#include <asm/io.h>   // virt_to_phys

#define CMP_REG_CNFG_MSG_CREDIT_SEL 14
#define CMP_REG_CTRL_REG         0
#define CMP_REG_DMA_CREDITS_REG  1
#define CMP_REG_SPILL_ADDR0_REG  2
#define CMP_REG_SPILL_ADDR1_REG  3
#define CMP_REG_SPILL_SIZE_REG   4
#define CMP_REG_SPILL_BYTES_REG  5
#define CMP_REG_CRC_ADLER_SPILL  6
#define CMP_REG_SCRATCH_PAGE     7
#define CMP_REG_INTERRUPT_VEC    8
#define CMP_REG_INTERRUPT_MASK   9
#define CMP_REG_FREE_DESC_THRES  10
#define CMP_REG_DESC_FIFO_COUNT  11
#define CMP_REG_RESET_REG        12
#define CMP_REG_ERROR_RESET_MASK 13
#define CMP_REG_READ_ERROR_LIST0 14
#define CMP_REG_READ_ERROR_LIST1 15

//defines needed to be declared
#define CMP_MSG_BUCKET0_SIZE 0x320
#define CMP_MSG_BUCKET1_SIZE 0x321


static inline uint32_t cmp_read_reg(int reg)
{
  nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_COMP_OFFSET);

  return netlogic_read_reg(mmio, reg);
}

static inline void cmp_write_reg(int reg, uint32_t value)
{
  nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_COMP_OFFSET);

  netlogic_write_reg(mmio, reg, value);
}

static __inline__ int make_fd_msg(struct msgrng_msg *msg, void *free_addr)
{
  int stid = MSGRNG_STNID_CMP_0;

  msg->msg0 = ( ((uint64_t)virt_to_phys(free_addr) & 0xffffffffffULL)
                );
  return stid;
}

static __inline__ int make_cmp_msg(struct msgrng_msg *msg, int rtn_bkt,
				   int op, int length, void *src_addr)
{
  int stid = MSGRNG_STNID_CMP_1;
  
  msg->msg0 = ( ((uint64_t) op << 60)  | 
                ((uint64_t) rtn_bkt << 54) |
                ((uint64_t) length << 40) |
                ((uint64_t) virt_to_phys(src_addr) & 0xffffffffffULL)
                );
  return stid;
}


static __inline__ uint64_t make_src_desc(int eof, int type, int sod, int sob, int save,
					 int restore, int eob, int length, void *src_addr)
{
  uint64_t src_desc = 0;
  src_desc = (  ((uint64_t) eof << 63) | 
                ((uint64_t) type << 61) |
                ((uint64_t) sod << 60) |
                ((uint64_t) sob << 59) |
                ((uint64_t) save << 58) |
                ((uint64_t) restore << 57) |
                ((uint64_t) eob << 56) |
                ((uint64_t) length << 40) |
                ((uint64_t) virt_to_phys(src_addr) & 0xffffffffffULL)
                );
  return src_desc;
}

static __inline__ uint64_t get_dest_desc(uint64_t dest_addr)
{
  uint64_t *desc;
  desc = phys_to_virt(dest_addr);
  return *desc;
}

static __inline__ int read_cmp_msg(char *buffer, uint64_t payload) 
{
  uint64_t i,j,num_bytes;
  int offset = 0;
  uint64_t *desc, dest_addr;
  int num_desc = (payload >> 40) & 0x3fff;
  char * tmp_ptr;

  //  printk("num_desc = %d\n", num_desc);

  for (i = 0; i < num_desc; i++) {
    desc = phys_to_virt(payload & 0xffffffffffUll) + i*8; //64 byte descriptors //dliao: why i*8??

    num_bytes = (*desc >> 40) & 0xffff;

    //    printk("num_bytes = %lld\n", num_bytes);

    dest_addr = *desc & 0xffffffffffUll;

    tmp_ptr = (char *) phys_to_virt(dest_addr & 0xffffffffffUll);

    for (j = 0; j < num_bytes; j++) {
      buffer[offset+j] = tmp_ptr[j];
      //   buffer[offset + j] = phys_to_virt(dest_addr & 0xffffffffff)+j;
    }

    offset = offset + num_bytes;
  }

  return offset;
}


#ifndef CDE_MAJOR
#define CDE_MAJOR 0   /* dynamic major by default */
#endif


/*
 * Split minors in two parts
 */
#define TYPE(minor)	(((minor) >> 4) & 0xf)	/* high nibble */
#define NUM(minor)	((minor) & 0xf)		/* low  nibble */


#define CDE_IOC_MAGIC  'k'
#define CDE_IOCINFLATE _IO(CDE_IOC_MAGIC, 0)
#define CDE_IOCDEFLATE _IO(CDE_IOC_MAGIC, 1)
#define CDE_IOC_MAXNR  1

#define CDE_INFLATE 0
#define CDE_DEFLATE 1


#endif /* _NLM_CDE_H_ */
