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


#include <net/ip.h>
#include <net/xfrm.h>
#include <net/ah.h>
#include <net/icmp.h>
#include <linux/crypto.h>

#include <asm/io.h>

#include <asm/netlogic/debug.h>
#include <asm/netlogic/nlm_common_sec.h>

#define NUM_CHUNKS(size, bits) ( ((size)>>(bits)) + (((size)&((1<<(bits))-1))?1:0) )
//#define DEBUG

static u8 tmp_auth_data[MAX_AH_AUTH_LEN] __cacheline_aligned;
static PacketDescriptor_t pkt_descs[NR_CPUS] __cacheline_aligned;
static ControlDescriptor_t ctrl_descs[NR_CPUS] __cacheline_aligned;

static spinlock_t msgrng_lock;

void nlm_common_ah_hmac_digest(struct ah_data *ahp, struct sk_buff *skb, u8 *auth_data)
{
	struct crypto_tfm *tfm = ahp->tfm;
	u8 *key = ahp->key;
	unsigned int keylen = ahp->key_len;
	
	int len_dwords=0;
	PacketDescriptor_t *pkt_desc = &pkt_descs[smp_processor_id()];
	ControlDescriptor_t *ctrl_desc = &ctrl_descs[smp_processor_id()];
	unsigned long addr = virt_to_phys(skb->data);
	int offset = addr - (addr & ~(SMP_CACHE_BYTES - 1));
	int len=0, len_lastdword_remainder=0;
	int ctrl_desc_size=0, ctrl_len_cachelines=0;
	int i=0;
	struct msgrng_msg msg;
	int size=0, code=0, stid=0;
	unsigned long flags=0, msgrng_flags=0;

#ifdef DEBUG
        {
	  dbg_msg("Expected auth result:\n");
	  for(i=0;i<ahp->icv_trunc_len;i++) {
	    printk("%02x ", auth_data[i]);
	    if (i && (i % 16) == 0) printk("\n");
	  }
	  printk("\n");
        }
#endif	
	/* zero out the destination auth field */
	memset(auth_data, 0, ahp->icv_trunc_len);

	/* zero out the tmp auth field */
	//memset(tmp_auth_data, 0, ahp->icv_trunc_len);
	
	/* no big keys support */
	if ( (keylen > crypto_tfm_alg_blocksize(tfm)) || (keylen & 0x07) ) {
	  dbg_msg("keylen(=%d) > algo_blocksize(%d) or unaligned keylen\n", 
		 keylen, crypto_tfm_alg_blocksize(tfm));
	  return;
	}
#if 0
	/* no packet fragments support */
	if (skb_shinfo(skb)->nr_frags != 1) {
	  dbg_msg("fragmented packets (#frags=%d) not supported\n", 
		  skb_shinfo(skb)->nr_frags);
	  return;
	}
#endif	
	len = skb->len + offset;
	len_dwords = NUM_CHUNKS(len, 3);
	len_lastdword_remainder = len & 0x07;

#ifdef DEBUG
	dbg_msg("ctrl_desc=%p, pkt_desc=%p, addr=%lx, offset=%d, len=%d, len_dwords=%d\n", 
		ctrl_desc, pkt_desc, addr, offset, skb->len, len_dwords);
#endif

	/* Construct the packet descriptor */
	pkt_desc->srcLengthIVOffUseIVNext = ( ((uint64_t)1 << 63) | 
					      ((uint64_t)1 << 62) |
					      ((uint64_t)len_lastdword_remainder << 59) |
					      (((uint64_t)len_dwords & 0xeff)<<43) |  
					      ((uint64_t)addr & ~(SMP_CACHE_BYTES - 1)) |
					      ((uint64_t)offset & 0x07)
					      );
	pkt_desc->dstLLWMask = 0;
	pkt_desc->authDst = (uint64_t)virt_to_phys(tmp_auth_data);
	pkt_desc->ckSumDst = 0;
  
	memset(ctrl_desc, 0, sizeof(ControlDescriptor_t));
	/* Construct the control descriptor */
	ctrl_desc->instruction = ( ((uint64_t)(offset >> 3) << 18) |
				   ((uint64_t)1 << 20) |
				   ((uint64_t)HASH_MD5 << 21) |
				   ((uint64_t)1 << 23) |
				   ((uint64_t)1 << 36)
				   );
	len_dwords = NUM_CHUNKS(keylen, 3);
	for(i=0;i<len_dwords;i++)
	  ctrl_desc->cipherHashInfo.infoDwords[i] = *((uint64_t *)&key[i<<3]);
	ctrl_desc_size = 9<<3;

#ifdef DEBUG
	{      
	  dbg_msg("ctrl_desc_size=%d, ctrl_desc->instr=%llx, pkt_desc<%llx,%llx,%llx,%llx>\n",
		  ctrl_desc_size, ctrl_desc->instruction, 
		  pkt_desc->srcLengthIVOffUseIVNext, pkt_desc->dstLLWMask,
		  pkt_desc->authDst, pkt_desc->ckSumDst);
	}
#endif
	
	/* Send the message to the sec_engine */
	ctrl_len_cachelines = NUM_CHUNKS(ctrl_desc_size, 5);
	stid = make_sec_desc(&msg, ctrl_desc, ctrl_len_cachelines, pkt_desc);	  

#ifdef DEBUG	
	dbg_msg("cachelines=%d, ctrl_desc=%p, pkt_desc=%p, ctrl_desc->instr=%llx, "
		"pkt_desc<%llx,%llx,%llx,%llx>\n",
		ctrl_len_cachelines, ctrl_desc, pkt_desc, ctrl_desc->instruction, 
		pkt_desc->srcLengthIVOffUseIVNext, pkt_desc->dstLLWMask,
		pkt_desc->authDst, pkt_desc->ckSumDst);
#endif
	
	msgrng_access_save(&msgrng_lock, flags, msgrng_flags);

	while (message_send(2, MSGRNG_CODE_SEC, stid, &msg));
	
	/* Wait for the response */
	for(;;) {
	  int bucket = (cpu_logical_map(smp_processor_id()) & 0x03) + 4;
	  int ctrl_err=0, pkt_err=0;
#ifdef DEBUG	  
	  dbg_msg("waiting for a response from sec engine (bucket=%x)...\n", bucket);
#endif
	  msgrng_wait(1 << bucket);

	  if (message_receive(bucket, &size, &code, &stid, &msg)) 
	    continue;

	  ctrl_desc = (ControlDescriptor_t *)phys_to_virt(msg.msg0 & 0xffffffffe0ULL);
	  pkt_desc = (PacketDescriptor_t *)phys_to_virt(msg.msg1 & 0xffffffffe0ULL);
	  
	  ctrl_err = (msg.msg0 >> 40) & 0x1ff;
	  pkt_err = (msg.msg1 >> 40) & 0x1ff;
	  if (ctrl_err || pkt_err) {
	    dbg_msg("error (ctrl_err=%x, pkt_err=%x) reported by sec_engine\n", 
		   ctrl_err, pkt_err);
	    goto out;
	  }

	  /* copy the auth result */
	  if (pkt_desc->authDst != (uint64_t)virt_to_phys(tmp_auth_data)) {
	    dbg_msg("bad authDst in sec engine response\n");
	    goto out;
	  }
	  
	  memcpy(auth_data, tmp_auth_data, ahp->icv_trunc_len);
#ifdef DEBUG
	  {
	    dbg_msg("sec engine auth result:\n");
	    for(i=0;i<ahp->icv_trunc_len;i++) {
	      printk("%02x ", auth_data[i]);
	      if (i && (i % 16) == 0) printk("\n");
	    }
	    printk("\n");
	  }
#endif	  
	  break;
	}
 out:
	msgrng_access_restore(&msgrng_lock, flags, msgrng_flags);
} 

static int __init nlm_common_sec_init(void)
{
  int i=0;
  nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_SECURITY_OFFSET);

  spin_lock_init(&msgrng_lock);

  netlogic_write_reg(mmio, SEC_DMA_CREDIT, 0x00924924);
  
  for(i=0;i<8;i++)
    netlogic_write_reg(mmio, SEC_MSG_BUCKET0_SIZE + i, bucket_sizes.bucket[MSGRNG_STNID_SEC + i]);
  
  for(i=0;i<128;i++)
    netlogic_write_reg(mmio, SEC_CC_CPU0_0 + i, cc_table_sec.counters[i>>3][i&0x07]);

  dbg_msg("Intialized Phoenix security engine\n");

  return 0;
}

module_init(nlm_common_sec_init);
