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


#include <linux/kernel.h>
#include <linux/gfp.h>

#include <linux/skbuff.h>
#include <linux/netdevice.h>

#include "os_layer.h"

#define NLM_RX_BUF_SIZE (MAX_FRAME_SIZE+SPI4_BYTE_OFFSET+MAC_PREPAD+ \
		                MAC_SKB_BACK_PTR_SIZE+SMP_CACHE_BYTES)


void os_free_buffer(char *addr){
	kfree(addr);
	return;
}

char*	os_malloc(unsigned long size){
	return	kmalloc(size, GFP_KERNEL);

}


char*	os_malloc_buffer(){

	return  kmalloc(NLM_RX_BUF_SIZE, GFP_KERNEL);

}



void* os_cacheline_aligned_kmalloc(int size){

	void *buf = kmalloc(size+SMP_CACHE_BYTES, GFP_KERNEL);
	if (buf){
		buf = (void *)(CACHELINE_ALIGNED_ADDR(
			(unsigned long)buf+SMP_CACHE_BYTES));
	}
	return buf;
}


struct sk_buff *os_alloc_skb(void){

	int offset=0;
	struct sk_buff *skb = __dev_alloc_skb(NLM_RX_BUF_SIZE, GFP_KERNEL);

	if (!skb) {
		return NULL;
	}

	/* align the data to the next cache line */
	offset = ((unsigned long)skb->data + SMP_CACHE_BYTES) &
		~(SMP_CACHE_BYTES - 1);
	skb_reserve(skb, (offset - (unsigned long)skb->data));

	return skb;
}


void os_free(struct sk_buff *skb){

	//	dev_kfree_skb(skb);
	dev_kfree_skb_irq(skb);

	return;
}
