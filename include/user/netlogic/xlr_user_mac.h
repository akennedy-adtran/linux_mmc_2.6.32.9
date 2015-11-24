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


#ifndef __USER_NLM_XLR_USER_MAC_H
#define __USER_NLM_XLR_USER_MAC_H

#include <asm/ioctl.h>

#define USER_MAC_IOC_MAGIC 'M'

#define    USER_MAC_IOC_GSHMPHYS            _IOR(USER_MAC_IOC_MAGIC, 1, unsigned int)
#define    USER_MAC_IOC_GSHMVIRT            _IOR(USER_MAC_IOC_MAGIC, 2, unsigned int)
#define    USER_MAC_IOC_GSHMSIZE            _IOR(USER_MAC_IOC_MAGIC, 3, unsigned int)
#define    USER_MAC_IOC_GMMAP_START         _IOR(USER_MAC_IOC_MAGIC, 4, unsigned int)
#define    USER_MAC_IOC_SWRITE_REG          _IOW(USER_MAC_IOC_MAGIC, 10, unsigned int)
#define    USER_MAC_IOC_GREAD_REG           _IOR(USER_MAC_IOC_MAGIC, 11, unsigned int)
#define    USER_MAC_IOC_SPERF               _IOW(USER_MAC_IOC_MAGIC, 12, unsigned int)
#define    USER_MAC_IOC_GPHYS_CPU_PRESENT_MAP _IOR(USER_MAC_IOC_MAGIC, 13, unsigned int)
#define    USER_MAC_IOC_GCPU_ONLINE_MAP     _IOR(USER_MAC_IOC_MAGIC, 14, unsigned int)
#define    USER_MAC_IOC_HYBRID_MODE_SETUP   _IOR(USER_MAC_IOC_MAGIC, 15, unsigned int)
#define    USER_MAC_IOC_HUGETLB_SHM_VIRT_ADDR _IOR(USER_MAC_IOC_MAGIC, 16, unsigned int)
#define    USER_MAC_IOC_EARLY_MEM_INIT		_IOR(USER_MAC_IOC_MAGIC, 17, unsigned int)

#define NLM_USER_MAC_CHRDEV_NAME "xlr_user_mac"

#define MAX_USER_MAC_PKTS 3072
#define MAX_USER_MAC_FRIN_PKTS (MAX_USER_MAC_PKTS - 256)
#define USER_MAC_FIFO_SIZE 128
#define USER_MAC_PKT_BUF_SIZE 1600

struct packet_data {
  unsigned char data[USER_MAC_PKT_BUF_SIZE];
};

struct packet_desc {
  unsigned int offset;
  int len;
  int port;
  int type;
  int xgmac; //ignore in gmac. 1 xgmac loopback, 2, xgmac crossover
  int device; //0 xgmac0, 1 xgmac1
  int free;
  unsigned char priv[48];
  uint64_t priv_ptr;	//uint32_t *priv_ptr;
};

#define USER_MAC_TXQ_FREE 0
#define USER_MAC_TXQ_TX 1
#define USER_MAC_TXQ_HOST 2

struct user_mac_time {
  unsigned int hi;
  unsigned int lo;
};
struct user_mac_data {
  struct packet_data pkt_data[MAX_USER_MAC_PKTS];
  struct packet_desc pkt_desc[MAX_USER_MAC_PKTS];
  struct user_mac_time time;
  struct timespec ktime;
  int host_pkt_next_free[32];
};

/* copy all the user_mac_data which are accessed from the kernal to user_mac_kernal_data */
struct user_mac_kernal_data {
	struct user_mac_time time;
	struct timespec ktime;
};

static __inline__ unsigned char *user_mac_host_pkt_alloc(struct user_mac_data *user_mac, int cpu)
{
	int num_pkts = (MAX_USER_MAC_PKTS - MAX_USER_MAC_FRIN_PKTS) / 32;
	int start_index = MAX_USER_MAC_FRIN_PKTS + (cpu * num_pkts);
	int next_free = user_mac->host_pkt_next_free[cpu];
	int i=0;

	if (next_free < start_index || next_free >= (start_index + num_pkts)) 
		return NULL;
	
	for (i=next_free; i<(start_index+num_pkts) ;i++) {
		if (user_mac->pkt_desc[i].free) {
			user_mac->pkt_desc[i].free = 0;
			user_mac->host_pkt_next_free[cpu] = i;
			return user_mac->pkt_data[i].data;
		}
	}
	
	for (i=start_index; i<next_free; i++) {
		if (user_mac->pkt_desc[i].free) {
			user_mac->pkt_desc[i].free = 0;
			user_mac->host_pkt_next_free[cpu] = i;
			return user_mac->pkt_data[i].data;
		}		
	}

	return NULL;
}

static __inline__ int user_mac_host_pkt_free(struct user_mac_data *user_mac, int index, int cpu)
{
	/* This function can be called from any cpu */
	if (index < MAX_USER_MAC_FRIN_PKTS || index >= MAX_USER_MAC_PKTS)
		return -1;
	
	if (user_mac->pkt_desc[index].free) return -1;

	user_mac->pkt_desc[index].free = 1;

	return 0;
}

#endif
