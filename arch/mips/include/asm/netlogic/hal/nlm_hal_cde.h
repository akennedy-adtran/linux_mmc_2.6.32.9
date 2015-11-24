
/*-
 * Copyright (c) 2003-2012 Broadcom Corporation
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


#ifndef _NLM_HAL_CDE_H_
#define _NLM_HAL_CDE_H_

#include "nlm_hal.h"

#define nlm_hal_write_cde_reg(reg, val) nlm_hal_write_32bit_reg \
					(nlm_hal_get_dev_base \
					(XLP_CDE_NODE, XLP_CDE_BUS, XLP_CDE_DEVICE, XLP_CDE_FUNC), \
					 (reg), (val))
#define nlm_hal_read_cde_reg(reg) nlm_hal_read_32bit_reg( \
					nlm_hal_get_dev_base \
					(XLP_CDE_NODE, XLP_CDE_BUS, XLP_CDE_DEVICE, XLP_CDE_FUNC), \
					 (reg))

static __inline__ int nlm_hal_cde_make_fd_msg(uint64_t *msg0, uint64_t free_addr)
{
  int stid = XLP_STNID_CMP;

  *msg0 = ( ((uint64_t)free_addr & 0xffffffffffULL)
                );
  return stid;
}

static __inline__ int nlm_hal_cde_make_cmp_msg(uint64_t *msg0, int readp, int op, int rtn_bkt,
				   int length, uint64_t src_addr)
{
  int stid = XLP_STNID_CMP;
  
  *msg0 = ( ((uint64_t) readp <<63) |
		((uint64_t) op << 62)  | 
                ((uint64_t) rtn_bkt << 50) |
                ((uint64_t) length << 40) |
                ((uint64_t) src_addr & 0xffffffffffULL)
                );
  return stid;
}


static __inline__ uint64_t nlm_hal_cde_make_data_desc(int eof, int type, int sod, int sob, int save,
					 int restore, int eob, int length, uint64_t src_addr)
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
                ((uint64_t) src_addr & 0xffffffffffULL)
                );
  return src_desc;
}

/* Return error when parameter are invalid */
static __inline__ int nlm_hal_cde_send_request(uint32_t dst_vc, uint64_t entry0)
{
	return nlm_hal_send_msg1(dst_vc, 0, entry0);

}

static __inline__ void nlm_hal_cde_receive_response(uint32_t rx_vc, uint64_t *entry0, uint64_t *entry1)
{
        uint32_t size = 0, code = 0, src = 0;
/*        uint64_t entry0 = 0, entry1 = 0; */

        nlm_hal_recv_msg2(rx_vc, &src, &size, &code, entry0, entry1);
}
#endif				/*#ifndef _NLM_HAL_CDE_H_ */

