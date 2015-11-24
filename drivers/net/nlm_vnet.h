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
 * $Id: $
 *
 * nlm_vnet.h - Shared memory based virtual network driver defines
 *
 */

#ifndef _NLM_VNET_H
#define _NLM_VNET_H


extern uint32_t rmik_en;
extern int nlm_vnet_debug;

#define VNET_DBG(format, args...) \
    if (nlm_vnet_debug) printk(KERN_DEBUG format, ##args );

#define NLM_VNET_TIMEOUT	5

static inline nlm_addr_t nlm_vnet_dom_map_get_addr(void)
{
	nlm_addr_t addr = 0x0ULL;
	nlm_resource_t res;

	if (nlm_resource_getref("vnet", &res, NULL) != 0) {
		return 0;
	}

	if (nlm_get_addr_property(res, "dom-map", &addr) < 0) {
		nlm_resource_putref(res);
		return 0;
	}

	nlm_resource_putref(res);
	return addr;

}

static inline void nlm_vnet_dom_map_set_clear_bit(nlm_addr_t vnet_dom_map_addr,
						  nlm_dom_t domid, int set)
{
	if (domid < 0 || domid >= NLM_MAX_DOMAINS)
		return;

	if (set) {
		xlr_atomic_bit_set_u64(domid,
				       nlm_addr_to_ptr(vnet_dom_map_addr));
	} else {
		xlr_atomic_bit_clear_u64(domid,
					 nlm_addr_to_ptr(vnet_dom_map_addr));
	}
}


static inline nlm_addr_t nlm_vnet_promisc_map_get_addr(void)
{
	nlm_addr_t addr = 0x0ULL;
	nlm_resource_t res;

	if (nlm_resource_getref("vnet", &res, NULL) != 0) {
		return 0;
	}

	if (nlm_get_addr_property(res, "promisc-dom-map", &addr) < 0) {
		nlm_resource_putref(res);
		return 0;
	}

	nlm_resource_putref(res);
	return addr;

}

static inline void nlm_vnet_promisc_map_set_clear_bit(
					    nlm_addr_t promisc_dom_map_addr,
					    nlm_dom_t domid, int set)
{
	if (domid < 0 || domid >= NLM_MAX_DOMAINS)
		return;

	if (set) {
		xlr_atomic_bit_set_u64(domid,
				       nlm_addr_to_ptr(promisc_dom_map_addr));
	} else {
		xlr_atomic_bit_clear_u64(domid,
					 nlm_addr_to_ptr(promisc_dom_map_addr));
	}
}

#endif /* _NLM_VNET_H */
