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
 * RapidIO Tsi578 switch support
 *
 */

#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/rio_ids.h>
#include "../rio.h"
#include <linux/delay.h>

#ifdef NLM_SRIO_DEBUG 
#define Message(a,b...) printk("\nFunction[%s]-Line[%d] \n"a"\n",__FUNCTION__,__LINE__,##b)
#else
#define Message(a,b...)
#endif

static int init_done = 0;
static int
tsi578_route_add_entry(struct rio_mport *mport, u16 destid, u8 hopcount, u16 table, u16 route_destid, u8 route_port)
{
	u32 offset0 = 0x70;
	u32 offset1 = 0x74;
    uint32_t data = 0;
    Message("route_destid = %#x",route_destid);
    Message("destid=%d, hopcount=%d, table=%d, route_port=%d",
            destid,hopcount,table,route_port);

    if(!init_done){
        rio_mport_read_config_32(mport, 0xffff, 0, 0x10004, &data);
        data = data & ~(1<<24);
        rio_mport_write_config_32(mport, 0xffff, 0, 0x10004, data);
        init_done = 1; 
    }

	if (table == 0xff) {
        Message("Writing in to per port route table..");
        Message("Writing @ Offset %#x and Offset %#x",offset0, offset1);
        rio_mport_write_config_32(mport, destid, hopcount, offset0, 
                                    route_destid);
        rio_mport_write_config_32(mport, destid, hopcount, offset1, 
                                    route_port);
	}
	return 0;
}

static int
tsi578_route_get_entry(struct rio_mport *mport, u16 destid, u8 hopcount, u16 table, u16 route_destid, u8 *route_port)
{
	u32 offset0 = 0x70;
	u32 offset1 = 0x74;
    u32 result = 0;
    Message("route_destid = %#x",route_destid);
    Message("destid=%d, hopcount=%d, table=%d, route_port=%d",
            destid,hopcount,table,route_port);

    if(route_destid > 15)
       return -1; 

	if (table == 0xff) {
        Message("Reading from switch..");
	    rio_mport_write_config_32(mport, destid, hopcount, offset0, 
                                    route_destid);
	    rio_mport_read_config_32(mport, destid, hopcount, offset1, &result);
        *route_port = (u8)result;
	    return 0;
	}
    return -1;
}

DECLARE_RIO_ROUTE_OPS(RIO_VID_TUNDRA, RIO_DID_TSI578, tsi578_route_add_entry, tsi578_route_get_entry);
