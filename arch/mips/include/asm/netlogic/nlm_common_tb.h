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

#ifndef _NLM_NETLOGIC_TB_H
#define _NLM_NETLOGIC_TB_H

#include <linux/types.h>
#include <asm/netlogic/iomap.h>
#include <linux/nlm_common_tb.h>

#define TB_REG_SIZE				4
#define TB_NO_RDDATA_REGS		4
#define TB_MAX_ENTRIES			256
#define TB_ENTRY_SIZE		(TB_NO_RDDATA_REGS * TB_REG_SIZE)
#define TB_SIZE				(TB_MAX_ENTRIES * TB_ENTRY_SIZE)

/* ---------------------------------------------------------------------------- */
/*                             RD/WR macros                                     */
/* ---------------------------------------------------------------------------- */

static inline unsigned int tb_read_reg_be32(unsigned int reg)
{
    nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_TB_OFFSET);
    return netlogic_read_reg(mmio, reg);
}

static inline void tb_write_reg_be32 (unsigned int reg, unsigned int value)
{
    nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_TB_OFFSET);
    netlogic_write_reg(mmio, reg, value);
}

static inline unsigned int tb_read_reg_le32 (unsigned int reg)
{
    nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_TB_OFFSET);
    return netlogic_read_reg_le32(mmio, reg);
}

static inline void tb_write_reg_le32 (unsigned int reg, unsigned int value)
{
    nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_TB_OFFSET);
    netlogic_write_reg_le32(mmio, reg, value);
}

#define tb_read_status_reg() tb_read_reg_be32(TB_STATUS_REG)
#define tb_read_ctrl_reg() tb_read_reg_be32(TB_CTRL_REG)

#define tb_read_reqmatch_reg(i) tb_read_reg_be32 (TB_REQMATCH_REGS + i)
#define tb_read_raddr_reg(i) tb_read_reg_be32 (TB_RADDR_REGS + i)
#define tb_read_rddata_reg(i) tb_read_reg_be32 (TB_RDDATA_REGS + i)

#define tb_write_ctrl_reg(val) tb_write_reg_be32 (TB_CTRL_REG, val)
#define tb_write_reqmatch_reg(i, val) tb_write_reg_be32 ((TB_REQMATCH_REGS+i), val)
#define tb_write_raddr_reg(i, val) tb_write_reg_be32 ((TB_RADDR_REGS+i), val)

#define tb_reinit(void) {tb_write_reg_be32(TB_INIT_REG, 0x0);tb_write_reg_be32(TB_INIT_REG, 0x1);}
#define tb_pop_entry(void) tb_write_reg_be32(TB_ACCESS_REG, 0)
#define disable_tb()    tb_write_reg_be32(TB_CTRL_REG, 0x01000000)

typedef struct _tb_dev_t {
	unsigned int	size;
//	unsigned char   data[TB_SIZE];
	unsigned char   *data;
} tb_dev_t;

ssize_t	tb_read (struct file *, char *, size_t, loff_t *);
int		tb_open (struct inode *, struct file *);
int		tb_ioctl (struct inode *, struct file *, unsigned int, unsigned long);
int		tb_release (struct inode *, struct file *);

#endif
