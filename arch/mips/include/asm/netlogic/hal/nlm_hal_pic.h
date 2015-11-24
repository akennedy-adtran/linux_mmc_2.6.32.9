
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


#ifndef _NLM_HAL_PIC_H
#define _NLM_HAL_PIC_H

#include "nlm_hal.h"

#define TIMER_CYCLES_MAXVAL        0xffffffffffffffffULL

/*
 *    IRT Map
 */
#define PIC_NUM_IRTS               160

#define PIC_IRT_WD_0_INDEX         0
#define PIC_IRT_WD_1_INDEX         1
#define PIC_IRT_WD_NMI_0_INDEX     2
#define PIC_IRT_WD_NMI_1_INDEX     3
#define PIC_IRT_TIMER_0_INDEX      4
#define PIC_IRT_TIMER_1_INDEX      5
#define PIC_IRT_TIMER_2_INDEX      6
#define PIC_IRT_TIMER_3_INDEX      7
#define PIC_IRT_TIMER_4_INDEX      8
#define PIC_IRT_TIMER_5_INDEX      9
#define PIC_IRT_TIMER_6_INDEX      10
#define PIC_IRT_TIMER_7_INDEX      11
#define PIC_IRT_CLOCK_INDEX        PIC_IRT_TIMER_7_INDEX

#define PIC_NUM_MSG_Q_IRTS         32
#define PIC_IRT_MSG_Q0_INDEX       12
#define PIC_IRT_MSG_Q_INDEX(qid)   ((qid) + PIC_IRT_MSG_Q0_INDEX) /* 12 - 43 */

#define PIC_IRT_MSG_0_INDEX        44
#define PIC_IRT_MSG_1_INDEX        45

#define PIC_NUM_PCIE_MSIX_IRTS     32
#define PIC_IRT_PCIE_MSIX_0_INDEX  46
#define PIC_IRT_PCIE_MSIX_INDEX(num) ((num) + PIC_IRT_PCIE_MSIX_0_INDEX) /* 46 - 77 */

#define PIC_NUM_PCIE_LINK_IRTS     4
#define PIC_IRT_PCIE_LINK_0_INDEX  78
#define PIC_IRT_PCIE_LINK_INDEX(num) ((num) + PIC_IRT_PCIE_LINK_0_INDEX) /* 78 - 81 */

#define PIC_NUM_NA_IRTS            32
#define PIC_IRT_NA_0_INDEX         82
#define PIC_IRT_NA_INDEX(num)      ((num) + PIC_IRT_NA_0_INDEX) /* 82 - 113 */

#define PIC_IRT_POE_INDEX          114

#define PIC_NUM_USB_IRTS           6
#define PIC_IRT_USB_0_INDEX        115
#define PIC_IRT_USB_INDEX(num) ((num) + PIC_IRT_USB_0_INDEX) /* 115 - 120 */

#define PIC_IRT_GDX_INDEX          121
#define PIC_IRT_SEC_INDEX          122
#define PIC_IRT_RSA_INDEX          123

#define PIC_NUM_COMP_IRTS          4
#define PIC_IRT_COMP_0_INDEX       124
#define PIC_IRT_COMP_INDEX(num)    ((num) + PIC_IRT_COMP_0_INDEX) /* 124 - 127 */

#define PIC_IRT_ICC_0_INDEX        129 /* ICC - Inter Chip Coherency */
#define PIC_IRT_ICC_1_INDEX        130
#define PIC_IRT_ICC_2_INDEX        131
#define PIC_IRT_CAM_INDEX          132
#define PIC_IRT_UART_0_INDEX       133
#define PIC_IRT_UART_1_INDEX       134
#define PIC_IRT_I2C_0_INDEX        135
#define PIC_IRT_I2C_1_INDEX        136
#define PIC_IRT_SYS_0              137
#define PIC_IRT_SYS_1              138
#define PIC_IRT_JTAG_INDEX         139
#define PIC_IRT_PIC                140

#define PIC_NUM_GPIO_IRTS          4
#define PIC_IRT_GPIO_0_INDEX       146
#define PIC_IRT_GPIO_INDEX(num)    ((num) + PIC_IRT_GPIO_0_INDEX) /* 146 - 149 */

#define PIC_IRT_NOR                150
#define PIC_IRT_NAND               151
#define PIC_IRT_SPI                152
#define PIC_IRT_MMC                153
#define PIC_IRT_NBU                154
#define PIC_IRT_TCU                155
#define PIC_IRT_GCU                156 /* GBC - Global Coherency */
#define PIC_IRT_DMC_0_INDEX        157
#define PIC_IRT_DMC_1_INDEX        158
#define PIC_IRT_TCB                159

/*
 *     Register Offsets
 */
#define PIC_CTRL             0x00
#define PIC_BYTESWAP         0x01
#define PIC_STATUS           0x02
#define PIC_INT_TIMEOUT      0x03
#define PIC_ICI0_INT_TIMEOUT 0x04
#define PIC_ICI1_INT_TIMEOUT 0x05
#define PIC_ICI2_INT_TIMEOUT 0x06
#define PIC_IPI_CTL          0x07
#define PIC_INT_ACK          0x08
#define PIC_INT_PENDING0     0x09
#define PIC_INT_PENDING1     0x0a
#define PIC_INT_PENDING2     0x0b

#define PIC_WD0_MAX_VAL      0x0c
#define PIC_WD0_COUNT        0x0d
#define PIC_WD0_MASK_0       0x0e
#define PIC_WD0_MASK_1       0x0f
#define PIC_WD0_HEARBEATCMD  0x10
#define PIC_WD0_HEARBEAT_0   0x11
#define PIC_WD0_HEARBEAT_1   0x12

#define PIC_WD_MAX_VAL(id)    (PIC_WD0_MAX_VAL + ((id) ? 7 : 0))
#define PIC_WD_COUNT(id)      (PIC_WD0_COUNT + ((id) ? 7 : 0))
#define PIC_WD_MASK_0(id)     (PIC_WD0_MASK_0 + ((id) ? 7 : 0))
#define PIC_WD_MASK_1(id)     (PIC_WD0_MASK_1 + ((id) ? 7 : 0))
#define PIC_WD_HEARBEAT_0(id) (PIC_WD0_HEARTBEAT_0 + ((id) ? 7 : 0))
#define PIC_WD_HEARBEAT_1(id) (PIC_WD0_HEARTBEAT_1 + ((id) ? 7 : 0))

#define PIC_SYS_TIMER_0_MAX_VAL   0x1a
#define PIC_SYS_TIMER_MAX_VAL(id) (PIC_SYS_TIMER_0_MAX_VAL + (id))

#define PIC_SYS_TIMER_0_COUNTER   0x22
#define PIC_SYS_TIMER_COUNTER(id) (PIC_SYS_TIMER_0_COUNTER + (id))

#define PIC_TIMER_0_MAXVAL   PIC_SYS_TIMER_0_MAX_VAL
#define PIC_TIMER_0_COUNTER  PIC_SYS_TIMER_0_COUNTER
#define PIC_TIMER_7_MAXVAL   PIC_SYS_TIMER_MAX_VAL(7)
#define PIC_TIMER_7_COUNTER  PIC_SYS_TIMER_COUNTER(7)
#define PIC_TIMER_6_MAXVAL   PIC_SYS_TIMER_MAX_VAL(6)
#define PIC_TIMER_6_COUNTER  PIC_SYS_TIMER_COUNTER(6)

#define PIC_INT_THR_ENABLE_0_N01   0x2a
#define PIC_INT_THR_ENABLE_0_N23   0x2b
#define PIC_INT_THR_ENABLE_N01(id) (PIC_INT_THR_ENABLE_0_N01 + ((id) * 2))
#define PIC_INT_THR_ENABLE_N23(id) (PIC_INT_THR_ENABLE_0_N23 + ((id) * 2))

#define PIC_IRT_0   0x3a
#define PIC_IRT(id) (PIC_IRT_0 + (id))

#define PIC_IRT_WD_0        PIC_IRT(PIC_IRT_WD0_INDEX)
#define PIC_IRT_WD_1        PIC_IRT(PIC_IRT_WD1_INDEX)
#define PIC_IRT_TIMER_0     PIC_IRT(PIC_IRT_TIMER_0_INDEX)
#define PIC_IRT_TIMER_1     PIC_IRT(PIC_IRT_TIMER_1_INDEX)
#define PIC_IRT_TIMER_2     PIC_IRT(PIC_IRT_TIMER_2_INDEX)
#define PIC_IRT_TIMER_3     PIC_IRT(PIC_IRT_TIMER_3_INDEX)
#define PIC_IRT_TIMER_4     PIC_IRT(PIC_IRT_TIMER_4_INDEX)
#define PIC_IRT_TIMER_5     PIC_IRT(PIC_IRT_TIMER_5_INDEX)
#define PIC_IRT_TIMER_6     PIC_IRT(PIC_IRT_TIMER_6_INDEX)
#define PIC_IRT_TIMER_7     PIC_IRT(PIC_IRT_TIMER_7_INDEX)
#define PIC_IRT_CLOCK       PIC_IRT_TIMER_7
#define PIC_IRT_UART_0      PIC_IRT(PIC_IRT_UART_0_INDEX)
#define PIC_IRT_UART_1      PIC_IRT(PIC_IRT_UART_1_INDEX)
#define PIC_IRT_I2C_0       PIC_IRT(PIC_IRT_I2C_0_INDEX)
#define PIC_IRT_I2C_1       PIC_IRT(PIC_IRT_I2C_1_INDEX)

#define PIC_CLOCK_TIMER     7
#define PIC_IRQ_BASE        8

#define ASM_XLP_IO_PIC_OFFSET        0xffffffffb8004100 /* TODO: This will change in to function */
#define C_XLP_IO_PIC_OFFSET        0xffffffffb8004100ULL /* TODO: This will change in to function */

#ifndef __ASSEMBLY__
enum {
	WD0 = 0,
	WD1 = 1
};
extern int irt_irq_table[PIC_NUM_IRTS][4];
extern int find_irt_from_irq(int irq_num);
extern int nlm_hal_request_shared_irq(int irt);
extern void nlm_hal_unrequest_shared_irq(int irt);

static __inline__ int nlm_hal_irt_to_irq(int irt_num)
{
	if(irt_num < 0 || irt_num > PIC_NUM_IRTS)
		return -1;

	return irt_irq_table[irt_num][0];
}

static __inline__ int nlm_hal_irq_to_irt(int irq_num)
{
	int irt = find_irt_from_irq(irq_num);
	return irt;
}

static __inline__ int nlm_hal_is_shared_irt(int irt_num)
{
	return irt_irq_table[irt_num][1];
}

#define PIC_IRT_FIRST_IRQ        (PIC_IRQ_BASE)
#define PIC_WD_0_IRQ             nlm_hal_irt_to_irq(PIC_IRT_WD_0_INDEX)
#define PIC_WD_1_IRQ             nlm_hal_irt_to_irq(PIC_IRT_WD_1_INDEX)
#define PIC_TIMER_0_IRQ          nlm_hal_irt_to_irq(PIC_IRT_TIMER_0_INDEX)
#define PIC_TIMER_1_IRQ          nlm_hal_irt_to_irq(PIC_IRT_TIMER_1_INDEX)
#define PIC_TIMER_2_IRQ          nlm_hal_irt_to_irq(PIC_IRT_TIMER_2_INDEX)
#define PIC_TIMER_3_IRQ          nlm_hal_irt_to_irq(PIC_IRT_TIMER_3_INDEX)
#define PIC_TIMER_4_IRQ          nlm_hal_irt_to_irq(PIC_IRT_TIMER_4_INDEX)
#define PIC_TIMER_5_IRQ          nlm_hal_irt_to_irq(PIC_IRT_TIMER_5_INDEX)
#define PIC_TIMER_6_IRQ          nlm_hal_irt_to_irq(PIC_IRT_TIMER_6_INDEX)
#define PIC_TIMER_7_IRQ          nlm_hal_irt_to_irq(PIC_IRT_TIMER_7_INDEX)
#define PIC_CLOCK_IRQ            (PIC_TIMER_7_IRQ)
#define PIC_UART_0_IRQ           17
#define PIC_UART_1_IRQ           18
#define PIC_I2C_0_IRQ            nlm_hal_irt_to_irq(PIC_IRT_I2C_0_INDEX)
#define PIC_I2C_1_IRQ            nlm_hal_irt_to_irq(PIC_IRT_I2C_1_INDEX)
#define PIC_GPIO_IRQ(num)        nlm_hal_irt_to_irq(PIC_IRT_GPIO_INDEX(num))
#define PIC_IRT_LAST_IRQ_        (PIC_IRQ_BASE + PIC_NUM_IRTS - 1)
#define PIC_IRT_LAST_IRQ()       PIC_IRT_LAST_IRQ_

/*
 *   Misc
 */
#define IRT_VALID       	1
#define LOCAL_SCHEDULING    1
#define GLOBAL_SCHEDULING   0
#define PIC_IRQ_IS_IRT(irq) ((irq >= PIC_IRT_FIRST_IRQ) && (irq <= PIC_IRT_LAST_IRQ_))
#define PIC_IRQ_IS_EDGE_TRIGGERED(irq) 0 /* XLP interrupts are level triggered */

/*
 *
 */

#define NODE_OFFSET(node) ((node) << 18)
#define CPU_TO_NODE(cpu) ((cpu) >> 5)

static __inline__ int nlm_hal_cpu_id(void)
{
	int cpu;

	__asm__ __volatile__ (
		".set push\n"
		".set noreorder\n"
		".set mips32\n"
		"mfc0 %0, $15, 1\n"
		"andi %0, %0, 0x3ff\n"
		".set pop\n"
		: "=r"(cpu)
		);

	return cpu;
}

#define XLP_IO_PIC_OFFSET        C_XLP_IO_PIC_OFFSET

typedef volatile unsigned long long pic_reg_t;

static __inline__ pic_reg_t* nlm_hal_pic_offset(void)
{
	uint32_t cpu = nlm_hal_cpu_id();

	return ( (pic_reg_t *) (unsigned long) (XLP_IO_PIC_OFFSET + NODE_OFFSET( CPU_TO_NODE(cpu) )) );
}

#ifdef CONFIG_64BIT

static __inline__ void nlm_hal_write_pic_reg(pic_reg_t *base, unsigned int offset, unsigned long long value)
{
	base[offset] = value;
}
static __inline__ unsigned long long nlm_hal_read_pic_reg(pic_reg_t *base, unsigned int offset)
{
	return ((base)[offset]);
}

#else

static __inline__ void nlm_hal_write_pic_reg(pic_reg_t *base, unsigned int offset, unsigned long long value)
{
        uint32_t lsw, msw;
        uint64_t val;
        uint32_t ls, ms;
        unsigned long flags;

        lsw = (uint32_t) ((unsigned long)base + offset);
        msw = (uint32_t) 0xffffffffUL;
        val = (uint64_t)value;

        ls = (uint32_t) (val & 0xffffffff);
        ms = (uint32_t) (val >> 32);

        enable_KX(flags);
        __asm__ __volatile__(".set push\n"
                        ".set noreorder\n"
                        ".set mips64\n"
                        ".set noat\n"
                        "dsll32 $1, %2, 0\n"
                        "dsll32 %1, 0\n"
                        "dsrl32 %1, 0\n"
                        "or $1, $1, %1\n"
                        "dsll32 $8, %4, 0\n"
                        "dsll32 %3, 0\n"
                        "dsrl32 %3, 0\n"
                        "or $8, $8, %3\n"
                        "sd $8, 0($1) \n"
                        ".set at\n"
                        ".set pop\n"
                        :
                        :"r"(val), "r"(lsw), "r"(msw), "r"(ls), "r"(ms)
                        :"$1", "$8");
        disable_KX(flags);
}

static __inline__ unsigned long long nlm_hal_read_pic_reg(pic_reg_t *base, unsigned int offset)
{
        uint32_t lsw, msw;
        uint64_t value = 0;
        uint32_t lo, hi;
        unsigned long flags;

        lsw = (uint32_t) ((unsigned long)base + offset);
        msw = (uint32_t) 0xffffffffUL;

        enable_KX(flags);
        __asm__ __volatile__(".set push\n"
                        ".set noreorder\n"
                        ".set mips64\n"
                        ".set noat\n"
                        "dsll32 $1, %3, 0\n"
                        "dsll32 %2, 0\n"
                        "dsrl32 %2, 0\n"
                        "or $1, $1, %2\n"
                        "ld $8, 0($1) \n"
                        "dsrl32 %1, $8, 0\n"
                        "dsll32 $8, $8, 0\n"
                        "dsrl32 %0, $8, 0\n"
                        ".set at\n"
                        ".set pop\n"
                        :"=r"(lo), "=r"(hi)
                        :"r"(lsw), "r"(msw)
                        :"$1", "$8");

        disable_KX(flags);
        value = hi;
        value = (uint64_t) ((value<<32) | lo);
        return (value);
}

#endif /* #ifdef CONFIG_64BIT */
static __inline__ void nlm_hal_pic_send_ipi(int nmi, int vec, int node, int cpu)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	unsigned long long ipi = (nmi << 31) | (vec << 20) | (node << 17) | (1 << (cpu & 0xf));
	if (cpu > 15) {
		ipi |= 0x10000; /* Setting bit 16 to select cpus 16-31 */
	}

	nlm_hal_write_pic_reg(mmio, PIC_IPI_CTL, ipi);
}

static __inline__ unsigned long long nlm_hal_pic_read_control(void)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	return nlm_hal_read_pic_reg(mmio, PIC_CTRL);
}

static __inline__ void nlm_hal_pic_write_control(unsigned long long control)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	nlm_hal_write_pic_reg(mmio, PIC_CTRL, control);
}

static __inline__ void nlm_hal_pic_update_control(unsigned long long control)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	nlm_hal_write_pic_reg(mmio, PIC_CTRL, (control | nlm_hal_read_pic_reg(mmio, PIC_CTRL)));
}

static __inline__ void nlm_hal_ack_pic(int irt_num)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	nlm_hal_write_pic_reg(mmio, PIC_INT_ACK, irt_num);

	/* Ack the Status register for Watchdog & System timers */
	if (irt_num < 12) {
		nlm_hal_write_pic_reg(mmio, PIC_STATUS, (1 << irt_num));
	}
}

static __inline__ unsigned long long nlm_hal_pic_read_irt(int irt_num)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	return nlm_hal_read_pic_reg(mmio, PIC_IRT(irt_num));
}

static __inline__ void nlm_hal_pic_write_irt(int irt_num, int en, int nmi, int sch, int vec, int dt, int db, int dte)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	unsigned long long val = (((en & 0x1) << 31) | ((nmi & 0x1) << 29) | ((sch & 0x1) << 28) |
				  ((vec & 0x3f) << 20) | ((dt & 0x1 ) << 19) | ((db & 0x7) << 16) |
				  (dte & 0xffff));

	nlm_hal_write_pic_reg(mmio, PIC_IRT(irt_num), val);
}

#define CPUIDBITS01(X) ((X) & 0x3)
#define CPUIDBIT2(X) ((X >> 2) & 0x1)

static __inline__ void nlm_hal_pic_write_irt_direct(int irt_num, int en, int nmi, int sch, int vec, int cpu)
{
	nlm_hal_pic_write_irt(irt_num, en, nmi, sch, vec, 1, CPUIDBIT2(cpu), CPUIDBITS01(cpu));
	/* Does not support multi node support yet */
}

static __inline__ unsigned long long nlm_hal_pic_read_timer(int timer)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	return nlm_hal_read_pic_reg(mmio, PIC_SYS_TIMER_COUNTER(timer));
}

static __inline__ void nlm_hal_pic_write_timer(int timer, pic_reg_t value)
{
	pic_reg_t *mmio = nlm_hal_pic_offset();

	nlm_hal_write_pic_reg(mmio, PIC_SYS_TIMER_COUNTER(timer), value);
}

#endif /* __ASSEMBLY__ */

#endif /* _NLM_HAL_PIC_H */
