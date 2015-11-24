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


#ifndef _ASM_RFI_IO_H
#define _ASM_RFI_IO_H

#if defined(CONFIG_NLM_XLR)
#define DEFAULT_NETLOGIC_IO_BASE 0xffffffffbef00000ULL
#define NETLOGIC_IO_DDR2_CHN0_OFFSET       0x01000
#define NETLOGIC_IO_DDR2_CHN1_OFFSET       0x02000
#define NETLOGIC_IO_DDR2_CHN2_OFFSET       0x03000
#define NETLOGIC_IO_DDR2_CHN3_OFFSET       0x04000
#define NETLOGIC_IO_PIC_OFFSET             0x08000
#define NETLOGIC_IO_UART_0_OFFSET          0x14000
#define NETLOGIC_IO_UART_1_OFFSET          0x15100

#else
#define DEFAULT_NETLOGIC_IO_BASE 0xffffffffb8000000ULL
#define NETLOGIC_IO_DDR2_CHN0_OFFSET       0x14000
#define NETLOGIC_IO_DDR2_CHN1_OFFSET       0x15000
#define NETLOGIC_IO_DDR2_CHN2_OFFSET       0x16000
#define NETLOGIC_IO_DDR2_CHN3_OFFSET       0x17000
#define NETLOGIC_IO_PIC_OFFSET             0x04000
#define NETLOGIC_IO_UART_0_OFFSET          0x30100
#define NETLOGIC_IO_UART_1_OFFSET          0x31100
#define NETLOGIC_IO_SYS_OFFSET             0x35100
#endif /* CONFIG_NLM_XLP */

#define NETLOGIC_IO_SIZE                   0x1000

#define NETLOGIC_IO_BRIDGE_OFFSET          0x00000

#define NETLOGIC_IO_RLD2_CHN0_OFFSET       0x05000
#define NETLOGIC_IO_RLD2_CHN1_OFFSET       0x06000

#define NETLOGIC_IO_SRAM_OFFSET            0x07000

#define NETLOGIC_IO_PCIX_OFFSET            0x09000
#define NETLOGIC_IO_HT_OFFSET              0x0A000

#define NETLOGIC_IO_SECURITY_OFFSET        0x0B000

#define NETLOGIC_IO_GMAC_0_OFFSET          0x0C000
#define NETLOGIC_IO_GMAC_1_OFFSET          0x0D000
#define NETLOGIC_IO_GMAC_2_OFFSET          0x0E000
#define NETLOGIC_IO_GMAC_3_OFFSET          0x0F000

#if defined(XLS) || defined(CONFIG_NLM_XLP)
#define NETLOGIC_IO_GMAC_4_OFFSET          0x20000
#define NETLOGIC_IO_GMAC_5_OFFSET          0x21000
#define NETLOGIC_IO_GMAC_6_OFFSET          0x22000
#define NETLOGIC_IO_GMAC_7_OFFSET          0x23000

#define NETLOGIC_IO_PCIE_0_OFFSET          0x1E000
#define NETLOGIC_IO_PCIE_1_OFFSET          0x1F000
#define NETLOGIC_IO_SRIO_0_OFFSET          0x1E000
#define NETLOGIC_IO_SRIO_1_OFFSET          0x1F000

#define NETLOGIC_IO_USB_0_OFFSET           0x24000
#define NETLOGIC_IO_USB_1_OFFSET           0x25000

#define NETLOGIC_IO_COMP_OFFSET            0x1D000

#endif /* XLS or XLP */

#define NETLOGIC_IO_SPI4_0_OFFSET          0x10000
#define NETLOGIC_IO_XGMAC_0_OFFSET         0x11000
#define NETLOGIC_IO_SPI4_1_OFFSET          0x12000
#define NETLOGIC_IO_XGMAC_1_OFFSET         0x13000

#define NETLOGIC_IO_I2C_0_OFFSET           0x16000
#define NETLOGIC_IO_I2C_1_OFFSET           0x17000

#define NETLOGIC_IO_GPIO_OFFSET            0x18000

#define NETLOGIC_IO_FLASH_OFFSET           0x19000

#define NETLOGIC_IO_TB_OFFSET           	  0x1C000

#define NETLOGIC_CPLD_OFFSET               0xffffffffbd840000ULL

/* Base Address (Virtual) of the PCI Config address space
 * For now, choose 256M phys in kseg1 = 0xA0000000 + (1<<28)
 * Config space spans 256 (num of buses) * 256 (num functions) * 256 bytes
 * ie 1<<24 = 16M
 */ 
#define DEFAULT_PCI_CONFIG_BASE         0x18000000
#define DEFAULT_HT_TYPE0_CFG_BASE       0x16000000
#define DEFAULT_HT_TYPE1_CFG_BASE       0x17000000

#ifndef __ASSEMBLY__

#include <linux/types.h>
#include <asm/byteorder.h>

typedef volatile __u32 nlm_reg_t;
extern unsigned long netlogic_io_base;

#define netlogic_io_mmio(offset) ((nlm_reg_t *)(netlogic_io_base+(offset)))

/* XLP_MERGE_TODO */
#if defined(NLM_BRIDGE_WKAROUND)
#include "nlm_rw_lock.h"
extern nlm_rwlock_t *nlm_bridge_lock;
extern int nlm_enable_br_wrkaround;

static inline void nlm_preempt_enable(void)
{
    uint32_t status=0;
    __asm__ volatile(
#ifdef CONFIG_64BIT
            "lw %0, 36($28)\n"
#else
            "lw %0, 20($28)\n"
#endif
            "addiu %0, %0, -1 \n"
#ifdef CONFIG_64BIT
            "sw %0, 36($28) \n"
#else
            "sw %0, 20($28) \n"
#endif
            :"=r"(status)
            );    
}

static inline void nlm_preempt_disable(void)
{
    uint32_t status=0;
    __asm__ volatile(
#ifdef CONFIG_64BIT
            "lw %0, 36($28)\n"
#else
            "lw %0, 20($28)\n"
#endif
            "addiu %0, %0, 1 \n"
#ifdef CONFIG_64BIT
            "sw %0, 36($28) \n"
#else
            "sw %0, 20($28) \n"
#endif
            :"=r"(status)
            );    
}

static inline uint32_t nlm_br_read_lock(void)
{
    uint32_t ret = 0;
	if(nlm_enable_br_wrkaround){
         nlm_preempt_disable();
		 ret = nlm_read_lock_irq_save(nlm_bridge_lock);
         nlm_preempt_enable();
    }
	return ret;
}
static inline void nlm_br_read_unlock(unsigned int flags)
{
	if(nlm_enable_br_wrkaround){
        nlm_preempt_disable();
		nlm_read_unlock_irq_restore(nlm_bridge_lock, flags);
        nlm_preempt_enable();
    }
}

static inline uint32_t nlm_br_write_lock(void)
{
    uint32_t ret = 0;
	if(nlm_enable_br_wrkaround){
        nlm_preempt_disable();
		ret = nlm_write_lock_irq_save(nlm_bridge_lock);
        nlm_preempt_enable();
    }
	return ret;
}

static inline void nlm_br_write_unlock(unsigned int flags)
{
	if(nlm_enable_br_wrkaround){
        nlm_preempt_disable();
		nlm_write_unlock_irq_restore(nlm_bridge_lock, flags);
        nlm_preempt_enable();
    }
}

static inline uint32_t nlm_read_reg_locked(nlm_reg_t *base, 
		unsigned int offset) 	
{
	unsigned int flags, val;

	flags = nlm_br_read_lock();
	val = (be32_to_cpu((base)[(offset)])); 
	nlm_br_read_unlock(flags);

	return val;
}
static inline uint32_t nlm_read_reg_le_locked(nlm_reg_t *base, 
		unsigned int offset) 	
{
	unsigned int flags, val;
	flags = nlm_br_read_lock();
	val = (le32_to_cpu((base)[(offset)])); 
	nlm_br_read_unlock(flags);

	return val;
}
static inline void nlm_write_reg_locked(nlm_reg_t *base, 
		 unsigned int offset,  unsigned int value)
{
	unsigned int flags;
	flags = nlm_br_write_lock();
	((base)[(offset)] = cpu_to_be32((value)));
	nlm_br_write_unlock(flags);
}

static inline void nlm_write_reg_le_locked(nlm_reg_t *base, 
		 unsigned int offset,  unsigned int value)
{
	unsigned int flags;
	flags = nlm_br_write_lock();
	((base)[(offset)] = cpu_to_le32((value)));
	nlm_br_write_unlock(flags);
}

#define netlogic_read_reg(base, offset) nlm_read_reg_locked(base, offset)
#define netlogic_write_reg(base, offset, value) \
	nlm_write_reg_locked(base, offset, value)

#define netlogic_read_reg_le32(base, offset) \
	nlm_read_reg_le_locked(base, offset)
#define netlogic_write_reg_le32(base, offset, value) \
	nlm_write_reg_le_locked(base, offset, value)

#else /* NLM_BRIDGE_WORKAROUND */

static inline uint32_t nlm_br_read_lock(void) 
{
	return 0;
}

static inline void nlm_br_read_unlock(unsigned int flags)
{
}

static inline uint32_t nlm_br_write_lock(void)
{
	return 0;
}

static inline void nlm_br_write_unlock(unsigned int flags)
{
}
#ifdef CONFIG_CPU_LITTLE_ENDIAN

#define netlogic_read_reg(base, offset) ((base)[(offset)])
#define netlogic_write_reg(base, offset, value) ((base)[(offset)] = (value))

#else

#define netlogic_read_reg(base, offset) (be32_to_cpu((base)[(offset)]))
#define netlogic_write_reg(base, offset, value) ((base)[(offset)] = cpu_to_be32((value)))

#endif

#define netlogic_read_reg_le32(base, offset) (le32_to_cpu((base)[(offset)]))
#define netlogic_write_reg_le32(base, offset, value) \
	((base)[(offset)] = cpu_to_le32((value)))

#endif /* NLM_BRIDGE_WORKAROUND */

extern void on_chip_init(void);

#endif /* __ASSEMBLY__ */

#endif
