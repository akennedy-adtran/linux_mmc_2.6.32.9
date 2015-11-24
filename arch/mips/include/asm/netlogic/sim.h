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

#ifdef CONFIG_CPU_XLR

#ifndef _ASM_SIM_H
#define _ASM_SIM_H

#include <linux/types.h>
#include <asm/cpu.h>
#include <asm/mipsregs.h>

#define MAX_CPU_REV_LEN	 100
#define PSB_INFO_VERSION 0x0001

struct psb_info {
	uint64_t boot_level;
	uint64_t io_base;
	uint64_t output_device;
	uint64_t uart_print;
	uint64_t led_output;
	uint64_t init;
	uint64_t exit;
	uint64_t warm_reset;
	uint64_t wakeup;
	uint64_t nlm_cpu_online_map;
	uint64_t master_reentry_sp;
	uint64_t master_reentry_gp;
	uint64_t master_reentry_fn;
	uint64_t slave_reentry_fn;
	uint64_t magic_dword;
	uint64_t uart_putchar;
	uint64_t size;
	uint64_t uart_getchar;
	uint64_t nmi_handler;
	uint64_t psb_version;
	uint64_t mac_addr;
	uint64_t cpu_frequency;
	uint64_t board_version;
	uint64_t malloc;
	uint64_t free;
	uint64_t global_shmem_addr;
	uint64_t global_shmem_size;
	uint64_t psb_os_cpu_map;
	uint64_t userapp_cpu_map;
	uint64_t wakeup_os;
	uint64_t psb_mem_map;
	uint64_t board_major_version;
	uint64_t board_minor_version;
	uint64_t board_manf_revision;
	uint64_t board_serial_number;
	uint64_t psb_physaddr_map;
	uint64_t xlr_loaderip_config;
	uint64_t bldr_envp;
	uint64_t avail_mem_map;
};


enum {
        NETLOGIC_IO_SPACE = 0x10,
        PCIX_IO_SPACE,
        PCIX_CFG_SPACE,
        PCIX_MEMORY_SPACE,
        HT_IO_SPACE,
        HT_CFG_SPACE,
        HT_MEMORY_SPACE,
        SRAM_SPACE,
        FLASH_CONTROLLER_SPACE
};

#define MAX_ENV_BUF 0x00020000 // 128 KB = One sector of Intel flash.
struct environment
{
        unsigned int crc;
        unsigned char envbuf[MAX_ENV_BUF - 20]; // 4 bytes for CRC and 16 bytes reserved.
        unsigned char reserved[16];
};

#define NLM_XLR_BOARD_ARIZONA_I   1
#define NLM_XLR_BOARD_ARIZONA_II  2
#define NLM_XLR_BOARD_ARIZONA_III 3
#define NLM_XLR_BOARD_ARIZONA_IV  4
#define NLM_XLR_BOARD_ARIZONA_V   5
#define NLM_XLR_BOARD_ARIZONA_VI   6  /* XLS boards */
#define NLM_XLR_BOARD_ARIZONA_VII 7 /*XLS 2xx boards*/
#define NLM_XLR_BOARD_ARIZONA_VIII 8 /*XLS LTE boards*/
#define NLM_XLR_BOARD_ARIZONA_XI 11
#define NLM_XLR_BOARD_ARIZONA_XII  12

extern struct smp_boot_info smp_boot;
extern void prom_boot_cpus_secondary(void *);

extern __u32 xlr_board_major_version;
extern __u32 xlr_board_minor_version;

#define XLR_REVISION_A0 0xc0000
#define XLR_REVISION_A1 0xc0001
#define XLR_REVISION_B0 0xc0002
#define XLR_REVISION_B1 0xc0003
#define XLR_REVISION_B2 0xc0004
#define XLR_REVISION_C0 0xc0005
#define XLR_REVISION_C1 0xc0006
#define XLR_REVISION_C2 0xc0007
#define XLR_REVISION_C3 0xc0008
#define XLR_REVISION_C4 0xc0009

static __inline__ unsigned int xlr_revision(void)
{
	return read_c0_prid() & 0xff00ff;
}

static __inline__ int xlr_revision_a0(void)
{
	return xlr_revision() == XLR_REVISION_A0;
}

static __inline__ int xlr_revision_b0(void)
{
	return xlr_revision() == XLR_REVISION_B0;
}

static __inline__ int xlr_revision_b1(void)
{
        return xlr_revision() == XLR_REVISION_B1;
}

static __inline__ int xlr_revision_c(void)
{
    uint32_t prid = read_c0_prid();
    if(prid>=XLR_REVISION_C0 && prid<=XLR_REVISION_C4)
        return 1;        
    return 0; 
}

static __inline__ int xlr_board_atx_i(void)
{
	return xlr_board_major_version == NLM_XLR_BOARD_ARIZONA_I;
}

static __inline__ int xlr_board_atx_ii(void)
{
	return xlr_board_major_version == NLM_XLR_BOARD_ARIZONA_II;
}

static __inline__ int xlr_board_atx_ii_b(void)
{
	return (xlr_board_major_version == NLM_XLR_BOARD_ARIZONA_II)
		&& (xlr_board_minor_version == 1);
}

static __inline__ int xlr_board_atx_iii(void)
{
	return xlr_board_major_version == NLM_XLR_BOARD_ARIZONA_III;
}

static __inline__ int xlr_board_atx_iv(void)
{
	return (xlr_board_major_version == NLM_XLR_BOARD_ARIZONA_IV)
		&& (xlr_board_minor_version == 0);
}

static __inline__ int xlr_board_atx_iv_b(void)
{
	return (xlr_board_major_version == NLM_XLR_BOARD_ARIZONA_IV)
		&& (xlr_board_minor_version == 1);
}

static __inline__ int xlr_board_atx_v(void)
{
	return xlr_board_major_version == NLM_XLR_BOARD_ARIZONA_V;
}

static __inline__ int xlr_board_atx_iii_256(void)
{
	return (xlr_board_major_version == NLM_XLR_BOARD_ARIZONA_III)
		&& (xlr_board_minor_version == 0);
}

static __inline__ int xlr_board_atx_iii_512(void)
{
	return (xlr_board_major_version == NLM_XLR_BOARD_ARIZONA_III)
		&& (xlr_board_minor_version == 1);
}

static __inline__ int xlr_board_atx_v_512(void)
{
	return (xlr_board_major_version == NLM_XLR_BOARD_ARIZONA_V)
		&& (xlr_board_minor_version == 1);
}

static __inline__ int xlr_board_atx_vi(void)
{
	return (xlr_board_major_version == NLM_XLR_BOARD_ARIZONA_VI);
}

static __inline__ int xlr_board_atx_vii(void)
{
	return (xlr_board_major_version == NLM_XLR_BOARD_ARIZONA_VII);
}

static __inline__ int xlr_board_atx_viii(void)
{
	return (xlr_board_major_version == NLM_XLR_BOARD_ARIZONA_VIII);
}

static __inline__ int xlr_board_atx_xi(void)
{
	return (xlr_board_major_version == NLM_XLR_BOARD_ARIZONA_XI);
}

static __inline__ int xlr_board_atx_xii(void)
{
	return (xlr_board_major_version == NLM_XLR_BOARD_ARIZONA_XII);
}

static __inline__ int xlr_board_atx_xaui_rework(void)
{
	if ((xlr_board_atx_xi() || xlr_board_atx_xii()) &&
			(xlr_board_minor_version == 4))
		return 1;
	else
		return 0;
}


#define XLR_HYBRID_NONE              0
#define XLR_HYBRID_USER_MAC          1
#define XLR_HYBRID_RMIOS_IPSEC       2
#define XLR_HYBRID_RMIOS_TCPIP_STACK 3
#define XLR_HYBRID_USER_MAC_GMAC     4
#define XLR_HYBRID_USER_MAC_XGMAC    5
#define XLR_HYBRID_USER_MAC_SPI4     6
#define XLR_HYBRID_USER_MAC_GMAC_XGMAC    7
#define XLR_HYBRID_USER_MAC_GMAC_SPI4     8

extern int xlr_hybrid;

static __inline__ int xlr_hybrid_user_mac(void)
{
	return xlr_hybrid == XLR_HYBRID_USER_MAC;
}

static __inline__ int xlr_hybrid_user_mac_xgmac(void)
{
	return (xlr_hybrid == XLR_HYBRID_USER_MAC_XGMAC || 
		xlr_hybrid == XLR_HYBRID_USER_MAC_SPI4);
}

static __inline__ int xlr_hybrid_rmios_tcpip_stack(void)
{
        return xlr_hybrid == XLR_HYBRID_RMIOS_TCPIP_STACK;
}

static __inline__ int xlr_hybrid_rmios_ipsec(void)
{
	return xlr_hybrid == XLR_HYBRID_RMIOS_IPSEC;
}

static __inline__ int xlr_hybrid_none(void)
{
	return xlr_hybrid == XLR_HYBRID_NONE;
}

struct boot_mem_map_entry *psb_get_physaddr_base_address(unsigned long type);


#define CHIP_PROCESSOR_ID_XLS_608   0x80
#define CHIP_PROCESSOR_ID_XLS_408   0x88
#define CHIP_PROCESSOR_ID_XLS_404   0x8c
#define CHIP_PROCESSOR_ID_XLS_208   0x8e
#define CHIP_PROCESSOR_ID_XLS_204   0x8f
#define CHIP_PROCESSOR_ID_XLS_108   0xce
#define CHIP_PROCESSOR_ID_XLS_104   0xcf

/* Defines for XLS B0*/
#define CHIP_PROCESSOR_ID_XLS_616_B0   0x40
#define CHIP_PROCESSOR_ID_XLS_608_B0   0x4a
#define CHIP_PROCESSOR_ID_XLS_416_B0   0x44
#define CHIP_PROCESSOR_ID_XLS_412_B0   0x4c
#define CHIP_PROCESSOR_ID_XLS_408_B0   0x4e
#define CHIP_PROCESSOR_ID_XLS_404_B0   0x4f

#define CHIP_PROCESSOR_ID_XLR_B_308   0x06
#define CHIP_PROCESSOR_ID_XLR_B_508   0x07
#define CHIP_PROCESSOR_ID_XLR_B_516   0x08
#define CHIP_PROCESSOR_ID_XLR_B_532   0x09
#define CHIP_PROCESSOR_ID_XLR_B_716   0x0a
#define CHIP_PROCESSOR_ID_XLR_B_732   0x0b

#define CHIP_PROCESSOR_ID_XLR_C_308   0x0F
#define CHIP_PROCESSOR_ID_XLR_C_508   0x0b
#define CHIP_PROCESSOR_ID_XLR_C_516   0x0a
#define CHIP_PROCESSOR_ID_XLR_C_532   0x08
#define CHIP_PROCESSOR_ID_XLR_C_716   0x02

#if defined(CONFIG_NLM_XLP)
/* Fake Values for bring-up */
#define CHIP_PROCESSOR_ID_XLP_A_832   0x00
#define CHIP_PROCESSOR_ID_XLR_C_732   0xff
#else
/* Real Values */
#define CHIP_PROCESSOR_ID_XLP_A_832   0x90
#define CHIP_PROCESSOR_ID_XLR_C_732   0x00
#endif

/*  fill the xls chip family types 
 */
extern int chip_is_xls6xx;
extern int chip_is_xls4xx;
extern int chip_is_xls2xx;
extern int chip_is_xls1xx;
extern int chip_is_xls;
extern int chip_is_xls_b0;
extern int chip_is_xls6xx_b0;
extern int chip_is_xls4xx_b0;

static __inline__ void set_xls_chip_family_types(void)
{
	int processor_id = ((read_c0_prid() & 0xff00) >> 8);
	chip_is_xls = 1;
	switch (processor_id) {
        case CHIP_PROCESSOR_ID_XLS_608: 
		{
			chip_is_xls6xx = 1;
			break;
		}
        case CHIP_PROCESSOR_ID_XLS_408:
        case CHIP_PROCESSOR_ID_XLS_404:
		{
			chip_is_xls4xx = 1;
			break;
		}
        case CHIP_PROCESSOR_ID_XLS_208:
        case CHIP_PROCESSOR_ID_XLS_204:
		{
			chip_is_xls2xx = 1;
			break;
		}
        case CHIP_PROCESSOR_ID_XLS_108:
        case CHIP_PROCESSOR_ID_XLS_104:
		{
			chip_is_xls1xx = 1;
			break;
		}
        case CHIP_PROCESSOR_ID_XLS_616_B0:
        case CHIP_PROCESSOR_ID_XLS_608_B0:
		{
			chip_is_xls_b0 = 1;
			chip_is_xls6xx_b0 = 1;
			break;
		}
        case CHIP_PROCESSOR_ID_XLS_416_B0:
        case CHIP_PROCESSOR_ID_XLS_412_B0:
        case CHIP_PROCESSOR_ID_XLS_408_B0:
        case CHIP_PROCESSOR_ID_XLS_404_B0:
		{
			chip_is_xls_b0 = 1;
			chip_is_xls4xx_b0 = 1;
			break;
		}
        default:
			chip_is_xls = 0;
	}
	return;
}

static __inline__ int is_xls(void)
{
	return chip_is_xls;
}

static __inline__ int is_xls2xx(void)
{
	return chip_is_xls2xx;
}

static __inline__ int is_xls1xx(void)
{
	return chip_is_xls1xx;
}

static __inline__ int is_xls4xx(void)
{
	return chip_is_xls4xx;
}

static __inline__ int is_xls6xx(void)
{
	return chip_is_xls6xx;
}

static __inline__ int is_xls_b0_4xx(void)
{
	return chip_is_xls4xx_b0;
}

static __inline__ int is_xls_b0_6xx(void)
{
	return chip_is_xls6xx_b0;
}

static __inline__ int is_xls_b0(void)
{
	return chip_is_xls_b0;
}



#define NR_CORES 8
#define NR_CPUS_PER_CORE 4
#define NLM_MAX_ARGS 64
#define NLM_MAX_ENVS 32

extern char cpu_model_info[MAX_CPU_REV_LEN];
extern char* get_cpu_info(void);

#endif /* _ASM_SIM_H */
#endif /* CONFIG_CPU_XLR */
