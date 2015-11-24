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

#include <linux/smp.h>
#include <asm/system.h>

#include <asm/netlogic/iomap.h>
#include <asm/netlogic/hal/nlm_hal.h>
#include <asm/netlogic/hal/nlm_hal_macros.h>
#include <asm/netlogic/xlp.h>
#include <asm/netlogic/xlp_irq.h>
#include <asm/netlogic/hal/nlm_hal_xlp_dev.h>
#include <asm/inst.h>
#include <asm/ptrace.h>
#include <asm/fpu.h>
#include <asm/asm.h>
#include <asm/mipsregs.h>
#include <hal/nlm_hal_macros.h>

unsigned char nlm_cerr_stack[8192];
volatile int nlm_cerr_lock;

static void load_registers(struct pt_regs *regs, unsigned long *pErrepc);
static void restore_registers(struct pt_regs *regs, unsigned long errepc);
static int  compute_return_registers(struct pt_regs *regs, unsigned long *pErrepc);
static int  update_registers(void);

static __inline__ void cerr_cpu_halt(void)
{
	for(;;) {
		__asm__ __volatile__(
			".set push      \n"
			".set mips64    \n"
			"1: wait        \n"
			"   b 1b        \n"
			"   nop         \n"
			".set pop       \n"
		);
	}
}

#define UART_RHR 0
#define UART_THR 0
#define UART_IER 1
#define UART_IIR 2
#define UART_FCR 2
#define UART_LCR 3
#define UART_MCR 4
#define UART_LSR 5
#define UART_MSR 6
static void cerr_outbyte(char ch)
{
	volatile uint32_t *mmio = netlogic_io_mmio(NETLOGIC_IO_UART_0_OFFSET);
	int lsr;
	for (;;) {

		lsr = be32_to_cpu(mmio[UART_LSR]);

		/* Tx Fifo empty */
		if (lsr & 0x20) {
			mmio[UART_THR] = cpu_to_be32((int)ch);
			break;
		}
	}
}

static char cerr_printk_buf[2048];
static void cerr_printk(const char *fmt, ...)
{
	va_list args;
	int len;
	int i = 0;

	va_start(args, fmt);
	len = vsnprintf(cerr_printk_buf, sizeof(cerr_printk_buf), fmt, args);
	va_end(args);

	for (i = 0; i <= len; i++) {

		if (cerr_printk_buf[i] == 0)
			continue;

		if (cerr_printk_buf[i] == '\n')
			cerr_outbyte('\r');

		cerr_outbyte(cerr_printk_buf[i]);
	}

}

static void print_error_header(void)
{
	cerr_printk("*******************************************************************************************\n");
	cerr_printk("cpu_%d received a bus/cache error(warning)\n", hard_smp_processor_id());
	cerr_printk("*******************************************************************************************\n");
}

#ifdef CONFIG_NLM_XLR

static char *bridge_aerr_intr_devstat[] = {
	[0] = "cpu 0",
	[1] = "cpu 1",
	[2] = "cpu 2",
	[3] = "cpu 3",
	[4] = "cpu 4",
	[5] = "cpu 5",
	[6] = "cpu 6",
	[7] = "cpu 7",

	[8] = "L2",
	[9] = "XGS 0",
	[10] = "XGS 1",
	[11] = "GMAC",
	[12] = "SEC",
	[13] = "PCIX",
	[14] = "HT",
	[15] = "DMA",
};

static int print_cerr_info(void)
{
	__u64 cerr_cpu_log = 0;
	int i = 0;
	nlm_reg_t *mmio = netlogic_io_mmio(NETLOGIC_IO_BRIDGE_OFFSET);
	__u32 tmp = 0;

	print_error_header();

	cerr_printk("Bridge: Phys Addr = 0x%010llx, Device_AERR = 0x%08x\n",
			( ((__u64)netlogic_read_reg(mmio, 39)<<5) | ((__u64)netlogic_read_reg(mmio, 40)<<37) ),
			netlogic_read_reg(mmio, 41));

	cerr_printk("Bridge: The devices reporting AERR are:\n");
	tmp = netlogic_read_reg(mmio, 41);
	for(i = 0; i < 16; i++) {
		if (tmp & (1<<i)) cerr_printk("\t%s\n", bridge_aerr_intr_devstat[i]);
	}

	cerr_cpu_log = read_64bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRLOG_REGID);
	cerr_printk("CPU: (XLR specific) Cache Error log = 0x%016llx, Phy Addr = 0x%010llx\n",
			cerr_cpu_log, ((cerr_cpu_log >> 10) & 0xffffffffffULL) << 3);

	return 1;
}

#else

static void dump_cerr_info(void)
{
	int n, num_controllers, ret;
	uint32_t dram_log1, dram_log2;
	int node = hard_smp_processor_id() / NLM_MAX_CPU_PER_NODE;
	uint64_t mmio = nlm_hal_get_dev_base(node, 0, NLH_BRIDGE, 0);

	uint32_t nbu_reg0 = nlm_hal_read_32bit_reg(mmio, 0xA2);
	uint32_t nbu_reg1 = nlm_hal_read_32bit_reg(mmio, 0xA3);
	uint32_t nbu_reg2 = nlm_hal_read_32bit_reg(mmio, 0xA4);

	uint32_t icu_log0 = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_ICU, ICU_CERRLOG0_REGID);
	uint32_t icu_log1 = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_ICU, ICU_CERRLOG1_REGID);
	uint32_t icu_log2 = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_ICU, ICU_CERRLOG2_REGID);

	uint32_t lsu_log0 = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRLOG0_REGID);
	uint64_t lsu_log1 = read_64bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRLOG1_REGID);

	uint32_t scu_log0 = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_SCU, SCU_CERRLOG0_REGID);
	uint32_t scu_log1 = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_SCU, SCU_CERRLOG1_REGID);
	uint32_t scu_log2 = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_SCU, SCU_CERRLOG2_REGID);

	uint32_t l3_reg0 = nlm_hal_read_32bit_reg(mmio, 0xD9);
	uint32_t l3_reg1 = nlm_hal_read_32bit_reg(mmio, 0xDA);
	uint32_t l3_reg2 = nlm_hal_read_32bit_reg(mmio, 0xDB);

	uint32_t cop2_txmsgstatus, cop2_msgstatus1;

	cop2_txmsgstatus = (uint32_t) _read_32bit_cp2_register($2);
	cop2_msgstatus1  = (uint32_t) _read_32bit_cp2_register($4);

	cerr_printk("CPU (XLP specific) registers dump: Node=%d \n", node);
	cerr_printk("    COP2: TxMsgStatus = 0x%08x, MsgStatus1 = 0x%08x\n",
				cop2_txmsgstatus, cop2_msgstatus1 );

	cerr_printk("     ICU: log0 = 0x%08x, log1 = 0x%08x, log2 = 0x%08x\n",
				icu_log0, icu_log1, icu_log2);

	cerr_printk("     LSU: log0 = 0x%08x, log1 = 0x%016llx\n", lsu_log0, lsu_log1);

	cerr_printk("     SCU: log0 = 0x%08x, log1 = 0x%08x, log2 = 0x%08x\n",
				scu_log0, scu_log1, scu_log2);

	cerr_printk("     TCU: reg0 = 0x%08x, reg1 = 0x%08x, reg2 = 0x%08x\n",
				l3_reg0, l3_reg1, l3_reg2);

	cerr_printk("     NBU: reg0 = 0x%08x, reg1 = 0x%08x, reg2 = 0x%08x\n",
				nbu_reg0, nbu_reg1, nbu_reg2);

	num_controllers = 4;	//8xx has 4 controller 
	ret=is_nlm_xlp3xx_rev(XLP_REVISION_ANY);
	if( ret ) num_controllers = 2;
	else {
		ret=is_nlm_xlp2xx();
		if( ret ) num_controllers = 1;
	}

	for(n=0; n<num_controllers; n++)
	{
		dram_log1 = nlm_hal_read_32bit_reg(mmio, n*0x80+0x11D);
		dram_log2 = nlm_hal_read_32bit_reg(mmio, n*0x80+0x11E);
		cerr_printk("  DRAM_%c: reg1 = 0x%08x, reg2 = 0x%08x\n",
                (char)(n+'A'), dram_log1, dram_log2);
	}

	cerr_printk("\n     CPU: epc = 0x%lx, errorepc = 0x%016llx, cacheerr = 0x%08x\n",
			read_c0_epc(), read_c0_errorepc(), read_c0_cacheerr());
	cerr_printk("*******************************************************************************************\n");
}

static int check_COP2_error(void)
{
	uint32_t cop2_txmsgstatus, cop2_msgstatus1;
	cop2_txmsgstatus = (uint32_t)_read_32bit_cp2_register( $2 );
	cop2_msgstatus1  = (uint32_t)_read_32bit_cp2_register( $4 );

	if( ((cop2_txmsgstatus >> 3) & 1) == 0 && ((cop2_msgstatus1 >> 12) & 1) == 0 ) return 0;

	cerr_printk("COP2 Cache error: \n");
	if( (cop2_txmsgstatus >> 3) & 1 )
	{
		cerr_printk("  TQF: %s\n", (cop2_txmsgstatus >> 24) ? "Queue full" : "Queue not full");
		cerr_printk("  DFC: 0x%04x\n", (cop2_txmsgstatus >> 4) & 0xFFFF);
		cerr_printk(" DFCF: ECC error during OQ credit RAM read from COP2 RAM\n");
		cerr_printk("   PS: %s\n", (cop2_txmsgstatus >> 2) & 1 ? "failed due to pending synchronization" : "Status good");
		cerr_printk("  IQC: %s\n", (cop2_txmsgstatus >> 1) & 1 ? "error insufficient input credit" : "Status good" );
		cerr_printk("  OQC: %s\n", (cop2_txmsgstatus >> 0) & 1 ? "error insufficient output credit": "Status good" );
	}

	if( (cop2_msgstatus1 >> 12) & 1 )
	{
		cerr_printk("      VCE: 0x%x\n", (cop2_msgstatus1 >> 24 ) & 0xF );
		cerr_printk("      RWF: %d\n",   (cop2_msgstatus1 >> 23 ) & 1 );
		cerr_printk("      ICO: %d\n",   (cop2_msgstatus1 >> 22 ) & 1 );
		cerr_printk("      OCO: %d\n",   (cop2_msgstatus1 >> 21 ) & 1 );
		cerr_printk("       ME: %d\n",   (cop2_msgstatus1 >> 20 ) & 1 );
		cerr_printk("  VC_Pend: %d\n",   (cop2_msgstatus1 >> 16 ) & 0xF );
		cerr_printk("   MSGSTA: ECC error in COP2 RAM\n");
		cerr_printk("     OQID: 0x%03x\n", cop2_msgstatus1 & 0xFFF );
	}

	return -1;
}

static char * c_icu_errinfo[]={
	[0] = "L1 tag RAM",
	[1] = "L1 data RAM",
	[2] = "Not defined"
};

static int check_ICU_error(void)
{
	uint32_t icu_log0 = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_ICU, ICU_CERRLOG0_REGID);
	uint32_t icu_log1 = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_ICU, ICU_CERRLOG1_REGID);
	uint32_t icu_log2 = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_ICU, ICU_CERRLOG2_REGID);
	uint64_t phyaddr = ( (uint64_t)icu_log2 << 32) | icu_log1;
	uint32_t errinfo = (icu_log0 >> 12) & 0xF;
	uint32_t errtype = (icu_log0 >> 9) & 7;
	uint32_t oprtype = (icu_log0 >> 6) & 7;

	if( (icu_log0&0xF) == 0 && icu_log1 == 0 && icu_log2 == 0 ) return 0;

	cerr_printk("ICU Cache %s:\n", (icu_log0&0xF) ? "Error" : "Warning" );
	cerr_printk("   log0=0x%08x, phyaddr=0x%016llx\n", icu_log0, phyaddr);
	if( (icu_log0 & 0xF) == 0 )
	{
		write_32bit_nlm_ctrl_reg(CPU_BLOCKID_ICU,ICU_CERRLOG1_REGID, 0);
		write_32bit_nlm_ctrl_reg(CPU_BLOCKID_ICU,ICU_CERRLOG2_REGID, 0);
		return 1;
	}
	else
	{
		cerr_printk("          errinfo: %s\n", errinfo < 2  ? c_icu_errinfo[errinfo] : c_icu_errinfo[2] );
		cerr_printk("          errtype: %s\n", errtype == 0 ? "Single‐bit error" : "Not defined");
		cerr_printk("          oprtype: %s\n", oprtype == 0 ? "Read access" : "Not defined");
		cerr_printk("    Uncorrectable: %d\n", ( icu_log0 >> 5) & 1);
		cerr_printk("         Overflow: %d\n", ( icu_log0 >> 4 ) & 1);
		cerr_printk("     Threads Mask: %x\n", icu_log0 & 0xF );
		return -1;
	}
}

static int check_LSU_error(void)
{
	uint32_t lsu_log0 = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRLOG0_REGID);
	uint64_t lsu_log1 = read_64bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRLOG1_REGID);
	if( (lsu_log0 & 0xF) == 0 && lsu_log1 == 0 ) return 0;

	cerr_printk("LSU Cache %s:\n", (lsu_log0&0xF) ? "Error" : "Warning" );
	cerr_printk("   log0=0x%016llx, phyaddr=0x%010llx\n", lsu_log0, lsu_log1);
	if( (lsu_log0 & 0xF ) == 0 )
	{
		write_64bit_nlm_ctrl_reg(CPU_BLOCKID_LSU, LSU_CERRLOG1_REGID, 0);
		return 1;
	}
	else
	{
		cerr_printk("                 Info: %s\n", ((lsu_log0>>11)&1)==0 ? "TAG" : "DATA" );
		cerr_printk("              ErrType: %s\n", ((lsu_log0>>10)&1)==0 ? "Parity Error" :  "Reserved");
		cerr_printk("               OpType: %s\n", ((lsu_log0>> 6)&1)==0 ? "Read" :  "Reserved" );
		cerr_printk("   Ucorrectable Error: %d\n", (lsu_log0>>5)&1);
		cerr_printk("       Error Overflow: %d\n", (lsu_log0>>4)&1);
		cerr_printk("   Error threads mask: %x\n", lsu_log0&0xF);
		return -1;
	}
}

static char *c_l2_errtype[] = {
	[0] = "Single‐bit Tag RAM error",
	[1] = "Double‐bit Tag RAM Error",
	[2] = "Valid Array Parity Error",
	[3] = "Single‐bit Data RAM Error",
	[4] = "Double‐bit Data RAM Error",
	[5] = "External Error from Data or Completion for Fill",
	[6] = "Evict Completion Error. Registers do not hold the correct address/way",
	[7] = "Reserved"
};

static char *c_l2_erroptype[] = {
	[0] = "Load",
	[1] = "Probe",
	[2] = "Store",
	[3] = "Load Fill",
	[4] = "Store Fill",
	[5] = "Evict Completion",
	[6] = "Reserved"
};

static int check_SCU_error(void)
{
	uint32_t scu_log0 = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_SCU, SCU_CERRLOG0_REGID);
	uint32_t scu_log1 = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_SCU, SCU_CERRLOG1_REGID);
	uint32_t scu_log2 = read_32bit_nlm_ctrl_reg(CPU_BLOCKID_SCU, SCU_CERRLOG2_REGID);

	uint64_t phyaddr = ((uint64_t)scu_log2 << 32) | scu_log1;
	int errtype  = ( scu_log0 >> 8 ) & 0xF;
	int erroptyp = ( scu_log0 >> 4 ) & 7;

	if( ( scu_log0 & 1) == 0 && phyaddr == 0 )	return 0;

	cerr_printk("SCU Cache %s:\n", (scu_log0&1) ? "Error" : "Warning" );
	cerr_printk("   log0=0x%08x, phyaddr=0x%016llx\n", scu_log0, phyaddr);
	if( ( scu_log0 & 1 ) == 0 )
	{
		write_32bit_nlm_ctrl_reg(CPU_BLOCKID_SCU, SCU_CERRLOG1_REGID, 0);
		write_32bit_nlm_ctrl_reg(CPU_BLOCKID_SCU, SCU_CERRLOG2_REGID, 0);
		return 1;
	}
	else
	{
		cerr_printk("          Way Info: %s\n", (scu_log0>>12)&0xF );
		cerr_printk("          ErrType: %s\n", errtype<7 ? c_l2_errtype[errtype] : c_l2_errtype[7] );
		cerr_printk("        ErrOpType: %s\n", erroptyp<6 ? c_l2_erroptype[erroptyp] : c_l2_erroptype[6] );
		cerr_printk("    Uncorrectable: %d\n", (scu_log0>>2)&1);
		cerr_printk("         Overflow: %d\n", (scu_log0>>1)&1);
		cerr_printk("      Error Valid: %d\n", scu_log0&1);
		return -1;
	}
}

static char *c_l3_inf[] = {
	[0] = "TAG",
	[1] = "STATE",
	[2] = "DATA",
	[3] = "Not defined"
};

static char *c_l3_ErTy[] = {
	[0] = "Single‐bit ECC",
	[1] = "Multi‐bit ECC",
	[2] = "Not defined"
};

static char *c_l3_ErOpTy[] = {
	[0] = "Read",
	[1] = "Write",
	[2] = "Msg",
	[3] = "Reserved"
};

static int check_L3_error(void)
{
	int node = hard_smp_processor_id() / NLM_MAX_CPU_PER_NODE;
	uint64_t mmio = nlm_hal_get_dev_base(node, 0, NLH_BRIDGE, 0);

	uint32_t reg0 = nlm_hal_read_32bit_reg(mmio, 0xD9);
	uint32_t reg1 = nlm_hal_read_32bit_reg(mmio, 0xDA);
	uint32_t reg2 = nlm_hal_read_32bit_reg(mmio, 0xDB);

	uint64_t phyaddr = ( (uint64_t)reg2 << 32 ) | reg1;
	uint8_t  erinf = (reg0 >> 11) & 0x1F;
	uint8_t  erty  = (reg0 >> 8 ) & 7;
	uint8_t  opty  = (reg0 >> 4 ) & 7;

	if( ( reg0 & 1) == 0 && phyaddr == 0 ) return 0;

	cerr_printk("L3 Cache %s:\n", (reg0&1) ? "Error" : "Warning" );
	cerr_printk("   log0=0x%08x, phyaddr=0x%016llx\n", reg0, phyaddr);
	if( ( reg0 & 1) == 0)
	{
		nlm_hal_write_32bit_reg(mmio, 0xDA, 0);
		nlm_hal_write_32bit_reg(mmio, 0xDB, 0);
		return 1;
	}
	else
	{
		cerr_printk("      Error Info: %s\n", erinf < 3 ? c_l3_inf[erinf]   : c_l3_inf[3]   );
		cerr_printk("      Error Type: %s\n", erty  < 2 ? c_l3_ErTy[erty]   : c_l3_ErTy[2]  );
		cerr_printk("     Option Type: %s\n", opty  < 4 ? c_l3_ErOpTy[opty] : c_l3_ErOpTy[3]);
		cerr_printk("   Uncorrectable: %d\n", ( reg0 >> 2 ) & 1);
		cerr_printk("        Overflow: %d\n", ( reg0 >> 1 ) & 1);
		cerr_printk("           Valid: %d\n", reg0 & 1 );
		return -1;
	}
}

static int check_DRAM_error(void)
{
	int n, error=0;
	uint32_t log1, log2;
	int node = hard_smp_processor_id() / NLM_MAX_CPU_PER_NODE;
	uint64_t mmio = nlm_hal_get_dev_base(node, 0, NLH_BRIDGE, 0);

	int num_controllers = 4;	//8xx has 4 controller 
	int ret=is_nlm_xlp(300, XLP_REVISION_ANY, CPU_EXTPID_XLP_3XX_ANY);
	if( ret ) num_controllers = 2;
	else
	{
		ret=is_nlm_xlp(200, XLP_REVISION_ANY, 0 );
		if( ret ) num_controllers = 1;
	}

	for(n=0; n<num_controllers; n++)
	{
		log1 = nlm_hal_read_32bit_reg(mmio, n*0x80+0x11D);
		log2 = nlm_hal_read_32bit_reg(mmio, n*0x80+0x11E);
		if( ( log1 & 1 ) == 0 ) continue;

		if(error==0)	cerr_printk("DRAM exception: \n");
		cerr_printk("  Bank (%c): log1=0x%08x, log1=0x%08x\n", (char)(n+'A'),log1, log2);
		cerr_printk("       Syndrome: %x\n",  (log1>>16) );
		cerr_printk("     Error Type: %x\n",  (log1>>8) & 0xF );
		cerr_printk("       Opt Type: %x\n",  (log1>>4) & 0xF );
		cerr_printk("  Uncorrectable: %d\n",  (log1>>2) & 1 );
		cerr_printk("       Overflow: %d\n",  (log1>>1) & 1 );
		cerr_printk("    Error Valid: %d\n",  log1&1 );

		cerr_printk("    Error Rank: %x\n", (log2>>12)&3 );
		cerr_printk("       ChunkID: %d 16-byte chunk\n", (log2>>8)&3 );
		cerr_printk(" Error Bit pos: %d\n",  log1&0x3F );

		error++;
	}

	return 0 < error ? -1 : 0;
}

static char *c_nbu_reqsrc[] = {
	[0] = "Core 0",
	[1] = "Core 1",
	[2] = "Core 2",
	[3] = "Core 3",
	[4] = "Core 4",
	[5] = "Core 5",
	[6] = "Core 6",
	[7] = "Core 7",
	[8] = "L3",
	[9] = "DRAM",
	[10] = "IO",
	[11] = "GCU",
	[12] = "NBU",
	[13] = "Invalid",
	[14] = "Invalid",
	[15] = "Invalid",
};

static char *c_nbu_reqtype[] = {
	[0] = "Invalidate",
	[1] = "Read",
	[2] = "Read Exclusive",
	[3] = "Read Upgrade",
	[4] = "Writeback",
	[5] = "IO Read",
	[6] = "IO Write",
	[7] = "IO Read Exclusive",
};

static int check_NBU_error(void)
{
	int node = hard_smp_processor_id() / NLM_MAX_CPU_PER_NODE;
	uint64_t nbu_mmio = nlm_hal_get_dev_base(node, 0, NLH_BRIDGE, 0);

	uint32_t nbu_reg0 = nlm_hal_read_32bit_reg(nbu_mmio, 0xA2);
	uint32_t nbu_reg1 = nlm_hal_read_32bit_reg(nbu_mmio, 0xA3);
	uint32_t nbu_reg2 = nlm_hal_read_32bit_reg(nbu_mmio, 0xA4);

	uint8_t src  = (nbu_reg0 >> 7) & 0xF;
	uint8_t type = (nbu_reg0 >> 4) & 0x7;
	uint8_t overflow = (nbu_reg0 >> 3) & 0x1;
	uint8_t valid = (nbu_reg0 >> 2) & 0x1;
	uint64_t phyaddr = ( (uint64_t)nbu_reg2 << 32) | nbu_reg1;

	if( valid == 0 && phyaddr == 0)	return 0;

	cerr_printk("NBU Cache %s:\n", valid ? "Error" : "Warning" );
	cerr_printk("   log0=0x%08x, phyaddr=0x%016llx\n", nbu_reg0, phyaddr);

	if( valid == 0 )
	{
		nlm_hal_write_32bit_reg(nbu_mmio, 0xA3, 0);
		nlm_hal_write_32bit_reg(nbu_mmio, 0xA4, 0);
		return 1;
	}
	else
	{
		cerr_printk("          ReqSrc: %s\n", c_nbu_reqsrc[src] );
		cerr_printk("         ReqType: %s\n", c_nbu_reqtype[type] );
		cerr_printk("        Overflow: %d\n", overflow);
		cerr_printk("  	  Error Valid: %d\n", valid);
		return -1;
	}
}

static int  print_cerr_info(void)
{
	int ret = 0, error=0, warning=0 ;

	print_error_header();
	dump_cerr_info();

	ret = check_COP2_error();
	if(ret < 0) error++;
	else if(ret > 0) warning++;

	ret = check_ICU_error();
	if(ret < 0) error++;
	else if(ret > 0) warning++;

	ret = check_LSU_error();
	if(ret < 0) error++;
	else if(ret > 0) warning++;

	ret = check_SCU_error();
	if(ret < 0) error++;
	else if(ret > 0) warning++;

	ret = check_L3_error();
	if(ret < 0) error++;
	else if(ret > 0) warning++;

	ret = check_NBU_error();
	if(ret < 0) error++;
	else if(ret > 0) warning++;

	ret = check_DRAM_error();
	if(ret < 0) error++;
	else if(ret > 0) warning++;

	ret = 0;
	if( 0 < error ) ret = -1;
	else if( warning <= 0 )
	{
		cerr_printk(" Unknown Cache error in exception handler!\n");
		ret = -1;
	}

	return ret;
}
#endif

/* On XLR/XLP, errors reported by bridge (like misconfigured BARS etc) are also
 * reported as cache errors. Need to check if it is really a cache error or a "bus error"
 * and take action appropriately.
 * For now, treat it as a cache error
 */
asmlinkage void nlm_cache_error(void)
{
	int ret = 0 ;

	ret = print_cerr_info();

	if( ret == 0 )	ret=update_registers();

	if( ret == 0 )	cerr_printk("\n ------- CPU return to normal execution! ------------\n\n");
	else
	{
		cerr_printk("Can not handle bus/cache error - Halting cpu\n\n");
		cerr_cpu_halt();
	}
}

static int update_registers(void)
{
	unsigned long errepc;
	struct pt_regs regs;
	int ret=0;

	load_registers(&regs, &errepc);

	ret=compute_return_registers(&regs, &errepc);
	if(ret!=0)
	{
		cerr_printk("  Update registers fail! CPU could not return to normal execution!\n");
		return -1;
	}

	restore_registers(&regs, errepc);
	return 0;
}

static void load_registers(struct pt_regs *regs, unsigned long *pErrepc)
{
	int iRegPos, n;
	uint64_t *pReg;
	unsigned long k0, k1;

	if(regs==NULL || pErrepc==NULL)	return;
	memset(regs, 0, sizeof(*regs));

	//copy original registers value from stack
	iRegPos=0x2000-40*8;
	pReg = (uint64_t*)( nlm_cerr_stack + iRegPos );
	for( n=0; n<32; n++) regs->regs[n] = (unsigned long)pReg[n] ;

	//copy original register: sp, ra, errepc
	pReg=(uint64_t*)(nlm_cerr_stack + 0x2000);	//top of the stack
	regs->regs[29] = *(pReg - 1);	//sp
	regs->regs[31] = *(pReg - 2);	//ra
	*pErrepc = *( pReg -3 );		//ErrEpc

	//copy k0, k1 from scratch register
	#ifdef CONFIG_64BIT
	__asm__ volatile(
		".set push         \n"
		".set mips64r2     \n"
		"dmfc0 %0, $22, 3  \n"
		"dmfc0 %1, $22, 4  \n"
		".set pop          \n"
		:"=r" (k0), "=r" (k1)
	);
	#else	//32 bits
	__asm__ volatile(
		".set push         \n"
		".set mips64r2     \n"
		"mfc0 %0, $22, 3   \n"
		"mfc0 %1, $22, 4   \n"
		".set pop          \n"
		:"=r" (k0), "=r" (k1)
	);
	#endif //CONFIG_64BIT

	regs->regs[26] = k0 ;
	regs->regs[27] = k1 ;
}

static void restore_registers(struct pt_regs *regs, unsigned long errepc)
{
	int iRegPos, n;
	uint64_t *pReg;
	unsigned long  k0, k1;

	if(regs == NULL )	return;

	//copy original registers value from stack
	iRegPos=0x2000-40*8;
	pReg=(uint64_t*)(nlm_cerr_stack + iRegPos );
	for(n=0; n<32; n++) pReg[n] = regs->regs[n];

	//copy original register: sp, ra, errepc
	pReg=(uint64_t*)(nlm_cerr_stack + 0x2000);	//top of the stack
	*(pReg - 1) = regs->regs[29] ;  //sp
	*(pReg - 2) = regs->regs[31] ;  //ra
	*(pReg - 3) = errepc;           //ErrEpc

	//copy k0, k1 from scratch register
	k0 = regs->regs[26] ;
	k1 = regs->regs[27] ;

	#ifdef CONFIG_64BIT
	__asm__ volatile(
		".set push         \n"
		".set noreorder    \n"
		".set mips64r2     \n"
		"dmtc0 %0, $22, 3  \n"
		"dmtc0 %1, $22, 4  \n"
		".set pop          \n"
		::"r" (k0), "r" (k1)
	);
	#else  //32bits
	__asm__ volatile(
		".set push         \n"
		".set noreorder    \n"
		".set mips64r2     \n"
		"mtc0 %0, $22, 3   \n"
		"mtc0 %1, $22, 4   \n"
		".set pop          \n"
		::"r" (k0), "r" (k1)
	);
	#endif //CONFIG_64BIT
}

static int compute_return_registers(struct pt_regs *regs, unsigned long *pErrepc)
{
	unsigned int  *addr;
	unsigned int bit, fcr31, dspcontrol;
	unsigned long errepc;
	union mips_instruction insn;

	if(pErrepc==NULL)	return -1;

	/*	it has slight chance that prev_errepc may not in the tlb and tlb handling of prev_errepc may trigger
		some issues, or prev_errepc may point to non insn. Since we do not have enough informaiton, have to 
		access as normal.
 	*/
	errepc = *pErrepc;
	addr = (uint32_t*) errepc;
	insn.word = *addr;

	regs->regs[0] = 0;
	switch (insn.i_format.opcode) {
	// jr and jalr are in r_format format.
	case spec_op:
		switch (insn.r_format.func) {
		case jalr_op:
			regs->regs[insn.r_format.rd] = errepc + 8;
			// Fall through
		case jr_op:
			errepc = regs->regs[insn.r_format.rs];
			break;
		}
		break;

	/*
	 * This group contains:
	 * bltz_op, bgez_op, bltzl_op, bgezl_op,
	 * bltzal_op, bgezal_op, bltzall_op, bgezall_op.
	 */
	case bcond_op:
		switch (insn.i_format.rt) {
		case bltz_op:
		case bltzl_op:
			if ((long)regs->regs[insn.i_format.rs] < 0)
				errepc = errepc + 4 + (insn.i_format.simmediate << 2);
			else
				errepc += 8;
			break;

		case bgez_op:
		case bgezl_op:
			if ((long)regs->regs[insn.i_format.rs] >= 0)
				errepc = errepc + 4 + (insn.i_format.simmediate << 2);
			else
				errepc += 8;
			break;

		case bltzal_op:
		case bltzall_op:
			regs->regs[31] = errepc + 8;
			if ((long)regs->regs[insn.i_format.rs] < 0)
				errepc = errepc + 4 + (insn.i_format.simmediate << 2);
			else
				errepc += 8;
			break;

		case bgezal_op:
		case bgezall_op:
			regs->regs[31] = errepc + 8;
			if ((long)regs->regs[insn.i_format.rs] >= 0)
				errepc = errepc + 4 + (insn.i_format.simmediate << 2);
			else
				errepc += 8;
			break;
		case bposge32_op:
			if (!cpu_has_dsp)  return -10;

			dspcontrol = rddsp(0x01);

			if (dspcontrol >= 32) {
				errepc = errepc + 4 + (insn.i_format.simmediate << 2);
			} else
				errepc += 8;
			break;
		}
		break;

	/*
	 * These are unconditional and in j_format.
	 */
	case jal_op:
		regs->regs[31] = errepc + 8;
	case j_op:
		errepc += 4;
		errepc >>= 28;
		errepc <<= 28;
		errepc |= (insn.j_format.target << 2);
		break;

	/*
	 * These are conditional and in i_format.
	 */
	case beq_op:
	case beql_op:
		if (regs->regs[insn.i_format.rs] ==
		    regs->regs[insn.i_format.rt])
			errepc = errepc + 4 + (insn.i_format.simmediate << 2);
		else
			errepc += 8;
		break;

	case bne_op:
	case bnel_op:
		if (regs->regs[insn.i_format.rs] !=
		    regs->regs[insn.i_format.rt])
			errepc = errepc + 4 + (insn.i_format.simmediate << 2);
		else
			errepc += 8;
		break;

	case blez_op: /* not really i_format */
	case blezl_op:
		/* rt field assumed to be zero */
		if ((long)regs->regs[insn.i_format.rs] <= 0)
			errepc = errepc + 4 + (insn.i_format.simmediate << 2);
		else
			errepc += 8;
		break;

	case bgtz_op:
	case bgtzl_op:
		/* rt field assumed to be zero */
		if ((long)regs->regs[insn.i_format.rs] > 0)
			errepc = errepc + 4 + (insn.i_format.simmediate << 2);
		else
			errepc += 8;
		break;

	/*
	 * And now the FPA/cp1 branch instructions.
	 */
	case cop1_op:
		if (is_fpu_owner())
			asm volatile("cfc1\t%0,$31" : "=r" (fcr31));
		else
			fcr31 = current->thread.fpu.fcr31;

		bit = (insn.i_format.rt >> 2);
		bit += (bit != 0);
		bit += 23;
		switch (insn.i_format.rt & 3) {
		case 0:	/* bc1f */
		case 2:	/* bc1fl */
			if (~fcr31 & (1 << bit))
				errepc = errepc + 4 + (insn.i_format.simmediate << 2);
			else
				errepc += 8;
			break;

		case 1:	/* bc1t */
		case 3:	/* bc1tl */
			if (fcr31 & (1 << bit))
				errepc = errepc + 4 + (insn.i_format.simmediate << 2);
			else
				errepc += 8;
			break;
		}
		break;
	default:
		errepc += 4;
	}

	*pErrepc = errepc;
	return 0;
}
