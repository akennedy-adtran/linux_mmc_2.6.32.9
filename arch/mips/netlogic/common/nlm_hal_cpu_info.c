
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

#include "nlm_hal.h"
#include "nlm_hal_macros.h"
#include "nlm_hal_xlp_dev.h"

extern void *memset(void *s, int c, size_t n);
/* bitcount moved to nlm_hal.h - backport of 2.3.1 patch */

__inline__ uint32_t efuse_cfg0(void)
{
	return  nlm_hal_read_32bit_reg(SYS_REG_BASE, (SYS_REG_INDEX(EFUSE_DEVICE_CFG0)));
}

 __inline__ uint32_t efuse_cfg1(void)
{
	return  nlm_hal_read_32bit_reg(SYS_REG_BASE, (SYS_REG_INDEX(EFUSE_DEVICE_CFG1)));
}

 __inline__ uint32_t efuse_cfg2(void)
{
	return  nlm_hal_read_32bit_reg(SYS_REG_BASE, (SYS_REG_INDEX(EFUSE_DEVICE_CFG2)));
}

 __inline__ uint32_t efuse_cfg3(void)
{
	return  nlm_hal_read_32bit_reg(SYS_REG_BASE, (SYS_REG_INDEX(EFUSE_DEVICE_CFG3)));
}

__inline__ uint32_t efuse_cfg6(void)
{
	return  nlm_hal_read_32bit_reg(SYS_REG_BASE, (SYS_REG_INDEX(EFUSE_DEVICE_CFG6)));
}

int nlm_xlp2xx_has_cmp(void)
{
    unsigned int cfg1 = efuse_cfg1();
    return !(cfg1 & (1 << XLP2XX_CMP_BLOCK_INDEX));
}

int nlm_xlp2xx_has_crypto(void)
{
    unsigned int cfg1 = efuse_cfg1();
    return !(cfg1 & (1 << XLP2XX_CRYPTO_BLOCK_INDEX));
}

int nlm_xlp2xx_has_rsa(void)
{
    unsigned int cfg2 = efuse_cfg2();
    return !(cfg2 & (1 << XLP2XX_RSA_BLOCK_INDEX));
}

int nlm_xlp2xx_has_regx(void)
{
    unsigned int cfg2 = efuse_cfg2();
    return !(cfg2 & (1 << XLP2XX_REGX_BLOCK_INDEX));
}

uint32_t get_proc_id(void)
{
	unsigned int prid = nlm_read_prid();
	prid = (prid >> 8) & 0xff;
	if (!prid) { /*May be its Non fused part.*/
#ifdef  CONFIG_NETL3XX
		prid = CHIP_PROCESSOR_ID_XLP_3XX;
#elif  defined (CONFIG_NETL2XX)
		prid = CHIP_PROCESSOR_ID_XLP_2XX;
#else
		prid = CHIP_PROCESSOR_ID_XLP_8_4_XX;
#endif
	}
	return prid;
}

__inline__ int get_nlm_xlp8xx_rev(void)
{
	return nlm_read_prid() & 0xff;
}

__inline__ int get_nlm_xlp3xx_rev(void)
{
	int hw_rev = nlm_read_prid() & 0xff;
	int sw_rev;
	if(hw_rev>=XLP3XX_REVISION_B0)
		sw_rev = hw_rev + 1;
	else
		sw_rev = hw_rev;
	return sw_rev;
}

/* No Firefly A2 (like Storm) so handle sw rev the same as Storm */
__inline__ int get_nlm_xlp2xx_rev(void)
{
	int hw_rev = nlm_read_prid() & 0xff;
	int sw_rev;
	if(hw_rev>=XLP2XX_REVISION_B0)
		sw_rev = hw_rev + 1;
	else
		sw_rev = hw_rev;
	return sw_rev;
}

/***************************************************************************************
* match the chip revision with 'rev'
 * rev:  revision number
		single match: XLP_REVISION_A0 etc
		multi-match:  XLP_REVISION_AX/_BX/_XX
****************************************************************************************/
__inline__ int is_nlm_xlp8xx_rev_xx(uint32_t rev)
{
	int sw_rev=get_nlm_xlp8xx_rev();
	uint32_t rev_b0 = XLP_REVISION_B0;
	if( rev==sw_rev)        return 1;
	if( rev==XLP_REVISION_AX && sw_rev<rev_b0)	return 1;
	if( rev==XLP_REVISION_BX && (rev_b0<=sw_rev))  	return 1;
	if( rev==XLP_REVISION_ANY)			return 1;
	return 0;
}

/***************************************************************************************
* match the chip revision with 'rev'
 * rev:  revision number
		single match: XLP_REVISION_A0 etc
		multi-match:  XLP_REVISION_AX/_BX/_XX
****************************************************************************************/
__inline__ int is_nlm_xlp3xx_rev_xx(uint32_t rev)
{
	int sw_rev=get_nlm_xlp3xx_rev();
	uint32_t rev_b0 = XLP_REVISION_B0;
	if( rev==sw_rev)	return 1;
	if( rev==XLP_REVISION_AX && sw_rev<rev_b0)	return 1;
	if( rev==XLP_REVISION_BX && (rev_b0<=sw_rev))  	return 1;
	if( rev==XLP_REVISION_ANY)			return 1;

	return 0;
}


__inline__ int is_nlm_xlp2xx_rev_xx(uint32_t rev)
{
	int sw_rev=get_nlm_xlp2xx_rev();
	uint32_t rev_b0 = XLP_REVISION_B0;
	if( rev==sw_rev)	return 1;
	if( rev==XLP_REVISION_AX && sw_rev<rev_b0)	return 1;
	if( rev==XLP_REVISION_BX && (rev_b0<=sw_rev))  	return 1;
	if( rev==XLP_REVISION_ANY)			return 1;

	return 0;
}

/***************************************************************************************
match legacy eagle Ax: xlp832, xlp816, xlp432, xlp416, xlp408, xlp208, xlp204, xlp104
chipid: 832, 816, 432, 416, 408, 208, 204, 104; match any of the chip in eagle Ax
rev:    revision
***************************************************************************************/
inline int is_xlp8xx_legacy(int chipid, uint32_t rev)
{
	uint32_t pid=get_proc_id();
	uint32_t sw_rev = get_nlm_xlp8xx_rev();

	if( rev==XLP_REVISION_ANY )	rev=XLP_REVISION_AX;
	if( sw_rev!=rev && rev!=XLP_REVISION_AX )	return 0;
	if( XLP_REVISION_B0<=sw_rev ) return 0;
	if(	(( (chipid==0x8084) || (chipid==0x8000) ) &&  (pid==CHIP_PROCESSOR_ID_XLP_832)) ||
		(( (chipid==0x8044) || (chipid==0x8000) ) &&  (pid==CHIP_PROCESSOR_ID_XLP_816)) ||
		(( (chipid==0x4084) || (chipid==0x4000) ) &&  (pid==CHIP_PROCESSOR_ID_XLP_432)) ||
		(( (chipid==0x4044) || (chipid==0x4000) ) &&  (pid==CHIP_PROCESSOR_ID_XLP_416)) ||
		(( (chipid==0x4024) || (chipid==0x4000) ) &&  (pid==CHIP_PROCESSOR_ID_XLP_408)) )
		return 1;

    return 0;
}

/*
 * match xlp8xx
 * num_cpu: 32-xlp832, 24-xlp824, 16-xlp816, 0 any cpu number is valid match
 * rev:  chip revision,
 */
#define CPU_NUM_ANY  0 /* any cpu number will be matched */
inline int is_xlp8xx(uint8_t num_cpu, uint32_t rev)
{
	uint32_t pid, core_mask, cfg1;
	int ret, hw_cpu_num;

	ret=is_nlm_xlp8xx_rev_xx(rev);
	if(ret!=1)	return 0;

	pid=get_proc_id();

	if( pid == CHIP_PROCESSOR_ID_XLP_8_4_XX)
	{
		core_mask = efuse_cfg0() & 0xff;
		cfg1 = efuse_cfg1() & 0x7;

		hw_cpu_num=(8-bitcount(core_mask))<<2;
		if( ((hw_cpu_num==num_cpu)||(num_cpu==CPU_NUM_ANY)) && cfg1!=0x7 )	return 1;
	}

	return 0;
}

/*
 * match xlp4xx
 * num_cpu: 32-xlp832, 24-xlp824, 16-xlp816, 0xff any cpu number is valid match
 * rev:  chip revision,
 */
inline int is_xlp4xx(uint8_t num_cpu, uint32_t rev)
{
	uint32_t pid, core_mask, cfg1;
	int ret, hw_cpu_num;

	ret=is_nlm_xlp8xx_rev_xx(rev);
	if(ret!=1)  return 0;

	pid=get_proc_id();

	if( pid == CHIP_PROCESSOR_ID_XLP_8_4_XX )
	{
		core_mask = efuse_cfg0() & 0xff;
		cfg1 = efuse_cfg1() & 0x7;

		hw_cpu_num=(8-bitcount(core_mask))<<2;
		if( ((hw_cpu_num==num_cpu)||(num_cpu==CPU_NUM_ANY)) && cfg1==0x7 )	return 1;
	}

    return 0;
}

/*
 * match xlp3xx
 * num_cpu: 16-xlp316,  8-xlp308, 4-xlp304;  0xff any cpu number is valid match
 * rev:  chip revision,
 * type: CPU_EXTPID_XLP_3XX_NONE, CPU_EXTPID_XLP_3XX_L, CPU_EXTPID_XLP_3XX_LP, CPU_EXTPID_XLP_3XX_LP2
 * xlp316 : 4x4 threads
 * xlp308   2x4 threads
 * xlp304   1x4 threads
 * xlp208a  2x4 trheads
 * xlp108a  2x4 trheads
 * xlp204a  1x4 trheads
 * xlp104a  1x4 trheads
 * xlp202a  2x1 trheads
 * xlp201a  1x1 trheads
 * xlp101a  1x1 trheads
 */
inline int xlp3xx_get_num_of_threads_per_core(uint32_t core_mask, uint32_t epid)
{
	int nthreads=1;
	switch(epid) {
		case CPU_EXTPID_XLP_3XX_BASE :
		case CPU_EXTPID_XLP_3XX_L    :
		case CPU_EXTPID_XLP_3XX_LP   :
		case CPU_EXTPID_XLP_3XX_LP2  :
		case CPU_EXTPID_XLP_208a     :
		case CPU_EXTPID_XLP_108a     :
		case CPU_EXTPID_XLP_204a     :
		case CPU_EXTPID_XLP_104a     : nthreads = 4; break;
		case CPU_EXTPID_XLP_202a     :
		case CPU_EXTPID_XLP_201a     :
		case CPU_EXTPID_XLP_101a     : nthreads = 1; break;
		default: break;
	};
	return nthreads;
}

inline int xlp3xx_get_num_of_cores(uint32_t core_mask, uint32_t epid)
{
	int ncores=1;
	switch(epid) {
		case CPU_EXTPID_XLP_3XX_BASE :
		case CPU_EXTPID_XLP_3XX_L    :
		case CPU_EXTPID_XLP_3XX_LP   :
		case CPU_EXTPID_XLP_3XX_LP2  :
		case CPU_EXTPID_XLP_208a     :
		case CPU_EXTPID_XLP_108a     :
		case CPU_EXTPID_XLP_204a     :
		case CPU_EXTPID_XLP_104a     :
		{
 			ncores = 4-bitcount(core_mask);
			break;
		}
		case CPU_EXTPID_XLP_202a     : ncores = 2; break;
		case CPU_EXTPID_XLP_201a     :
		case CPU_EXTPID_XLP_101a     : ncores = 1; break;
		default: break;
	};
	return ncores;
}

inline int is_xlp3xx(uint8_t num_cores, uint8_t num_threads, uint32_t rev, uint32_t exttype)
{
	uint32_t pid, cfg0, core_mask;
	uint8_t epid;
	int ret, ncores, nthreads;

	ret=is_nlm_xlp3xx_rev_xx(rev);
	if(ret!=1)  return 0;

	pid=get_proc_id();
	if( pid == CHIP_PROCESSOR_ID_XLP_3XX )
	{
		cfg0=efuse_cfg0();
		core_mask = cfg0  & 0xf;
		epid = (uint8_t)(( cfg0>>4 )  & 0xf);

		if( exttype ==CPU_EXTPID_XLP_3XX_ANY)
			return 1;
		else if (exttype == epid ) {
			ncores=xlp3xx_get_num_of_cores(core_mask, epid);
			nthreads=xlp3xx_get_num_of_threads_per_core(core_mask, epid);
			if ((num_cores*num_threads) == CPU_NUM_ANY) return 1;
  			else if ((ncores == num_cores) && (nthreads == num_threads)) return 1;
		}
	}

    return 0;
}

/*
 * match xlp2xx
 * num_cpu: 8-xlp208, 4-xlp204;  0xff any cpu number is valid match
 * rev:  chip revision,
 */
inline int is_xlp2xx(uint8_t num_cores, uint8_t num_threads, uint32_t rev)
{
	uint32_t pid, cfg0, core_mask;
	int ret, ncores, nthreads;
	int xlp2xx_threads_tbl[4] = { 4, 2, 2, 1 };

	ret=is_nlm_xlp2xx_rev_xx(rev);
	if(ret!=1)  return 0;

	pid=get_proc_id();
	if(pid == CHIP_PROCESSOR_ID_XLP_2XX)
	{
		cfg0=efuse_cfg0();
		core_mask = cfg0  & 0x3;
		nthreads = xlp2xx_threads_tbl[(cfg0 >> 28) & 0x3];
		ncores   = (2-bitcount(core_mask));
		if ((num_cores*num_threads) == CPU_NUM_ANY) return 1;
		else if ((ncores == num_cores) && (nthreads == num_threads)) return 1;
	}
	return 0;
}

/*
 * match xlpxx
 * chipid: 832, 316, 308, 208, etc, 800: any in 8xx group, 0 for all xlp group
 * rev:    XLP_REVISION_A0, XLP_REVISION_A0 etc, or XLP_REVISION_AX (a0,a1,a2) XLP_REVISION_ANY(both: ax,bx)
 * exttype: current only for 3xx:
 *  CPU_EXTPID_XLP_3XX_BASE  0x00
 *  CPU_EXTPID_XLP_3XX_L    0x01
 *  CPU_EXTPID_XLP_3XX_LP   0x02
 *  CPU_EXTPID_XLP_3XX_LP2  0x03
 *  CPU_EXTPID_XLP_208a     0x06
 *  CPU_EXTPID_XLP_108a     0x07
 *  CPU_EXTPID_XLP_204a     0x05
 *  CPU_EXTPID_XLP_104a     0x04
 *  CPU_EXTPID_XLP_202a     0x08
 *  CPU_EXTPID_XLP_201a     0x09
 *  CPU_EXTPID_XLP_101a     0x0A
 */
int is_nlm_xlp(unsigned int chipid, unsigned int rev, unsigned int exttype)
{
	uint32_t group=(chipid>>12) & 0xF;
	uint32_t num_cores=(chipid>>4) & 0xFF;
	uint32_t num_threads_per_core=chipid & 0xF;
	uint8_t num_cpu=num_cores*num_threads_per_core;
	int b_rc=0;

	if ( group==8 )
	{
		b_rc=is_xlp8xx(num_cpu, rev);
		if( b_rc==1 )	return 1;
	}

	if ( group==4 )
	{
		b_rc=is_xlp4xx(num_cpu, rev);
		if( b_rc==1 )	return 1;
	}

	if ( group==3 )
	{
		b_rc=is_xlp3xx(num_cores, num_threads_per_core, rev, exttype);
		return b_rc;
	}

	if ( group==2 )
	{
		b_rc=is_xlp2xx(num_cores, num_threads_per_core, rev);
		return b_rc;
	}

	/* for legacy chips: */
	if( rev<=XLP_REVISION_A2 || rev==XLP_REVISION_AX || rev==XLP_REVISION_ANY )
		b_rc=is_xlp8xx_legacy(chipid, rev);

	return b_rc;
}

int nlm_hal_get_chipid(void)
{
	int xlp2xx_threads_tbl[4] = { 4, 2, 2, 1 };
	int chipid=-1, ncores, nthreads;
	unsigned int cfg0, cfg1;
	uint32_t pid=get_proc_id();

	cfg0 = efuse_cfg0();
	cfg1 = efuse_cfg1();

	/* eagle 4xx 8xx BX, unfused chip treat as 8xx */
	if(pid==0 || pid==CHIP_PROCESSOR_ID_XLP_8_4_XX)
	{
		ncores=(8-bitcount(cfg0&0xFF));
		nthreads = 4;
		chipid = (cfg1&7)==7 ? 0x4000 : 0x8000;
		chipid+= ((ncores<<4) + nthreads);
		return chipid;
	}
	if(pid==CHIP_PROCESSOR_ID_XLP_3XX)
	{
		ncores=(4-bitcount(cfg0&0xF));
		nthreads = 4;
		chipid = (0x3000 + (ncores<<4) + nthreads);
		return chipid;
	}
	if (pid == CHIP_PROCESSOR_ID_XLP_2XX)
	{
		ncores=(2-bitcount(cfg0&0x3));
		nthreads = xlp2xx_threads_tbl[(cfg0 >> 28) & 0x3];
		chipid = (0x2000 + (ncores<<4) + nthreads);
		return chipid;
	}
	/* eagle 8xx AX */
	switch(pid)
	{
	case CHIP_PROCESSOR_ID_XLP_832 :    chipid=0x8084;   break;
	case CHIP_PROCESSOR_ID_XLP_816 :    chipid=0x8044;   break;
	case CHIP_PROCESSOR_ID_XLP_432 :    chipid=0x4084;   break;
	case CHIP_PROCESSOR_ID_XLP_416 :    chipid=0x4044;   break;
	case CHIP_PROCESSOR_ID_XLP_408 :    chipid=0x4024;   break;
	default: break;
	}

	return chipid;
}

static const char* nlm_hal_get_chipid_str(void)
{
	if(is_nlm_xlp8xx_832()) return "XLP832";
	if(is_nlm_xlp8xx_824()) return "XLP824";
	if(is_nlm_xlp8xx_816()) return "XLP816";

	if(is_nlm_xlp8xx_432()) return "XLP432";
	if(is_nlm_xlp8xx_424()) return "XLP424";
	if(is_nlm_xlp8xx_416()) return "XLP416";

	if(is_nlm_xlp316()) return "XLP316";
	if(is_nlm_xlp312()) return "XLP312";
	if(is_nlm_xlp308()) return "XLP308";
	if(is_nlm_xlp304()) return "XLP304";

	if(is_nlm_xlp3xx_208a()) return "XLP3XX_208a";
	if(is_nlm_xlp3xx_108a()) return "XLP3XX_108a";

	if(is_nlm_xlp3xx_204a()) return "XLP3XX_204a";
	if(is_nlm_xlp3xx_104a()) return "XLP3XX_104a";

	if(is_nlm_xlp3xx_202a()) return "XLP3XX_202a";
	if(is_nlm_xlp3xx_201a()) return "XLP3XX_201a";
	if(is_nlm_xlp3xx_101a()) return "XLP3XX_101a";

	if(is_nlm_xlp208()) return "XLP208";
	if(is_nlm_xlp108()) return "XLP108";

	if(is_nlm_xlp204()) return "XLP204";
	if(is_nlm_xlp104()) return "XLP104";

	if(is_nlm_xlp202()) return "XLP202";
	if(is_nlm_xlp201()) return "XLP201";
	if(is_nlm_xlp101()) return "XLP101";

	return "XLP???";
}

int  nlm_hal_get_cpuinfo(struct nlm_netl_proc_info* cpu_info)
{
	const static char c_typename[][8]={"\0", "Lite", "Lite+", "Lite+2", "Unknown"};
	unsigned int sw_rev, chipid, type, i;
	uint32_t cfg0, pid;
	const char* chipid_str;

	pid=get_proc_id();
	chipid=nlm_hal_get_chipid();

	memset(cpu_info, 0, sizeof(struct nlm_netl_proc_info));
	cpu_info->proc_id=pid;
	cpu_info->chipid=chipid;

	for(i=0; i<8; i++) {
		cpu_info->efuse_config[i] =
			nlm_hal_read_32bit_reg(SYS_REG_BASE, (SYS_REG_INDEX(EFUSE_DEVICE_CFG0+i)));
	}
	chipid_str = nlm_hal_get_chipid_str();

	if( pid == CHIP_PROCESSOR_ID_XLP_2XX )
	{
		sw_rev = get_nlm_xlp2xx_rev();
		cpu_info->revision= sw_rev;
		sprintf(cpu_info->cpu_info_str, "%s Rev %c%1d",
			chipid_str,(char)('A'+sw_rev/XLP_REVISION_B0), sw_rev%XLP_REVISION_B0);
	}
	else if( pid != CHIP_PROCESSOR_ID_XLP_3XX )
	{
		sw_rev = get_nlm_xlp8xx_rev();
		cpu_info->revision= sw_rev;
		sprintf(cpu_info->cpu_info_str, "%s Rev %c%1d",
			chipid_str,(char)('A'+sw_rev/XLP_REVISION_B0), sw_rev%XLP_REVISION_B0);
	}
	else
	{
		sw_rev = get_nlm_xlp3xx_rev();
		cpu_info->revision = sw_rev;
		cfg0 =  efuse_cfg0();
		type= (cfg0>>4) & 0xF;
		if(CPU_EXTPID_XLP_3XX_LP2<type) type=CPU_EXTPID_XLP_3XX_LP2+1;
		sprintf(cpu_info->cpu_info_str, "%s%s Rev %c%1d", chipid_str, c_typename[type],
			(char)('A'+sw_rev/XLP_REVISION_B0), sw_rev%XLP_REVISION_B0);
	}

	return 0;
}

int is_nlm_xlp208(void)
{
	return ( is_nlm_xlp(0x2024, XLP_REVISION_ANY, 0)
			&& (nlm_xlp2xx_has_cmp())
			&& (nlm_xlp2xx_has_crypto())
			&& (nlm_xlp2xx_has_rsa())
			&& (nlm_xlp2xx_has_regx()) );
}

int is_nlm_xlp108(void)
{
	return ( is_nlm_xlp(0x2024, XLP_REVISION_ANY, 0)
			&& (!nlm_xlp2xx_has_cmp())
			&& (nlm_xlp2xx_has_crypto())
			&& (nlm_xlp2xx_has_rsa())
			&& (nlm_xlp2xx_has_regx()) );
}

int is_nlm_xlp204(void)
{
	return ( is_nlm_xlp(0x2014, XLP_REVISION_ANY, 0)
				&& (nlm_xlp2xx_has_cmp())
				&& (nlm_xlp2xx_has_crypto())
				&& (nlm_xlp2xx_has_rsa())
				&& (nlm_xlp2xx_has_regx()) );
}

int is_nlm_xlp104(void)
{
	return ( is_nlm_xlp(0x2014, XLP_REVISION_ANY, 0)
				&& (!nlm_xlp2xx_has_cmp())
				&& (nlm_xlp2xx_has_crypto())
				&& (nlm_xlp2xx_has_rsa())
				&& (nlm_xlp2xx_has_regx()) );
}

int is_nlm_xlp202(void)
{
	return ( is_nlm_xlp(0x2021, XLP_REVISION_ANY, 0)
				&& (!nlm_xlp2xx_has_cmp())
				&& (!nlm_xlp2xx_has_crypto())
				&& (!nlm_xlp2xx_has_rsa())
				&& (!nlm_xlp2xx_has_regx()) );
}

int is_nlm_xlp201(void)
{
	return ( is_nlm_xlp(0x2011, XLP_REVISION_ANY, 0)
				&& (!nlm_xlp2xx_has_cmp())
				&& (!nlm_xlp2xx_has_crypto())
				&& (!nlm_xlp2xx_has_rsa())
				&& (nlm_xlp2xx_has_regx()) );
}

int is_nlm_xlp101(void)
{
	return ( is_nlm_xlp(0x2011, XLP_REVISION_ANY, 0) \
				&& (!nlm_xlp2xx_has_cmp()) \
				&& (!nlm_xlp2xx_has_crypto()) \
				&& (!nlm_xlp2xx_has_rsa()) \
				&& (!nlm_xlp2xx_has_regx()) );
}

#ifdef NLM_HAL_LINUX_KERNEL
#include <linux/types.h>
#include <linux/module.h>
/*Add  API here if any API  from above is needed*/
EXPORT_SYMBOL(efuse_cfg0);
EXPORT_SYMBOL(efuse_cfg1);
EXPORT_SYMBOL(efuse_cfg6);
EXPORT_SYMBOL(get_proc_id);
EXPORT_SYMBOL(is_nlm_xlp);
EXPORT_SYMBOL(nlm_hal_get_cpuinfo);
EXPORT_SYMBOL(is_nlm_xlp208);
EXPORT_SYMBOL(is_nlm_xlp108);
EXPORT_SYMBOL(is_nlm_xlp204);
EXPORT_SYMBOL(is_nlm_xlp104);
EXPORT_SYMBOL(is_nlm_xlp202);
EXPORT_SYMBOL(is_nlm_xlp201);
EXPORT_SYMBOL(is_nlm_xlp101);
#endif
