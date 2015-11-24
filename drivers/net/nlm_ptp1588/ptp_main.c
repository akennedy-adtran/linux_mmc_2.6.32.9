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

#include <linux/module.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>	
#include <linux/slab.h>		
#include <linux/fs.h>		
#include <linux/errno.h>	
#include <linux/types.h>	
#include <linux/proc_fs.h>
#include <linux/fcntl.h>	
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/time.h>
#include <linux/skbuff.h>

//#include <linux/config.h>
#include <linux/init.h>
#include <asm/netlogic/debug.h>
#include <asm/netlogic/pci.h>
#include <asm/netlogic/pic.h>
#include <asm/netlogic/xlr_mac.h>
#include <asm/netlogic/mips-exts.h>
#include <asm/netlogic/msgring.h>
#include <asm/netlogic/sim.h>
#include <asm/netlogic/xlr_user_mac.h>
#include <asm/netlogic/atx_cpld.h>
#include <asm/netlogic/xgmac_mdio.h>
#include <asm/netlogic/proc.h>
#include <asm-mips/smp.h>
#include <asm-mips/rmi/iomap.h>
#include <asm-mips/rmi/gpio.h>
#include <user/rmi/xlr_user_mac.h>
#include <asm-mips/div64.h>
#include "ptp_common.h"
#include "ptp_mod.h"
extern struct psb_info *prom_info;
void dump_all_interface(u32 reg);
u32 nlm_macreg_get(u32 reg, u32 val);
void nlm_macreg_set_all(u32 reg, u32 val, u32 mask);
//int nlm_macreg_set_all(int inf, u32 reg, u32 val, u32 mask);
int  nlm_macreg_set(int inf, u32 reg, u32 val);
void nlm_register_ptp_ts_fp(void (*fp) (u32,u32,ktime_t *, u32));
void nlm_clr_ptp_ts_fp(void);
void tgl_timer_bit(u32, u32);
int nlm_mac_get_inf_idx(char *infname);


dev_t dev;
struct cdev cdev;
spinlock_t ptp_lock;
struct timespec g_srttime;
intf_type_t intf;

#define GMAC_CORE_0                 0
#define GMAC_CORE_1                 4
ptp_clk_t clk;
ptp_ts_t     ptp_ts;
#define PTP1588_CONTROL                 0x077

#define PTP1588_FREQ_MUL                3    
#define PTP1588_FREQ_INS                2
#define PTP_TICKS_PER_SEC               66666666ULL // assume 66MHZ
#define PTP_FRM_MASK                    0x3
#define ANY_INF                         2
#define CLK_SRC_DIV                    7
#define CLK_FREQ_DIV                   10000000
#define dbg_ptp(...)            

int ptp_ioctl(struct inode *inode, struct file *filp, u32 cmd, unsigned long arg);
   
int ptp_open(struct inode *inode, struct file *filp);

struct file_operations ptp_fops = {
        .owner = THIS_MODULE,
        .ioctl = ptp_ioctl,
        .open = ptp_open,
    };
#if 0
unsigned long tmp_buff[1000];
void dump_offset(void)
{
    static u32 prev = 0, cur = 0, idx = 0, iter = 0, cur_sec = 0 , 
               cur_msec= 0 ,prev_sec = 0, prev_msec = 0, cnt = 0; 
    int i = 0;
    u32  msb = 0;
    u64 cur_tic, tmp_tic;
    static u64 prev_tic = 0;
    tgl_timer_bit(1<<12, 1<<12);
    cur = nlm_macreg_get( 2, PTP_TIMER_LATCH_VAL);
    msb = nlm_macreg_get( 2, PTP_TIMER_LATCH_VAL1);

 
//    get_cur_clk_val( &cur, &msb);
    tmp_tic = cur_tic = ((u64)msb <<32|cur); 
    #if 0
    dbg_ptp("dump %u %u off %u systic %lu\n", nlm_macreg_get( 2, PTP_TIMER_LATCH_VAL1), cur, 
                                   (cur - prev),   nlm_common_timer_get_stats(0)); 
    #endif
    tmp_buff[idx] = (cur - prev);
    if(!(idx = (++idx)%1000)) {
        iter++;
     //   for(i = 0; i < 1000 ; i ++) {
            dbg_ptp("iter %d off[%d] %lu\n",iter, i, tmp_buff[0]);
       // }
    }
    cur_msec  = do_div(tmp_tic, PTP_TICKS_PER_SEC);
    cur_sec = tmp_tic;
//    dbg_ptp("cur %lu %lu diff %lu %lu\n", cur_sec, cur_msec, 
//                                        (cur_sec - prev_sec), (cur_msec- prev_msec));
    if(!(cnt%1000))
        dbg_ptp("%u num seco %d tic %lld curtic %lld sec %u %u\n", cnt, cnt/1000, cur_tic - prev_tic, cur_tic, cur_sec, cur_msec);
    prev_tic = cur_tic;
    prev_sec = cur_sec;
    prev_msec = cur_msec;

    cnt ++; 
    
    prev = nlm_macreg_get( 2, PTP_TIMER_LATCH_VAL);
}
#endif



void tgl_timer_bit(u32 ctr, u32 mask)
{
    nlm_macreg_set_all( PTP1588_CONTROL, 0, mask);
    nlm_macreg_set_all( PTP1588_CONTROL, ctr, mask);
    return;
}

int get_cur_clk_val( u32 *cur, u32 *msb)
{    
    tgl_timer_bit( 1<<O_PTP_CTRL_RTC_LATCH, NUM_BITS_1<<O_PTP_CTRL_RTC_LATCH);
    *cur = nlm_macreg_get( ANY_INF, PTP_TIMER_LATCH_VAL);
    *msb = nlm_macreg_get( ANY_INF, PTP_TIMER_LATCH_VAL1);
   return 0;
}

void set_prepad_frm(u32 frm)
{
    nlm_macreg_set_all( R_RX_CONTROL, frm<<O_PTP_Rx1588TS, NUM_BITS_2<<O_PTP_Rx1588TS);
    dbg_ptp("set format %x\n", 1<<11);
    return;
}


void set_prepad(u32 enable)
{
    nlm_macreg_set_all( R_DESC_PACK_CTRL, (enable<<O_DESC_PACK_CTRL__PrePadEnable) , (NUM_BITS_1<<O_DESC_PACK_CTRL__PrePadEnable));
    return;
}

/* 

    Assume only one instance of ptp can run in the box
    Convert Ticks to seconds and nano seconds
 
*/
void __inline__ ptp_set_tx_ts(u32 sec , u32 usec)
{
    spin_lock(&ptp_lock);
    ptp_ts.ts_msb = usec;
    ptp_ts.ts_lsb = sec; 
    spin_unlock(&ptp_lock);
}

void ptp_set_ts(u32 msb, u32 lsb, ktime_t *tv, u32 flags)
{
   u64 cur_tic = ((u64)msb <<32|lsb) ;
   // total nano seconds 
   u32 sec = 0, nsec = 0;
  
  nsec = (do_div( cur_tic, PTP_TICKS_PER_SEC));
  sec = cur_tic;
  nsec += g_srttime.tv_nsec;
  sec  += g_srttime.tv_sec ;

  if(!tv) {
    ptp_set_tx_ts( sec , nsec/1000);
#if 0 
       dbg_ptp(" TX TV %u %u hex %x %x \n", ptp_ts.ts_msb, ptp_ts.ts_lsb, 
                                           ptp_ts.ts_msb, ptp_ts.ts_lsb);
#endif

    } else {
	    *tv = ktime_set(sec, nsec);
 #if 0 
   struct timespec l_time;
       l_time       = current_kernel_time(); 
       dbg_ptp("RX TV sec %u  usec%u hex %x %x\n", tv->off_sec, tv->off_usec,
                                         tv->off_sec, tv->off_usec); 
       dbg_ptp("gxtime %lu %lu\n", g_srttime.tv_sec, g_srttime.tv_nsec);
       dbg_ptp("xtime  %lu %lu\n", l_time.tv_sec, l_time.tv_nsec);
       dbg_ptp("sxtime %u %u\n",  tv->off_sec , 
                                   tv->off_usec);
#endif
    }

    return;
}

void ptp_get_tx_ts(ptp_ts_t *ts)
{
    spin_lock(&ptp_lock);
    ts->ts_msb = ptp_ts.ts_msb; 
    ts->ts_lsb = ptp_ts.ts_lsb; 
    spin_unlock(&ptp_lock);
    return;
}
int  ptp_set_clk(ptp_clk_t *clk, int flag)
{ 
 dbg_ptp("clk %d,div %d, den %d, num %d, off0 %d, off1 %d\n", clk->src_clk, clk->src_div,
                                                             clk->frac_div, clk->frac_mul,
                                                             clk->offset0, clk->offset1);

 nlm_macreg_set_all( PTP_SOURCE     , clk->src_clk , NUM_BITS_1); 
 nlm_macreg_set_all( PTP_SOURCE_DIV , clk->src_div , NUM_BITS_3); 
 nlm_macreg_set_all( PTP_FRAC_DIV   , clk->frac_div, NUM_BITS_32); 
 nlm_macreg_set_all( PTP_FRAC_MUL   , clk->frac_mul, NUM_BITS_32); 
 nlm_macreg_set_all( PTP_OFFSET0    , clk->offset0 , NUM_BITS_32); 
 nlm_macreg_set_all( PTP_OFFSET1    , clk->offset1 , NUM_BITS_32);
 return 0;
}

int ptp_open(struct inode *inode, struct file *filp)
{
    dbg_ptp("%s\n", __FUNCTION__);
    return 0;
}

int init_ptp(void)
{
u32  cur = 0 , msb = 0, biu_clk_div = 1;
u64  cur64, tmp64;
u32 gpio_reset_cfg = 0;
nlm_reg_t *gpio_mmio =
                    (nlm_reg_t *)(netlogic_io_base + NETLOGIC_IO_GPIO_OFFSET); 
u64 clk_freq = prom_info->cpu_frequency;

gpio_reset_cfg = netlogic_read_reg(gpio_mmio,NETLOGIC_GPIO_PWRON_RESET_CFG_REG);

 do_div( clk_freq, CLK_SRC_DIV);
 if(gpio_reset_cfg & (PTP_BIU_HALF_CLOCK) ) {
        biu_clk_div = 2;
  } else {
        biu_clk_div = 1;
  }
 memset( &clk, 0, sizeof(ptp_clk_t)); 
 
 clk.src_clk  = PTP_CLK_SRC_CORE; 
// (Pic clock hz)/(clock hz/src_div)
 clk.frac_mul = 4444444*biu_clk_div;
 dbg_ptp("multi %d \n", (PTP_TICKS_PER_SEC*1000000/(u32)clk_freq));
// clk.frac_mul = (prom_info->cpu_frequency)*biu_clk_div;

 clk.frac_div = CLK_FREQ_DIV;
 clk.src_div  = CLK_SRC_DIV;

// Reset clock 
 tgl_timer_bit(1, NUM_BITS_1);

/*  Set clock
    u need to toggle the control bits for the setting to work,hardware anomaly */

 ptp_set_clk( &clk, 0);

//set prepad format
 set_prepad(1);
 set_prepad_frm(2);

 tgl_timer_bit( (1<<PTP1588_FREQ_MUL), (1<<PTP1588_FREQ_MUL));
//Register timestamp 
nlm_register_ptp_ts_fp(ptp_set_ts);



// store the number the "time zero"

 tgl_timer_bit((1<<O_PTP_CTRL_RTC_LATCH), (1<<O_PTP_CTRL_RTC_LATCH));
 cur = nlm_macreg_get( ANY_INF, PTP_TIMER_LATCH_VAL);
 msb = nlm_macreg_get( ANY_INF, PTP_TIMER_LATCH_VAL1);
   
 tmp64=  cur64 = ((u64)msb<<32 | cur);
 g_srttime=  current_kernel_time();

 dbg_ptp("INIT gstart %lu %lu \n", g_srttime.tv_sec , g_srttime.tv_nsec);
 g_srttime.tv_nsec -= do_div(cur64, PTP_TICKS_PER_SEC);;
 g_srttime.tv_sec  -= (u32) cur64;

 dbg_ptp("INIT start %llu sec %lu nsec %lu \n", tmp64, g_srttime.tv_sec, g_srttime.tv_nsec );
 return 0;
}

void deinit_ptp(void)
{
 dbg_ptp("%s\n", __FUNCTION__); 
// set_prepad(0);
 //set_prepad_frm(0);
 tgl_timer_bit(( 1<<PTP1588_FREQ_MUL) , ( 1<<PTP1588_FREQ_MUL));
 nlm_clr_ptp_ts_fp();
 return;
}

int ptp_ioctl(struct inode *inode, struct file *filp,
                 u32 cmd, unsigned long arg)
{
int  rc = 0;
ptp_ts_t ptp;
ptp_clk_t ptp_clk;

switch (cmd) {
    case PTP_INIT:
        dbg_ptp("init %s\n", __FUNCTION__);
        rc= init_ptp();
        break;
    case PTP_GET_INF_IDX:
        break;
    case PTP_SET_INF_IDX:
        copy_from_user( &intf, (const char __user *)arg, sizeof(intf_type_t));
        dbg_ptp("intfname %s num %ld\n", &intf.name[0][0], intf.numif); 
        break;
    case PTP_TX_TIMESTAMP:  
        dbg_ptp("%s %d\n", __FUNCTION__, cmd);
        memset( &ptp,  0 , sizeof(ptp));
        ptp_get_tx_ts(&ptp);   
        copy_to_user((void *)arg, &ptp, sizeof(ptp_ts_t));
        break;
    case PTP_SET_TIME:
        memset( &ptp_clk, 0, sizeof(ptp_clk_t));
        copy_from_user( &ptp_clk, (const char __user *)arg, sizeof(ptp_clk_t));
        rc = ptp_set_clk( &ptp_clk, 0);
        break;
    default:
        dbg_ptp("default\n");
        break;
   }
return rc;
}
static int __init init_mod(void)
{
int  major = 0;
   major = register_chrdev( PTP_DEV_MAJOR_NUM, PTP_DRV_NAME, &ptp_fops);
   
   spin_lock_init(&ptp_lock);
   return 0;
}
static void __exit exit_mod(void)
{
    dbg_ptp("%s \n", __FUNCTION__);
    unregister_chrdev_region(0, dev);
    deinit_ptp(); 
    return;
}
EXPORT_SYMBOL(ptp_set_ts);
module_init(init_mod);
module_exit(exit_mod);
