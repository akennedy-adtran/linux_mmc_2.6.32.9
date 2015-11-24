
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

/* */
/* v1p7 mwj 2011-05-12 */
/*  add transpose (1.c20c.9) when RX_LOS asserted for LED link status */
/* v1p8 pc  2011-05-24 update using margaret ocal_tx_losXposee_tgt_v1p8.uc */
/*  add LC PLL lookup table to reliably run a wider temperature range */
/* v1p9 mwj 2011-05-31 */
/*  add lrcv/lrxmit reset and xaui squelch during RXLOS to ensure no LED */
/*     when RXLOS = 1 and no incoming signal */
/*  changed wait time after SRAM load to 300ms to ensure contvideal values */
/*     are udpated */
/* v1p9B mwj 2011-06-30 */
/*  store temp to 1.eff0 */
/*  increased wait time to 400ms in case customer MDIO is faster and */
/*     reaches RCON faster */
/* v1p10 mwj 2011-07-02 */
/*  store temp to 1.ef2c to stay consistent with other firmware */
/*  temp sense / contvideal LUT increased to 10 steps from 3 */
/* v1p10B mwj 2011-07-05 */
/*  optimized LUT values */

/* MPS - Pete Moore, convert to .c functions for XLP SDK HAL */


/* calico_C2 settings... */
/* -------------------------------------------------- */
/* NOTE, no reset (h/  nlm_nlp_phy_mod(pa,or x.0000.15) here */

/* PART 1, nlp_init ---------------------------------- */
uint16_t nlm_xaui_phy_read(int phyaddr, int devaddr, int regidx);
void nlm_xaui_phy_write(int phyaddr, int devaddr, int regidx, uint16_t val);

static inline void nlm_nlp_phy_mod(int pa, int da, uint16_t ra, int bh, int bl,
	uint16_t val)
{
  uint16_t mask = 0xffff;
  uint16_t tval;
  int i;

  for (i=bl; i<(bh+1); i++) {
    mask &= ~(1 << i);
  }

  tval = nlm_xaui_phy_read(pa, da, ra);
  /*nlm_print("Phymod read %x:%x:%x: 0x%02x\n", pa, da, ra, tval); */
  tval &= mask;
  tval |= (val << bl);
  nlm_xaui_phy_write(pa, da, ra, tval);
  /*nlm_print("Phymod Write %x:%x:%x: 0x%02x\n", pa, da, ra, tval); */
}

static inline void nlm_nlp1042_init(int pa)
{
  nlm_nlp_phy_mod(pa,1,0xca42,10,8, 0x1); 	/*  cmu_e2o_tx,vcocaplrsel */
  nlm_nlp_phy_mod(pa,1,0xca44,15,12, 0x8); 	/*  cmu_e2o_tx,reg_vcotailsel */
  nlm_nlp_phy_mod(pa,1,0xca44,9,5, 0x10); 	/*  cmu_e2o_tx,reg_contvideal */
  nlm_nlp_phy_mod(pa,1,0xca46,8,8, 0x1); 	/*  cmu_e2o_tx,reg_cp_highv_en */
  nlm_nlp_phy_mod(pa,1,0xca46,11,9 , 0x4); 	/*  cmu_e2o_tx,reg_cp_highvctl */
  nlm_nlp_phy_mod(pa,1,0xcb0e,13,12, 0x1); 	/*  bt_237_236_ir25txcmu1 */
  nlm_nlp_phy_mod(pa,1,0xca4c,5,0  , 0x2); 	/*  cmu_e2o_tx,antesten */
  nlm_nlp_phy_mod(pa,1,0xca12,10,8 , 0x1); 	/*  cmu_o2e_lim,vcocaplrsel */
  nlm_nlp_phy_mod(pa,1,0xca14,15,12, 0x8); 	/*  cmu_o2e_lim,reg_vcotailsel */
  nlm_nlp_phy_mod(pa,1,0xca14,9,5  , 0x10); 	/*  cmu_o2e_lim,reg_contvideal */
  nlm_nlp_phy_mod(pa,1,0xca16,8,8, 0x1); 	/*  cmu_o2e_lim,reg_cp_highv_en */
  nlm_nlp_phy_mod(pa,1,0xca16,11,9 , 0x4); 	/*  cmu_o2e_lim,reg_cp_highvctl */
  nlm_nlp_phy_mod(pa,1,0xcb06,13,12, 0x1); 	/*  bt_109_108_ir25rxfcmu1 */
  nlm_nlp_phy_mod(pa,1,0xca1c,5,0  , 0x2); 	/*  cmu_o2e_lim,antesten */
  nlm_nlp_phy_mod(pa,1,0xca42,13,11, 0x1); 	/*  cmu_e2o_tx,reg_bufcaplrsel */
  nlm_nlp_phy_mod(pa,1,0xca43,14,12, 0x2); 	/*  cmu_e2o_tx,bufcmc */
  nlm_nlp_phy_mod(pa,1,0xc246,9,7  , 0x3); 	/*  csbiasmuxsel */
  nlm_nlp_phy_mod(pa,1,0xc242,2,0  , 0x0); 	/*  lkbiassel */
  nlm_nlp_phy_mod(pa,1,0xc240,7,4  , 0x0); 	/*  ldbiassel */
  nlm_nlp_phy_mod(pa,1,0xc240,13,11, 0x7); 	/*  cmbiassel */
  nlm_nlp_phy_mod(pa,1,0xc242,15,13, 0x0); 	/*  cspredrvsel */
  nlm_nlp_phy_mod(pa,1,0xc246,0,0, 0x0); 	/*  sfpmodeen */
  nlm_nlp_phy_mod(pa,1,0xc246,1,1, 0x1); 	/*  krmodeen */
  nlm_nlp_phy_mod(pa,1,0xc243,4,0, 0x1); 	/*  drvpre */
  nlm_nlp_phy_mod(pa,1,0xc243,10,5, 0x3f); 	/*  drvmain */
  nlm_nlp_phy_mod(pa,1,0xc243,15,11, 0x2); 	/*  drvpost */
  nlm_nlp_phy_mod(pa,1,0xcb0b,1,0, 0x1); 	/*  bt_177_176_ic25drvbias0 */
  nlm_nlp_phy_mod(pa,1,0xcb0b,5,4, 0x1); 	/*  bt_181_180_ir25drvbias0 */
  nlm_nlp_phy_mod(pa,1,0xcb0b,9,8, 0x3); 	/*  bt_185_184_ir25drvbias2 */
  nlm_nlp_phy_mod(pa,1,0xcb0b,11,10, 0x3); 	/*  bt_187_186_ir25drvbias3 */
  nlm_nlp_phy_mod(pa,1,0xcb0b,13,12, 0x3); 	/*  bt_189_188_ir25drvbias4 */
  nlm_nlp_phy_mod(pa,1,0xcb0b,15,14, 0x3); 	/*  bt_191_190_ir25drvbias5 */
  nlm_nlp_phy_mod(pa,1,0xcc00,7,0, 0x0); 	/*  lsatxfi_o2e_rxrxfcdrqoffs */
  nlm_nlp_phy_mod(pa,1,0xcc02,7,7, 0x1); 	/*  lsatxfi_o2e_rxrxfcdrwrpmltgl */
  nlm_nlp_phy_mod(pa,1,0xcc02,7,7, 0x0); 	/*  lsatxfi_o2e_rxrxfcdrwrpmltgl */
  nlm_nlp_phy_mod(pa,1,0xcc07,4,0, 0x17); 	/*  lsatxfi_o2e_rxrxfeqklp */
  nlm_nlp_phy_mod(pa,1,0xcc08,5,0, 0x1f); 	/*  lsatxfi_o2e_rxrxfdfetap1 */
  nlm_nlp_phy_mod(pa,1,0xcc08,11,6, 0x1f); 	/*  lsatxfi_o2e_rxrxfdfetap2 */
  nlm_nlp_phy_mod(pa,1,0xcc09,5,0, 0x1f); 	/*  lsatxfi_o2e_rxrxfdfetap3 */
  nlm_nlp_phy_mod(pa,1,0xcc09,11,6, 0x1f); 	/*  lsatxfi_o2e_rxrxfdfetap4 */
  nlm_nlp_phy_mod(pa,1,0xcc07,8,5, 0x0); 	/*  lsatxfi_o2e_rxrxfeqmres */
  nlm_nlp_phy_mod(pa,1,0xcc0e,0,0, 0x1); 	/*  lsatxfi_o2e_rxrxfdfetapstrobe */
  nlm_nlp_phy_mod(pa,1,0xcc0e,1,1, 0x1); 	/*  lsatxfi_o2e_rxrxfdfeosdacstrobe */
  nlm_nlp_phy_mod(pa,1,0xcc0e,2,2, 0x1); 	/*  lsatxfi_o2e_rxrxfeqosdacstrobe */
  nlm_nlp_phy_mod(pa,1,0xcc0e,0,0, 0x0); 	/*  lsatxfi_o2e_rxrxfdfetapstrobe */
  nlm_nlp_phy_mod(pa,1,0xcc0e,1,1, 0x0); 	/*  lsatxfi_o2e_rxrxfdfeosdacstrobe */
  nlm_nlp_phy_mod(pa,1,0xcc0e,2,2, 0x0); 	/*  lsatxfi_o2e_rxrxfeqosdacstrobe */
  nlm_nlp_phy_mod(pa,1,0xcc02,5,5, 0x1); 	/*  lsatxfi_o2e_rxrxfclk16inv */
  nlm_nlp_phy_mod(pa,1,0xcc04,0,0, 0x1); 	/*  lsatxfi_o2e_rxrxfvgaovrden */
  nlm_nlp_phy_mod(pa,1,0xcc04,8,1, 0x5a); 	/*  lsatxfi_o2e_rxrxfvgaovrd */
  nlm_nlp_phy_mod(pa,1,0xcc11,5,3, 0x0); 	/*  lsatxfi_o2e_rxrxfdfetap1adj */
  nlm_nlp_phy_mod(pa,1,0xcc11,8,6, 0x0); 	/*  lsatxfi_o2e_rxrxfdfetap2adj */
  nlm_nlp_phy_mod(pa,1,0xcc11,11,9, 0x0); 	/*  lsatxfi_o2e_rxrxfdfetap3adj */
  nlm_nlp_phy_mod(pa,1,0xcc11,14,12, 0x0); 	/*  lsatxfi_o2e_rxrxfdfetap4adj */
  nlm_nlp_phy_mod(pa,1,0xcc11,2,0, 0x0); 	/*  lsatxfi_o2e_rxrxfdfemainadj */
  nlm_nlp_phy_mod(pa,1,0xcb02,7,4, 0xf); 	/*  bt_039_036_ir50rxfeq1 */
  nlm_nlp_phy_mod(pa,1,0xcb02,3,0, 0xf); 	/*  bt_035_032_ir50rxfeq0 */
  nlm_nlp_phy_mod(pa,1,0xcb21,15,12, 0x1); 	/*  bt_543_540_ir50xtpll3 */
  nlm_nlp_phy_mod(pa,1,0xca01,0,0, 0x1); 	/*  cmu_o2e_tx,cpdac_ovrd */
  nlm_nlp_phy_mod(pa,1,0xca01,7,1, 0x8); 	/*  cmu_o2e_tx,ov_cpdac */
  nlm_nlp_phy_mod(pa,1,0xcb1b,15,12, 0x1); 	/*  bt_447_444_ir50xrpll3 */
  nlm_nlp_phy_mod(pa,1,0xca51,0,0, 0x1); 	/*  cmu_e2o_rx,cpdac_ovrd */
  nlm_nlp_phy_mod(pa,1,0xca51,7,1, 0x8); 	/*  cmu_e2o_rx,ov_cpdac */
  nlm_nlp_phy_mod(pa,1,0xc2c3,3,0, 0xf); 	/*  pwdpostb */
  nlm_nlp_phy_mod(pa,1,0xc2ca,15,11, 0x8); 	/*  drvpost0 */
  nlm_nlp_phy_mod(pa,1,0xc2d0,15,11, 0x8); 	/*  drvpost1 */
  nlm_nlp_phy_mod(pa,1,0xc2d6,15,11, 0x8); 	/*  drvpost2 */
  nlm_nlp_phy_mod(pa,1,0xc2dc,15,11, 0x8); 	/*  drvpost3 */
  nlm_nlp_phy_mod(pa,1,0xc2f1,5,1, 0xa); 	/*  xreqklp0 */
  nlm_nlp_phy_mod(pa,1,0xc2f1,8,6, 0x0); 	/*  xreqmlpf0 */
  nlm_nlp_phy_mod(pa,1,0xc2f5,5,1, 0xa); 	/*  xreqklp1 */
  nlm_nlp_phy_mod(pa,1,0xc2f5,8,6, 0x0); 	/*  xreqmlpf1 */
  nlm_nlp_phy_mod(pa,1,0xc2f9,5,1, 0xa); 	/*  xreqklp2 */
  nlm_nlp_phy_mod(pa,1,0xc2f9,8,6, 0x0); 	/*  xreqmlpf2 */
  nlm_nlp_phy_mod(pa,1,0xc2fd,5,1, 0xa); 	/*  xreqklp3 */
  nlm_nlp_phy_mod(pa,1,0xc2fd,8,6, 0x0); 	/*  xreqmlpf3 */
  nlm_nlp_phy_mod(pa,1,0xc2e8,8,0, 0x100); 	/*  xrcdrqoffs0 */
  nlm_nlp_phy_mod(pa,1,0xc2e9,8,0, 0x100); 	/*  xrcdrqoffs1 */
  nlm_nlp_phy_mod(pa,1,0xc2ea,8,0, 0x100); 	/*  xrcdrqoffs2 */
  nlm_nlp_phy_mod(pa,1,0xc2eb,8,0, 0x100); 	/*  xrcdrqoffs3 */
  nlm_nlp_phy_mod(pa,1,0xc2e6,11,11, 0x1); 	/*  xrcdrwrpmltgl */
  nlm_nlp_phy_mod(pa,1,0xc2e6,11,11, 0x0); 	/*  xrcdrwrpmltgl */
  nlm_nlp_phy_mod(pa,1,0xc010,14,14, 0x1); 	/*  ber_los_mask */
  nlm_nlp_phy_mod(pa,1,0xc017,13,13, 0x0);   /*  rx_pmainpcsfault_mask */
  nlm_nlp_phy_mod(pa,1,0xc010,13,13, 0x1);   /*  los_opt_intlos_mask */
  nlm_nlp_phy_mod(pa,1,0xcd40,3,0, 0x1);   /*  pdtrgmsk_10gbrx */
  nlm_nlp_phy_mod(pa,1,0xc019,10,10, 0x1);   /*  sd_intlos_mask */
  nlm_nlp_phy_mod(pa,1,0xff2a,15,0, 0x4a);

/* PART 2, uc code  ---------------------------------- */

/* cmdUC processing binary and source file... */
/* cmdUC Writing binary into memory... */
  nlm_nlp_phy_mod(pa,1,0xd008,0,0, 0x1);
/* config uC */
  nlm_udelay(100000); /* 100ms */
  nlm_nlp_phy_mod(pa,1,0xd000,15,0, 0x5200);
  nlm_udelay(100000); /* 100ms */
/* writing binary into uC SRAM... */
  nlm_nlp_phy_mod(pa,1,0xd800,15,0, 0x2fff);
  nlm_nlp_phy_mod(pa,1,0xd801,15,0, 0x300f);
  nlm_nlp_phy_mod(pa,1,0xd802,15,0, 0x2ff4);
  nlm_nlp_phy_mod(pa,1,0xd803,15,0, 0x3ef4);
  nlm_nlp_phy_mod(pa,1,0xd804,15,0, 0x20ae);
  nlm_nlp_phy_mod(pa,1,0xd805,15,0, 0x301e);
  nlm_nlp_phy_mod(pa,1,0xd806,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd807,15,0, 0x22c4);
  nlm_nlp_phy_mod(pa,1,0xd808,15,0, 0x3ef4);
  nlm_nlp_phy_mod(pa,1,0xd809,15,0, 0x400e);
  nlm_nlp_phy_mod(pa,1,0xd80a,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd80b,15,0, 0x220e);
  nlm_nlp_phy_mod(pa,1,0xd80c,15,0, 0x300e);
  nlm_nlp_phy_mod(pa,1,0xd80d,15,0, 0x2124);
  nlm_nlp_phy_mod(pa,1,0xd80e,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd80f,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd810,15,0, 0x23fe);
  nlm_nlp_phy_mod(pa,1,0xd811,15,0, 0x3c1e);
  nlm_nlp_phy_mod(pa,1,0xd812,15,0, 0x2214);
  nlm_nlp_phy_mod(pa,1,0xd813,15,0, 0x3ca4);
  nlm_nlp_phy_mod(pa,1,0xd814,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd815,15,0, 0x20a4);
  nlm_nlp_phy_mod(pa,1,0xd816,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd817,15,0, 0x2dfe);
  nlm_nlp_phy_mod(pa,1,0xd818,15,0, 0x307e);
  nlm_nlp_phy_mod(pa,1,0xd819,15,0, 0x6e24);
  nlm_nlp_phy_mod(pa,1,0xd81a,15,0, 0x6e24);
  nlm_nlp_phy_mod(pa,1,0xd81b,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd81c,15,0, 0x20e4);
  nlm_nlp_phy_mod(pa,1,0xd81d,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd81e,15,0, 0x402e);
  nlm_nlp_phy_mod(pa,1,0xd81f,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd820,15,0, 0x400e);
  nlm_nlp_phy_mod(pa,1,0xd821,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd822,15,0, 0x2014);
  nlm_nlp_phy_mod(pa,1,0xd823,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd824,15,0, 0x64de);
  nlm_nlp_phy_mod(pa,1,0xd825,15,0, 0x6e8f);
  nlm_nlp_phy_mod(pa,1,0xd826,15,0, 0x400e);
  nlm_nlp_phy_mod(pa,1,0xd827,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd828,15,0, 0x2044);
  nlm_nlp_phy_mod(pa,1,0xd829,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd82a,15,0, 0x64de);
  nlm_nlp_phy_mod(pa,1,0xd82b,15,0, 0x6e8f);
  nlm_nlp_phy_mod(pa,1,0xd82c,15,0, 0x201e);
  nlm_nlp_phy_mod(pa,1,0xd82d,15,0, 0x300e);
  nlm_nlp_phy_mod(pa,1,0xd82e,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd82f,15,0, 0x20d4);
  nlm_nlp_phy_mod(pa,1,0xd830,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd831,15,0, 0x64de);
  nlm_nlp_phy_mod(pa,1,0xd832,15,0, 0x6e8f);
  nlm_nlp_phy_mod(pa,1,0xd833,15,0, 0x21fe);
  nlm_nlp_phy_mod(pa,1,0xd834,15,0, 0x300e);
  nlm_nlp_phy_mod(pa,1,0xd835,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd836,15,0, 0x20e4);
  nlm_nlp_phy_mod(pa,1,0xd837,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd838,15,0, 0x404e);
  nlm_nlp_phy_mod(pa,1,0xd839,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd83a,15,0, 0x400e);
  nlm_nlp_phy_mod(pa,1,0xd83b,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd83c,15,0, 0x21f5);
  nlm_nlp_phy_mod(pa,1,0xd83d,15,0, 0x3005);
  nlm_nlp_phy_mod(pa,1,0xd83e,15,0, 0xb805);
  nlm_nlp_phy_mod(pa,1,0xd83f,15,0, 0x8556);
  nlm_nlp_phy_mod(pa,1,0xd840,15,0, 0x8557);
  nlm_nlp_phy_mod(pa,1,0xd841,15,0, 0x8558);
  nlm_nlp_phy_mod(pa,1,0xd842,15,0, 0x8559);
  nlm_nlp_phy_mod(pa,1,0xd843,15,0, 0x855a);
  nlm_nlp_phy_mod(pa,1,0xd844,15,0, 0x400d);
  nlm_nlp_phy_mod(pa,1,0xd845,15,0, 0x6d8f);
  nlm_nlp_phy_mod(pa,1,0xd846,15,0, 0x2032);
  nlm_nlp_phy_mod(pa,1,0xd847,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xd848,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd849,15,0, 0x2132);
  nlm_nlp_phy_mod(pa,1,0xd84a,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xd84b,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd84c,15,0, 0x21c2);
  nlm_nlp_phy_mod(pa,1,0xd84d,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xd84e,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd84f,15,0, 0x2302);
  nlm_nlp_phy_mod(pa,1,0xd850,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xd851,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd852,15,0, 0x23a2);
  nlm_nlp_phy_mod(pa,1,0xd853,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xd854,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd855,15,0, 0x24c2);
  nlm_nlp_phy_mod(pa,1,0xd856,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xd857,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd858,15,0, 0x2562);
  nlm_nlp_phy_mod(pa,1,0xd859,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xd85a,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd85b,15,0, 0x6f7e);
  nlm_nlp_phy_mod(pa,1,0xd85c,15,0, 0x4004);
  nlm_nlp_phy_mod(pa,1,0xd85d,15,0, 0xb814);
  nlm_nlp_phy_mod(pa,1,0xd85e,15,0, 0x5e43);
  nlm_nlp_phy_mod(pa,1,0xd85f,15,0, 0x3d7);
  nlm_nlp_phy_mod(pa,1,0xd860,15,0, 0x2032);
  nlm_nlp_phy_mod(pa,1,0xd861,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xd862,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd863,15,0, 0x200e);
  nlm_nlp_phy_mod(pa,1,0xd864,15,0, 0x300e);
  nlm_nlp_phy_mod(pa,1,0xd865,15,0, 0x2);
  nlm_nlp_phy_mod(pa,1,0xd866,15,0, 0xd01e);
  nlm_nlp_phy_mod(pa,1,0xd867,15,0, 0x6e8f);
  nlm_nlp_phy_mod(pa,1,0xd868,15,0, 0x20fe);
  nlm_nlp_phy_mod(pa,1,0xd869,15,0, 0x300e);
  nlm_nlp_phy_mod(pa,1,0xd86a,15,0, 0xb80e);
  nlm_nlp_phy_mod(pa,1,0xd86b,15,0, 0xd01d);
  nlm_nlp_phy_mod(pa,1,0xd86c,15,0, 0x5de3);
  nlm_nlp_phy_mod(pa,1,0xd86d,15,0, 0x249e);
  nlm_nlp_phy_mod(pa,1,0xd86e,15,0, 0x301e);
  nlm_nlp_phy_mod(pa,1,0xd86f,15,0, 0x135e);
  nlm_nlp_phy_mod(pa,1,0xd870,15,0, 0x6f7e);
  nlm_nlp_phy_mod(pa,1,0xd871,15,0, 0x6f7e);
  nlm_nlp_phy_mod(pa,1,0xd872,15,0, 0x20d4);
  nlm_nlp_phy_mod(pa,1,0xd873,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd874,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd875,15,0, 0x20e4);
  nlm_nlp_phy_mod(pa,1,0xd876,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd877,15,0, 0x404e);
  nlm_nlp_phy_mod(pa,1,0xd878,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd879,15,0, 0x400e);
  nlm_nlp_phy_mod(pa,1,0xd87a,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd87b,15,0, 0x6f7e);
  nlm_nlp_phy_mod(pa,1,0xd87c,15,0, 0x2044);
  nlm_nlp_phy_mod(pa,1,0xd87d,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd87e,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd87f,15,0, 0x6f7e);
  nlm_nlp_phy_mod(pa,1,0xd880,15,0, 0x2014);
  nlm_nlp_phy_mod(pa,1,0xd881,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd882,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd883,15,0, 0x200e);
  nlm_nlp_phy_mod(pa,1,0xd884,15,0, 0x300e);
  nlm_nlp_phy_mod(pa,1,0xd885,15,0, 0x2124);
  nlm_nlp_phy_mod(pa,1,0xd886,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd887,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd888,15,0, 0x2504);
  nlm_nlp_phy_mod(pa,1,0xd889,15,0, 0x3cd4);
  nlm_nlp_phy_mod(pa,1,0xd88a,15,0, 0x4015);
  nlm_nlp_phy_mod(pa,1,0xd88b,15,0, 0x65c4);
  nlm_nlp_phy_mod(pa,1,0xd88c,15,0, 0x2514);
  nlm_nlp_phy_mod(pa,1,0xd88d,15,0, 0x3cd4);
  nlm_nlp_phy_mod(pa,1,0xd88e,15,0, 0x64d5);
  nlm_nlp_phy_mod(pa,1,0xd88f,15,0, 0xb145);
  nlm_nlp_phy_mod(pa,1,0xd890,15,0, 0xb115);
  nlm_nlp_phy_mod(pa,1,0xd891,15,0, 0x65c4);
  nlm_nlp_phy_mod(pa,1,0xd892,15,0, 0x2bd2);
  nlm_nlp_phy_mod(pa,1,0xd893,15,0, 0x3012);
  nlm_nlp_phy_mod(pa,1,0xd894,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd895,15,0, 0x678f);
  nlm_nlp_phy_mod(pa,1,0xd896,15,0, 0x2514);
  nlm_nlp_phy_mod(pa,1,0xd897,15,0, 0x3cd4);
  nlm_nlp_phy_mod(pa,1,0xd898,15,0, 0x64d5);
  nlm_nlp_phy_mod(pa,1,0xd899,15,0, 0xb145);
  nlm_nlp_phy_mod(pa,1,0xd89a,15,0, 0xb105);
  nlm_nlp_phy_mod(pa,1,0xd89b,15,0, 0x65c4);
  nlm_nlp_phy_mod(pa,1,0xd89c,15,0, 0x2bd2);
  nlm_nlp_phy_mod(pa,1,0xd89d,15,0, 0x3012);
  nlm_nlp_phy_mod(pa,1,0xd89e,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd89f,15,0, 0x6f78);
  nlm_nlp_phy_mod(pa,1,0xd8a0,15,0, 0xe78e);
  nlm_nlp_phy_mod(pa,1,0xd8a1,15,0, 0x22c5);
  nlm_nlp_phy_mod(pa,1,0xd8a2,15,0, 0x3ef5);
  nlm_nlp_phy_mod(pa,1,0xd8a3,15,0, 0x6ec5);
  nlm_nlp_phy_mod(pa,1,0xd8a4,15,0, 0x2084);
  nlm_nlp_phy_mod(pa,1,0xd8a5,15,0, 0x3034);
  nlm_nlp_phy_mod(pa,1,0xd8a6,15,0, 0x2005);
  nlm_nlp_phy_mod(pa,1,0xd8a7,15,0, 0x3d75);
  nlm_nlp_phy_mod(pa,1,0xd8a8,15,0, 0xc451);
  nlm_nlp_phy_mod(pa,1,0xd8a9,15,0, 0x2f62);
  nlm_nlp_phy_mod(pa,1,0xd8aa,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xd8ab,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd8ac,15,0, 0x2444);
  nlm_nlp_phy_mod(pa,1,0xd8ad,15,0, 0x3ca4);
  nlm_nlp_phy_mod(pa,1,0xd8ae,15,0, 0x2ed2);
  nlm_nlp_phy_mod(pa,1,0xd8af,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xd8b0,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd8b1,15,0, 0x2144);
  nlm_nlp_phy_mod(pa,1,0xd8b2,15,0, 0x3ca4);
  nlm_nlp_phy_mod(pa,1,0xd8b3,15,0, 0x2ed2);
  nlm_nlp_phy_mod(pa,1,0xd8b4,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xd8b5,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd8b6,15,0, 0x2f02);
  nlm_nlp_phy_mod(pa,1,0xd8b7,15,0, 0x3012);
  nlm_nlp_phy_mod(pa,1,0xd8b8,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd8b9,15,0, 0x28c2);
  nlm_nlp_phy_mod(pa,1,0xd8ba,15,0, 0x3012);
  nlm_nlp_phy_mod(pa,1,0xd8bb,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd8bc,15,0, 0x0);
  nlm_nlp_phy_mod(pa,1,0xd8bd,15,0, 0x628f);
  nlm_nlp_phy_mod(pa,1,0xd8be,15,0, 0x4007);
  nlm_nlp_phy_mod(pa,1,0xd8bf,15,0, 0x2524);
  nlm_nlp_phy_mod(pa,1,0xd8c0,15,0, 0x3cd4);
  nlm_nlp_phy_mod(pa,1,0xd8c1,15,0, 0x64d5);
  nlm_nlp_phy_mod(pa,1,0xd8c2,15,0, 0x2005);
  nlm_nlp_phy_mod(pa,1,0xd8c3,15,0, 0x9575);
  nlm_nlp_phy_mod(pa,1,0xd8c4,15,0, 0x65c4);
  nlm_nlp_phy_mod(pa,1,0xd8c5,15,0, 0x678f);
  nlm_nlp_phy_mod(pa,1,0xd8c6,15,0, 0x2dd2);
  nlm_nlp_phy_mod(pa,1,0xd8c7,15,0, 0x3012);
  nlm_nlp_phy_mod(pa,1,0xd8c8,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd8c9,15,0, 0x6f77);
  nlm_nlp_phy_mod(pa,1,0xd8ca,15,0, 0x2514);
  nlm_nlp_phy_mod(pa,1,0xd8cb,15,0, 0x3cd4);
  nlm_nlp_phy_mod(pa,1,0xd8cc,15,0, 0x64d5);
  nlm_nlp_phy_mod(pa,1,0xd8cd,15,0, 0xbd05);
  nlm_nlp_phy_mod(pa,1,0xd8ce,15,0, 0xbf45);
  nlm_nlp_phy_mod(pa,1,0xd8cf,15,0, 0x2db2);
  nlm_nlp_phy_mod(pa,1,0xd8d0,15,0, 0x3012);
  nlm_nlp_phy_mod(pa,1,0xd8d1,15,0, 0x5553);
  nlm_nlp_phy_mod(pa,1,0xd8d2,15,0, 0x1302);
  nlm_nlp_phy_mod(pa,1,0xd8d3,15,0, 0x2006);
  nlm_nlp_phy_mod(pa,1,0xd8d4,15,0, 0x3016);
  nlm_nlp_phy_mod(pa,1,0xd8d5,15,0, 0x5763);
  nlm_nlp_phy_mod(pa,1,0xd8d6,15,0, 0x13c2);
  nlm_nlp_phy_mod(pa,1,0xd8d7,15,0, 0xd017);
  nlm_nlp_phy_mod(pa,1,0xd8d8,15,0, 0x2bf2);
  nlm_nlp_phy_mod(pa,1,0xd8d9,15,0, 0x3012);
  nlm_nlp_phy_mod(pa,1,0xd8da,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd8db,15,0, 0x6f72);
  nlm_nlp_phy_mod(pa,1,0xd8dc,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd8dd,15,0, 0x628f);
  nlm_nlp_phy_mod(pa,1,0xd8de,15,0, 0x2514);
  nlm_nlp_phy_mod(pa,1,0xd8df,15,0, 0x3cd4);
  nlm_nlp_phy_mod(pa,1,0xd8e0,15,0, 0x64d5);
  nlm_nlp_phy_mod(pa,1,0xd8e1,15,0, 0x4026);
  nlm_nlp_phy_mod(pa,1,0xd8e2,15,0, 0x9655);
  nlm_nlp_phy_mod(pa,1,0xd8e3,15,0, 0x65c4);
  nlm_nlp_phy_mod(pa,1,0xd8e4,15,0, 0x648f);
  nlm_nlp_phy_mod(pa,1,0xd8e5,15,0, 0x401d);
  nlm_nlp_phy_mod(pa,1,0xd8e6,15,0, 0x2f22);
  nlm_nlp_phy_mod(pa,1,0xd8e7,15,0, 0x3012);
  nlm_nlp_phy_mod(pa,1,0xd8e8,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd8e9,15,0, 0x6f74);
  nlm_nlp_phy_mod(pa,1,0xd8ea,15,0, 0x2fd6);
  nlm_nlp_phy_mod(pa,1,0xd8eb,15,0, 0x3ff6);
  nlm_nlp_phy_mod(pa,1,0xd8ec,15,0, 0x8655);
  nlm_nlp_phy_mod(pa,1,0xd8ed,15,0, 0x65c4);
  nlm_nlp_phy_mod(pa,1,0xd8ee,15,0, 0x6f72);
  nlm_nlp_phy_mod(pa,1,0xd8ef,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd8f0,15,0, 0x22cd);
  nlm_nlp_phy_mod(pa,1,0xd8f1,15,0, 0x301d);
  nlm_nlp_phy_mod(pa,1,0xd8f2,15,0, 0x2108);
  nlm_nlp_phy_mod(pa,1,0xd8f3,15,0, 0x3808);
  nlm_nlp_phy_mod(pa,1,0xd8f4,15,0, 0x628f);
  nlm_nlp_phy_mod(pa,1,0xd8f5,15,0, 0x5dd3);
  nlm_nlp_phy_mod(pa,1,0xd8f6,15,0, 0x2012);
  nlm_nlp_phy_mod(pa,1,0xd8f7,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xd8f8,15,0, 0x1302);
  nlm_nlp_phy_mod(pa,1,0xd8f9,15,0, 0x63a8);
  nlm_nlp_phy_mod(pa,1,0xd8fa,15,0, 0x2b72);
  nlm_nlp_phy_mod(pa,1,0xd8fb,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xd8fc,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd8fd,15,0, 0xdffd);
  nlm_nlp_phy_mod(pa,1,0xd8fe,15,0, 0x2f52);
  nlm_nlp_phy_mod(pa,1,0xd8ff,15,0, 0x3012);
  nlm_nlp_phy_mod(pa,1,0xd900,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd901,15,0, 0x6f72);
  nlm_nlp_phy_mod(pa,1,0xd902,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd903,15,0, 0x2214);
  nlm_nlp_phy_mod(pa,1,0xd904,15,0, 0x3ca4);
  nlm_nlp_phy_mod(pa,1,0xd905,15,0, 0x64de);
  nlm_nlp_phy_mod(pa,1,0xd906,15,0, 0x2ef4);
  nlm_nlp_phy_mod(pa,1,0xd907,15,0, 0x3ff4);
  nlm_nlp_phy_mod(pa,1,0xd908,15,0, 0x8e4e);
  nlm_nlp_phy_mod(pa,1,0xd909,15,0, 0x2214);
  nlm_nlp_phy_mod(pa,1,0xd90a,15,0, 0x3ca4);
  nlm_nlp_phy_mod(pa,1,0xd90b,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd90c,15,0, 0x2104);
  nlm_nlp_phy_mod(pa,1,0xd90d,15,0, 0x3004);
  nlm_nlp_phy_mod(pa,1,0xd90e,15,0, 0x9e4e);
  nlm_nlp_phy_mod(pa,1,0xd90f,15,0, 0x2214);
  nlm_nlp_phy_mod(pa,1,0xd910,15,0, 0x3ca4);
  nlm_nlp_phy_mod(pa,1,0xd911,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd912,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd913,15,0, 0x2294);
  nlm_nlp_phy_mod(pa,1,0xd914,15,0, 0x3ca4);
  nlm_nlp_phy_mod(pa,1,0xd915,15,0, 0x64db);
  nlm_nlp_phy_mod(pa,1,0xd916,15,0, 0x8bbc);
  nlm_nlp_phy_mod(pa,1,0xd917,15,0, 0xb84b);
  nlm_nlp_phy_mod(pa,1,0xd918,15,0, 0x300c);
  nlm_nlp_phy_mod(pa,1,0xd919,15,0, 0xdf0b);
  nlm_nlp_phy_mod(pa,1,0xd91a,15,0, 0xdf0c);
  nlm_nlp_phy_mod(pa,1,0xd91b,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd91c,15,0, 0xc5b5);
  nlm_nlp_phy_mod(pa,1,0xd91d,15,0, 0xc6c6);
  nlm_nlp_phy_mod(pa,1,0xd91e,15,0, 0x855e);
  nlm_nlp_phy_mod(pa,1,0xd91f,15,0, 0xb84e);
  nlm_nlp_phy_mod(pa,1,0xd920,15,0, 0x866c);
  nlm_nlp_phy_mod(pa,1,0xd921,15,0, 0xb84c);
  nlm_nlp_phy_mod(pa,1,0xd922,15,0, 0xb60c);
  nlm_nlp_phy_mod(pa,1,0xd923,15,0, 0x9cee);
  nlm_nlp_phy_mod(pa,1,0xd924,15,0, 0x20a4);
  nlm_nlp_phy_mod(pa,1,0xd925,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd926,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd927,15,0, 0x20e4);
  nlm_nlp_phy_mod(pa,1,0xd928,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd929,15,0, 0x202e);
  nlm_nlp_phy_mod(pa,1,0xd92a,15,0, 0x300e);
  nlm_nlp_phy_mod(pa,1,0xd92b,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd92c,15,0, 0x200e);
  nlm_nlp_phy_mod(pa,1,0xd92d,15,0, 0x300e);
  nlm_nlp_phy_mod(pa,1,0xd92e,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd92f,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd930,15,0, 0x22b4);
  nlm_nlp_phy_mod(pa,1,0xd931,15,0, 0x3ca4);
  nlm_nlp_phy_mod(pa,1,0xd932,15,0, 0x64db);
  nlm_nlp_phy_mod(pa,1,0xd933,15,0, 0x8bbc);
  nlm_nlp_phy_mod(pa,1,0xd934,15,0, 0xb84b);
  nlm_nlp_phy_mod(pa,1,0xd935,15,0, 0xb80c);
  nlm_nlp_phy_mod(pa,1,0xd936,15,0, 0xb84c);
  nlm_nlp_phy_mod(pa,1,0xd937,15,0, 0xdf0b);
  nlm_nlp_phy_mod(pa,1,0xd938,15,0, 0xdf0c);
  nlm_nlp_phy_mod(pa,1,0xd939,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd93a,15,0, 0xc7b7);
  nlm_nlp_phy_mod(pa,1,0xd93b,15,0, 0xc8c8);
  nlm_nlp_phy_mod(pa,1,0xd93c,15,0, 0x877e);
  nlm_nlp_phy_mod(pa,1,0xd93d,15,0, 0xb84e);
  nlm_nlp_phy_mod(pa,1,0xd93e,15,0, 0x888c);
  nlm_nlp_phy_mod(pa,1,0xd93f,15,0, 0xb84c);
  nlm_nlp_phy_mod(pa,1,0xd940,15,0, 0xb60c);
  nlm_nlp_phy_mod(pa,1,0xd941,15,0, 0x9cee);
  nlm_nlp_phy_mod(pa,1,0xd942,15,0, 0x20b4);
  nlm_nlp_phy_mod(pa,1,0xd943,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd944,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd945,15,0, 0x20e4);
  nlm_nlp_phy_mod(pa,1,0xd946,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd947,15,0, 0x402e);
  nlm_nlp_phy_mod(pa,1,0xd948,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd949,15,0, 0x400e);
  nlm_nlp_phy_mod(pa,1,0xd94a,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd94b,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd94c,15,0, 0x22a4);
  nlm_nlp_phy_mod(pa,1,0xd94d,15,0, 0x3ca4);
  nlm_nlp_phy_mod(pa,1,0xd94e,15,0, 0x64db);
  nlm_nlp_phy_mod(pa,1,0xd94f,15,0, 0x8bbc);
  nlm_nlp_phy_mod(pa,1,0xd950,15,0, 0xb84b);
  nlm_nlp_phy_mod(pa,1,0xd951,15,0, 0xb80c);
  nlm_nlp_phy_mod(pa,1,0xd952,15,0, 0xb84c);
  nlm_nlp_phy_mod(pa,1,0xd953,15,0, 0xdf0b);
  nlm_nlp_phy_mod(pa,1,0xd954,15,0, 0xdf0c);
  nlm_nlp_phy_mod(pa,1,0xd955,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd956,15,0, 0xc9b9);
  nlm_nlp_phy_mod(pa,1,0xd957,15,0, 0xcaca);
  nlm_nlp_phy_mod(pa,1,0xd958,15,0, 0x899e);
  nlm_nlp_phy_mod(pa,1,0xd959,15,0, 0xb84e);
  nlm_nlp_phy_mod(pa,1,0xd95a,15,0, 0x8aac);
  nlm_nlp_phy_mod(pa,1,0xd95b,15,0, 0xb84c);
  nlm_nlp_phy_mod(pa,1,0xd95c,15,0, 0xb60c);
  nlm_nlp_phy_mod(pa,1,0xd95d,15,0, 0x9cee);
  nlm_nlp_phy_mod(pa,1,0xd95e,15,0, 0x20c4);
  nlm_nlp_phy_mod(pa,1,0xd95f,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd960,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd961,15,0, 0x20e4);
  nlm_nlp_phy_mod(pa,1,0xd962,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd963,15,0, 0x402e);
  nlm_nlp_phy_mod(pa,1,0xd964,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd965,15,0, 0x400e);
  nlm_nlp_phy_mod(pa,1,0xd966,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd967,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd968,15,0, 0x628f);
  nlm_nlp_phy_mod(pa,1,0xd969,15,0, 0x20a4);
  nlm_nlp_phy_mod(pa,1,0xd96a,15,0, 0x3004);
  nlm_nlp_phy_mod(pa,1,0xd96b,15,0, 0x64d9);
  nlm_nlp_phy_mod(pa,1,0xd96c,15,0, 0x899e);
  nlm_nlp_phy_mod(pa,1,0xd96d,15,0, 0xbf0e);
  nlm_nlp_phy_mod(pa,1,0xd96e,15,0, 0xbf4e);
  nlm_nlp_phy_mod(pa,1,0xd96f,15,0, 0x4012);
  nlm_nlp_phy_mod(pa,1,0xd970,15,0, 0x2f0e);
  nlm_nlp_phy_mod(pa,1,0xd971,15,0, 0x300e);
  nlm_nlp_phy_mod(pa,1,0xd972,15,0, 0x2054);
  nlm_nlp_phy_mod(pa,1,0xd973,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd974,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd975,15,0, 0x2064);
  nlm_nlp_phy_mod(pa,1,0xd976,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd977,15,0, 0x64de);
  nlm_nlp_phy_mod(pa,1,0xd978,15,0, 0x8ee9);
  nlm_nlp_phy_mod(pa,1,0xd979,15,0, 0xbe0e);
  nlm_nlp_phy_mod(pa,1,0xd97a,15,0, 0xbf4e);
  nlm_nlp_phy_mod(pa,1,0xd97b,15,0, 0x5e23);
  nlm_nlp_phy_mod(pa,1,0xd97c,15,0, 0x382);
  nlm_nlp_phy_mod(pa,1,0xd97d,15,0, 0x8);
  nlm_nlp_phy_mod(pa,1,0xd97e,15,0, 0x401e);
  nlm_nlp_phy_mod(pa,1,0xd97f,15,0, 0x99e9);
  nlm_nlp_phy_mod(pa,1,0xd980,15,0, 0x69c4);
  nlm_nlp_phy_mod(pa,1,0xd981,15,0, 0x280e);
  nlm_nlp_phy_mod(pa,1,0xd982,15,0, 0x300e);
  nlm_nlp_phy_mod(pa,1,0xd983,15,0, 0x63ae);
  nlm_nlp_phy_mod(pa,1,0xd984,15,0, 0x8);
  nlm_nlp_phy_mod(pa,1,0xd985,15,0, 0x2fee);
  nlm_nlp_phy_mod(pa,1,0xd986,15,0, 0x3ffe);
  nlm_nlp_phy_mod(pa,1,0xd987,15,0, 0x89e9);
  nlm_nlp_phy_mod(pa,1,0xd988,15,0, 0x69c4);
  nlm_nlp_phy_mod(pa,1,0xd989,15,0, 0x280e);
  nlm_nlp_phy_mod(pa,1,0xd98a,15,0, 0x300e);
  nlm_nlp_phy_mod(pa,1,0xd98b,15,0, 0x63ae);
  nlm_nlp_phy_mod(pa,1,0xd98c,15,0, 0x6f72);
  nlm_nlp_phy_mod(pa,1,0xd98d,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd98e,15,0, 0x628f);
  nlm_nlp_phy_mod(pa,1,0xd98f,15,0, 0x20a4);
  nlm_nlp_phy_mod(pa,1,0xd990,15,0, 0x3004);
  nlm_nlp_phy_mod(pa,1,0xd991,15,0, 0x64d9);
  nlm_nlp_phy_mod(pa,1,0xd992,15,0, 0x899e);
  nlm_nlp_phy_mod(pa,1,0xd993,15,0, 0xbf0e);
  nlm_nlp_phy_mod(pa,1,0xd994,15,0, 0xbf4e);
  nlm_nlp_phy_mod(pa,1,0xd995,15,0, 0x4012);
  nlm_nlp_phy_mod(pa,1,0xd996,15,0, 0x5e23);
  nlm_nlp_phy_mod(pa,1,0xd997,15,0, 0x3d2);
  nlm_nlp_phy_mod(pa,1,0xd998,15,0, 0x1d);
  nlm_nlp_phy_mod(pa,1,0xd999,15,0, 0x2f0e);
  nlm_nlp_phy_mod(pa,1,0xd99a,15,0, 0x300e);
  nlm_nlp_phy_mod(pa,1,0xd99b,15,0, 0x2054);
  nlm_nlp_phy_mod(pa,1,0xd99c,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd99d,15,0, 0x6ec4);
  nlm_nlp_phy_mod(pa,1,0xd99e,15,0, 0x2064);
  nlm_nlp_phy_mod(pa,1,0xd99f,15,0, 0x3cc4);
  nlm_nlp_phy_mod(pa,1,0xd9a0,15,0, 0x64de);
  nlm_nlp_phy_mod(pa,1,0xd9a1,15,0, 0x8ee9);
  nlm_nlp_phy_mod(pa,1,0xd9a2,15,0, 0xbe0e);
  nlm_nlp_phy_mod(pa,1,0xd9a3,15,0, 0xbf4e);
  nlm_nlp_phy_mod(pa,1,0xd9a4,15,0, 0x5e23);
  nlm_nlp_phy_mod(pa,1,0xd9a5,15,0, 0x382);
  nlm_nlp_phy_mod(pa,1,0xd9a6,15,0, 0x9);
  nlm_nlp_phy_mod(pa,1,0xd9a7,15,0, 0x2fee);
  nlm_nlp_phy_mod(pa,1,0xd9a8,15,0, 0x3ffe);
  nlm_nlp_phy_mod(pa,1,0xd9a9,15,0, 0x89e9);
  nlm_nlp_phy_mod(pa,1,0xd9aa,15,0, 0x69c4);
  nlm_nlp_phy_mod(pa,1,0xd9ab,15,0, 0x280e);
  nlm_nlp_phy_mod(pa,1,0xd9ac,15,0, 0x300e);
  nlm_nlp_phy_mod(pa,1,0xd9ad,15,0, 0x63ae);
  nlm_nlp_phy_mod(pa,1,0xd9ae,15,0, 0x7);
  nlm_nlp_phy_mod(pa,1,0xd9af,15,0, 0x401e);
  nlm_nlp_phy_mod(pa,1,0xd9b0,15,0, 0x99e9);
  nlm_nlp_phy_mod(pa,1,0xd9b1,15,0, 0x69c4);
  nlm_nlp_phy_mod(pa,1,0xd9b2,15,0, 0x280e);
  nlm_nlp_phy_mod(pa,1,0xd9b3,15,0, 0x300e);
  nlm_nlp_phy_mod(pa,1,0xd9b4,15,0, 0x63ae);
  nlm_nlp_phy_mod(pa,1,0xd9b5,15,0, 0x6f72);
  nlm_nlp_phy_mod(pa,1,0xd9b6,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd9b7,15,0, 0x628f);
  nlm_nlp_phy_mod(pa,1,0xd9b8,15,0, 0x2844);
  nlm_nlp_phy_mod(pa,1,0xd9b9,15,0, 0x3c04);
  nlm_nlp_phy_mod(pa,1,0xd9ba,15,0, 0x64de);
  nlm_nlp_phy_mod(pa,1,0xd9bb,15,0, 0x2d62);
  nlm_nlp_phy_mod(pa,1,0xd9bc,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xd9bd,15,0, 0x1e52);
  nlm_nlp_phy_mod(pa,1,0xd9be,15,0, 0x20c4);
  nlm_nlp_phy_mod(pa,1,0xd9bf,15,0, 0x3c24);
  nlm_nlp_phy_mod(pa,1,0xd9c0,15,0, 0x64de);
  nlm_nlp_phy_mod(pa,1,0xd9c1,15,0, 0x2ff9);
  nlm_nlp_phy_mod(pa,1,0xd9c2,15,0, 0x3fd9);
  nlm_nlp_phy_mod(pa,1,0xd9c3,15,0, 0x89e9);
  nlm_nlp_phy_mod(pa,1,0xd9c4,15,0, 0x69c4);
  nlm_nlp_phy_mod(pa,1,0xd9c5,15,0, 0x26c4);
  nlm_nlp_phy_mod(pa,1,0xd9c6,15,0, 0x3c04);
  nlm_nlp_phy_mod(pa,1,0xd9c7,15,0, 0x64de);
  nlm_nlp_phy_mod(pa,1,0xd9c8,15,0, 0x2ff9);
  nlm_nlp_phy_mod(pa,1,0xd9c9,15,0, 0x3fa9);
  nlm_nlp_phy_mod(pa,1,0xd9ca,15,0, 0x89e9);
  nlm_nlp_phy_mod(pa,1,0xd9cb,15,0, 0x69c4);
  nlm_nlp_phy_mod(pa,1,0xd9cc,15,0, 0x2894);
  nlm_nlp_phy_mod(pa,1,0xd9cd,15,0, 0x3c04);
  nlm_nlp_phy_mod(pa,1,0xd9ce,15,0, 0x64de);
  nlm_nlp_phy_mod(pa,1,0xd9cf,15,0, 0x2f79);
  nlm_nlp_phy_mod(pa,1,0xd9d0,15,0, 0x3ff9);
  nlm_nlp_phy_mod(pa,1,0xd9d1,15,0, 0x89e9);
  nlm_nlp_phy_mod(pa,1,0xd9d2,15,0, 0x69c4);
  nlm_nlp_phy_mod(pa,1,0xd9d3,15,0, 0x2eb2);
  nlm_nlp_phy_mod(pa,1,0xd9d4,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xd9d5,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd9d6,15,0, 0x2894);
  nlm_nlp_phy_mod(pa,1,0xd9d7,15,0, 0x3c04);
  nlm_nlp_phy_mod(pa,1,0xd9d8,15,0, 0x64de);
  nlm_nlp_phy_mod(pa,1,0xd9d9,15,0, 0x2089);
  nlm_nlp_phy_mod(pa,1,0xd9da,15,0, 0x3009);
  nlm_nlp_phy_mod(pa,1,0xd9db,15,0, 0x99e9);
  nlm_nlp_phy_mod(pa,1,0xd9dc,15,0, 0x69c4);
  nlm_nlp_phy_mod(pa,1,0xd9dd,15,0, 0x26c4);
  nlm_nlp_phy_mod(pa,1,0xd9de,15,0, 0x3c04);
  nlm_nlp_phy_mod(pa,1,0xd9df,15,0, 0x64de);
  nlm_nlp_phy_mod(pa,1,0xd9e0,15,0, 0x2009);
  nlm_nlp_phy_mod(pa,1,0xd9e1,15,0, 0x3059);
  nlm_nlp_phy_mod(pa,1,0xd9e2,15,0, 0x99e9);
  nlm_nlp_phy_mod(pa,1,0xd9e3,15,0, 0x69c4);
  nlm_nlp_phy_mod(pa,1,0xd9e4,15,0, 0x20c4);
  nlm_nlp_phy_mod(pa,1,0xd9e5,15,0, 0x3c24);
  nlm_nlp_phy_mod(pa,1,0xd9e6,15,0, 0x64de);
  nlm_nlp_phy_mod(pa,1,0xd9e7,15,0, 0x2009);
  nlm_nlp_phy_mod(pa,1,0xd9e8,15,0, 0x3029);
  nlm_nlp_phy_mod(pa,1,0xd9e9,15,0, 0x99e9);
  nlm_nlp_phy_mod(pa,1,0xd9ea,15,0, 0x69c4);
  nlm_nlp_phy_mod(pa,1,0xd9eb,15,0, 0x6f72);
  nlm_nlp_phy_mod(pa,1,0xd9ec,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd9ed,15,0, 0x64d6);
  nlm_nlp_phy_mod(pa,1,0xd9ee,15,0, 0x21f7);
  nlm_nlp_phy_mod(pa,1,0xd9ef,15,0, 0x3fc7);
  nlm_nlp_phy_mod(pa,1,0xd9f0,15,0, 0x8676);
  nlm_nlp_phy_mod(pa,1,0xd9f1,15,0, 0xb505);
  nlm_nlp_phy_mod(pa,1,0xd9f2,15,0, 0x9566);
  nlm_nlp_phy_mod(pa,1,0xd9f3,15,0, 0x66c4);
  nlm_nlp_phy_mod(pa,1,0xd9f4,15,0, 0xb545);
  nlm_nlp_phy_mod(pa,1,0xd9f5,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xd9f6,15,0, 0x628f);
  nlm_nlp_phy_mod(pa,1,0xd9f7,15,0, 0x6138);
  nlm_nlp_phy_mod(pa,1,0xd9f8,15,0, 0x5883);
  nlm_nlp_phy_mod(pa,1,0xd9f9,15,0, 0x2062);
  nlm_nlp_phy_mod(pa,1,0xd9fa,15,0, 0x3032);
  nlm_nlp_phy_mod(pa,1,0xd9fb,15,0, 0x1302);
  nlm_nlp_phy_mod(pa,1,0xd9fc,15,0, 0x2ff7);
  nlm_nlp_phy_mod(pa,1,0xd9fd,15,0, 0x3007);
  nlm_nlp_phy_mod(pa,1,0xd9fe,15,0, 0x8786);
  nlm_nlp_phy_mod(pa,1,0xd9ff,15,0, 0xb887);
  nlm_nlp_phy_mod(pa,1,0xda00,15,0, 0x8785);
  nlm_nlp_phy_mod(pa,1,0xda01,15,0, 0xb8c5);
  nlm_nlp_phy_mod(pa,1,0xda02,15,0, 0x5e63);
  nlm_nlp_phy_mod(pa,1,0xda03,15,0, 0x2f72);
  nlm_nlp_phy_mod(pa,1,0xda04,15,0, 0x3022);
  nlm_nlp_phy_mod(pa,1,0xda05,15,0, 0x13c2);
  nlm_nlp_phy_mod(pa,1,0xda06,15,0, 0x6f72);
  nlm_nlp_phy_mod(pa,1,0xda07,15,0, 0x1002);
  nlm_nlp_phy_mod(pa,1,0xda08,15,0, 0xc82);
  nlm_nlp_phy_mod(pa,1,0xda09,15,0, 0xd87);
  nlm_nlp_phy_mod(pa,1,0xda0a,15,0, 0xe8d);
  nlm_nlp_phy_mod(pa,1,0xda0b,15,0, 0x1092);
  nlm_nlp_phy_mod(pa,1,0xda0c,15,0, 0x1198);
  nlm_nlp_phy_mod(pa,1,0xda0d,15,0, 0x129d);
  nlm_nlp_phy_mod(pa,1,0xda0e,15,0, 0x14a3);
  nlm_nlp_phy_mod(pa,1,0xda0f,15,0, 0x16a8);
  nlm_nlp_phy_mod(pa,1,0xda10,15,0, 0x17ae);
  nlm_nlp_phy_mod(pa,1,0xda11,15,0, 0x18b4);
  nlm_nlp_phy_mod(pa,1,0xda12,15,0, 0x0);
  nlm_nlp_phy_mod(pa,1,0xda13,15,0, 0x0);

/* done with uC binary */
/* cmdUC verifying binary... */
  nlm_nlp_phy_mod(pa,1,0xd080,15,0, 0x100);
  nlm_nlp_phy_mod(pa,1,0xd092,15,0, 0x0);
  nlm_udelay(400000);  /* wait for uC to settle offset calibration */

#if 0 /* Sequence from SDK */
  nlm_nlp_phy_mod(pa,1,0xc088,3,0  , 0x0);
  nlm_nlp_phy_mod(pa,1,0xc012,3,0  , 0x0);
  nlm_nlp_phy_mod(pa,1,0xc012,7,4  , 0x0);
  nlm_nlp_phy_mod(pa,1,0xc017,13,13  , 0x0);
  nlm_nlp_phy_mod(pa,1,0xcd40,3,0  , 0x1);
  nlm_nlp_phy_mod(pa,1,0xc0f0,15,0  , 0x0102);
  nlm_nlp_phy_mod(pa,1,0xc0f1,15,0  , 0x0056);
  nlm_nlp_phy_mod(pa,1,0xc20d,1,1  , 0x1);
  nlm_udelay(200000);  /* sleep 200ms */

#else /* Sequence from Support */
  nlm_nlp_phy_mod(pa,1,0xcc08,5,0  , 0x6); 	/*  lsatxfi_o2e_rxrxfdfetap1 */
  nlm_nlp_phy_mod(pa,1,0xcc08,11,6 , 0x23); 	/*  lsatxfi_o2e_rxrxfdfetap2 */
  nlm_nlp_phy_mod(pa,1,0xcc09,5,0  , 0x28); 	/*  lsatxfi_o2e_rxrxfdfetap3 */
  nlm_nlp_phy_mod(pa,1,0xcc09,11,6 , 0x2d); 	/*  lsatxfi_o2e_rxrxfdfetap4 */
  nlm_nlp_phy_mod(pa,1,0xcc0e,0,0, 0x1); 	/*  lsatxfi_o2e_rxrxfdfetapstrobe */
  nlm_nlp_phy_mod(pa,1,0xcc0e,0,0, 0x0); 	/*  lsatxfi_o2e_rxrxfdfetapstrobe */
/* end of s_eepromLoad */

/* PART 3, speed mode -------------------------------- */

/* Evaluating, 'xaui_init ' */
  nlm_nlp_phy_mod(pa,1,0xc2e1,3,0, 0xf);   /*  mci_lpbk_glue */
  nlm_nlp_phy_mod(pa,1,0xc088,3,0, 0x0);   /*  espeed_mode */
  nlm_nlp_phy_mod(pa,1,0xc012,3,0, 0x0);   /*  cfg_pwrdnxr */
  nlm_nlp_phy_mod(pa,1,0xc012,7,4, 0x0);   /*  cfg_pwrdnxt */
  nlm_nlp_phy_mod(pa,1,0xc0f0,15,0, 0x010a);/*  revision */
  nlm_nlp_phy_mod(pa,1,0xc0f1,15,0, 0x0056);/*  clk mode */
  nlm_nlp_phy_mod(pa,1,0xc20d,1,1, 0x1);   /*  rConStart */
  nlm_udelay(200000);  /* sleep 200ms */
#endif

  /* Add setting the GPIO config for our LEDs */

  /* GPIO port2 gpioobcfg */
  nlm_nlp_phy_mod(pa,1,0xc112,15,0, 0x33ff);

  /* GPIO port2 gpioobcfg */
  nlm_nlp_phy_mod(pa,1,0xc113,15,0, 0xb3ff);

  /* GPIO Ctrl port2 and port3 to probe mode */
  /*nlm_nlp_phy_mod(pa,1,0xc108,15,0, 0x8800); */
  /* GPIO Ctrl port2 and port3 to traffic indication mode */
  nlm_nlp_phy_mod(pa,1,0xc108,15,0, 0xdd00);
}
