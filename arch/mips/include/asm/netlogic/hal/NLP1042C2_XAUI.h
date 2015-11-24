/*-
 * Copyright (c) 2003-2013 Broadcom Corporation
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
//#NLM_C45_WRITE (0, bus_num, 0, 1, 0xc2e1, data);


NLM_C45_WRITE (0, bus_num, 0, 1, 0x0000, 0xA040);
//s 200000);
//sleep   300000;  # wait for uC to settle offset calibration
//for(i=0;i<100000;i++){
for(i=0;i<10000;i++){
NLM_C45_PRINTF(".");
}
NLM_C45_PRINTF("\nDone");
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC019, 0x1531);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC010, 0x4000);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC017, 0xDF20);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC010, 0x6000);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCD40, 0x0001);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC019, 0x1531);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA42, 0x0100);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA44, 0x81F8);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA44, 0x8218);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA46, 0x0100);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA46, 0x0900);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCB0E, 0x1000);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA4C, 0x0002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA12, 0x0100);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA14, 0x81F8);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA14, 0x8218);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA16, 0x0100);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA16, 0x0900);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCB06, 0x1000);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA1C, 0x0002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA42, 0x0900);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA43, 0x2492);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC246, 0x0181);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC242, 0x8100);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC240, 0x1000);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC240, 0x3800);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC242, 0x0100);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC246, 0x0180);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC246, 0x0182);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC243, 0x07E1);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC243, 0x17E1);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCB0B, 0x0001);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCB0B, 0x0011);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCB0B, 0x0311);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCB0B, 0x0F11);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCB0B, 0x3F11);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCB0B, 0xFF11);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC00, 0x00F6);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC02, 0x0081);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC02, 0x0001);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC07, 0x0117);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC07, 0x0017);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC0E, 0x0001);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC0E, 0x0003);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC0E, 0x0007);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC0E, 0x0006);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC0E, 0x0004);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC0E, 0x0000);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC02, 0x0021);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC04, 0x0001);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC04, 0x00B5);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC11, 0x4904);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC11, 0x4804);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC11, 0x4004);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC11, 0x0004);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC11, 0x0000);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCB02, 0x00F0);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCB02, 0x00FF);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCB21, 0x1000);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA01, 0x0001);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA01, 0x0011);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCB1B, 0x1000);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA51, 0x0001);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCA51, 0x0011);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2C3, 0x000F);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2CA, 0x43C0);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2D0, 0x43C0);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2D6, 0x43C0);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2DC, 0x43C0);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2F1, 0x5F15);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2F1, 0x5E15);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2F5, 0x5F15);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2F5, 0x5E15);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2F9, 0x5F15);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2F9, 0x5E15);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2FD, 0x5F15);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2FD, 0x5E15);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2E8, 0x0100);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2E9, 0x0100);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2EA, 0x0100);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2EB, 0x0100);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2E6, 0x0800);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC2E6, 0x0000);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC05, 0x001F);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC03, 0x0020);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xFF2A, 0x004A);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCD40, 0x000F);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCD4A, 0x0400);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCD40, 0x0001);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC0F0, 0x0102);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC0F1, 0x0056);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xC20D, 0x0002);

//s 200000);
//s 100000);
//sleep   300000;  # wait for uC to settle offset calibration
//for(i=0;i<100000;i++){
for(i=0;i<10000;i++){
NLM_C45_PRINTF(".");
}
NLM_C45_PRINTF("\nDone");

NLM_C45_WRITE (0, bus_num, 0, 1, 0xC0F1, 0x0056);
//s 200000);
//sleep   300000;  # wait for uC to settle offset calibration
//for(i=0;i<100000;i++){
for(i=0;i<10000;i++){
NLM_C45_PRINTF(".");
}
NLM_C45_PRINTF("\nDone");


NLM_C45_WRITE (0, bus_num, 0, 1, 0xD008, 0x0001);
//s 200000);
//sleep   300000;  # wait for uC to settle offset calibration
//for(i=0;i<100000;i++){
for(i=0;i<10000;i++){
NLM_C45_PRINTF(".");
}
NLM_C45_PRINTF("\nDone");

NLM_C45_WRITE (0, bus_num, 0, 1, 0xD000, 0x5200);
//s 200000);
//sleep   300000;  # wait for uC to settle offset calibration
//for(i=0;i<100000;i++){
for(i=0;i<10000;i++){
NLM_C45_PRINTF(".");
}
NLM_C45_PRINTF("\nDone");

NLM_C45_WRITE (0, bus_num, 0, 1, 0xD800, 0x2FFF);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD801, 0x300F);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD802, 0x220E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD803, 0x300E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD804, 0x2124);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD805, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD806, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD807, 0x23FE);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD808, 0x3C1E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD809, 0x2214);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD80A, 0x3CA4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD80B, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD80C, 0x20A4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD80D, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD80E, 0x2DFE);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD80F, 0x307E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD810, 0x6E24);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD811, 0x6E24);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD812, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD813, 0x20E4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD814, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD815, 0x402E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD816, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD817, 0x400E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD818, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD819, 0x2014);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD81A, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD81B, 0x64DE);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD81C, 0x6E8F);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD81D, 0x400E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD81E, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD81F, 0x2044);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD820, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD821, 0x64DE);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD822, 0x6E8F);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD823, 0x201E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD824, 0x300E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD825, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD826, 0x20D4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD827, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD828, 0x64DE);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD829, 0x6E8F);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD82A, 0x21FE);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD82B, 0x300E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD82C, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD82D, 0x20E4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD82E, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD82F, 0x404E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD830, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD831, 0x400E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD832, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD833, 0x21F5);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD834, 0x3005);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD835, 0xB805);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD836, 0x8556);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD837, 0x8557);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD838, 0x8558);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD839, 0x8559);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD83A, 0x855A);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD83B, 0x400D);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD83C, 0x6D8F);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD83D, 0x2DD2);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD83E, 0x3012);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD83F, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD840, 0x2ED2);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD841, 0x3012);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD842, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD843, 0x2F62);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD844, 0x3012);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD845, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD846, 0x20A2);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD847, 0x3022);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD848, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD849, 0x2142);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD84A, 0x3022);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD84B, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD84C, 0x2262);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD84D, 0x3022);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD84E, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD84F, 0x2302);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD850, 0x3022);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD851, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD852, 0x6F7E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD853, 0x4004);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD854, 0xB814);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD855, 0x5E43);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD856, 0x03D7);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD857, 0x2DD2);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD858, 0x3012);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD859, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD85A, 0x200E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD85B, 0x300E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD85C, 0x0002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD85D, 0xD01E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD85E, 0x6E8F);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD85F, 0x20FE);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD860, 0x300E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD861, 0xB80E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD862, 0xD01D);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD863, 0x5DE3);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD864, 0x240E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD865, 0x301E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD866, 0x135E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD867, 0x6F7E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD868, 0x6F7E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD869, 0x20D4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD86A, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD86B, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD86C, 0x20E4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD86D, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD86E, 0x404E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD86F, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD870, 0x400E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD871, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD872, 0x6F7E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD873, 0x2044);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD874, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD875, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD876, 0x6F7E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD877, 0x2014);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD878, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD879, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD87A, 0x200E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD87B, 0x300E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD87C, 0x2124);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD87D, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD87E, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD87F, 0x2504);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD880, 0x3CD4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD881, 0x4015);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD882, 0x65C4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD883, 0x2514);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD884, 0x3CD4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD885, 0x64D5);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD886, 0xB145);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD887, 0xB115);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD888, 0x65C4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD889, 0x29F2);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD88A, 0x3012);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD88B, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD88C, 0x678F);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD88D, 0x2514);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD88E, 0x3CD4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD88F, 0x64D5);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD890, 0xB145);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD891, 0xB105);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD892, 0x65C4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD893, 0x29F2);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD894, 0x3012);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD895, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD896, 0x6F78);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD897, 0xE78E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD898, 0x2D02);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD899, 0x3012);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD89A, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD89B, 0x2832);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD89C, 0x3012);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD89D, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD89E, 0x0000);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD89F, 0x628F);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8A0, 0x4007);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8A1, 0x2524);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8A2, 0x3CD4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8A3, 0x64D5);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8A4, 0x2005);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8A5, 0x9575);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8A6, 0x65C4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8A7, 0x678F);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8A8, 0x2BF2);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8A9, 0x3012);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8AA, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8AB, 0x6F77);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8AC, 0x2514);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8AD, 0x3CD4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8AE, 0x64D5);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8AF, 0xBD05);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8B0, 0xBF45);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8B1, 0x2BD2);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8B2, 0x3012);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8B3, 0x5553);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8B4, 0x1302);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8B5, 0x2006);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8B6, 0x3016);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8B7, 0x5763);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8B8, 0x13C2);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8B9, 0xD017);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8BA, 0x2A12);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8BB, 0x3012);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8BC, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8BD, 0x6F72);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8BE, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8BF, 0x628F);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8C0, 0x2514);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8C1, 0x3CD4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8C2, 0x64D5);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8C3, 0x4026);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8C4, 0x9655);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8C5, 0x65C4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8C6, 0x401D);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8C7, 0x2D22);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8C8, 0x3012);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8C9, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8CA, 0x2FD6);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8CB, 0x3FF6);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8CC, 0x8655);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8CD, 0x65C4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8CE, 0x6F72);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8CF, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8D0, 0x200D);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8D1, 0x302D);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8D2, 0x2408);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8D3, 0x35D8);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8D4, 0x5DD3);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8D5, 0x0307);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8D6, 0x8887);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8D7, 0x63A7);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8D8, 0x8887);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8D9, 0x63A7);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8DA, 0xDFFD);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8DB, 0x00F9);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8DC, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8DD, 0x2214);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8DE, 0x3CA4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8DF, 0x64DE);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8E0, 0x2EF4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8E1, 0x3FF4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8E2, 0x8E4E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8E3, 0x2214);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8E4, 0x3CA4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8E5, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8E6, 0x2104);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8E7, 0x3004);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8E8, 0x9E4E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8E9, 0x2214);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8EA, 0x3CA4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8EB, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8EC, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8ED, 0x2294);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8EE, 0x3CA4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8EF, 0x64DB);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8F0, 0x8BBC);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8F1, 0xB84B);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8F2, 0x300C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8F3, 0xDF0B);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8F4, 0xDF0C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8F5, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8F6, 0xC5B5);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8F7, 0xC6C6);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8F8, 0x855E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8F9, 0xB84E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8FA, 0x866C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8FB, 0xB84C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8FC, 0xB60C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8FD, 0x9CEE);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8FE, 0x20A4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD8FF, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD900, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD901, 0x20E4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD902, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD903, 0x202E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD904, 0x300E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD905, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD906, 0x200E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD907, 0x300E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD908, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD909, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD90A, 0x22B4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD90B, 0x3CA4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD90C, 0x64DB);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD90D, 0x8BBC);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD90E, 0xB84B);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD90F, 0xB80C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD910, 0xB84C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD911, 0xDF0B);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD912, 0xDF0C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD913, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD914, 0xC7B7);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD915, 0xC8C8);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD916, 0x877E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD917, 0xB84E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD918, 0x888C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD919, 0xB84C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD91A, 0xB60C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD91B, 0x9CEE);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD91C, 0x20B4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD91D, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD91E, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD91F, 0x20E4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD920, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD921, 0x402E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD922, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD923, 0x400E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD924, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD925, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD926, 0x22A4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD927, 0x3CA4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD928, 0x64DB);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD929, 0x8BBC);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD92A, 0xB84B);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD92B, 0xB80C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD92C, 0xB84C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD92D, 0xDF0B);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD92E, 0xDF0C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD92F, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD930, 0xC9B9);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD931, 0xCACA);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD932, 0x899E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD933, 0xB84E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD934, 0x8AAC);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD935, 0xB84C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD936, 0xB60C);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD937, 0x9CEE);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD938, 0x20C4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD939, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD93A, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD93B, 0x20E4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD93C, 0x3CC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD93D, 0x402E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD93E, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD93F, 0x400E);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD940, 0x6EC4);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD941, 0x1002);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD942, 0x0000);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD080, 0x0100);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xD092, 0x0000);
//s 300000);
//sleep   300000;  # wait for uC to settle offset calibration
//for(i=0;i<100000;i++){
for(i=0;i<10000;i++){
NLM_C45_PRINTF(".");
}
NLM_C45_PRINTF("\nDone");

NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC08, 0x07C6);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC08, 0x08C6);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC09, 0x07E8);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC09, 0x0B68);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC0E, 0x0001);
NLM_C45_WRITE (0, bus_num, 0, 1, 0xCC0E, 0x0000);
//);
