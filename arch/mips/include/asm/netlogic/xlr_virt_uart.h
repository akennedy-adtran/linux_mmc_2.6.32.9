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


#ifndef _ASM_XLR_VIRT_UART_H
#define _ASM_XLR_VIRT_UART_H

typedef struct test{
        volatile unsigned char *tx_fifo;
        volatile unsigned int *tx_pro;
        volatile unsigned int  *tx_con;
        volatile unsigned char *rx_fifo;
        volatile unsigned int *rx_pro;
        volatile unsigned int *rx_con;
        volatile int *status;
}virt_uart_struct;

typedef struct outbyte_struct{
      volatile unsigned char *rx_fifo;
      volatile unsigned int *rx_pro;
      volatile unsigned int  *rx_con;
      volatile unsigned char *tx_fifo;
      volatile unsigned int *tx_pro;
      volatile unsigned int *tx_con;
      volatile int *status;
}virt_uart;

#define USER_CMD_SIZE 			  (1*1024)
#define USER_RESULT_SIZE 		  (7*1024)
#define DELAY_TIME                         2
#define VIRTUAL_UART_CONSOLE      	  "virt_uart"
#define VIRTUAL_UART_CONSOLE_MAJOR        XLR_VIRT_UART_MAJOR
#define VIRTUAL_UART_CONSOLE_MINOR        200
#define VIRTUAL_UART_NR                   32
#define VIRT_UART_OPENED 		  0xeadbeef
#ifdef CONFIG_64BIT
#define VIRT_UART_BUF_START               ((48<<20) | 0xffffffff80000000ULL)
#else
#define VIRT_UART_BUF_START               ((48<<20) | 0x80000000)
#endif
#endif

