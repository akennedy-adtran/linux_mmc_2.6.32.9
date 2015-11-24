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


#ifndef __USER_NLM_XLR_MSG_RNG_H
#define __USER_NLM_XLR_MSG_RNG_H

#include <asm/ioctl.h>

struct msgring_msg_data {
  int		size;
  int           code;
  int           stid;
  int           rx_bucket;
  unsigned int  msgs[8];
};

#define MSG_RNG_IOC_MAGIC 'M'

#define    MSGRING_IOC_SSTNNO       _IOW(MSG_RNG_IOC_MAGIC, 0, unsigned int)
#define    MSGRING_IOC_GSHMPHYS     _IOR(MSG_RNG_IOC_MAGIC, 1, unsigned int)
#define    MSGRING_IOC_GSHMVIRT     _IOR(MSG_RNG_IOC_MAGIC, 2, unsigned int)
#define    MSGRING_IOC_GMMAP_START  _IOR(MSG_RNG_IOC_MAGIC, 3, unsigned int)
#define    MSGRING_IOC_SYSINIT      _IOR(MSG_RNG_IOC_MAGIC, 4, unsigned int)
#define    MSGRING_IOC_SYSPHYS      _IOR(MSG_RNG_IOC_MAGIC, 5, unsigned int)
#define    MSGRING_IOC_SYSCALL      _IOW(MSG_RNG_IOC_MAGIC, 6, unsigned int)

#define NLM_MSGRING_CHRDEV_NAME "xlr_msgring_shm"

#endif
