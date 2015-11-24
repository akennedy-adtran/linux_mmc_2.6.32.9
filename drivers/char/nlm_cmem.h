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

#ifndef NLM_CMEM_H_
#define NLM_CMEM_H_

#define NLM_CMEM_IOC 'n'

#define NLM_CMEM_SET_APP_PAGE_SIZE 	_IOWR(NLM_CMEM_IOC, 1, unsigned long long *)
#define NLM_CMEM_MADVISE_DONT_NEED 	_IOWR(NLM_CMEM_IOC, 2, unsigned long long *)
#define NLM_CMEM_MADVISE_NEED 		_IOWR(NLM_CMEM_IOC, 3, unsigned long long *)
#define NLM_CMEM_POST_CHILD_ATFORK	_IOWR(NLM_CMEM_IOC, 6, unsigned long long *)
#define NLM_CMEM_GET_VA_PA_INFO		_IOWR(NLM_CMEM_IOC, 7, unsigned long long *)

#endif /* NLM_CMEM_H_ */
