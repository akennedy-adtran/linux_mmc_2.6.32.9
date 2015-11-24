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


#ifndef __USER_RMI_PHNX_LOADER_H
#define __USER_RMI_PHNX_LOADER_H

#include <asm/ioctl.h>
#include <asm/types.h>

#define PHNX_LOADER_INFO_MAGIC 			0x600ddeed
#define XLR_MAP_SLAVE_DEVICE 			0x1
#define XLR_MAP_UNCACHED 			0x1
#define XLR_MAP_CACHED 				0x2
#define PHNX_APP_LOADER_CHRDEV_NAME 		"xlr_app_loader"
#define MAX_NUM_LOADER_MEM_BLK 			4
#define SHARED_MEM_ADDR            		(3 << 20)
#define XLP_APP_LOADER_MAJOR       		(245)

#define PSB_MEM_MAP_MAX 			32
#define PSB_IO_MAP_MAX 				32
#define MAX_FRAGMENTS 				32

#define MAX_TLB_MAPPINGS 			16
#define MAX_ARGS 				16
#define MAX_ARGV_LEN 				16

#define PHNX_LOADER_IOC_MAGIC 			'X'
#define PHNX_LOADER_IOC_SHMEM_SIZE 		_IOR(PHNX_LOADER_IOC_MAGIC, 0, unsigned int)
#define PHNX_LOADER_IOC_MMAP_SHMEM 		_IOR(PHNX_LOADER_IOC_MAGIC, 1, unsigned int)
#define PHNX_LOADER_IOC_LIB_BKP 		_IOR(PHNX_LOADER_IOC_MAGIC, 2, unsigned int)
#define PHNX_LOADER_IOC_MMAP_LOAD_ADDR  	_IOR(PHNX_LOADER_IOC_MAGIC, 3, unsigned int)
#define PHNX_LOADER_IOC_START_IPI 		_IOR(PHNX_LOADER_IOC_MAGIC, 4, unsigned int)
#define PHNX_LOADER_IOC_STOP_IPI  		_IOR(PHNX_LOADER_IOC_MAGIC, 5, unsigned int)
#define PHNX_LOADER_IOC_ALLOC_PERSISTENT_MEM  	_IOR(PHNX_LOADER_IOC_MAGIC, 6, unsigned int)
#define PHNX_LOADER_IOC_MMAP_PERSISTENT_MEM  	_IOR(PHNX_LOADER_IOC_MAGIC, 7, unsigned int)
#define PHNX_LOADER_IOC_FREE_PERSISTENT_MEM  	_IOR(PHNX_LOADER_IOC_MAGIC, 8, unsigned int)
#define PHNX_LOADER_IOC_SHMEM_KSEG_ADDR 	_IOR(PHNX_LOADER_IOC_MAGIC, 10, unsigned int)
#define PHNX_LOADER_IOC_LAUNCH_KSEG 		_IOR(PHNX_LOADER_IOC_MAGIC, 15, unsigned int)
#define PHNX_LOADER_IOC_APP_SHMEM_SIZE 		_IOR(PHNX_LOADER_IOC_MAGIC, 25, unsigned int)
#define PHNX_LOADER_IOC_APP_SHMEM_RESERVE 	_IOR(PHNX_LOADER_IOC_MAGIC, 35, unsigned int)
#define PHNX_LOADER_IOC_MMAP_APP_SHMEM 		_IOR(PHNX_LOADER_IOC_MAGIC, 45, unsigned int)
#define PHNX_LOADER_IOC_APP_SHMEM_PHYS 		_IOR(PHNX_LOADER_IOC_MAGIC, 55, unsigned int)
#define PHNX_LOADER_STORE_ENV 			_IOR(PHNX_LOADER_IOC_MAGIC, 65, unsigned int)
#define PHNX_LOADER_SEND_IPI 			_IOR(PHNX_LOADER_IOC_MAGIC, 75, unsigned int)
#define PHNX_LOADER_IOC_STORE_APP_SHMEM_INFO 	_IOR(PHNX_LOADER_IOC_MAGIC, 85, unsigned int)
#define PHNX_LOADER_IOC_GET_APP_SHMEM_INFO 	_IOR(PHNX_LOADER_IOC_MAGIC, 95, unsigned int)
#define PHNX_LOADER_IOC_FDT_CPUMASK 		_IOWR(PHNX_LOADER_IOC_MAGIC, 105, unsigned int)

enum { KUSEG_MODE, KSEG0_MODE };
typedef enum {
	STOP_THREAD=0xbeef,
	START_THREAD,
	RUN_FUNCTION, /* Used by wakeup and wakeup_os call */

}loader_cmd;

typedef enum {
	THREAD_STOPPED=0x600d,
	THREAD_RUNNING,
	THREAD_SCHEDULED,
}thread_status;

struct cpu_tlb_mapping {
	int page_size;
	int asid;
	int coherency;
	int attr;
	unsigned long virt;
	uint64_t phys;
};

struct cpu_wakeup_info {
	int            master_cpu;
	int            map_count;
	int            valid;
	unsigned long  func;
	unsigned long  args;
	int            argc;
	uint32_t       buddy_mask;
	uint32_t       cpu_mask;
	char          *argv[32]; /* RMIOS LIB NEEDS this to be 32 */
	char           buf[256];/* must be > MAX_ARGS * MAX_ARGV_LEN + some buffer */
	struct cpu_tlb_mapping map[MAX_TLB_MAPPINGS];
};

/* SHARED memory structure b/w loader app, linux and RMIOS apps */
typedef struct phnx_loader_shared_struct {
	unsigned long park_entry;
	loader_cmd    cmd;
	thread_status thr_status;
	unsigned long entry; /* Entry point address */
	int 	      run_mode;
	struct cpu_wakeup_info run_info;
	uint32_t 	app_sh_mem_sz; /* Size of the shared memory */
	unsigned long	sp;/* Used for reentry */
	unsigned long	gp;
}phnx_loader_shared_struct_t;


/* This structure is passed to all applications launched from the linux
   loader through OS 7 scratch register
   */
typedef struct phnx_loader_info {
	uint32_t magic;
	/* phnx_loader_shared_struct_t for CPU 0 will start here */
	unsigned long sh_mem_start;
	/* Size of the shared memory b/w linux apps and rmios apps  */
	uint32_t app_sh_mem_size;
	uint8_t printk_lock[16]; /* used for printk */
}phnx_loader_info_t;

struct psb_mem_map {
	int nr_map;
	struct psb_mem_map_entry {
		uint64_t addr;  /* start of memory segment */
		uint64_t size;  /* size of memory segment */
		uint32_t type;      /* type of memory segment */
	} map[PSB_MEM_MAP_MAX];
};

struct psb_io_map {
	int nr_map;
	struct psb_io_map_entry {
		uint64_t addr;  /* start of IO segment */
		uint64_t size;  /* size of IO segment */
		long type;      /* type of IO segment */
	} map[PSB_IO_MAP_MAX];
};

struct r_exception_region {
	    unsigned int data[1024];
};

struct xlr_rmios_pt_regs {
	unsigned long long pad0[6];

	unsigned long long regs[32];

	unsigned long long cp0_status;
	unsigned long long hi;
	unsigned long long lo;

	/*
	 * saved cp0 registers
	 */
	unsigned long long cp0_badvaddr;
	unsigned long long cp0_cause;
	unsigned long long cp0_epc;
};

struct domain_info
{
	uint32_t domain;
	uint32_t cpumask;	////cpu mask -- currently 32bit.
	uint32_t mastercpu;	///master cpu id
	uint32_t mode; //// 0-smp, 1-amp
	uint64_t app_addr;	//Hyperapp app load address
	uint64_t fdt_blob;	//FDT blob address.
};

struct wakeup_info
{
       int vcpu;
       unsigned long long func;
       unsigned long long data;
};

struct xlr_load_addr
{
       uint64_t phys;
       uint64_t size;
       uint32_t flag;
};

struct xlr_lib_shared_mem
{
	uint64_t entries;
	uint64_t tot_size;
	uint64_t addr[MAX_FRAGMENTS];
	uint64_t size[MAX_FRAGMENTS];
};

struct loader_mem_info{
	uint64_t size;
	uint64_t start_addr;
};

#endif
