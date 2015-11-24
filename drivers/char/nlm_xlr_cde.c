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


#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/poll.h>

#include <linux/slab.h>         /* kmalloc() */
#include <linux/fs.h>           /* everything... */
#include <linux/errno.h>        /* error codes */
#include <linux/types.h>        /* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>        /* O_ACCMODE */
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/module.h>

#include <asm/system.h>         /* cli(), *_flags */
#include <asm/uaccess.h>        /* copy_*_user */
#include <asm/netlogic/sim.h>        /* is_xls */

#include <asm/netlogic/nlm_common_cde.h>
#include <asm/netlogic/linux_crf.h>
#include <asm/netlogic/nlm_rw_lock.h>


#define CDE_NON      0
#define CDE_STATIC   1
#define CDE_DYNAMIC  2
#define CDE_DYNAMIC2 3

#define FD_BURST_SIZE 1
#define RTN_BKT 2
#define CC_CPU0_0 0x380

#define SPILL_SIZE   1024
#define NUM_FREE_DESCRIPTORS 100 //must be less or equal to MAX_NUM_PAGES
#define MAX_NUM_PAGES 100
#define MAX_BUFFER_SIZE 1024*16
#define CMP_PAGE_SIZE   1024
#define SCRATCH_SIZE    1024

#define MAX_NUM_MESSAGES 20

extern __u32 cpu_to_frstid[];

/*
typedef struct cmp_data_structure {
  char src[MAX_BUFFER_SIZE];    // source data
  char target[MAX_BUFFER_SIZE]; // inflated or deflated result
  unsigned long long src_desc[CMP_PAGE_SIZE];  // source descriptors
  unsigned long long scratch[SCRATCH_SIZE];    // scratch page
  unsigned long long src_size;    //size of source data
  unsigned long long target_size; // size of result
  unsigned long long num_desc;    // number of source descriptors including scratch desc
  short op;                       // to deflate or inflate
} cmp_data_t;

*/

typedef struct cmp_data_structure {
  char *src;    // source data
  char *target; // inflated or deflated result
  unsigned long long *src_desc;  // source descriptors
  unsigned long long *scratch;    // scratch page
  unsigned long long src_size;    //size of source data
  unsigned long long target_size; // size of result
  unsigned long long num_desc;    // number of source descriptors including scratch desc
  short op;                       // to deflate or inflate
} cmp_data_t;

typedef enum {
	CDE_READ_DONE = 0,
	CDE_WRITE_PENDING,
	CDE_WRITE_DONE
} nlm_common_cde_state;

typedef struct msgrng_msg msg;
wait_queue_head_t cde_write_queue;
static volatile int cde_write_completed;
static spinlock_t cde_read_write_lock;

// Data Structures
struct cde_dev {
  struct cdev cdev;	  /* Char device structure	    */
  void *data;
};

typedef struct free_page_structure {
	  char *data_array;
} fr_page;

typedef struct spill_page_structure {
	  char *data_array;
} sp_page;


volatile msg cmp_msg[MAX_NUM_MESSAGES]    __attribute__((aligned(32)));
volatile fr_page page_array[MAX_NUM_PAGES]    __attribute__((aligned(32)));
volatile sp_page spill_page  __attribute__((aligned(32)));
volatile char *spill_page_tmp_data_array __attribute__((aligned(32)));
volatile char *(page_array_tmp_data_array[MAX_NUM_PAGES])__attribute__((aligned(32)));


#define CACHELINE_ALIGNED_ADDR(addr) (((unsigned long)(addr)) & ~(SMP_CACHE_BYTES-1))

static __inline__ void *cacheline_aligned_kmalloc(int size, char **buf_addr, int gfp_mask)
{
	void *buf = kmalloc(size + SMP_CACHE_BYTES, gfp_mask);
	
	if (buf)	{
		*buf_addr = (char *)buf;
		buf = (void*)(CACHELINE_ALIGNED_ADDR((unsigned long)buf +
					SMP_CACHE_BYTES));
	}
	return buf;
}

static int cde_debug = 0;


static int config_cmp(void)
{
  int i;
  nlm_reg_t *cmp_mmio = netlogic_io_mmio(NETLOGIC_IO_COMP_OFFSET);
  
  spill_page.data_array = cacheline_aligned_kmalloc(SPILL_SIZE+SMP_CACHE_BYTES,(char **)&spill_page_tmp_data_array, GFP_KERNEL);
  if(!spill_page.data_array)
	  return -1;

  if (dev_tree_en) {
    char bkt[8], cc[16][8];
    int rv = fdt_get_cde_bucket_conf(bkt, 4, cc, 128);

    if (rv == -1) {
       printk(KERN_ERR "CDE: Could not get credit info from FDT\n");
       return -1;
    }
    netlogic_write_reg(cmp_mmio, CMP_MSG_BUCKET0_SIZE, bkt[0]);
    netlogic_write_reg(cmp_mmio, CMP_MSG_BUCKET1_SIZE, bkt[1]);

    for (i = 0; i < 32; i++)
       netlogic_write_reg(cmp_mmio, CC_CPU0_0 + i, cc[i/8][i%8]);
  } else {
    netlogic_write_reg(cmp_mmio, CMP_MSG_BUCKET0_SIZE, xls_bucket_sizes.bucket[MSGRNG_STNID_CMP_0]);
    netlogic_write_reg(cmp_mmio, CMP_MSG_BUCKET1_SIZE, xls_bucket_sizes.bucket[MSGRNG_STNID_CMP_1]);
    for (i = 0; i < 32; i++)
      netlogic_write_reg(cmp_mmio, CC_CPU0_0 + i, 
			xls_cc_table_cmp.counters[i >> 3][i & 0x7]);
  }
   
  cmp_write_reg(CMP_REG_CTRL_REG,        ((0x39CE << 16) | CMP_PAGE_SIZE)); //16'h{CMP_PAGE_SIZE}
  cmp_write_reg(CMP_REG_DMA_CREDITS_REG, 0x0FFFFFFF);
  cmp_write_reg(CMP_REG_SPILL_ADDR0_REG, (virt_to_phys(spill_page.data_array) >> 5) & 0xffffffffffffffffUll);
  cmp_write_reg(CMP_REG_SPILL_ADDR1_REG, ((__u64)virt_to_phys(spill_page.data_array) >> 36) & 0x7);
  cmp_write_reg(CMP_REG_SPILL_SIZE_REG,  SPILL_SIZE); //16'h{SPILL_SIZE}

  if (cde_debug) {
    printk("COMP OFFSET = 0x%x\n", NETLOGIC_IO_COMP_OFFSET);
    printk("register = 0x%0x data = 0x%0x\n", CMP_MSG_BUCKET0_SIZE,cmp_mmio[CMP_MSG_BUCKET0_SIZE]);
    printk("register = 0x%0x data = 0x%0x\n", CMP_MSG_BUCKET1_SIZE,cmp_mmio[CMP_MSG_BUCKET1_SIZE]);

    printk("register=0x%0x data=0x%0x\n", CMP_REG_CTRL_REG, cmp_read_reg(CMP_REG_CTRL_REG));
    printk("register=0x%0x data=0x%0x\n", CMP_REG_DMA_CREDITS_REG, cmp_read_reg(CMP_REG_DMA_CREDITS_REG));
    printk("register=0x%0x data=0x%0x\n", CMP_REG_SPILL_ADDR0_REG, cmp_read_reg(CMP_REG_SPILL_ADDR0_REG));
    printk("register=0x%0x data=0x%0x\n", CMP_REG_SPILL_ADDR1_REG, cmp_read_reg(CMP_REG_SPILL_ADDR1_REG));
    printk("register=0x%0x data=0x%0x\n", CMP_REG_SPILL_SIZE_REG, cmp_read_reg(CMP_REG_SPILL_SIZE_REG));
  }
  return 0;
}

static int send_message(int stid, struct msgrng_msg *msg)
{
  unsigned long mflags = 0;
  int ret = 0;

  msgrng_flags_save(mflags);
  ret = message_send_retry(1, 0, stid, msg);
  msgrng_flags_restore(mflags);

  return ret;
}


static int send_free_desc(void)
{
  int i;
  int status = 0;
  int stid;
  struct msgrng_msg fd_msg;


  // send free descriptors to cmp block
  for (i = 0; i < NUM_FREE_DESCRIPTORS; i++) {
    if (!page_array[i].data_array)	  
       page_array[i].data_array = cacheline_aligned_kmalloc(CMP_PAGE_SIZE+SMP_CACHE_BYTES, (char **)&page_array_tmp_data_array[i], GFP_KERNEL);
    if (!page_array[i].data_array)	{
	    printk("cacheline_aligned_kmalloc returmed error\n");
	    return -1;
    }

    stid = make_fd_msg(&fd_msg, page_array[i].data_array);
    //    printk("Free descriptor message [%0d] = 0x%016llx\n", i, fd_msg.msg0);

    status = send_message(MSGRNG_STNID_CMP_0, &fd_msg);

    if (status != 0) {
      printk("[%s@%d]: Free descriptor (%d) didnt not reach cmpm status=%0d\n",
	     __FUNCTION__, __LINE__, i, status);
      return -1;
    }
  }

  if (cde_debug) {
    printk("[%s@%d]: Sent %d free desc to comp engine\n",
	   __FUNCTION__, __LINE__, i);
  }

  return 0;
}


int create_message(cmp_data_t *cmp_data, int num_blk, int num_desc, int en_save_restore)
{
  int num_messages;
  int i,j,k,stid;

  int type = CDE_STATIC;
  int rtn_bkt;

  int cur_blk = 0;
  int cur_desc = 0;

  int eof  = 0;
  int sod  = 0;
  int sob  = 0;
  int eob  = 0;
  int save = 0;
  int restore = 0;

  int length = 0;
  int start = 0;
  int div = num_blk * num_desc;

  int desc_idx = 0;

  if (en_save_restore)
    num_messages = num_blk;
  else
    num_messages = 1;

  rtn_bkt = cpu_to_frstid[hard_smp_processor_id()];
  /*
   * dliao: num_messages = 1 for now. not sure how it works when num_messages > 0 or num_desc > 0
   * what is num_blk or num_desc for ?
   */
  for (i = 0; i < num_messages; i++) {
    //create scratch page descriptor
    cur_desc = 0;
    restore = ((num_messages > 1) & (cur_blk != 0));
    length = SCRATCH_SIZE;

    cmp_data->src_desc[desc_idx] =
      make_src_desc(0, 0, 0, 0, 0, restore, 0, length, cmp_data->scratch);

    if (cde_debug) {
      printk("scratch=0x%p, v2p=0x%lx, scratch_desc = 0x%016llx &scratch_desc=0x%p\n", 
	     cmp_data->scratch, virt_to_phys(cmp_data->scratch), cmp_data->src_desc[desc_idx], &(cmp_data->src_desc[0])); 
    }

    cur_desc++;

    for (j = 0; j < num_blk/num_messages; j++) {
      for (k = 0; k < num_desc; k++) {
	eof = (cur_blk == num_blk-1);

	sod = (start == 0);
	sob = (k == 0);
	eob = (k == num_desc-1);
	save = ((en_save_restore == 1) & (eob == 1) & (eof == 0));
	restore = 0; //restore only can be 1 on scratch descriptor

	//	length = (cmp_data->src_size - start) / div;
	length = (cmp_data->src_size - start); //dliao: assume div == 1

	cmp_data->src_desc[cur_desc] =
	  make_src_desc(eof, type, sod, sob, save, restore, eob, length, cmp_data->src + start);

	if (cde_debug) {
	  printk("[%s@%d]: eof=%d, sod=%d, sob=%d, eob=%d, save=%d, restore=0, length=%d\n",
		 __FUNCTION__, __LINE__, eof, sod, sob, eob, save, length);
	  
	  printk("[%s@%d]: cur_desc=%d, i=%d, j=%d, k=%d, &(cmp_data->src)=0x%p, src_desc[1]=0x%llx &(src_desc[1])=0x%p\n",
		 __FUNCTION__, __LINE__,
		 cur_desc, i, j, k, cmp_data->src, cmp_data->src_desc[cur_desc], &(cmp_data->src_desc[cur_desc]));
	}

	start = start + length;
	div--;
	cur_desc++;
      }

      if (type == 2) {//dynamic has 2x descriptors
	cur_desc = cur_desc + num_desc;
      }

      cur_blk++;
    }

    stid = make_cmp_msg((struct msgrng_msg *) (cmp_msg + i), rtn_bkt, cmp_data->op,
			cur_desc, cmp_data->src_desc + desc_idx);
    if (cde_debug) {
      printk("[%s@%d]: desc_idx=%d, &(src_desc[0])=0x%p, v2p=0x%lx, i=%d, cmp_msg.msg0 = 0x%016llx\n", __FUNCTION__, __LINE__, desc_idx, cmp_data->src_desc+0, virt_to_phys(cmp_data->src_desc+0),i, cmp_msg[i].msg0); 
    }

    desc_idx = desc_idx + cur_desc;
  }

  // returns the number of messages created
  return (num_messages);
}


void return_free_descriptors(msg *msg_list, int msg_index)
{
  int i, j, k, status, num_desc;
  msg return_msg, fd_msg;
  uint64_t * temp_desc;
  uint64_t dest_desc;
  int used_fd_index = 0;

  volatile uint64_t used_fd[NUM_FREE_DESCRIPTORS] __attribute__((aligned(32)));

  for (i = 0; i < msg_index; i++) {

    memcpy ((void *) &return_msg, (void *)(msg_list+i), sizeof(msg));
    used_fd[used_fd_index++] = ((uint64_t) return_msg.msg1 & 0xffffffffffUll);

    num_desc = (return_msg.msg1>>40 & 0xffff);

    temp_desc = (uint64_t *) phys_to_virt(return_msg.msg1 & 0xffffffffffUll);

    for (j = 0; j< num_desc; j++) {
      dest_desc = temp_desc[j];
      used_fd[used_fd_index++] = ((uint64_t) dest_desc & 0xffffffffffUll);
    }


    if (used_fd_index > FD_BURST_SIZE) {
      for (k = 0; k < used_fd_index; k++) {
	fd_msg.msg0 = ((uint64_t) used_fd[k]);
	status = send_message(MSGRNG_STNID_CMP_0, &fd_msg);
	if (status != 0)
	  printk("Return free descriptor didnt not reach cmp! status=%0d\n", status);
	}
        used_fd_index = 0;
    }
  }
}

static cmp_data_t *cmp_data = 0;

static void nlm_common_msgring_comp_int_handler(int bucket, int size, int code, int stid,
					  struct msgrng_msg *msg, void *data/* ignored */)
{
  int last = 0; 
  int msg_index = 0; 
  struct msgrng_msg msg_list[MAX_NUM_MESSAGES]; 
  int offset = cmp_data->target_size;
  
  if (cde_debug) {
    printk("[%s@%d]: bucket=%d, size=%d, code=%d, stid=%d "
	   " msg0=0x%016llx, msg1=0x%016llx\n",
	   __FUNCTION__, __LINE__, bucket, size, code, stid, msg->msg0, msg->msg1);
    printk("@msg = %p \n", msg);
  }

  // TODO need to fix not-last case
  last = ((msg->msg0 >> 63) & 0x1ULL); 
  
  if (cde_debug)
    printk("[%s@%d]: last = %d\n", __FUNCTION__, __LINE__, last); 
    
  offset = cmp_data->target_size; 
  cmp_data->target_size += read_cmp_msg((char *) cmp_data->target + offset, msg->msg1);

  if (last) {
	  spin_lock(&cde_read_write_lock);
	  cde_write_completed = CDE_WRITE_DONE;
	  spin_unlock(&cde_read_write_lock);
  }

  memcpy((void *)&msg_list[msg_index],(void *)msg, sizeof(struct msgrng_msg));

  if (cde_write_completed == CDE_WRITE_DONE) {
	  wake_up_interruptible(&cde_write_queue);
  }

  return_free_descriptors(msg_list, msg_index); 
//  printk("[\n%s@%d]:end of nlm_common_msgring_comp_int_handler\n", __FUNCTION__, __LINE__);
}

/*
 * Our parameters which can be set at load time.
 */

static int cde_major =   CDE_MAJOR;
static int cde_minor =   0;
static int cde_nr_devs = 1;

module_param(cde_major, int, S_IRUGO);
module_param(cde_minor, int, S_IRUGO);
module_param(cde_nr_devs, int, S_IRUGO);

struct cde_dev *cde_device;
static int cde_open_flag = 0;
static spinlock_t cde_open_lock; 


int cde_open(struct inode *inode, struct file *filp)
{
  spin_lock(&cde_open_lock);
  if (cde_open_flag)
  {       spin_unlock(&cde_open_lock);
	  return -EAGAIN;
  }
 
  cde_open_flag = 1;
  spin_unlock(&cde_open_lock);

  cmp_data = (cmp_data_t *) kmalloc(sizeof(cmp_data_t), GFP_KERNEL);
  if (!cmp_data)
    return -ENOMEM;
  memset(cmp_data, 0, sizeof(cmp_data_t));

  cmp_data->src = kmalloc(MAX_BUFFER_SIZE, GFP_KERNEL);
  if (! cmp_data->src)	{
	  printk("kmalloc returns Error : cmp_data->src\n");
	  kfree(cmp_data);
	  
	  return -ENOMEM;
  }
	  
  cmp_data->target = kmalloc(MAX_BUFFER_SIZE, GFP_KERNEL);
  if (! cmp_data->target)	{
	  printk("kmalloc returns Error : cmp_data->target\n");
	  kfree(cmp_data->src);
	  kfree(cmp_data);
	  return -ENOMEM;
  }
  cmp_data->src_desc = kmalloc(sizeof(unsigned long long) * CMP_PAGE_SIZE , GFP_KERNEL);
  if (! cmp_data->src_desc)	{
	  printk("kmalloc returns Error : cmp_data->src_desc\n");
	  kfree(cmp_data->src);
	  kfree(cmp_data->target);
	  kfree(cmp_data);
	  return -ENOMEM;
  }
  cmp_data->scratch = kmalloc(sizeof(unsigned long long) * SCRATCH_SIZE , GFP_KERNEL);
  if (! cmp_data->scratch)	{
	  printk("kmalloc returns Error : cmp_data->scratch\n");
	  kfree(cmp_data->src);
	  kfree(cmp_data->target);
	  kfree(cmp_data->src_desc);
	  kfree(cmp_data);
	  return -ENOMEM;
  }
  memset(cmp_data->src, 0, MAX_BUFFER_SIZE);
  memset(cmp_data->target, 0, MAX_BUFFER_SIZE);
  memset(cmp_data->src_desc, 0, sizeof(unsigned long long) * CMP_PAGE_SIZE);
  memset(cmp_data->scratch, 0, sizeof(unsigned long long) * SCRATCH_SIZE);

  if (config_cmp() == -1)
	return -ENOMEM;

  if (send_free_desc() == -1)
	return -ENOMEM;

  init_waitqueue_head(&cde_write_queue);
  return 0;
}


int cde_release(struct inode *inode, struct file *filp)
{ 
	
  int i;
  if (cmp_data->src)  
	  kfree(cmp_data->src);
  if (cmp_data->target)  
	  kfree(cmp_data->target);
  if (cmp_data->src_desc)  
	  kfree(cmp_data->src_desc);
  if (cmp_data->scratch)  
	  kfree(cmp_data->scratch);
  if (cmp_data)
    kfree(cmp_data);
    cmp_data->src = NULL;
    cmp_data->target = NULL;
    cmp_data->src_desc = NULL; 
    cmp_data->scratch = NULL;
    cmp_data = NULL;

  
  cmp_write_reg(CMP_REG_RESET_REG, 0x10);
  do {
    unsigned int ret = cmp_read_reg(CMP_REG_RESET_REG);
    if (((ret >> 4) & 0x1) == 1)
      break;
  } while (1);

  cmp_write_reg(CMP_REG_RESET_REG, 0x01);
  cmp_write_reg(CMP_REG_RESET_REG, 0x00);
  
  if (spill_page.data_array)	{
    kfree((void *)spill_page_tmp_data_array);
    spill_page_tmp_data_array = NULL;
    spill_page.data_array = NULL;
  }

  for (i = 0; i < NUM_FREE_DESCRIPTORS; i++) {
    if (page_array[i].data_array)	{
	kfree((void *)page_array_tmp_data_array[i]);
	page_array_tmp_data_array[i] = NULL;
	page_array[i].data_array = NULL;
    }	
  }
  spin_lock(&cde_open_lock);
	cde_open_flag = 0;
  spin_unlock(&cde_open_lock);
  return 0;
}


// TODO: 1. fix the case when more data than user want to read.
//       2. mutiple reads
ssize_t cde_read(struct file *filp, char __user *buf,
		 size_t count, loff_t *f_pos)
{
  int size = 0;
  unsigned long irq_flags;
  static volatile int readlock;

  spin_lock_irqsave(&cde_read_write_lock, irq_flags);
  if (readlock == 1)
  {
  	spin_unlock_irqrestore(&cde_read_write_lock, irq_flags);
	return -EAGAIN;
  }
  readlock = 1;
  spin_unlock_irqrestore(&cde_read_write_lock, irq_flags);

  if (cde_write_completed != CDE_WRITE_DONE)	{
	  wait_event_interruptible(cde_write_queue, 
			  (cde_write_completed == CDE_WRITE_DONE) );         
  }
  
  spin_lock_irqsave(&cde_read_write_lock, irq_flags);
  if (cde_write_completed != CDE_WRITE_DONE) {
	  readlock = 0;
	  spin_unlock_irqrestore(&cde_read_write_lock, irq_flags);
	  return -EAGAIN;
  }
  spin_unlock_irqrestore(&cde_read_write_lock, irq_flags);

  size = count < cmp_data->target_size ? count : cmp_data->target_size;
  
  if (copy_to_user(buf, cmp_data->target, size)) {
    printk("copy_to_user failed\n");
    size = -EFAULT;
  }

  cmp_data->target_size = 0;

  spin_lock_irqsave(&cde_read_write_lock, irq_flags);
  cde_write_completed = CDE_READ_DONE;
  readlock = 0;
  spin_unlock_irqrestore(&cde_read_write_lock, irq_flags);

  return size;
}


ssize_t cde_write(struct file *filp, const char __user *buf,
		  size_t count, loff_t *f_pos)
{
  int i;
  int num_messages = 0;
  unsigned long irq_flags;

  spin_lock_irqsave(&cde_read_write_lock, irq_flags);
  if (cde_write_completed != CDE_READ_DONE)
  {
	  spin_unlock_irqrestore(&cde_read_write_lock, irq_flags);
	  return -EAGAIN;
  }
  cde_write_completed = CDE_WRITE_PENDING;
  spin_unlock_irqrestore(&cde_read_write_lock, irq_flags);
  
  if (copy_from_user(cmp_data->src, buf, count)) {
	  spin_lock_irqsave(&cde_read_write_lock, irq_flags);
	  cde_write_completed = CDE_READ_DONE;
	  spin_unlock_irqrestore(&cde_read_write_lock, irq_flags);
	  return -EFAULT;
  }

  cmp_data->src_size = count;

  num_messages = create_message(cmp_data, 1, 1, 0);

  // send message
  for (i = 0; i < num_messages; i++) {

    int status = send_message(MSGRNG_STNID_CMP_1, (struct msgrng_msg *) (cmp_msg + i));

    if (status != 0) {
	    printk("Cmp Message didnt not reach cmp, status=%0d\n", status);
	    spin_lock_irqsave(&cde_read_write_lock, irq_flags);
	    cde_write_completed = CDE_READ_DONE;
	    spin_unlock_irqrestore(&cde_read_write_lock, irq_flags);
    }

  }

  return count;
}


int cde_ioctl(struct inode *inode, struct file *filp,
	      unsigned int cmd, unsigned long arg)
{
  int err = 0;
  /*
    if (_IOC_TYPE(cmd) != CDE_IOC_MAGIC)
    return -ENOTTY;

    if (_IOC_NR(cmd) > CDE_IOC_MAXNR)
    return -ENOTTY;

    if (_IOC_DIR(cmd) & _IOC_READ)
    err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));

    else if (_IOC_DIR(cmd) & _IOC_WRITE)
    err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

    if (err)
    return -EFAULT;
  */

  switch(cmd) {

  case CDE_INFLATE:
    cmp_data->op = CDE_INFLATE;
    break;

  case CDE_DEFLATE:
    cmp_data->op = CDE_DEFLATE;
    break;

  default:
    return -ENOTTY;
  }

  return err;
}

long  cde_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  unsigned long ret = -1;
  lock_kernel();
  ret = cde_ioctl(NULL, filp, cmd, arg);
  unlock_kernel();
  if(ret){
     printk("%s: ioctl error\n", __FUNCTION__);
     return -EINVAL;
  }
  return ret;
}

struct file_operations cde_fops = {
  .owner =    THIS_MODULE,
  .read =     cde_read,
  .write =    cde_write,
  .ioctl =    cde_ioctl,
  .compat_ioctl = cde_compat_ioctl,
  .open =     cde_open,
  .release =  cde_release,
};


void cde_cleanup_module(void)
{
  dev_t devno = MKDEV(cde_major, cde_minor);
  
  if (cde_device) {
    cdev_del(&(cde_device->cdev));
    kfree(cde_device);
  }

  /* cleanup_module is never called if registering failed */
  unregister_chrdev_region(devno, cde_nr_devs);
}


static int cde_setup_cdev(struct cde_dev *dev)
{
  int err, devno = MKDEV(cde_major, cde_minor);

  cdev_init(&dev->cdev, &cde_fops);
  dev->cdev.owner = THIS_MODULE;
  dev->cdev.ops = &cde_fops;
  err = cdev_add(&dev->cdev, devno, 1);

  if (err)
    printk(KERN_NOTICE "Error %d adding cde", err);

  return err;
}


#ifdef CONFIG_NLM_MSGRING_NAPI
extern int nlm_on_chip_napi;
extern int nlm_msgring_napi;
#endif /* CONFIG_NLM_MSGRING_NAPI */


int cde_init_module(void)
{
  int ret;
  dev_t dev = 0;

  if (!is_xls_b0())
    return 0;
  cde_write_completed = CDE_READ_DONE;
  spin_lock_init(&cde_open_lock);
  spin_lock_init(&cde_read_write_lock);

  if (dev_tree_en && fdt_get_cde_enabled() == 0) {
    printk("Compression Engine disabled in configuration, skipping...\n");
    return -ENODEV;
  }

#ifdef CONFIG_NLM_MSGRING_NAPI
  if (nlm_msgring_napi && !nlm_on_chip_napi) {
	  printk(KERN_ALERT "%s: RMI Compression Driver: Incompatibility with GMAC"
		 " NAPI mode\n", __FUNCTION__);
	  printk(KERN_ALERT "%s: RMI Compression Driver: Aborting init sequence.\n", 
		 __FUNCTION__);
	  return -EINVAL;
  }
#endif /* CONFIG_NLM_MSGRING_NAPI */

  if (cde_major) {
    dev = MKDEV(cde_major, cde_minor);
    ret = register_chrdev_region(dev, cde_nr_devs, "xls_cde");
  }
  else {
    ret = alloc_chrdev_region(&dev, cde_minor, cde_nr_devs, "xls_cde");
    cde_major = MAJOR(dev);
  }

  if (ret < 0) {
    printk(KERN_WARNING "xls_cde: can't get major %d\n", cde_major);
    return ret;
  }

  /*
   * allocate the devices or static variable?
   */
  cde_device = kmalloc(sizeof(struct cde_dev), GFP_KERNEL);
  if (!cde_device) {
    printk(KERN_WARNING "xls_cde: can't allocate memory\n");
    ret = -ENOMEM;
    goto fail;
  }

  memset(cde_device, 0, sizeof(struct cde_dev));

  ret = cde_setup_cdev(cde_device);
  if (ret)
    goto fail;
  ret = register_msgring_handler(TX_STN_CMP, nlm_common_msgring_comp_int_handler, NULL);
  if (ret) {
    printk("[%s@%d]: unable to register handler for msgring stations for Compression Station\n",
	   __FILE__, __LINE__);
    goto fail;
  }

  return 0;

 fail:
  cde_cleanup_module();
  return ret;
}

module_init(cde_init_module);
module_exit(cde_cleanup_module);
