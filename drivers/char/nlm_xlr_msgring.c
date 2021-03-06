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
#include <linux/module.h>

#include <asm/uaccess.h>
#include <asm/mman.h>

#include <asm/netlogic/pic.h>
#include <asm/netlogic/msgring.h>
#include <asm/netlogic/sim.h>
#include <asm/netlogic/devices.h>
#include <asm/netlogic/xlr_user_mac.h>

#include <user/netlogic/nlm_common_msgring.h>

#define MAX_MSG_FIFOS 12
#define MSG_FIFO_SIZE 1024
#define MSG_FIFO_MASK (MSG_FIFO_SIZE-1)

enum LEVEL {
	LOG_EMERG = 0,
		LOG_ALERT,
		LOG_CRIT,
		LOG_ERR,
		LOG_WARNING,
		LOG_NOTICE,
		LOG_INFO,
		LOG_DEBUG
};

#define PRINTK_ERR(FMT, ...) \
do { \
    if(msgring_debug_level >= LOG_ERR) { \
        printk(KERN_ERR FMT, ## __VA_ARGS__); \
    } \
} while(0)

#define PRINTK_INFO(FMT, ...) \
do { \
    if(msgring_debug_level >= LOG_INFO) { \
        printk(KERN_INFO FMT, ## __VA_ARGS__); \
    } \
} while(0)

typedef enum syscall_id_s {
	SYSCALL_exec,
	SYSCALL_socketcall,
	SYSCALL_open,
	SYSCALL_write,
	SYSCALL_read,
	SYSCALL_close,
	SYSCALL_ioctl,
	SYSCALL_select,
	SYSCALL_exit,
	SYSCALL_interrupt,
	SYSCALL_max
} SYSCALL_ID;

typedef struct syscall {
	wait_queue_head_t sleep;
	int done;
	void *process;
	int pid;
	SYSCALL_ID id;
	int err;
	int _errno;
	int num;
	int src_id;
	void *trace;
	union {
		struct {
			int call;
		} socketcall;
	} u;
} SYSCALL;

extern char *saved_command_line;
static char *hybrid = 0;
extern int msgring_debug_level;

extern void *nlm_common_psb_shm;
extern unsigned long nlm_common_psb_shm_size;

static int    msgring_major;


struct fifo {
	struct msgring_msg_data *data;
	int       size;
	int       head;
	int       tail;
	spinlock_t lock;
	wait_queue_head_t wq;
};

static void fifo_init (struct fifo *fifo, int fifo_size)
{
	void *ptr = kmalloc(fifo_size * sizeof(struct msgring_msg_data), GFP_KERNEL);

	if (!ptr) panic("[%s]: Unable to allocate memory for Tx Fifos\n", __FUNCTION__);

	fifo->data = ptr;
	fifo->head = fifo->tail = 0;
	fifo->size = fifo_size;
	spin_lock_init(&fifo->lock);
}

/* TODO: Change all modulos to boolean arithmetic */
__inline__ int fifo_next_index(volatile struct fifo *fifo, int index)
{
	return (index+1) & MSG_FIFO_MASK;
}

__inline__ int  fifo_next_head(volatile struct fifo *fifo)
{ return (fifo->head+1) & MSG_FIFO_MASK ; }

__inline__ int  fifo_next_tail(volatile struct fifo *fifo)
{ return (fifo->tail+1) & MSG_FIFO_MASK ; }

__inline__ int  fifo_empty(volatile struct fifo *fifo)
{ return (fifo->head == fifo->tail); }

static __inline__ int  fifo_full(volatile struct fifo *fifo)
{ return (fifo_next_tail(fifo) == fifo->head); }

static __inline__ int  fifo_count(volatile struct fifo *fifo)
{
	if (fifo->head <= fifo->tail)
		return fifo->tail - fifo->head;
	else
		return (fifo->size - fifo->head) + fifo->tail;
}
static __inline__ int fifo_dequeue(volatile struct fifo *fifo,
				   struct msgring_msg_data *data)
{
	if (fifo_empty(fifo))
		return 0;

	*data = fifo->data[fifo->head];
	fifo->head = fifo_next_head(fifo);

	return 1;
}
static __inline__ int fifo_enqueue(volatile struct fifo *fifo,
				   struct msgring_msg_data *data)
{
	int cnt = 0;
	while (fifo_full(fifo)) {
		++cnt;
	}
	if(cnt > 10000) {
		PRINTK_ERR("%s:%d fifo queue full cnt=%d.\n", __FUNCTION__, __LINE__,
			   cnt);
	}

	fifo->data[fifo->tail] = *data;
	fifo->tail = fifo_next_tail(fifo);

	return 1;
}

#define PRINT_NLM_FIFO_DATA(p) \
do { \
	PRINTK_INFO("%s:%d fifo=%p fifo size=%d head=%d tail=%d.\n", __FUNCTION__, \
		    __LINE__, p, p->size, p->head, p->tail); \
} while(0)

static struct fifo msg_fifos[MAX_MSG_FIFOS];

#ifdef CONFIG_NLMCOMMON_MAC
extern void nlm_xlr_rmios_msgring_handler(int bucket, int size, int code, int stid,
					   struct msgrng_msg *msg, void *data/* ignored */);
#else /* CONFIG_NLMCOMMON_MAC */
void nlm_xlr_rmios_msgring_handler(int bucket, int size, int code,
                                    int stid, struct msgrng_msg *msg,
                                    void *data /* ignored */ ) { }
#endif /* CONFIG_NLMCOMMON_MAC */
extern void nlm_nlm_common_mac_msgring_handler(int bucket, int size, int code, int stid,
                                           struct msgrng_msg *msg, void *data/* ignored */);
static void nlm_nlm_common_syscall_msgring_handler(int bucket, int size, int code, int stid,
                                           struct msgrng_msg *msg, void *data/* ignored */);



void nlm_common_msgring_drv_int_handler(int bucket, int size, int code, int stid,
				  struct msgrng_msg *msg, void *data/* ignored */)
{
	volatile struct fifo *msg_fifo = 0;
	struct msgring_msg_data msg_data;
	int ret = 0;
	int tx_stid = MAX_MSG_FIFOS;


	if (stid < MSGRING_STNID_DEVICES) {


		if (((code&0xf) == 0xf) && hybrid) {
			// forward to linux driver
			nlm_xlr_rmios_msgring_handler(bucket, size, code, stid, msg, data);
			return;
		}
		if (((code&0xf) == 0xe) && hybrid) {
			nlm_nlm_common_syscall_msgring_handler(bucket, size, code, stid, msg, data);
			return;
		}
		tx_stid = stid >> 3;
	}
	else {
		if (stid == 96 || (is_xls() && stid == 80)) {
			tx_stid = 8 + (msg->msg1 & 0x0f);
		}
		else {
			printk("[%s]: illegal tx_stid = %d, stid=%d\n", __FUNCTION__, tx_stid, stid);
			return;
		}
	}

	//printk("[%s:%d]: \n", __FUNCTION__, __LINE__);

	if (hybrid)
		tx_stid = (msg->msg0 & 0xffffffff);

	msg_fifo = &msg_fifos[tx_stid];

	msg_data.size = size;
	msg_data.code = code;
	msg_data.rx_bucket = bucket;
	msg_data.stid = stid;
	msg_data.msgs[0] = (msg->msg0 & 0xffffffff);
	msg_data.msgs[1] = (msg->msg0 >> 32);
	msg_data.msgs[2] = (msg->msg1 & 0xffffffff);
	msg_data.msgs[3] = (msg->msg1 >> 32);
	msg_data.msgs[4] = (msg->msg2 & 0xffffffff);
	msg_data.msgs[5] = (msg->msg2 >> 32);
	msg_data.msgs[6] = (msg->msg3 & 0xffffffff);
	msg_data.msgs[7] = (msg->msg3 >> 32);

	PRINTK_INFO("%s:%d adding message to txid=%d fifo:%p\n", __FUNCTION__, __LINE__,
		    tx_stid, msg_fifo);
	spin_lock((spinlock_t *)(&msg_fifo->lock));
	ret = fifo_enqueue(msg_fifo, &msg_data);
	spin_unlock((spinlock_t *)(&msg_fifo->lock));

	/* wake up any readers */
	wake_up_interruptible((wait_queue_head_t *)&msg_fifo->wq);

	if (!ret) {
		PRINTK_ERR("[%s]: Unable to queue message from %d tx_stid (stid=%d)\n", __FUNCTION__, tx_stid, stid);
	}

}

static int msgring_open (struct inode *inode, struct file *filp)
{
	//printk("msgring_open() invoked\n");

	filp->private_data = NULL;

	return 0;
}

static DECLARE_WAIT_QUEUE_HEAD(msgring_read_wait);
static DECLARE_WAIT_QUEUE_HEAD(msgring_write_wait);

static unsigned int msgring_poll(struct file *filp, struct poll_table_struct *wait)
{
	unsigned int mask;
	volatile struct fifo *msg_fifo = NULL;
	int count = 0;
	unsigned long flags;

	if (!filp->private_data) return -EINVAL;

	msg_fifo = filp->private_data;

	mask = 0;

	spin_lock_irqsave((spinlock_t *)&msg_fifo->lock, flags);
	count = fifo_count(msg_fifo);
	spin_unlock_irqrestore((spinlock_t *)&msg_fifo->lock, flags);

	if(!count) {
		poll_wait(filp, (wait_queue_head_t *)&msg_fifo->wq, wait);

		spin_lock_irqsave((spinlock_t *)&msg_fifo->lock, flags);
		count = fifo_count(msg_fifo);
		spin_unlock_irqrestore((spinlock_t *)&msg_fifo->lock, flags);
	}
	/* if there is data in the read buffer */
	if (count > 0) {
		PRINT_NLM_FIFO_DATA(msg_fifo);
		mask |= POLLIN | POLLRDNORM;
	} else {
		PRINT_NLM_FIFO_DATA(msg_fifo);
		PRINTK_INFO("%s:%d count=%d\n", __FUNCTION__, __LINE__, count);
	}

	/* mark it writable always! */
	mask |= POLLOUT | POLLWRNORM;

	return mask;
}

static ssize_t msgring_read (struct file *filp, char __user *buf, size_t count, loff_t *offset)
// offset: the offset in the file
// count : no of bytes to read
// buf   : start location (in user space) to which to copy the contents
// filp  : pointer to 'struct file'  of the file
{
	volatile struct fifo *msg_fifo = NULL;
	int msg_size = sizeof(struct msgring_msg_data);
	struct msgring_msg_data msg;
	int ret = 0;
	unsigned long flags;

	if (!filp->private_data) return -EINVAL;

	if (count < msg_size) return -EINVAL;

	msg_fifo = filp->private_data;

	// we don't care about the passed file offset, but will update it
	// with the no of bytes read
 retry:
	spin_lock_irqsave((spinlock_t *)&msg_fifo->lock, flags);
	ret = fifo_dequeue(msg_fifo, &msg);
	spin_unlock_irqrestore((spinlock_t *)&msg_fifo->lock, flags);

	if (!ret) {
		if (filp->f_flags & O_NONBLOCK) return -EAGAIN;
		if (wait_event_interruptible((((struct fifo *)(msg_fifo))->wq), 
					fifo_count(msg_fifo)))
			return -ERESTARTSYS;
		goto retry;
	}

	PRINTK_INFO("%s:%d reading message from fifo:%p\n", __FUNCTION__, __LINE__, msg_fifo);
	if ( __copy_to_user(buf, &msg, count) )
		return -EFAULT;

	filp->f_pos += count;
	return count;
}

static ssize_t msgring_write(struct file *filp, const char __user *buf, size_t count, loff_t *off)
{
	struct msgring_msg_data *umsg_data = NULL;
	int msg_size = sizeof(struct msgring_msg_data);
	struct msgrng_msg msg;
	int size=0, code=0, stid=0;
	unsigned long mflags = 0;
	struct msgring_msg_data msg_data;

	if (count < msg_size) return -EINVAL;

	umsg_data = (struct msgring_msg_data *)buf;

	__copy_from_user(&size, &umsg_data->size, 4);
	__copy_from_user(&code, &umsg_data->code, 4);
	__copy_from_user(&stid, &umsg_data->stid, 4);

	__copy_from_user(&msg_data.msgs[0], &umsg_data->msgs[0], 4);
	__copy_from_user(&msg_data.msgs[1], &umsg_data->msgs[1], 4);
	__copy_from_user(&msg_data.msgs[2], &umsg_data->msgs[2], 4);
	__copy_from_user(&msg_data.msgs[3], &umsg_data->msgs[3], 4);
	__copy_from_user(&msg_data.msgs[4], &umsg_data->msgs[4], 4);
	__copy_from_user(&msg_data.msgs[5], &umsg_data->msgs[5], 4);
	__copy_from_user(&msg_data.msgs[6], &umsg_data->msgs[6], 4);
	__copy_from_user(&msg_data.msgs[7], &umsg_data->msgs[7], 4);

	msg.msg0 = ((__u64)msg_data.msgs[1] << 32) | ((__u64)msg_data.msgs[0]);
	msg.msg1 = ((__u64)msg_data.msgs[3] << 32) | ((__u64)msg_data.msgs[2]);
	msg.msg2 = ((__u64)msg_data.msgs[5] << 32) | ((__u64)msg_data.msgs[4]);
	msg.msg3 = ((__u64)msg_data.msgs[7] << 32) | ((__u64)msg_data.msgs[6]);

	__sync();

	msgrng_flags_save(mflags);

	if (message_send_retry(size, code, stid, &msg)) {
		printk("Failed to send message!\n");
		msgrng_flags_restore(mflags);
		return -EAGAIN;
	}

	msgrng_flags_restore(mflags);


	return msg_size;
}

static int msgring_mmap_syscall (struct file *file, struct vm_area_struct *vma)
{
        unsigned long addr = __pa(file->private_data);
        unsigned long size = vma->vm_end - vma->vm_start;

	if (vma->vm_flags & VM_LOCKED) return -EPERM;
	vma->vm_flags |= (VM_RESERVED | VM_IO);
	if (remap_pfn_range(vma, vma->vm_start, (addr >> PAGE_SHIFT), size, vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static int msgring_mmap (struct file *file, struct vm_area_struct *vma)
{
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long shm_addr = __pa(nlm_common_psb_shm);
	unsigned long shm_size = nlm_common_psb_shm_size;
	unsigned long size = 0;
	unsigned long vm_size = vma->vm_end - vma->vm_start;

	if (file->private_data != 0 &&
           ((char *)file->private_data < (char *)&msg_fifos[0] ||
            (char *)file->private_data > (char *)&msg_fifos[MAX_MSG_FIFOS]))
		return msgring_mmap_syscall(file, vma);

	if (vma->vm_start != (unsigned long)NLM_USER_MAC_MMAP_VIRT_START)
		return -EINVAL;

	if (!shm_addr) return -ENXIO;

	if (offset >= shm_size) return -ESPIPE;

	if (vma->vm_flags & VM_LOCKED) return -EPERM;

	size = shm_size - offset;
	if (vm_size > size) return -ENOSPC;

	vma->vm_flags |= (VM_RESERVED | VM_IO);

	if (remap_pfn_range(vma, vma->vm_start, (shm_addr >> PAGE_SHIFT), size, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}
static DEFINE_PER_CPU(spinlock_t, msgring_lock) = SPIN_LOCK_UNLOCKED;
static int cpu_spread[16];

static int send_syscall (SYSCALL *syscall, int arg)
{
	struct msgrng_msg msg;
	unsigned long mflags = 0;
	int cpu, dst_stid, src_stid;
	int ret;

	cpu = hard_smp_processor_id();
	src_stid = (((cpu >> 2) << 3) | (cpu & 0x3));
	if (smp_boot.online_map == 0xff)
		dst_stid = 16 + src_stid + (((cpu_spread[cpu]++ % 3) & 0x3) << 4);
	else if (smp_boot.online_map == 0xf) {
		dst_stid = 8 + src_stid;
	}
	else
		dst_stid = 32 + src_stid;
	msg.msg0 = (1ULL<<63) | ((uint64_t)src_stid<<40) |
		((uint64_t)arg<<32) | (uint32_t)(unsigned long)syscall;
	msg.msg1 = msg.msg2 = msg.msg3 = 0;
	spin_lock(&__get_cpu_var(msgring_lock));
	msgrng_flags_save(mflags);
	ret = message_send_retry(1, 0, dst_stid, &msg);
	msgrng_flags_restore(mflags);
	spin_unlock(&__get_cpu_var(msgring_lock));

	return ret;
}

static int msgring_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned long shm_vaddr = (unsigned long)nlm_common_psb_shm;
	unsigned long shm_paddr = __pa(nlm_common_psb_shm); // assume it is in kseg0, for now

	switch (cmd) {

	case MSGRING_IOC_SSTNNO: {
		int tx_stn = arg;
		if (tx_stn >= MAX_MSG_FIFOS) {
			printk("[%s]: illegal tx stn id=%d\n", __FUNCTION__, tx_stn);
			return -EINVAL;
		}
		filp->private_data = &msg_fifos[tx_stn];
		PRINTK_INFO("%s:%d registering tx_stn=%d fifo=%p\n", __FUNCTION__, __LINE__,
			    tx_stn, filp->private_data);
		msg_fifos[tx_stn].head = msg_fifos[tx_stn].tail = 0;
	}
		break;

	case MSGRING_IOC_GSHMPHYS: {
		*(unsigned int *)arg = shm_paddr;
	}
		break;

	case MSGRING_IOC_GSHMVIRT: {
		*(unsigned int *)arg = shm_vaddr;
	}
		break;

	case MSGRING_IOC_GMMAP_START:{
			*(unsigned int *)arg =
			    (unsigned int)NLM_USER_MAC_MMAP_VIRT_START;
	}
		break;

	case MSGRING_IOC_SYSINIT: {
		/* allocate the socket interfacxe */
		SYSCALL *syscall;

		/* allocate the syscall interface */
		syscall = kmalloc(arg, GFP_KERNEL);
		if (!syscall) {
			printk("syscall: no memory\n");
			return -ENOMEM;
		}
		init_waitqueue_head(&syscall->sleep);
                syscall->process = 0;
		syscall->pid = current->pid;
		filp->private_data = syscall;
	}
		break;

	case MSGRING_IOC_SYSPHYS:
		*(unsigned int *)arg = (unsigned int)(unsigned long)filp->private_data;
		break;

	case MSGRING_IOC_SYSCALL: {
		SYSCALL *syscall;
		DEFINE_WAIT(wait);
		int not_sent;

		/* send a system call */
		syscall = filp->private_data;
		syscall->done = 0;
		syscall->num++;
		syscall->trace = 0;
		not_sent = send_syscall(syscall, arg);

		/* wait for the answer */
		if (syscall->done) {
			if (syscall->id == 10) /*SYSCALL_memcpy */
  		          return 100;
			/* the system call was acknowledged */
			break;
		}

		do {
			prepare_to_wait(&syscall->sleep, &wait, TASK_INTERRUPTIBLE);
			schedule_timeout(1);
			if (not_sent)
				/* we have not yet sent the system call */
				not_sent = send_syscall(syscall, arg);
		} while (!signal_pending(current) && !syscall->done);
		finish_wait(&syscall->sleep, &wait);
		if (syscall->done) {
			if (syscall->id == 10) /*SYSCALL_memcpy */
  		          return 100;

			/* the system call was acknowledged, stop, because rmios will not
			 * reply to an interrupt when it is idle */
			break;
		}
		if (not_sent)
			/* the system call was not sent and the process caught a signal,
			 * stop, because rmios will not reply to an interrupt when it is
			 * idle */
			return -EINTR;
		not_sent = send_syscall(syscall, SYSCALL_interrupt);
		if (syscall->done)
			/* the interrupt was acknowledged wither by a system call response
			 * or by an interrupt response */
			return -EINTR;
		do {
			prepare_to_wait(&syscall->sleep, &wait, TASK_UNINTERRUPTIBLE);
			schedule_timeout(1);
			if (not_sent)
				/* we have not yet sent the interrupt */
				not_sent = send_syscall(syscall, SYSCALL_interrupt);
		} while (!syscall->done);
		finish_wait(&syscall->sleep, &wait);
		/* the interrupt was acknowledged wither by a system call response
		 * or by an interrupt response */
		return -EINTR;
	}
		break;

	default: {
		printk("ioctl(): invalid command\n");
		return -EINVAL;
	}

	}

	//printk("[%s:%d]: \n", __FUNCTION__, __LINE__);

	return 0;
}

static long msgring_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned long ret = -1;
	lock_kernel();
	ret = msgring_ioctl(NULL,filp,cmd,arg);
	unlock_kernel();
	if(ret){
		printk("msgring_ioctl returned with an error %lx", ret);
		return -ENOIOCTLCMD;
	}
	return ret;
}

static void nlm_nlm_common_syscall_msgring_handler(int bucket, int size, int code, int stid,
                                      struct msgrng_msg *msg, void *data/* ignored */) {

	SYSCALL *syscall;
	syscall = (SYSCALL *)(unsigned long)msg->msg0;
	//printk("Wake up syscall %p sleep %p\n",syscall,&syscall->sleep);
	if (msg->msg0 & (1ULL<<62))
		syscall->done = 2;
	else
		syscall->done = 1;
	wake_up_interruptible(&syscall->sleep);
}

// called only when the reference count (maintained in inode) is zero
static int msgring_release (struct inode *inode, struct file *filp)
{
	if (filp->private_data != 0 &&
           ((char *)filp->private_data < (char *)&msg_fifos[0] ||
            (char *)filp->private_data > (char *)&msg_fifos[MAX_MSG_FIFOS])) {
		SYSCALL *syscall;
		syscall = filp->private_data;
		if (syscall->process) {
			int sig = test_tsk_thread_flag(current, TIF_SIGPENDING);
			if (sig)
				clear_tsk_thread_flag(current, TIF_SIGPENDING);
			msgring_ioctl(inode, filp, MSGRING_IOC_SYSCALL, SYSCALL_exit);
			if (sig)
				set_tsk_thread_flag(current, TIF_SIGPENDING);
		}
		kfree(filp->private_data);
		filp->private_data = 0;
	}
	return 0;
}

static struct file_operations msgring_fops = {
	owner:		THIS_MODULE,
	read:		msgring_read,
	write:		msgring_write,
	mmap:		msgring_mmap,
	open:		msgring_open,
	ioctl:		msgring_ioctl,
	poll:		msgring_poll,
	release:	msgring_release,
	compat_ioctl:	msgring_compat_ioctl,
};

// msgring_init(): invoked as part of the kernel bootup process
static int msgring_init(void)
{
	int err;
	int i=0;

	/* if support for loading apps on same core as Linux is enabled */
	if(!xlr_hybrid_rmios_ipsec())
		return -EINVAL;

	hybrid = strstr(saved_command_line, "hybrid=");

	// Intitialize 8 FIFO queues for each of the 8 cpu stations
	for (i = 0; i < MAX_MSG_FIFOS; ++i) {

		fifo_init(&msg_fifos[i], MSG_FIFO_SIZE);
		init_waitqueue_head(&msg_fifos[i].wq);

		if (i > 7) continue;

		err = register_msgring_handler(TX_STN_CPU_0 + i, nlm_common_msgring_drv_int_handler, NULL);
		if (err) {
			// should we panic or just return an error message
			panic("In %s at line %d: unable to register handler for msgring stations for stn %d\n",
			      __FILE__, __LINE__, i);
		}
	}

	msgring_major = register_chrdev (XLR_MSGRING_SHM_MAJOR, NLM_MSGRING_CHRDEV_NAME, &msgring_fops);
	if (msgring_major < 0) {
		printk("msgring_init() register_chrdev() failed\n");
		return msgring_major;
	}
	msgring_major = XLR_MSGRING_SHM_MAJOR;
	printk("Registered phoenix msgring driver: major=%d\n", msgring_major);

	return 0;
}

static void msgring_exit(void)
{
	unregister_chrdev (msgring_major, NLM_MSGRING_CHRDEV_NAME);
}

module_init (msgring_init);
module_exit (msgring_exit);

// Do we need to export any names ?
