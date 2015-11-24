/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */
/*
 * linux/fs/binfmt_elf.c
 *
 * These are the functions used to load ELF format executables as used
 * on SVr4 machines.  Information on the format may be found in the book
 * "UNIX SYSTEM V RELEASE 4 Programmers Guide: Ansi C and Programming Support
 * Tools".
 *
 * Copyright 1993, 1994: Eric Youngdale (ericy@cais.com).
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/stat.h>
#include <linux/time.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/a.out.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/binfmts.h>
#include <linux/string.h>
#include <linux/file.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/shm.h>
#include <linux/personality.h>
#include <linux/elfcore.h>
#include <linux/init.h>
#include <linux/highuid.h>
#include <linux/smp.h>
#include <linux/smp_lock.h>
#include <linux/compiler.h>
#include <linux/highmem.h>
#include <linux/pagemap.h>
#include <linux/security.h>
#include <linux/syscalls.h>
#include <linux/random.h>
#include <linux/elf.h>
#include <asm/uaccess.h>
#include <asm/param.h>
#include <asm/page.h>


#ifndef ELF_EXEC_PAGESIZE

#if ELF_EXEC_PAGESIZE > PAGE_SIZE
#define ELF_MIN_ALIGN	ELF_EXEC_PAGESIZE
#else
#define ELF_MIN_ALIGN	PAGE_SIZE
#endif

#endif

#ifndef ELF_CORE_EFLAGS
#define ELF_CORE_EFLAGS	0
#endif

/*
 * Note that some platforms still use traditional core dumps and not
 * the ELF core dump.  Each platform can select it as appropriate.
 */
#if defined(USE_ELF_CORE_DUMP) && defined(CONFIG_ELF_CORE)

/*
 * ELF core dumper
 *
 * Modelled on fs/exec.c:aout_core_dump()
 * Jeremy Fitzhardinge <jeremy@sw.oz.au>
 */
/*
 * These are the only things you should do on a core-file: use only these
 * functions to write out all the necessary info.
 */
static int dump_write(struct file *file, const void *addr, int nr)
{
	return file->f_op->write(file, addr, nr, &file->f_pos) == nr;
}

static int dump_seek(struct file *file, loff_t off)
{
	if (file->f_op->llseek && file->f_op->llseek != no_llseek) {
		if (file->f_op->llseek(file, off, SEEK_CUR) < 0)
			return 0;
	} else {
		char *buf = (char *)get_zeroed_page(GFP_KERNEL);
		if (!buf)
			return 0;
		while (off > 0) {
			unsigned long n = off;
			if (n > PAGE_SIZE)
				n = PAGE_SIZE;
			if (!dump_write(file, buf, n))
				return 0;
			off -= n;
		}
		free_page((unsigned long)buf);
	}
	return 1;
}

/*
 * Decide whether a segment is worth dumping; default is yes to be
 * sure (missing info is worse than too much; etc).
 * Personally I'd include everything, and use the coredump limit...
 *
 * I think we should skip something. But I am not sure how. H.J.
 */
static int maydump(struct vm_area_struct *vma)
{
	/* The vma can be set up to tell us the answer directly.  */
	if (vma->vm_flags & VM_ALWAYSDUMP)
		return 1;

	/* Do not dump I/O mapped devices or special mappings */
	if (vma->vm_flags & (VM_IO | VM_RESERVED))
		return 0;

	/* Dump shared memory only if mapped from an anonymous file. */
	if (vma->vm_flags & VM_SHARED)
		return vma->vm_file->f_path.dentry->d_inode->i_nlink == 0;

	/* If it hasn't been written to, don't write it out */
	if (!vma->anon_vma)
		return 0;

	return 1;
}

/* An ELF note in memory */
struct memelfnote
{
	const char *name;
	int type;
	unsigned int datasz;
	void *data;
};

static int notesize(struct memelfnote *en)
{
	int sz;

	sz = sizeof(struct elf_note);
	sz += roundup(strlen(en->name) + 1, 4);
	sz += roundup(en->datasz, 4);

	return sz;
}

#define DUMP_WRITE(addr, nr, foffset)	\
	do { if (!dump_write(file, (addr), (nr))) return 0; *foffset += (nr); } while(0)

static int alignfile(struct file *file, loff_t *foffset)
{
	static const char buf[4] = { 0, };
	DUMP_WRITE(buf, roundup(*foffset, 4) - *foffset, foffset);
	return 1;
}

static int writenote(struct memelfnote *men, struct file *file,
			loff_t *foffset)
{
	struct elf_note en;
	en.n_namesz = strlen(men->name) + 1;
	en.n_descsz = men->datasz;
	en.n_type = men->type;

	DUMP_WRITE(&en, sizeof(en), foffset);
	DUMP_WRITE(men->name, en.n_namesz, foffset);
	if (!alignfile(file, foffset))
		return 0;
	DUMP_WRITE(men->data, men->datasz, foffset);
	if (!alignfile(file, foffset))
		return 0;

	return 1;
}
#undef DUMP_WRITE

#define DUMP_WRITE(addr, nr)	\
	if ((size += (nr)) > limit || !dump_write(file, (addr), (nr))) \
		goto end_coredump;
#define DUMP_SEEK(off)	\
	if (!dump_seek(file, (off))) \
		goto end_coredump;

static void fill_elf_header(struct elfhdr *elf, int segs)
{
	memcpy(elf->e_ident, ELFMAG, SELFMAG);
	elf->e_ident[EI_CLASS] = ELF_CLASS;
	elf->e_ident[EI_DATA] = ELF_DATA;
	elf->e_ident[EI_VERSION] = EV_CURRENT;
	elf->e_ident[EI_OSABI] = ELF_OSABI;
	memset(elf->e_ident+EI_PAD, 0, EI_NIDENT-EI_PAD);

	elf->e_type = ET_CORE;
	elf->e_machine = ELF_ARCH;
	elf->e_version = EV_CURRENT;
	elf->e_entry = 0;
	elf->e_phoff = sizeof(struct elfhdr);
	elf->e_shoff = 0;
	elf->e_flags = ELF_CORE_EFLAGS;
	elf->e_ehsize = sizeof(struct elfhdr);
	elf->e_phentsize = sizeof(struct elf_phdr);
	elf->e_phnum = segs;
	elf->e_shentsize = 0;
	elf->e_shnum = 0;
	elf->e_shstrndx = 0;
	return;
}

static void fill_elf_note_phdr(struct elf_phdr *phdr, int sz, loff_t offset)
{
	phdr->p_type = PT_NOTE;
	phdr->p_offset = offset;
	phdr->p_vaddr = 0;
	phdr->p_paddr = 0;
	phdr->p_filesz = sz;
	phdr->p_memsz = 0;
	phdr->p_flags = 0;
	phdr->p_align = 0;
	return;
}

static void fill_note(struct memelfnote *note, const char *name, int type, 
		unsigned int sz, void *data)
{
	note->name = name;
	note->type = type;
	note->datasz = sz;
	note->data = data;
	return;
}

/*
 * fill up all the fields in prstatus from the given task struct, except
 * registers which need to be filled up separately.
 */
static void fill_prstatus(struct elf_prstatus *prstatus,
		struct task_struct *p, long signr)
{
	prstatus->pr_info.si_signo = prstatus->pr_cursig = signr;
	prstatus->pr_sigpend = p->pending.signal.sig[0];
	prstatus->pr_sighold = p->blocked.sig[0];
	prstatus->pr_pid = p->pid;
	prstatus->pr_ppid = p->parent->pid;
	prstatus->pr_pgrp = task_pgrp_nr(p);
	prstatus->pr_sid = task_session_nr(p);
	if (thread_group_leader(p)) {
		/*
		 * This is the record for the group leader.  Add in the
		 * cumulative times of previous dead threads.  This total
		 * won't include the time of each live thread whose state
		 * is included in the core dump.  The final total reported
		 * to our parent process when it calls wait4 will include
		 * those sums as well as the little bit more time it takes
		 * this and each other thread to finish dying after the
		 * core dump synchronization phase.
		 */
		cputime_to_timeval(cputime_add(p->utime, p->signal->utime),
				   &prstatus->pr_utime);
		cputime_to_timeval(cputime_add(p->stime, p->signal->stime),
				   &prstatus->pr_stime);
	} else {
		cputime_to_timeval(p->utime, &prstatus->pr_utime);
		cputime_to_timeval(p->stime, &prstatus->pr_stime);
	}
	cputime_to_timeval(p->signal->cutime, &prstatus->pr_cutime);
	cputime_to_timeval(p->signal->cstime, &prstatus->pr_cstime);
}

static int fill_psinfo(struct elf_prpsinfo *psinfo, struct task_struct *p,
		       struct mm_struct *mm)
{
	unsigned int i, len;
	
	/* first copy the parameters from user space */
	memset(psinfo, 0, sizeof(struct elf_prpsinfo));

	len = mm->arg_end - mm->arg_start;
	if (len >= ELF_PRARGSZ)
		len = ELF_PRARGSZ-1;
	if (copy_from_user(&psinfo->pr_psargs,
		           (const char __user *)mm->arg_start, len))
		return -EFAULT;
	for(i = 0; i < len; i++)
		if (psinfo->pr_psargs[i] == 0)
			psinfo->pr_psargs[i] = ' ';
	psinfo->pr_psargs[len] = 0;

	psinfo->pr_pid = p->pid;
	psinfo->pr_ppid = p->parent->pid;
	psinfo->pr_pgrp = task_pgrp_nr(p);
	psinfo->pr_sid = task_session_nr(p);

	i = p->state ? ffz(~p->state) + 1 : 0;
	psinfo->pr_state = i;
	psinfo->pr_sname = (i > 5) ? '.' : "RSDTZW"[i];
	psinfo->pr_zomb = psinfo->pr_sname == 'Z';
	psinfo->pr_nice = task_nice(p);
	psinfo->pr_flag = p->flags;
	SET_UID(psinfo->pr_uid, p->uid);
	SET_GID(psinfo->pr_gid, p->gid);
	strncpy(psinfo->pr_fname, p->comm, sizeof(psinfo->pr_fname));
	
	return 0;
}

/* Here is the structure in which status of each thread is captured. */
struct elf_thread_status
{
	struct list_head list;
	struct elf_prstatus prstatus;	/* NT_PRSTATUS */
	elf_fpregset_t fpu;		/* NT_PRFPREG */
	struct task_struct *thread;
#ifdef ELF_CORE_COPY_XFPREGS
	elf_fpxregset_t xfpu;		/* NT_PRXFPREG */
#endif
	struct memelfnote notes[3];
	int num_notes;
};

/*
 * In order to add the specific thread information for the elf file format,
 * we need to keep a linked list of every threads pr_status and then create
 * a single section for them in the final core file.
 */
static int elf_dump_thread_status(long signr, struct elf_thread_status *t)
{
	int sz = 0;
	struct task_struct *p = t->thread;
	t->num_notes = 0;

	fill_prstatus(&t->prstatus, p, signr);
	elf_core_copy_task_regs(p, &t->prstatus.pr_reg);	
	
	fill_note(&t->notes[0], "CORE", NT_PRSTATUS, sizeof(t->prstatus),
		  &(t->prstatus));
	t->num_notes++;
	sz += notesize(&t->notes[0]);

	if ((t->prstatus.pr_fpvalid = elf_core_copy_task_fpregs(p, NULL,
								&t->fpu))) {
		fill_note(&t->notes[1], "CORE", NT_PRFPREG, sizeof(t->fpu),
			  &(t->fpu));
		t->num_notes++;
		sz += notesize(&t->notes[1]);
	}

#ifdef ELF_CORE_COPY_XFPREGS
	if (elf_core_copy_task_xfpregs(p, &t->xfpu)) {
		fill_note(&t->notes[2], "LINUX", NT_PRXFPREG, sizeof(t->xfpu),
			  &t->xfpu);
		t->num_notes++;
		sz += notesize(&t->notes[2]);
	}
#endif	
	return sz;
}

static struct vm_area_struct *first_vma(struct task_struct *tsk,
					struct vm_area_struct *gate_vma)
{
	struct vm_area_struct *ret = tsk->mm->mmap;

	if (ret)
		return ret;
	return gate_vma;
}
/*
 * Helper function for iterating across a vma list.  It ensures that the caller
 * will visit `gate_vma' prior to terminating the search.
 */
static struct vm_area_struct *next_vma(struct vm_area_struct *this_vma,
					struct vm_area_struct *gate_vma)
{
	struct vm_area_struct *ret;

	ret = this_vma->vm_next;
	if (ret)
		return ret;
	if (this_vma == gate_vma)
		return NULL;
	return gate_vma;
}

/*
 * Actual dumper
 *
 * This is a two-pass process; first we find the offsets of the bits,
 * and then they are actually written out.  If we run out of core limit
 * we just truncate.
 */
int btlb_elf_core_dump(long signr, struct pt_regs *regs, struct file *file)
{
#define	NUM_NOTES	6
	int has_dumped = 0;
	mm_segment_t fs;
	int segs;
	size_t size = 0;
	int i;
	struct vm_area_struct *vma, *gate_vma;
	struct elfhdr *elf = NULL;
	loff_t offset = 0, dataoff, foffset;
	unsigned long limit = current->signal->rlim[RLIMIT_CORE].rlim_cur;
	int numnote;
	struct memelfnote *notes = NULL;
	struct elf_prstatus *prstatus = NULL;	/* NT_PRSTATUS */
	struct elf_prpsinfo *psinfo = NULL;	/* NT_PRPSINFO */
 	struct task_struct *g, *p;
 	LIST_HEAD(thread_list);
 	struct list_head *t;
	elf_fpregset_t *fpu = NULL;
#ifdef ELF_CORE_COPY_XFPREGS
	elf_fpxregset_t *xfpu = NULL;
#endif
	int thread_status_size = 0;
	elf_addr_t *auxv;

	/*
	 * We no longer stop all VM operations.
	 * 
	 * This is because those proceses that could possibly change map_count
	 * or the mmap / vma pages are now blocked in do_exit on current
	 * finishing this core dump.
	 *
	 * Only ptrace can touch these memory addresses, but it doesn't change
	 * the map_count or the pages allocated. So no possibility of crashing
	 * exists while dumping the mm->vm_next areas to the core file.
	 */
  
	/* alloc memory for large data structures: too large to be on stack */
	elf = kmalloc(sizeof(*elf), GFP_KERNEL);
	if (!elf)
		goto cleanup;
	prstatus = kmalloc(sizeof(*prstatus), GFP_KERNEL);
	if (!prstatus)
		goto cleanup;
	psinfo = kmalloc(sizeof(*psinfo), GFP_KERNEL);
	if (!psinfo)
		goto cleanup;
	notes = kmalloc(NUM_NOTES * sizeof(struct memelfnote), GFP_KERNEL);
	if (!notes)
		goto cleanup;
	fpu = kmalloc(sizeof(*fpu), GFP_KERNEL);
	if (!fpu)
		goto cleanup;
#ifdef ELF_CORE_COPY_XFPREGS
	xfpu = kmalloc(sizeof(*xfpu), GFP_KERNEL);
	if (!xfpu)
		goto cleanup;
#endif

	if (signr) {
		struct elf_thread_status *tmp;
		rcu_read_lock();
		do_each_thread(g,p)
			if (current->mm == p->mm && current != p) {
				tmp = kzalloc(sizeof(*tmp), GFP_ATOMIC);
				if (!tmp) {
					rcu_read_unlock();
					goto cleanup;
				}
				tmp->thread = p;
				list_add(&tmp->list, &thread_list);
			}
		while_each_thread(g,p);
		rcu_read_unlock();
		list_for_each(t, &thread_list) {
			struct elf_thread_status *tmp;
			int sz;

			tmp = list_entry(t, struct elf_thread_status, list);
			sz = elf_dump_thread_status(signr, tmp);
			thread_status_size += sz;
		}
	}
	/* now collect the dump for the current */
	memset(prstatus, 0, sizeof(*prstatus));
	fill_prstatus(prstatus, current, signr);
	elf_core_copy_regs(&prstatus->pr_reg, regs);
	
	segs = current->mm->map_count;
#ifdef ELF_CORE_EXTRA_PHDRS
	segs += ELF_CORE_EXTRA_PHDRS;
#endif

	gate_vma = get_gate_vma(current);
	if (gate_vma != NULL)
		segs++;

	/* Set up header */
	fill_elf_header(elf, segs + 1);	/* including notes section */

	has_dumped = 1;
	current->flags |= PF_DUMPCORE;

	/*
	 * Set up the notes in similar form to SVR4 core dumps made
	 * with info from their /proc.
	 */

	fill_note(notes + 0, "CORE", NT_PRSTATUS, sizeof(*prstatus), prstatus);
	fill_psinfo(psinfo, current->group_leader, current->mm);
	fill_note(notes + 1, "CORE", NT_PRPSINFO, sizeof(*psinfo), psinfo);
	
	numnote = 2;

	auxv = (elf_addr_t *)current->mm->saved_auxv;

	i = 0;
	do
		i += 2;
	while (auxv[i - 2] != AT_NULL);
	fill_note(&notes[numnote++], "CORE", NT_AUXV,
		  i * sizeof(elf_addr_t), auxv);

  	/* Try to dump the FPU. */
	if ((prstatus->pr_fpvalid =
	     elf_core_copy_task_fpregs(current, regs, fpu)))
		fill_note(notes + numnote++,
			  "CORE", NT_PRFPREG, sizeof(*fpu), fpu);
#ifdef ELF_CORE_COPY_XFPREGS
	if (elf_core_copy_task_xfpregs(current, xfpu))
		fill_note(notes + numnote++,
			  "LINUX", NT_PRXFPREG, sizeof(*xfpu), xfpu);
#endif	
  
	fs = get_fs();
	set_fs(KERNEL_DS);

	DUMP_WRITE(elf, sizeof(*elf));
	offset += sizeof(*elf);				/* Elf header */
	offset += (segs + 1) * sizeof(struct elf_phdr); /* Program headers */
	foffset = offset;

	/* Write notes phdr entry */
	{
		struct elf_phdr phdr;
		int sz = 0;

		for (i = 0; i < numnote; i++)
			sz += notesize(notes + i);
		
		sz += thread_status_size;

#ifdef ELF_CORE_WRITE_EXTRA_NOTES
		sz += ELF_CORE_EXTRA_NOTES_SIZE;
#endif

		fill_elf_note_phdr(&phdr, sz, offset);
		offset += sz;
		DUMP_WRITE(&phdr, sizeof(phdr));
	}

	dataoff = offset = roundup(offset, ELF_EXEC_PAGESIZE);

	/* Write program headers for segments dump */
	for (vma = first_vma(current, gate_vma); vma != NULL;
			vma = next_vma(vma, gate_vma)) {
		struct elf_phdr phdr;
		size_t sz;

		sz = vma->vm_end - vma->vm_start;

		phdr.p_type = PT_LOAD;
		phdr.p_offset = offset;
		phdr.p_vaddr = vma->vm_start;
		phdr.p_paddr = 0;
		phdr.p_filesz = maydump(vma) ? sz : 0;
		phdr.p_memsz = sz;
		offset += phdr.p_filesz;
		phdr.p_flags = vma->vm_flags & VM_READ ? PF_R : 0;
		if (vma->vm_flags & VM_WRITE)
			phdr.p_flags |= PF_W;
		if (vma->vm_flags & VM_EXEC)
			phdr.p_flags |= PF_X;
		phdr.p_align = ELF_EXEC_PAGESIZE;

		DUMP_WRITE(&phdr, sizeof(phdr));
	}

#ifdef ELF_CORE_WRITE_EXTRA_PHDRS
	ELF_CORE_WRITE_EXTRA_PHDRS;
#endif

 	/* write out the notes section */
	for (i = 0; i < numnote; i++)
		if (!writenote(notes + i, file, &foffset))
			goto end_coredump;

#ifdef ELF_CORE_WRITE_EXTRA_NOTES
	ELF_CORE_WRITE_EXTRA_NOTES;
#endif

	/* write out the thread status notes section */
	list_for_each(t, &thread_list) {
		struct elf_thread_status *tmp =
				list_entry(t, struct elf_thread_status, list);

		for (i = 0; i < tmp->num_notes; i++)
			if (!writenote(&tmp->notes[i], file, &foffset))
				goto end_coredump;
	}

	/* Align to page */
	DUMP_SEEK(dataoff - foffset);

	for (vma = first_vma(current, gate_vma); vma != NULL;
			vma = next_vma(vma, gate_vma)) {
		unsigned long addr;

		if (!maydump(vma))
			continue;
		{
		for (addr = vma->vm_start;
		     addr < vma->vm_end;
		     addr += PAGE_SIZE) {
			struct page *page;
			struct vm_area_struct *vma;

			if (get_user_pages(current, current->mm, addr, 1, 0, 1,
						&page, &vma) <= 0) {
				DUMP_SEEK(PAGE_SIZE);
			} else {
				if (page == ZERO_PAGE(addr)) {
					if (!dump_seek(file, PAGE_SIZE)) {
						page_cache_release(page);
						goto end_coredump;
					}
				} else {
					void *kaddr;
					flush_cache_page(vma, addr,
							 page_to_pfn(page));
					kaddr = kmap(page);
					if ((size += PAGE_SIZE) > limit ||
					    !dump_write(file, kaddr,
					    PAGE_SIZE)) {
						kunmap(page);
						page_cache_release(page);
						goto end_coredump;
					}
					kunmap(page);
				}
				page_cache_release(page);
			}
		}
		}
	}

#ifdef ELF_CORE_WRITE_EXTRA_DATA
	ELF_CORE_WRITE_EXTRA_DATA;
#endif

end_coredump:
	set_fs(fs);

cleanup:
	while (!list_empty(&thread_list)) {
		struct list_head *tmp = thread_list.next;
		list_del(tmp);
		kfree(list_entry(tmp, struct elf_thread_status, list));
	}

	kfree(elf);
	kfree(prstatus);
	kfree(psinfo);
	kfree(notes);
	kfree(fpu);
#ifdef ELF_CORE_COPY_XFPREGS
	kfree(xfpu);
#endif
	return has_dumped;
#undef NUM_NOTES
}

#endif		/* USE_ELF_CORE_DUMP */
