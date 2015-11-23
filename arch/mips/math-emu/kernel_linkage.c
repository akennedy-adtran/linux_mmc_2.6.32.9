/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */
/*
 *  Kevin D. Kissell, kevink@mips and Carsten Langgaard, carstenl@mips.com
 *  Copyright (C) 2000 MIPS Technologies, Inc.  All rights reserved.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * Routines corresponding to Linux kernel FP context
 * manipulation primitives for the Algorithmics MIPS
 * FPU Emulator
 */
#include <linux/sched.h>
#include <asm/processor.h>
#include <asm/signal.h>
#include <asm/uaccess.h>

#include <asm/fpu.h>
#include <asm/fpu_emulator.h>

#define SIGNALLING_NAN 0x7ff800007ff80000LL

#ifdef CONFIG_PROFILE_MATHEMU

#include <asm/inst.h>
#include <linux/proc_fs.h>

#define FPUEMU_PROC_ENT "fpuemu"
static struct proc_dir_entry *proc_ent;

/*
 * func field of cop1 instructions using d, s or w format.
 */
static char *cop1_sdw_func_names[] = {
  "fadd_op",     /* 0x00 */
  "fsub_op",     /* 0x01 */
  "fmul_op",     /* 0x02 */
  "fdiv_op",     /* 0x03 */
  "fsqrt_op",    /* 0x04 */
  "fabs_op",     /* 0x05 */
  "fmov_op",     /* 0x06 */
  "fneg_op",     /* 0x07 */
  "froundl_op",  /* 0x08 */
  "ftruncl_op",  /* 0x09 */
  "fceill_op",   /* 0x0a */
  "ffloorl_op",  /* 0x0b */
  "fround_op",   /* 0x0c */
  "ftrunc_op",   /* 0x0d */
  "fceil_op",    /* 0x0e */
  "ffloor_op",   /* 0x0f */
  NULL,          /* 0x10 */
  "fmovc_op",    /* 0x11 */
  "fmovz_op",    /* 0x12 */
  "fmovn_op",    /* 0x13 */
  NULL,          /* 0x14 */
  "frecip_op",   /* 0x15 */
  "frsqrt_op",   /* 0x16 */
  NULL,          /* 0x17 */
  NULL,          /* 0x18 */
  NULL,          /* 0x19 */
  NULL,          /* 0x1a */
  NULL,          /* 0x1b */
  NULL,          /* 0x1c */
  NULL,          /* 0x1d */
  NULL,          /* 0x1e */
  NULL,          /* 0x1f */
  "fcvts_op",    /* 0x20 */
  "fcvtd_op",    /* 0x21 */
  "fcvte_op",    /* 0x22 */
  NULL,          /* 0x23 */
  "fcvtw_op",    /* 0x24 */
  "fcvtl_op",    /* 0x25 */
  NULL,          /* 0x26 */
  NULL,          /* 0x27 */
  NULL,          /* 0x28 */
  NULL,          /* 0x29 */
  NULL,          /* 0x2a */
  NULL,          /* 0x2b */
  NULL,          /* 0x2c */
  NULL,          /* 0x2d */
  NULL,          /* 0x2e */
  NULL,          /* 0x2f */
  "fcmp_op"      /* 0x30 */
};

static int proc_read(char *buf , char **start, off_t offset,
                        int len, int *eof, void *data)
{
  char *p = buf;

  //printk("proc_read entered %p %p %d %d %p %p\n", buf, start, offset, len, eof, data);  
  p += sprintf(p, "%s\n", "FPUEMU Statistics:");
  p += sprintf(p, "emulated: %d\n", fpuemuprivate.stats.emulated);
  p += sprintf(p, "loads: %d\n", fpuemuprivate.stats.loads);
  p += sprintf(p, "stores: %d\n", fpuemuprivate.stats.stores);
  p += sprintf(p, "cp1ops: %d\n", fpuemuprivate.stats.cp1ops);
  p += sprintf(p, "cp1xops: %d\n", fpuemuprivate.stats.cp1xops);
  p += sprintf(p, "errors: %d\n", fpuemuprivate.stats.errors);

  p += sprintf(p, "format totals:\n");
  p += sprintf(p, "\ts_fmt: %d\n", fpuemuprivate.stats.s_format.total);
  if (fpuemuprivate.stats.s_format.total > 0) {
    int i, j;
    p += sprintf(p, "\tfunctions:");
    for (i = 0; i < 8; i++) {
      p += sprintf(p, "\n\t");
      for (j = 0; j < 8; j++) {
	p += sprintf(p,"%d ",
		     fpuemuprivate.stats.s_format.ops[i*8 + j]);
      }
    }
    p += sprintf(p, "\n");
  }
  p += sprintf(p, "\td_fmt: %d\n", fpuemuprivate.stats.d_format.total);
  if (fpuemuprivate.stats.d_format.total > 0) {
    int i, j;
    p += sprintf(p, "\tfunctions:");
    for (i = 0; i < 8; i++) {
      p += sprintf(p, "\n\t");
      for (j = 0; j < 8; j++) {
	p += sprintf(p,"%d ",
		     fpuemuprivate.stats.d_format.ops[i*8 + j]);
      }
    }
    p += sprintf(p, "\n");
  }
  p += sprintf(p, "\tw_fmt: %d\n", fpuemuprivate.stats.w_format.total);
  if (fpuemuprivate.stats.w_format.total > 0) {
    int i, j;
    p += sprintf(p, "\tfunctions:");
    for (i = 0; i < 8; i++) {
      p += sprintf(p, "\n\t");
      for (j = 0; j < 8; j++) {
	p += sprintf(p,"%d ",
		     fpuemuprivate.stats.w_format.ops[i*8 + j]);
      }
    }
    p += sprintf(p, "\n");
  }
  p += sprintf(p, "\tl_fmt: %d\n", fpuemuprivate.stats.l_format.total);
  if (fpuemuprivate.stats.l_format.total > 0) {
    int i, j;
    p += sprintf(p, "\tfunctions:");
    for (i = 0; i < 8; i++) {
      p += sprintf(p, "\n\t");
      for (j = 0; j < 8; j++) {
	p += sprintf(p,"%d ",
		     fpuemuprivate.stats.l_format.ops[i*8 + j]);
      }
    }
    p += sprintf(p, "\n");
  }
  
  if (fpuemuprivate.stats.s_format.total > 0) {
    int i, fcmp_total;
    p += sprintf(p, "\nSingle precision:\ttotal %d\n",
		 fpuemuprivate.stats.s_format.total);
    for (i = fadd_op; i < fcmp_op; i++)
      if (fpuemuprivate.stats.s_format.ops[i] > 0)
	p += sprintf(p, "\t%s:\t%d\n", cop1_sdw_func_names[i],
		     fpuemuprivate.stats.s_format.ops[i]);
    for (i = fcmp_op, fcmp_total = 0; i < 0x40; i++)
      fcmp_total += fpuemuprivate.stats.s_format.ops[i];
    if (fcmp_total > 0)
      p += sprintf(p, "\t%s:\t%d (all compare ops combined)\n",
		   cop1_sdw_func_names[fcmp_op], fcmp_total);
    p += sprintf(p, "\n");
  }

  if (fpuemuprivate.stats.d_format.total > 0) {
    int i, fcmp_total;
    p += sprintf(p, "\nDouble precision:\ttotal %d\n",
		 fpuemuprivate.stats.d_format.total);
    for (i = fadd_op; i < fcmp_op; i++)
      if (fpuemuprivate.stats.d_format.ops[i] > 0)
	p += sprintf(p, "\t%s:\t%d\n", cop1_sdw_func_names[i],
		     fpuemuprivate.stats.d_format.ops[i]);
    for (i = fcmp_op, fcmp_total = 0; i < 0x40; i++)
      fcmp_total += fpuemuprivate.stats.d_format.ops[i];
    if (fcmp_total > 0)
      p += sprintf(p, "\t%s:\t%d (all compare ops combined)\n",
		   cop1_sdw_func_names[fcmp_op], fcmp_total);
    p += sprintf(p, "\n");
  }

  if (fpuemuprivate.stats.w_format.total > 0) {
    int i, fcmp_total;
    p += sprintf(p, "\nw format:\ttotal %d\n",
		 fpuemuprivate.stats.w_format.total);
    for (i = fadd_op; i < fcmp_op; i++)
      if (fpuemuprivate.stats.w_format.ops[i] > 0)
	p += sprintf(p, "\t%s:\t%d\n", cop1_sdw_func_names[i],
		     fpuemuprivate.stats.w_format.ops[i]);
    for (i = fcmp_op, fcmp_total = 0; i < 0x40; i++)
      fcmp_total += fpuemuprivate.stats.w_format.ops[i];
    if (fcmp_total > 0)
      p += sprintf(p, "\t%s:\t%d (all compare ops combined)\n",
		   cop1_sdw_func_names[fcmp_op], fcmp_total);
    p += sprintf(p, "\n");
  }

  *eof = 1;
  return p - buf;
}

static int proc_write(struct file *file, const char *user_buffer,
                         unsigned long count, void *data)
{
  printk("FPUEMU: clearing stats\n");
  memset(&fpuemuprivate.stats, '\0', sizeof(fpuemuprivate.stats));
  return count;
}
#endif

void fpu_emulator_init_fpu(void)
{
	static int first = 1;
	int i;

	if (first) {
		first = 0;
		printk("Algorithmics/MIPS FPU Emulator v1.5\n");
#ifdef CONFIG_PROFILE_MATHEMU
		proc_ent = create_proc_entry(FPUEMU_PROC_ENT, S_IWUSR | S_IRUGO,
					     &proc_root);
		if (proc_ent) {
		  proc_ent->read_proc = proc_read;
		  proc_ent->write_proc = proc_write;
		}
#endif
	}

	current->thread.fpu.fcr31 = 0;
	for (i = 0; i < 32; i++) {
		current->thread.fpu.fpr[i] = SIGNALLING_NAN;
	}
}


/*
 * Emulator context save/restore to/from a signal context
 * presumed to be on the user stack, and therefore accessed
 * with appropriate macros from uaccess.h
 */

int fpu_emulator_save_context(struct sigcontext __user *sc)
{
	int i;
	int err = 0;

	for (i = 0; i < 32; i++) {
		err |=
		    __put_user(current->thread.fpu.fpr[i], &sc->sc_fpregs[i]);
	}
	err |= __put_user(current->thread.fpu.fcr31, &sc->sc_fpc_csr);

	return err;
}

int fpu_emulator_restore_context(struct sigcontext __user *sc)
{
	int i;
	int err = 0;

	for (i = 0; i < 32; i++) {
		err |=
		    __get_user(current->thread.fpu.fpr[i], &sc->sc_fpregs[i]);
	}
	err |= __get_user(current->thread.fpu.fcr31, &sc->sc_fpc_csr);

	return err;
}

#ifdef CONFIG_64BIT
/*
 * This is the o32 version
 */

int fpu_emulator_save_context32(struct sigcontext32 __user *sc)
{
	int i;
	int err = 0;

	for (i = 0; i < 32; i+=2) {
		err |=
		    __put_user(current->thread.fpu.fpr[i], &sc->sc_fpregs[i]);
	}
	err |= __put_user(current->thread.fpu.fcr31, &sc->sc_fpc_csr);

	return err;
}

int fpu_emulator_restore_context32(struct sigcontext32 __user *sc)
{
	int i;
	int err = 0;

	for (i = 0; i < 32; i+=2) {
		err |=
		    __get_user(current->thread.fpu.fpr[i], &sc->sc_fpregs[i]);
	}
	err |= __get_user(current->thread.fpu.fcr31, &sc->sc_fpc_csr);

	return err;
}
#endif
