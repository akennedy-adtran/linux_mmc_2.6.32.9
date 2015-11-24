/*-
 * Copyright 2003-2012 Broadcom Corporation
 *
 * This is a derived work from software originally provided by the entity or
 * entities identified below. The licensing terms, warranty terms and other
 * terms specified in the header of the original work apply to this derived work
 *
 * #BRCM_1# */

#ifndef _ASM_MACH_NLM_PGTABLE_BITS_XLP_H
#define _ASM_MACH_NLM_PGTABLE_BITS_XLP_H

#define PAGE_NONE        __pgprot(_PAGE_PRESENT | \
                                  _PAGE_RI | _PAGE_XI | \
                                  _CACHE_CACHABLE_NONCOHERENT)
#define PAGE_READONLY    __pgprot(_PAGE_PRESENT | _PAGE_READ | \
                                  _PAGE_XI | \
                                   PAGE_CACHABLE_DEFAULT)
#define PAGE_WRITEONLY   __pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_WRITE | \
                                  _PAGE_RI | _PAGE_XI | \
                                  PAGE_CACHABLE_DEFAULT)
#define PAGE_WRITE_READ  __pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_WRITE | \
                                  _PAGE_XI | \
                                  PAGE_CACHABLE_DEFAULT)
#define PAGE_EXECONLY    __pgprot(_PAGE_PRESENT | _PAGE_READ | \
                                  _PAGE_RI | \
                                  PAGE_CACHABLE_DEFAULT)
#define PAGE_EXEC_READ   __pgprot(_PAGE_PRESENT | _PAGE_READ | \
                                  PAGE_CACHABLE_DEFAULT)
#define PAGE_EXEC_WRITE  __pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_WRITE | \
                                  _PAGE_RI | \
                                  PAGE_CACHABLE_DEFAULT)
#define PAGE_ALL         __pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_WRITE | \
                                  PAGE_CACHABLE_DEFAULT)
#define PAGE_COPY        __pgprot(_PAGE_PRESENT | _PAGE_READ | \
                                  _PAGE_RI | _PAGE_XI | \
                                   PAGE_CACHABLE_DEFAULT)

#define PAGE_COPY_READ      PAGE_READONLY
#define PAGE_EXEC_COPY      PAGE_EXECONLY
#define PAGE_EXEC_COPY_READ PAGE_EXEC_READ

/*
 * FIXME: What do we do with kernel pages ? These are primarily 
 *        used for modules.
 */
#define PAGE_KERNEL	__pgprot(_PAGE_PRESENT | __READABLE | __WRITEABLE | \
			_PAGE_GLOBAL | PAGE_CACHABLE_DEFAULT)

#define __P000	PAGE_NONE
#define __P001	PAGE_READONLY
#define __P010	PAGE_COPY
#define __P011	PAGE_COPY_READ
#define __P100	PAGE_EXECONLY
#define __P101	PAGE_EXEC_READ
#define __P110	PAGE_EXEC_COPY
#define __P111	PAGE_EXEC_COPY_READ

#define __S000	PAGE_NONE
#define __S001	PAGE_READONLY
#define __S010	PAGE_WRITEONLY
#define __S011	PAGE_WRITE_READ
#define __S100	PAGE_EXECONLY
#define __S101	PAGE_EXEC_READ
#define __S110	PAGE_EXEC_WRITE
#define __S111	PAGE_ALL

#endif
