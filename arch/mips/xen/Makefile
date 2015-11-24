ifdef CONFIG_FUNCTION_TRACER
# Do not profile debug and lowlevel utilities
CFLAGS_REMOVE_spinlock.o = -pg
CFLAGS_REMOVE_time.o = -pg
CFLAGS_REMOVE_irq.o = -pg
endif

ifeq ($(CONFIG_32BIT),y)
BITS := 32
else
BITS := 64
endif

# Make sure early boot has no stackprotector
nostackp := $(call cc-option, -fno-stack-protector)
CFLAGS_enlighten.o		:= $(nostackp)
CFLAGS_mmu.o			:= $(nostackp)

obj-y		                	:= enlighten.o grant-table.o xen-head.o

#obj-y					+= mmu.o time.o xen-asm.o
#obj-y					+= xen-asm_$(BITS).o setup.o
#obj-y					+= multicall.o irq.o suspend.o

#obj-$(CONFIG_SMP)			+= smp.o
#obj-$(CONFIG_PARAVIRT_SPINLOCKS)	+= spinlock.o
obj-$(CONFIG_XEN_DEBUG_FS)		+= debugfs.o

#obj-$(CONFIG_XEN_DOM0)		+= vga.o
