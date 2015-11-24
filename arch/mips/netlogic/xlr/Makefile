EXTRA_CFLAGS := -Werror
EXTRA_CFLAGS := $(CFLAGS) -DNLM_HAL_LINUX_KERNEL -Iarch/mips/include/asm/netlogic/hal


obj-y                   = setup.o config_net.o board.o
obj-y += irq.o time.o on_chip.o mmu.o
obj-$(CONFIG_NLM_XLR) += platform.o
obj-$(CONFIG_SMP)      += smp.o smpboot.o
obj-$(CONFIG_KGDB)      += nmi.o

