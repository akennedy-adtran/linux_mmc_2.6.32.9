EXTRA_CFLAGS := -Werror
EXTRA_CFLAGS := $(CFLAGS) -DNLM_HAL_LINUX_KERNEL -Iarch/mips/include/asm/netlogic/hal

obj-$(CONFIG_XLP_PIC_TIMER)		+= timer-base.o
obj-$(CONFIG_XLP_REPLACE_R4K_TIMER)	+= pic-timer.o
obj-$(CONFIG_XLP_PIC_TIMER_GENERIC)	+= timer-wd.o
obj-y					+= ite.o
