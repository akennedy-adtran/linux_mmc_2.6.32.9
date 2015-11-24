#EXTRA_CFLAGS := -Werror
EXTRA_CFLAGS := $(CFLAGS)
ifneq ("$(HAL_DIR)", "")
	EXTRA_CFLAGS += -DNLM_HAL_LINUX_KERNEL -Iarch/mips/include/asm/netlogic/hal
endif

obj-y							= setup.o nmi.o
obj-y							+= irq.o time.o on_chip.o mmu.o
obj-y							+= platform.o board.o xlp_hal_pic.o
obj-$(CONFIG_NLM_XLP_EEPROM)	+= nlm_eeprom.o
obj-$(CONFIG_SMP)       		+= smp.o
obj-y							+= pic/
obj-y							+= cop2.o
obj-$(CONFIG_XLP_CPUFREQ)		+= xlp-cpufreq.o
obj-$(CONFIG_RAPIDIO)			+= xlp_srio.o
obj-$(CONFIG_XLP_GPIO)			+= xlp_gpio.o
obj-$(CONFIG_USB)				+= usb-init.o

ifeq ($(CONFIG_XEN),y)
	obj-y += xenbootinfo.o
endif

obj-$(CONFIG_NLM_XLP)			+= cpu_control.o cpu_control_asm.o
obj-$(CONFIG_NUMA)				+= numa.o
