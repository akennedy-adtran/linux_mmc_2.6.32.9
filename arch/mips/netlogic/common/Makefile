#EXTRA_CFLAGS := -Werror
EXTRA_CFLAGS := $(CFLAGS)
ifneq ("$(HAL_DIR)", "")
	EXTRA_CFLAGS += -DNLM_HAL_LINUX_KERNEL -Iarch/mips/include/asm/netlogic/hal
endif

obj-y :=   cpu_proc.o memory.o bootinfo.o

#Changes to build with SDK 2.3.0 HAL - removed nlm_hal_nae, vsemi_config and nlm_eeprom
obj-$(CONFIG_NLM_XLP)			+= nlm_hal_fmn_config.o nlm_hal_cpu_info.o nlm_hal_sys.o
obj-$(CONFIG_NLM_XLP)			+= nlm_hal.o fdt_helper.o nlm_hal_mdio.o
obj-$(CONFIG_NLM_XLP_EVP_CPLD)	+= nlm_evp_cpld.o
obj-$(CONFIG_NLM_FDT_PROC)		+= fdt_proc.o

ifeq ($(CONFIG_NLM_CORTINA_SUPPORT),y)
obj-y += cortina_cs34x7/nlm_cortina_cs34x7.o cortina_cs34x7/nlm_cortina_cs34x7_p1.o cortina_cs34x7/nlm_cortina_cs34x7_p2.o \
		 cortina_cs34x7/nlm_cortina_cs34x7_p3.o cortina_cs34x7/nlm_cortina_cs34x7_p4.o cortina_cs34x7/nlm_cortina_cs34x7_p5.o \
		 cortina_cs34x7/nlm_cortina_cs34x7_p6.o cortina_cs34x7/nlm_cortina_cs34x7_p7.o cortina_cs34x7/nlm_cortina_cs34x7_p8.o \
		 cortina_cs34x7/nlm_cortina_cs34x7_p9.o cortina_cs34x7/nlm_cortina_cs34x7_p10.o cortina_cs34x7/nlm_cortina_cs34x7_p11.o \
		 cortina_cs34x7/nlm_cortina_cs34x7_p12.o
EXTRA_CFLAGS += -DNLM_CORTINA_SUPPORT
endif

obj-$(CONFIG_SMP)                 	+= smp.o

EXTRA_AFLAGS := $(CFLAGS)

clean-files += nlm_hal_fmn_config.o nlm_hal_cpu_info.o nlm_hal_sys.o
clean-files += nlm_hal.o fdt_helper.o nlm_hal_mdio.o
clean-files += nlm_evp_cpld.o fdt_proc.o
clean-files += srio.o dma.o smp.o cpu_proc.o memory.o bootinfo.o

