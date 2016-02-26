#
# Makefile for the kernel mmc core.
#

# 4_2_4 Make use of slot-gpio optional (see Kconfig)
obj-$(CONFIG_MMC)		+= mmc_core.o
mmc_core-y			:= core.o bus.o host.o \
				   mmc.o mmc_ops.o sd.o sd_ops.o \
				   sdio.o sdio_ops.o sdio_bus.o \
				   sdio_cis.o sdio_io.o sdio_irq.o \
				   quirks.o
mmc_core-$(CONFIG_MMC_USE_SLOT_GPIO) += slot-gpio.o
mmc_core-$(CONFIG_OF)		+= pwrseq.o pwrseq_simple.o pwrseq_emmc.o
mmc_core-$(CONFIG_DEBUG_FS)	+= debugfs.o
