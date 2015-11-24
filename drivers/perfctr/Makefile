# $Id: Makefile,v 1.1.2.2 2006-06-06 20:28:30 pboddupalli Exp $
# Makefile for the Performance-monitoring counters driver.

ifeq ("$(CONFIG_PERFCTR)","y")

# This also covers x86_64.
perfctr-objs-$(CONFIG_X86) := x86.o
tests-objs-$(CONFIG_X86) := x86_tests.o

perfctr-objs-$(CONFIG_PPC32) := ppc.o
tests-objs-$(CONFIG_PPC32) := ppc_tests.o

perfctr-objs-$(CONFIG_MIPS) := mips.o
tests-objs-$(CONFIG_MIPS) := mips_tests.o

perfctr-objs-y += init.o
perfctr-objs-$(CONFIG_PERFCTR_INIT_TESTS) += $(tests-objs-y)
perfctr-objs-$(CONFIG_PERFCTR_VIRTUAL) += virtual.o

perfctr-objs		:= $(perfctr-objs-y)
obj-$(CONFIG_PERFCTR)	:= perfctr.o

else
obj-y := dummy-syscalls.o
endif
