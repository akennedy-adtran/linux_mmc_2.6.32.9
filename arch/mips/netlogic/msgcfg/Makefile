#EXTRA_CFLAGS := -Werror
LOCAL_HAL_DIR :=$(srctree)/arch/mips/include/asm/netlogic/hal
EXTRA_CFLAGS := $(CFLAGS) -DNLM_HAL_LINUX_KERNEL -I$(LOCAL_HAL_DIR)
EXTRA_AFLAGS := $(CFLAGS)

ifneq ($(CONFIG_NLM_XLR), "")
clean-files += msgring_shared.lex.c
clean-files += msgring_xls.lex.c
clean-files += msgring.yacc.c
clean-files += msgring
clean-files += msgring.yacc.h
clean-files += msgring_xls.yacc.c
clean-files += msgring_shared.yacc.c
clean-files += msgring_shared
clean-files += msgring_xls
clean-files += msgring_xls.yacc.h
clean-files += msgring_shared.yacc.h
clean-files += msgring.lex.c
endif

shared_sources:
	$(Q)mkdir -p $(LOCAL_HAL_DIR)
ifneq ("$(HAL_DIR)", "")
	$(Q) echo "***Caching NLM HAL code from $(HAL_DIR)***"
	$(Q)cp -fr $(HAL_DIR)/*.h $(LOCAL_HAL_DIR)/
	$(Q)cp -fr $(FDT_DIR)/libfdt/contrib/fdt_helper.h $(LOCAL_HAL_DIR)/
	$(Q)cp -fr $(HAL_DIR)/nlm_hal.c  ../common
	$(Q)cp -fr $(HAL_DIR)/nlm_hal_sys.c  ../common
	$(Q)cp -fr $(HAL_DIR)/nlm_hal_fmn_config.c  ../common
	$(Q)cp -fr $(HAL_DIR)/nlm_evp_cpld.c  ../common
	$(Q)cp -fr $(HAL_DIR)/nlm_eeprom.c  ../common
	$(Q)cp -fr $(HAL_DIR)/nlm_hal_mdio.c  ../common
	$(Q)cp -fr $(HAL_DIR)/nlm_hal_cpu_info.c  ../common
	$(Q)cp -fr $(FDT_DIR)/libfdt/contrib/fdt_helper.c  ../common
	/bin/rm -rf $(LOCAL_HAL_DIR)/nlm_hal_pic.h
endif

ifneq ($(CONFIG_NLM_XLR), "")
msgring:
	make -f Makefile.msgring all
	make -f Makefile.msgring.xls all
	make -f Makefile.msgring.shared all
endif

.PHONY: msgring
