# Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG

DIRS += configure
DIRS += $(wildcard *Sup)
DIRS += $(wildcard *App)
ifeq ($(BUILD_IOCS),YES)
DIRS += $(wildcard iocBoot)
endif

define DIR_template
 $(1)_DEPEND_DIRS = configure
endef
$(foreach dir, $(filter-out configure, $(DIRS)), $(eval $(call DIR_template,$(dir))))

iocBoot_DEPEND_DIRS += $(filter %App,$(DIRS))

include $(TOP)/configure/RULES_TOP
