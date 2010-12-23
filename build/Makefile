# $Id: Makefile 4885 2009-03-26 20:44:53Z henry $
# Marvell linux 2.6.28 (Aspen) top-level makefile
# Wrapper makefile for kernel

include ../config/config.mk

export PLATFORM_TARGET=$(TARGET)-$(CNPLATFORM)

# May be overridden
CONFIGNAME=${CNPLATFORM}_aspen
PLATFORM_CONFIG=../config/kernel-config-$(CONFIGNAME)
EXPORTS=$(abspath ../exports)
OUTPUT=$(abspath ../output/$(PLATFORM_TARGET))
TARGET_EXPORTS=$(EXPORTS)/$(PLATFORM_TARGET)
LINUXDIR=../src
KERNEL_DIR=../output/$(PLATFORM_TARGET)

all: kernel

ksource:

kernel: $(LINUXDIR)/.config ksource $(KERNEL_DIR)
	$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)
	ls -l $(LINUXDIR)/arch/$(ARCH)/boot/zImage
	cp $(LINUXDIR)/arch/$(ARCH)/boot/zImage $(KERNEL_DIR)/k1
ifeq ($(SKIP_K2_UPDATE),1)
	@echo "Warning: skipping k2 update based on SKIP_K2_UPDATE"
else
	mkdir -p $(INSTALL_DIR)/rfs1/boot
	cp $(LINUXDIR)/arch/$(ARCH)/boot/zImage $(INSTALL_DIR)/rfs1/boot/vmlinuz
	cp $(LINUXDIR)/arch/$(ARCH)/boot/zImage $(KERNEL_DIR)/k2
endif
	INSTALL_MOD_PATH=$(INSTALL_DIR)/rfs1 make -C $(LINUXDIR) CROSS_COMPILE=$(CROSS_COMPILE) ARCH=$(ARCH) modules_install

kconfig: ../output
	$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) menuconfig
	@echo "------------ kernel configs ------------"
	@ls -l $(LINUXDIR)/.config $(PLATFORM_CONFIG)
	@echo "------------ start config diff ---------"
	-diff $(LINUXDIR)/.config $(PLATFORM_CONFIG) >../output/$(PLATFORM_TARGET)-diff.txt
	@echo "------------ diff is in ../output/$(PLATFORM_TARGET)-diff.txt ----------"

../output:
	@echo "Creating output dir..."
	-mkdir -p $@

koldconfig:
	cp $(PLATFORM_CONFIG) $(LINUXDIR)/.config
	$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) oldconfig

korgconfig:
	$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) pxa168_defconfig

ksaveconfig:
	-cp $(PLATFORM_CONFIG) $(PLATFORM_CONFIG).bak 2>/dev/null
	cp $(LINUXDIR)/.config $(PLATFORM_CONFIG)

kclean:
	$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) clean

kmrproper:
	$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) mrproper
	-rm $(LINUXDIR)/.cross_compile
	cp $(PLATFORM_CONFIG) $(LINUXDIR)/.config
	$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) oldconfig


$(LINUXDIR)/.config: $(PLATFORM_CONFIG)
	cp $(PLATFORM_CONFIG) $@
	$(MAKE) -C $(LINUXDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) oldconfig

help:
	@echo "Meta-targets:"
	@echo "help build-all clean-all commit"
	@echo "Build process uses:"
	@echo " make -C build clean-all PRODUCTION=1"
	@echo " make -C build build-all PRODUCTION=1"
	@echo " make -C build commit PRODUCTION=1"
	@echo "Utility targets:"
	@echo "diff trunk-merge bumpbuild"

clean:

exports:

clean-exports:

build-all:
	$(MAKE)

clean-all:
	$(MAKE) kclean
	$(MAKE) clean-exports

$(KERNEL_DIR):
	@echo "Creating directory $@"
	-mkdir -p $@

.PHONY: all help clean exports clean-exports build-all clean-all commit \
	kernel ksource kconfig korgconfig koldconfig ksaveconfig kclean kmrproper

