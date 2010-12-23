#
# Makefile for Marvell's PXA168 processors line
#

obj-y				+= common.o clock.o devices.o irq.o time.o

# SoC support
obj-$(CONFIG_CPU_PXA168)	+= pxa168.o
obj-$(CONFIG_CPU_PXA910)	+= pxa910.o pxa910-squ.o

# board support
ifneq (${CONFIG_MACH_CHUMBY_SILVERMOON},y)
obj-$(CONFIG_MACH_ASPENITE)	+= aspenite.o
endif
obj-$(CONFIG_MACH_ZYLONITE2)	+= aspenite.o
obj-$(CONFIG_MACH_TETON_BGA)	+= teton_bga.o
obj-$(CONFIG_MACH_WAYLAND)	+= wayland.o
obj-$(CONFIG_MACH_TTC_DKB)	+= ttc_dkb.o
obj-$(CONFIG_MACH_TAVOREVB)	+= tavorevb.o
obj-$(CONFIG_MACH_CHUMBY_SILVERMOON) += silvermoon.o

obj-$(CONFIG_DVFM)		+= dvfm.o
obj-$(CONFIG_DVFM_PXA168)	+= pxa168_dvfm.o pxa168_dfc_ll.o
obj-$(CONFIG_DVFM_PXA910)	+= pxa910_dvfm.o
obj-$(CONFIG_MSPM_PXA168)	+= pxa168_mspm_idle.o pxa168_mspm_prof.o
obj-$(CONFIG_MSPM_PXA910)	+= pxa910_mspm_idle.o pxa910_mspm_prof.o
obj-$(CONFIG_PM_PXA168)		+= pxa168_pm.o pxa168_pm_ll.o
obj-$(CONFIG_PM_PXA910)		+= pxa910_pm.o pxa910_pm_ll.o
obj-$(CONFIG_TIMER_SERVICES_MMP)	+= timer_services.o
obj-$(CONFIG_MACH_WAYLAND)	+= am300epd.o
obj-$(CONFIG_PCI)		+= pxa168_pcie.o io.o
