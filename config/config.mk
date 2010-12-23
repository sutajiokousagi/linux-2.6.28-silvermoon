# $Id: config.mk 27 2008-06-11 03:28:46Z henry $
# config.mk - project-specific configuration details

ifeq (${CNPLATFORM},)
export CNPLATFORM=silvermoon
endif
export ARCH=arm
export TARGET=$(ARCH)-linux
export CROSS_COMPILE=$(TARGET)-
