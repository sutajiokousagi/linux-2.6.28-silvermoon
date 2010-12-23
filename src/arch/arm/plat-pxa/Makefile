#
# Makefile for code common across different PXA processor families
#

obj-y	:= dma.o mfp.o ssp.o generic.o pxa3xx_pmic.o spi_flashinit.o

obj-$(CONFIG_GENERIC_GPIO)	+= gpio.o
obj-$(CONFIG_HAVE_PWM)		+= pwm.o

obj-$(CONFIG_IMM)		+= imm/
