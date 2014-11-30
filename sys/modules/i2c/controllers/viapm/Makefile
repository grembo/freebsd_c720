# $FreeBSD: head/sys/modules/i2c/controllers/viapm/Makefile 272017 2014-09-23 06:31:15Z rpaulo $

.PATH:		${.CURDIR}/../../../../dev/viapm
KMOD		= viapm
SRCS		= device_if.h bus_if.h iicbb_if.h isa_if.h pci_if.h smbus_if.h \
		  opt_isa.h viapm.c

.include <bsd.kmod.mk>
