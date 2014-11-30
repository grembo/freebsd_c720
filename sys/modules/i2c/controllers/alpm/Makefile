# $FreeBSD: head/sys/modules/i2c/controllers/alpm/Makefile 272017 2014-09-23 06:31:15Z rpaulo $

.PATH:		${.CURDIR}/../../../../dev/alpm
KMOD		= alpm
SRCS		= device_if.h bus_if.h iicbus_if.h smbus_if.h pci_if.h \
		  alpm.c

.include <bsd.kmod.mk>
