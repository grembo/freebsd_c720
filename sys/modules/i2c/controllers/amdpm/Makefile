# $FreeBSD: head/sys/modules/i2c/controllers/amdpm/Makefile 272017 2014-09-23 06:31:15Z rpaulo $

.PATH:		${.CURDIR}/../../../../dev/amdpm
KMOD		= amdpm
SRCS		= device_if.h bus_if.h pci_if.h smbus_if.h \
		  amdpm.c

.include <bsd.kmod.mk>
