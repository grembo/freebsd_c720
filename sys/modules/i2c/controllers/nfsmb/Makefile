# $FreeBSD: head/sys/modules/i2c/controllers/nfsmb/Makefile 272016 2014-09-23 05:54:18Z rpaulo $

.PATH: ${.CURDIR}/../../../../dev/nfsmb

KMOD=	nfsmb
SRCS=	nfsmb.c
SRCS+=	device_if.h smbus_if.h pci_if.h bus_if.h

.include <bsd.kmod.mk>
