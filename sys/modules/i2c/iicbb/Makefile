# $FreeBSD: head/sys/modules/i2c/iicbb/Makefile 262522 2014-02-26 03:26:00Z lwhsu $

.PATH:		${.CURDIR}/../../../dev/iicbus
KMOD		= iicbb
SRCS		= device_if.h bus_if.h iicbus_if.h \
		  iicbb_if.h iicbb_if.c iicbb.c opt_platform.h

.include <bsd.kmod.mk>
