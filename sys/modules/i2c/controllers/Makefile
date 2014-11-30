# $FreeBSD: head/sys/modules/i2c/controllers/Makefile 266474 2014-05-20 19:55:06Z jimharris $

.if ${MACHINE} == "pc98"
SUBDIR = lpbb
.else
SUBDIR = alpm amdpm amdsmb ichsmb intpm ismt nfsmb viapm lpbb pcf
.endif

.include <bsd.subdir.mk>
