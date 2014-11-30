/*
 * Michael Gmelin <freebsd@grem.de>
 */

#ifndef _ISL_H_
#define _ISL_H_

/* Command register 1  (bits 7-5) */
#define REG_CMD1		0x00
#define CMD1_MASK_POWER_DOWN	0x00      /* 00000000 */
#define CMD1_MASK_ALS_ONCE	0x01 << 5 /* 00100000 */
#define CMD1_MASK_IR_ONCE	0x02 << 5 /* 01000000 */
#define CMD1_MASK_PROX_ONCE	0x03 << 5 /* 01100000 */
/* RESERVED */                            /* 10000000 */
#define CMD1_MASK_ALS_CONT	0x05 << 5 /* 10100000 */
#define CMD1_MASK_IR_CONT	0x06 << 5 /* 11000000 */
#define CMD1_MASK_PROX_CONT	0x07 << 5 /* 11100000 */

/* Command register 2 (bits) */
#define REG_CMD2		0x01

/* data registers */
#define REG_DATA1		0x02
#define REG_DATA2		0x03
#define CMD2_SHIFT_RANGE	0x00
#define CMD2_MASK_RANGE		0x03 << CMD2_SHIFT_RANGE
#define CMD2_SHIFT_RESOLUTION	0x02
#define CMD2_MASK_RESOLUTION	0x03 << CMD2_SHIFT_RESOLUTION

/* Interrupt registers */
#define REG_INT_LO_LSB		0x04
#define REG_INT_LO_MSB		0x05
#define REG_INT_HI_LSB		0x06
#define REG_INT_HI_MSB		0x07


/* Test register (should hold 0x00 at all times */
#define REG_TEST		0x08


#endif
