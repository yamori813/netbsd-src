/*	$NetBSD: msg_222.c,v 1.5 2023/03/28 14:44:35 rillig Exp $	*/
# 3 "msg_222.c"

// Test for message: conversion of negative constant to unsigned type [222]

/* lint1-extra-flags: -X 351 */

/* expect+1: warning: initialization of unsigned with negative constant [221] */
unsigned int global = -1;

void take_unsigned_int(unsigned int);

void
function(void)
{
	/* expect+1: warning: initialization of unsigned with negative constant [221] */
	unsigned int local = -1;

	/* expect+1: warning: conversion of negative constant to unsigned type, arg #1 [296] */
	take_unsigned_int(-1);

	if (local & -1)
		return;

	/* expect+1: warning: operator '<' compares 'unsigned int' with 'negative constant' [162] */
	if (local < -1)
		return;

	local &= -1;

	/* expect+1: warning: conversion of negative constant to unsigned type [222] */
	local += -1;
}
