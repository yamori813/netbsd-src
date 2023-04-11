/*	$NetBSD: msg_018.c,v 1.6 2023/03/28 14:44:34 rillig Exp $	*/
# 3 "msg_018.c"

// Test for message: illegal use of 'void' [18]

/* lint1-extra-flags: -X 351 */

/* expect+1: error: void type for 'x' [19] */
void x;

/* expect+1: error: cannot take size/alignment of void [146] */
unsigned long sizeof_void = sizeof(void);

/* expect+2: error: illegal use of 'void' [18] */
/* expect+1: warning: empty array declaration for 'void_array' [190] */
void void_array[];
