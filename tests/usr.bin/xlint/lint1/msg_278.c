/*	$NetBSD: msg_278.c,v 1.5 2023/03/28 14:44:35 rillig Exp $	*/
# 3 "msg_278.c"

// Test for message: combination of '%s' and '%s', arg #%d [278]

/* lint1-extra-flags: -e -X 351 */

enum E {
	E1
};

void sink_enum(enum E);
void sink_int(int);

void
example(enum E e, int i)
{
	sink_enum(e);
	/* expect+1: warning: combination of 'enum E' and 'int', arg #1 [278] */
	sink_enum(i);

	/* expect+1: warning: combination of 'int' and 'enum E', arg #1 [278] */
	sink_int(e);
	sink_int(i);
}
