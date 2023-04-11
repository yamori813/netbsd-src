/*	$NetBSD: msg_139.c,v 1.5 2023/03/28 14:44:34 rillig Exp $	*/
# 3 "msg_139.c"

// Test for message: division by 0 [139]

/* lint1-extra-flags: -X 351 */

void sink_int(int);
void sink_double(double);

void
example(int i)
{
	enum {
		zero = 0
	};

	sink_int(i / 0);	/* only triggers in constant expressions */
	sink_int(i / zero);	/* only triggers in constant expressions */
	sink_double(i / 0.0);

	/* expect+1: error: division by 0 [139] */
	sink_int(13 / 0);
	/* expect+1: error: division by 0 [139] */
	sink_int(13 / zero);
	/* expect+1: error: division by 0 [139] */
	sink_double(13 / 0.0);	/* XXX: Clang doesn't warn */
	/* expect+1: error: division by 0 [139] */
	sink_double(13 / -0.0);	/* XXX: Clang doesn't warn */
}
