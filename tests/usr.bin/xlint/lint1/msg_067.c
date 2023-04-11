/*	$NetBSD: msg_067.c,v 1.6 2023/03/28 14:44:34 rillig Exp $	*/
# 3 "msg_067.c"

// Test for message: cannot return incomplete type [67]

/* lint1-extra-flags: -X 351 */

/* expect+1: warning: struct 'incomplete' never defined [233] */
struct incomplete;

struct incomplete function_declaration(void);

struct incomplete
function_definition(void)
/* expect+1: error: cannot return incomplete type [67] */
{
	/* expect+1: error: 'r' has incomplete type 'incomplete struct incomplete' [31] */
	struct incomplete r;

	/* expect+1: error: cannot return incomplete type [212] */
	return r;
}
