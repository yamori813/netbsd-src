/*	$NetBSD: msg_230_uchar.c,v 1.11 2023/02/27 21:59:14 rillig Exp $	*/
# 3 "msg_230_uchar.c"

// Test for message: nonportable character comparison '%s' [230]

/* lint1-flags: -S -g -p -w */
/* lint1-only-if: uchar */

/*
 * C11 6.2.5p15 defines that 'char' has the same range, representation, and
 * behavior as either 'signed char' or 'unsigned char'.
 *
 * The portable range of 'char' is from 0 to 127 since all lint platforms
 * define CHAR_SIZE to be 8.
 *
 * See msg_162.c, which covers 'signed char' and 'unsigned char'.
 */

void
compare_plain_char(char c)
{
	/* expect+1: warning: nonportable character comparison '== -129' [230] */
	if (c == -129)
		return;
	/* expect+1: warning: nonportable character comparison '== -128' [230] */
	if (c == -128)
		return;
	/* expect+1: warning: nonportable character comparison '== -1' [230] */
	if (c == -1)
		return;
	if (c == 0)
		return;
	if (c == 127)
		return;
	/* expect+1: warning: nonportable character comparison '== 128' [230] */
	if (c == 128)
		return;
	/* expect+1: warning: nonportable character comparison '== 255' [230] */
	if (c == 255)
		return;
	/* expect+1: warning: nonportable character comparison '== 256' [230] */
	if (c == 256)
		return;
}

void
compare_plain_char_yoda(char c)
{
	/* expect+1: warning: nonportable character comparison '-129 == ?' [230] */
	if (-129 == c)
		return;
	/* expect+1: warning: nonportable character comparison '-128 == ?' [230] */
	if (-128 == c)
		return;
	/* expect+1: warning: nonportable character comparison '-1 == ?' [230] */
	if (-1 == c)
		return;
	if (0 == c)
		return;
	if (127 == c)
		return;
	/* expect+1: warning: nonportable character comparison '128 == ?' [230] */
	if (128 == c)
		return;
	/* expect+1: warning: nonportable character comparison '255 == ?' [230] */
	if (255 == c)
		return;
	/* expect+1: warning: nonportable character comparison '256 == ?' [230] */
	if (256 == c)
		return;
}

void
compare_greater(char c)
{

	/* expect+1: warning: nonportable character comparison '> -2' [230] */
	if (c > -2)
		return;
	/* expect+1: warning: nonportable character comparison '>= -1' [230] */
	if (c >= -1)
		return;

	/*
	 * XXX: The following two comparisons have the same effect, yet lint
	 * only warns about one of them.
	 */
	/* expect+1: warning: nonportable character comparison '> -1' [230] */
	if (c > -1)
		return;
	/*
	 * This warning only occurs on uchar platforms since on these
	 * platforms it is always true.  Code that needs this ordered
	 * comparison on values of type plain char is questionable since it
	 * behaves differently depending on the platform.  Such a comparison
	 * should never be needed.
	 */
	/* expect+1: warning: operator '>=' compares 'char' with '0' [162] */
	if (c >= 0)
		return;

	/*
	 * XXX: The following two comparisons have the same effect, yet lint
	 * only warns about one of them.
	 */
	if (c > 127)
		return;
	/* expect+1: warning: nonportable character comparison '>= 128' [230] */
	if (c >= 128)
		return;

	/* expect+1: warning: nonportable character comparison '> 128' [230] */
	if (c > 128)
		return;
	/* expect+1: warning: nonportable character comparison '>= 129' [230] */
	if (c >= 129)
		return;
}

void
compare_with_character_literal(char ch)
{
	/*
	 * FIXME: These comparisons are portable since the character constant
	 *  is interpreted using the type 'char' on the exact same platform
	 *  as where the comparison takes place.
	 */
	/* expect+1: warning: nonportable character comparison '== 128' [230] */
	if (ch == '\200')
		return;
	/* expect+1: warning: nonportable character comparison '== 255' [230] */
	if (ch == '\377')
		return;
	if (ch == '\000')
		return;
}
