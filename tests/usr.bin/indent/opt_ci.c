/* $NetBSD: opt_ci.c,v 1.11 2023/06/10 08:17:04 rillig Exp $ */

/*
 * Tests for the option '-ci', which controls the indentation of continuation
 * lines in statements and declarations, but only inside a function.
 */

/*
 * Top level expressions with and without parentheses.
 */
//indent input
int top_level = 1 +
 2;
int top_level = (1 +
 2 + (
  3));
//indent end

//indent run -ci0
int		top_level = 1 +
2;
int		top_level = (1 +
			     2 + (
				  3));
//indent end

//indent run-equals-prev-output -ci2

//indent run-equals-prev-output -ci4

//indent run-equals-prev-output -ci8

//indent run -ci0 -nlp
int		top_level = 1 +
2;
int		top_level = (1 +
	2 + (
		3));
//indent end

//indent run -ci2 -nlp
int		top_level = 1 +
2;
int		top_level = (1 +
  2 + (
    3));
//indent end

/*
 * Between 2019-04-04 and 2023-06-09, there was a special rule that prevented
 * indentation based on the number of open parentheses, in the case that the
 * continuation indentation is half an indentation level, maybe to prevent that
 * the continuation line has the same indentation as a follow-up statement,
 * such as in 'if' statements. To prevent such ambiguities, see '-eei'.
 */
//indent run -ci4 -nlp
int		top_level = 1 +
2;
int		top_level = (1 +
    2 + (
	3));
//indent end


/*
 * Declarations in functions without parentheses.
 */
//indent input
int
sum(int a, int b)
{
	return a +
	 b;
	return first +
	 second;
}
//indent end

//indent run -ci0
int
sum(int a, int b)
{
	return a +
		b;
	return first +
		second;
}
//indent end

//indent run -ci2
int
sum(int a, int b)
{
	return a +
	  b;
	return first +
	  second;
}
//indent end

//indent run -ci4
int
sum(int a, int b)
{
	return a +
	    b;
	return first +
	    second;
}
//indent end

//indent run -ci8
int
sum(int a, int b)
{
	return a +
		b;
	return first +
		second;
}
//indent end


/*
 * Continued statements with expressions in parentheses.
 */
//indent input
int
sum(int a, int b)
{
	return (a +
	b);
	return (first +
	second + (
	third));
}
//indent end

//indent run -ci0
int
sum(int a, int b)
{
	return (a +
		b);
	return (first +
		second + (
			  third));
}
//indent end

//indent run-equals-prev-output -ci2

//indent run-equals-prev-output -ci4

//indent run-equals-prev-output -ci8

//indent run -ci2 -nlp
int
sum(int a, int b)
{
	return (a +
	  b);
	return (first +
	  second + (
	    third));
}
//indent end

//indent run -ci4 -nlp
int
sum(int a, int b)
{
	return (a +
	    b);
	return (first +
	    second + (
		third));
}
//indent end

//indent run -ci8 -nlp
int
sum(int a, int b)
{
	return (a +
		b);
	return (first +
		second + (
			third));
}
//indent end


/*
 * In the default configuration, the indentation level from '-i' is the same
 * as the continuation indentation from '-ci'.  The difference between these
 * becomes visible for structural macros like 'forever' or 'foreach'.
 */
//indent input
#define forever for (;;)
void
function(void)
{
	forever
		stmt();

	forever {
		stmt();
	}
}
//indent end

//indent run-equals-input

/*
 * The difference between the block indentation and the continuation
 * indentation only becomes visible when these two differ.
 */
//indent run -i8 -ci4
#define forever for (;;)
void
function(void)
{
	forever
	    stmt();

	forever {
		stmt();
	}
}
//indent end


//indent input
{
	size_t last_word_len = com.len
	    - (size_t)(last_blank + 1);
}
//indent end

//indent run-equals-input -ldi0 -ci4
