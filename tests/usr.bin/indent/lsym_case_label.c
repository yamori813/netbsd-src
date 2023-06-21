/* $NetBSD: lsym_case_label.c,v 1.11 2023/06/15 09:19:07 rillig Exp $ */

/*
 * Tests for the tokens lsym_case and lsym_default, which represent the
 * keywords 'case' and 'default', which are both used in 'switch' statements.
 *
 * Since C11, the keyword 'default' is used in _Generic selections as well.
 *
 * See also:
 *	opt_cli.c
 *	psym_switch_expr.c
 *	C11 6.5.1.1		"Generic selection"
 */

/*
 * A case label can be used in a 'switch' statement.
 */
//indent input
void function(void){switch(expr){case 1:;case 2:break;default:switch(inner){case 4:break;}}}
//indent end

//indent run
void
function(void)
{
	switch (expr) {
	case 1:	;
	case 2:
		break;
	default:
		switch (inner) {
		case 4:
			break;
		}
	}
}
//indent end


/*
 * If there is a '{' after a case label, it gets indented using tabs instead
 * of spaces. Indent does not necessarily insert a space in this situation,
 * which looks strange.
 */
//indent input
void
function(void)
{
	switch (expr) {
	case 1: {
		break;
	}
	case 11: {
		break;
	}
	}
}
//indent end

//indent run
void
function(void)
{
	switch (expr) {
	/* $ The space between the ':' and the '{' is actually a tab. */
	case 1:	{
			break;
		}
	case 11: {
			break;
		}
	}
}
//indent end


/*
 * Since C11, the _Generic selection expression allows a switch on the data
 * type of an expression.
 */
//indent input
const char *type_name = _Generic(
	' ',
	int: "character constants have type 'int'",
	char: "character constants have type 'char'",
	default: "character constants have some other type"
);
//indent end

//indent run -di0
const char *type_name = _Generic(
// $ XXX: It's strange to align the arguments at the parenthesis even though
// $ XXX: the first argument is already on a separate line.
				 ' ',
// $ The indentation is so large that the strings would spill over the right
// $ margin.  To prevent that, the code is right-aligned.  Since the base
// $ indentation of this declaration is 0, the code might even start at the
// $ beginning of the line.
				 int: "character constants have type 'int'",
			       char: "character constants have type 'char'",
			 default: "character constants have some other type"
);
//indent end

//indent run-equals-input -di0 -nlp


/*
 * Multi-line case expressions are rare but still should be processed in a
 * sensible way.
 */
//indent input
{
	switch (expr) {
// $ FIXME: The line containing the 'case' must be indented like a 'case'.
		case 1
		    + 2
// $ FIXME: This continuation line must be indented by 4 columns.
	+ 3:
		stmt;
	}
}
//indent end

//indent run-equals-input -ci4
