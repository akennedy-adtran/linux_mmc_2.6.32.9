%{
#include <stdio.h>
#include <string.h>
#include "msgring.yacc.h"


int yyerror(const char *str)
{
  fprintf(stderr, "%s\n", str);
  return 1;
}

%}

%x c_comment

%%

"/*"                    BEGIN(c_comment); /* c-style comment */
<c_comment>[^*\n]*        /* eat anything that is not a * */
<c_comment>"*"+[^*/\n]*   /* eat up * s not followed by a / */
<c_comment>\n
<c_comment>"*"+"/"      BEGIN(INITIAL);

"//"[^\n]*"\n"         ; /* eat up one-line (C++-style) commens */

bucket                  yylval = (unsigned long)"bucket"; return TOK_BUCKET;
size                    yylval = (unsigned long)"size"; return TOK_SIZE;
[a-zA-Z_][a-zA-Z_0-9]*    yylval = (unsigned long)strdup(yytext); return TOK_WORD;
[0-9]+                  yylval = atoi(yytext); return TOK_INTEGER;
\"                      yylval = (unsigned long)"\""; return TOK_QUOTE;
\{                      yylval = (unsigned long)"{"; return TOK_OPEN_BRACE;
\}                      yylval = (unsigned long)"}"; return TOK_CLOSE_BRACE;
;                       yylval = (unsigned long)";"; return TOK_SEMICOLON;
\n                      /* ignore newline */;
[ \t]+                  /* ignore whitespace */;
.                       yyerror("Unknown token\n");
%%

int yywrap()
{
  return 1;
}

