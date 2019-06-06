# Pablo Syntax

## Expression Syntax

``` bnf
<expression> ::=  <term> | <expression> "|" <term> | <expression> "^" <term>
<term> ::=  <arithmetic-expr> | <term> "&" <factor>
<factor> ::= <primitive> | "~" <primitive>
<primitive> ::= <literal> | <variable> | <function-call> | "(" <expression> ")"
<literal> ::= <int> | "<" <int> ">"
<variable> ::= <identifier> | <identifier> "[" <int> "]" | <identifier> "."  <identifier>
<function-call> ::= <identifier> "(" <expression> {"," <expression>} ")"
<int> = '[0-9]+' | '0x[0-9a-fA-f]+' | '0[0-7]+'
```

## Statements

``` bnf
<block> ::= {<statement>}
<statement> ::= <assignment> | <if> | <while>
<assignment> ::= <variable> "=" <expression> | <variable> "|=" <expression> | <variable> "&=" <expression>
<if> ::= "if" <expr>  <block>
<while> ::= "while" <expr>  <block>
```

## Types

``` bnf
<type> ::= <integer-type> | <stream-type> | <stream-set-type> |  <identifier>
<integer-type> ::= "i" <int>
<stream_type> ::= "<" <integer-type> ">"
<stream_set-type> ::= <stream_type> "[" <integer> "]" ["{" <identifier> {"," <identifier>} "}"]
<type-definition> ::= "type" <identifier> = <type>
```

## Kernels

``` bnf
<kernel> ::= "kernel" <identifier> "::" <signature> "{" block "}"
<signature> ::= <parameter_list> "->" <parameter_list>
<parameter_list> ::= [<parameter_spec> {"," <parameter_spec>} ]
<parameter_spec> ::= <type> <identifier>
<declaration> ::= <type-definition> | <kernel>
```
