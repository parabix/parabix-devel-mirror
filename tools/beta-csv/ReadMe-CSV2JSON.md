## CSV to JSON Conversion

This is a CSV to JSON conversion tool, which converts Comma Separated Value   to JavaScript Object Notation. Specifically, it converts a input.csv file to a output.json file.

### Iteration 1

In the first iteration, the “CSV to JSON” program is supposed to convert a  Comma Separated Value file with no **comma** or **newline** in the values and creates a JavaScript Object Notation file. In the JavaScript Object Notation file, each record occupies one line.

See as the following examples.

#### test case 1- group

> + input:
>
>   group,project,numer of members,comment
>   alpha,pinin grep,4,a
>   beta,csv,3,our group
>   gamma,,4,
>   delta,radicalgrep,3,
>
> + output:
>
>   [
>   {"group":"alpha","project":"pinin grep","numer of members":"4","comment":"a"},
>   {"group":"beta","project":"csv","numer of members":"3","comment":"our group"},
>   {"group":"gamma","project":"","numer of members":"4","comment":""},
>   {"group":"delta","project":"radicalgrep","numer of members":"3","comment":""}
>   ]