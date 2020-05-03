#	Testsuite Design

##	Black Box Testing

To attain adequate test coverage using black box testing, the domain of inputs is broken into partitions for each input parameter.

| Input Parameters | Partitions |
|---|---|
|File-Level||
|- number of fields|{0, 1, >1}|
|- number of records|{0, 1, >1}|
|- delimiter value|{len=1, len>1}|
|- line-break value|{len=1, len>1}|
|Record-Level||
|- number of entries |{1, 2, >2}|
|- number of non-empty entries |{0, 1, >1}|
|- ordering of empty/non-empty entries|{has "01", no "01"}|
|- number of element types contained|{1,2,...,6}|
|Field-Level||
|- element type|{int, bool, null, obj, array, str}|
|- whitespace|{left, right, both, none}|
|- element length|{len=0, len=1, len>1}|
|- escape sequences|{has, no}|

Using combinatorial testing with the partitions above, a large test suite will be required. 

To do:
- [ ] develop testing program
- [ ] choose n to use for n-way testing
- [ ] determine which input conditions are incompatible with each other
- [ ] create test case framework
- [ ] develop test cases using test case framework

##	White Box Testing

White box testing will not be implemented until the parabix version of the csv2json tool is written.