#	Testsuite Design

##	Input Format

- Any .csv file. The goal is to design a tool that can handle any input by throwing an error, throwing a warning and writing to an output file, or only writing to an output file. 

##	Output Format

- .json
- At the beginning of the file there is an array of strings containing the field names in order.
- The records are output in an array of objects. Each object consists of 0 or more key-element pairs.
- If a field in a given record is empty in the input file, that field should not occur in the corresponding object in the output file.
- If a record consists entirely of empty fields, it should be output as an empty object, ie. {}.
- If a record contains more fields than there are in the header, throw a warning and go to the next record, skipping the remaining entries
- If an input file contains only a header, ie. only 1 line of data, output the field names in an array and an empty array of objects, ie. []
- If an input file is entirely empty, throw a warning and output a file containing two empty arrays.

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