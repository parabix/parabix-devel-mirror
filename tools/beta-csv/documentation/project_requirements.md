

# Project Requirements

Authored by Group Beta: Joshua Malmberg, Vincent Chen, Josie Zhou

Last Updated: April 21st, 2020

## CSV Specification

There are many standards for the CSV file format that exist and are in use. For this project, the default file format used is the IETF standard, described in IETF RFC 4180 (https://tools.ietf.org/html/rfc4180). The specification is given below:

+ A CSV file consists of plain text using a character set/encoding such as ASCII, Unicode, etc.
+ A CSV is composed of **records**, where each line is one record. Records are delimited by a line break (carriage return and line feed).
+ The last record in the file may or may not have an ending line break.
+ Each record is divided into one or more **fields** separated by delimiters (typically a comma is used as the delimiter).
+ Every record has the same sequence and number of fields.
+ The last field in a record must not be followed by a comma.
+ Typically, the first line of the file is reserved as a **header** which provides a name for each of the fields in subsequent records.
+ Each field may or may not be surrounded in double quotes. If a field is surrounded by double quotes, the text inside the outer quotes can itself contain double quotes, line breaks, and commas. Otherwise, double quotes, line breaks, and commas will be interpreted as syntactic elements rather than field contents.
+ If a field surrounded by double quotes, any double quotes contained within must be escaped by preceding it with another double quote. For example:
```
    "Hello","World!","She said:""Hello.""\r\n"\r\n
```
​		In this example, the above record the third field is parsed as:
```
	She said:"Hello".\r\n
```

+ white-space in a field should not be ignored.

## JSON Specification

+ Basic Data Types
    - **Number**
    > A signed decimal number that may contain a fractional part and may use exponential E notation
    - **String**
    > A sequence of zero or more Unicode characters. Delimited with double quotation marks and support backslash escaping syntax.
    - **Boolean**
    > Either of the values true or false.
    - **Array**
    > A sequence of zero or more elements, each of which can be of any type. Delimited with square brackets, elements delimited with commas.
    - **Object**
    > An ordered collection of name-value pairs, where the names are strings. Delimited with curly brackets, commas separate pairs, colons separate the name and value in each pair.
    - **Null**
    > An empty value, using the word null.
+ A JSON file consists of data represented using the basic data types described above.

## Project Objective

Write a program that converts CSV files to JSON using the Parabix framework.

### Basic Requirements

-	Interprets the first line of the CSV file as a header containing field names.
-   Outputs an array containing the field names at the beginning of the file. Then outputs an array of objects containing the records of the CSV file.
-	Converts each record in the CSV to an object in the JSON document. Each object occupies only one line in the JSON file. 
-	Assumes the value in each field of the CSV file is of string type and writes it as a string in the JSON output.
-	Does not ignore extra white-space.
-	Accepts only commas as the delimiter in the CSV document.
-	Recognises fields in double quotes and parses the contents as described in the CSV specification, allowing for commas, double quotes, and line breaks inside of a fields.
# Advanced Requirements
-	Option to specify the delimiter used in the CSV document.
-	Option to includes white-space formatting (tabs and page returns) in the JSON document to display the structure in each object.
-	Option to allow the user to specify the type of values present in each field of the CSV file.
-	Option to ignore white-space in the CSV file.
-	Option to interpret backslash escape sequencing in the CSV document and includes it in the JSON file accordingly.
-	Option to write the JSON file as an array of arrays, each sub-array containing 1 record.

#	Input Format

- The csv2json executable can be called using the following command:

  ```
  ~/parabix-devel$ build/bin/csv2json <input file path> <output file path>
  ```

- If the output file location does not exist, it will be created in the location specified.

- The input file must have extension .csv. The ouput file must have extension .json.

- To run the test driver, go to the build directory and execute the following command:

  ```
  make csv2jsontest
  ```

  

###	Output Format

- If the ouput file is not of .json extension, throw an error and write nothing to the file.
- If the input file is not of .csv extension, throw an error and write nothing to the file.
- At the beginning of the file there is an array of strings containing the field names in order.
- The records are output in an array of objects. Each object consists of 0 or more key-element pairs.
- If a field in a given record is empty in the input file, that field should not occur in the corresponding object in the output file.
- If a record consists entirely of empty fields, it should be output as an empty object, ie. {}.
- If a record contains more fields than there are in the header, throw a warning and go to the next record, skipping the remaining entries
- If a record has an extra comma after the final field, throw a warning and go to the next record.
- If an input file contains only a header, ie. only 1 line of data, output the field names in an array and an empty array of objects, ie. [].
- If an input file is entirely empty, throw a warning and output a file containing two empty arrays

## References

- https://en.wikipedia.org/wiki/JSON
- https://en.wikipedia.org/wiki/Comma-separated_values
- https://tools.ietf.org/html/rfc4180
