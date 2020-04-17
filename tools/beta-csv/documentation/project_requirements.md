# Project Requirements

Authored by Group Beta: Joshua Malmberg, Vincent Chen, Josie Zhou

# CSV Specification

+ A CSV file consists of plain text using a character set/encoding such as ASCII, Unicode, etc.

+ A CSV is composed of records, where each line is one record.

+ Each record is divided into fields separated by delimiters (typically a comma is used as the delimiter).

+ Every record has the same sequence and number of fields.

+ Typically, the first line of the file is reserved as a header which provides a name for each of the fields in subsequent records.

# JSON Specification
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

# Project Objective

Write a program that converts CSV files to JSON using the Parabix framework.

# Basic Requirements

-	Interprets the first line of the CSV file as a header containing.
-	Converts each record in the CSV to an object in the JSON document. Each object occupies only one line in the JSON file.
-	Assumes the value in each field of the CSV file is of string type.
-	Does not ignore extra whitespace.
-	Accepts only commas as the delimiter in the CSV document.
-	Add square brackets at both the beginning and the end of the JSON file.
# Advanced Requirements
-	User able to specify the delimiter used in the CSV document.
-	Includes whitespace formatting (tabs and page returns) in the JSON document to display the structure in each object.
-	Allows the user to specify the type of values present in each field of the CSV file. Otherwise, predicts the data type and assigns the type in the JSON file accordingly.
-	Ignores whitespace in the CSV file.
-	Interprets backslash escape sequencing in the CSV document and includes it in the JSON file accordingly.
-	Support for optional fields
# References

- https://en.wikipedia.org/wiki/JSON
- https://en.wikipedia.org/wiki/Comma-separated_values
