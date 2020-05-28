## CSV to JSON Conversion

This is a CSV to JSON conversion tool, which converts Comma Separated Value to JavaScript Object Notation. Specifically, it converts a input.csv file to a output.json file.

CSV2JSON accepts as input standard CSV files using **commas** as field delimiters and **line breaks** as record delimiters. A JSON file consisting of an array of objects is output. 

Each record in the CSV file is represented as an object in the JSON file. Each field in a given CSV record is represented by an name-value pair in the related JSON object. White-space surrounding field contents in the CSV file is not ignored and will be preserved in the JSON name-value pair.



### How to use CSV2JSON?

CSV2JSON can be called from the parabix-devel directory using the following terminal command:

````
~/parabix-devel$ build\bin\csv2json <filename>
````

For example:

````
~/parabix-devel$ bin\csv2json ..\QA\csv2json\testcases\testcase_group.csv
````

Should yield the following output:

~~~~
[
{"group":"alpha","project":"pinin grep","numer of members":"4","comment":"a"},
{"group":"beta","project":"csv","numer of members":"3","comment":"our group"},
{"group":"gamma","project":"","numer of members":"4","comment":""},
{"group":"delta","project":"radicalgrep","numer of members":"3","comment":""}
]
~~~~



### Iteration 1

In the first iteration, the “CSV to JSON” program is supposed to convert a  Comma Separated Value file with no **comma** or **newline** in the values and creates a JavaScript Object Notation file. In the JavaScript Object Notation file, each record occupies one line.

See as the following examples.

#### Test case 1 - group



##### Input:

```
group,project,numer of members,comment
alpha,pinin grep,4,a
beta,csv,3,our group
gamma,,4,
delta,radicalgrep,3,
```



##### Output:

```json
[
{"group":"alpha","project":"pinin grep","numer of members":"4","comment":"a"},
{"group":"beta","project":"csv","numer of members":"3","comment":"our group"},
{"group":"gamma","project":"","numer of members":"4","comment":""},
{"group":"delta","project":"radicalgrep","numer of members":"3","comment":""}
]
```



###  Iteration 2

In the second iteration, we solve the limitation in Iteration 1-- fields are not allowed to include commas or newlines. Besides that, the field can also include double quotes. In all, in this iteration, we can deal with CSV file with its field values containing commas, newlines and double quotes.

This “CSV to JSON” program is supposed to convert a  Comma Separated Value file to a JavaScript Object Notation file. And in the JavaScript Object Notation file, each record occupies one line.

See as the following examples.

#### Test case 2 - conversation

This test case displays how the program handles CSV file with embedded commas, newlines and double quotes. Embedded newline is converted to **\n** , embedded double quote is converted to **\\"**  and embedded comma stays the same as **,** .

##### Input:

```
who,says,is a teacher,is a student
professor Cameron,"Group Beta,
why is your submission so late?",yes,no
Josie,Because I am a bad procrastinator.,no,yes
Joshua,"Yes, she is.",no,yes
Vincent,"Yes, she is.",no,yes
Josie,"Yes, I ""really"" am.",no,yes
```



##### Output:

```json
[
 {"who": "professor Cameron","says": "Group Beta,\nwhy is your submission so late?","is a teacher": "yes","is a student": "no"},
 {"who": "Josie","says": "Because I am a bad procrastinator.","is a teacher": "no","is a student": "yes"},
 {"who": "Joshua","says": "Yes, she is.","is a teacher": "no","is a student": "yes"},
 {"who": "Vincent","says": "Yes, she is.","is a teacher": "no","is a student": "yes"},
 {"who": "Josie","says": "Yes, I \"really\" am.","is a teacher": "no","is a student": "yes"}
]
```





Authored by Group Beta: Joshua Malmberg, Vincent Chen, Josie Zhou

Last Updated: May 28th, 2020