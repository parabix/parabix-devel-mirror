# CSV2JSON

CSV2JSON is a tool that converts CSV files to JSON. 

## Specification

​	CSV2JSON accepts as input standard CSV files using commas as field delimiters and line breaks as record delimiters. A JSON file consisting of an array of objects is output. 

​	Each record in the CSV file is represented as an object in the JSON file. Each field in a given CSV record is represented by an name-value pair in the related JSON object. White-space surrounding field contents in the CSV file is not ignored and will be preserved in the JSON name-value pair.



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



