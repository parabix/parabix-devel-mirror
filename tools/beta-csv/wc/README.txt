csv_count_records.cpp is just a rough draft.
This program borrowed codes from ucount and csv_escape_quotes.
greping "modified" in the implementation of the program will find the modified areas. 
program counts the number of records in input file(s).

basically the program translates literal newlines(newlines in string) from '\r' to '\n' in a basisbit stream, and then counts nonliteral newlines in a translated stream.


to test the program, you may enter the following commands:
bin/csv_count_records ../tools/beta-csv/wc/example1.csv ../tools/beta-csv/wc/example2.csv ../tools/beta-csv/wc/example3.csv
bin/csv_count_records ../tools/beta-csv/wc/example1.csv
...


first command should output:
     10       ../tools/beta-csv/wc/example1.csv
     10       ../tools/beta-csv/wc/example2.csv
     10       ../tools/beta-csv/wc/example3.csv
    30 total


to count '\r' uncomment line 224 and comment out line 225
to count '\n' comment out line 224 and uncomment line 225 
