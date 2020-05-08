csv_count_records.cpp is just a rough draft.
This program borrowed codes from ucount and csv_escape_quotes.
greping "modified" in the implementation of the program will find the modified areas. 

basically the program translates literal newlines(newlines in string) from '\r' to '\n', and then counts the character class input(second arg).


to test the program, you may enter the following commands:
bin/csv_count_records '\r' ../tools/beta-csv/wc/example1.csv ../tools/beta-csv/wc/example2.csv
bin/csv_count_records '\n' ../tools/beta-csv/wc/example1.csv ../tools/beta-csv/wc/example2.csv
bin/csv_count_records [abcd'\r'] ../tools/beta-csv/wc/example1.csv
...


first command should output:
     10       ../tools/beta-csv/wc/example1.csv
     10       ../tools/beta-csv/wc/example2.csv
    20 total


second command should output:
     12       ../tools/beta-csv/wc/example1.csv
     10       ../tools/beta-csv/wc/example2.csv
    22 total

