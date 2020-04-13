README-radicalgrep.txt

Radical Grep

Radical grep is a tool built off of icgrep. It imports a file from the user and will prompt them to input a radical to be searched, followed by a place holder "_".
After it performs the search, it will output the whole line which the characters with the specified radicals in.

Iteration 1:
Below lists the implementations for the first iteration.

- Phrase Search 
- Input two radicals and return a phrase (two consecutive characters) with the corresponding radicals.

Sample:
Input: grep 亻_心_
Output: Return all phrases with the radicals 亻and 心. The order of the outputted characters must be the same as the inputted. 
For instance, phrases with 心_亻_will not be returned.

Other Ideas:
- Implement number of strokes in a character?


BUILD
cd Parabix-devel
mkdir build
cd build
make 
make check

COMMAND
iteration 1:
cd QA
python greptest.py -t radicaltest/radicaltest.xml -d . ../build/bin/icgrep