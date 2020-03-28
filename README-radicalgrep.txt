README-radicalgrep.txt

Radical grep is a tool built off of icgrep. It imports a file from the user and will prompt them to input a radical to be searched, followed by a place holder "_".
After it performs the search, it will output characters with the specified radicals.

Iteration 1:
Below lists the implementations for the first iteration.

1. Single Radical Search - Input any one radical and search for all characters with the corresponding radical.
Input: grep 氵
Output: Return all characters in the file with the radical 氵.

2. Phrase Search - Input two radicals and return a phrase (two consecutive characters) with the corresponding radicals.
Input: grep 亻心
Output: Return all phrases with the radicals 亻心. The order of the outputted characters must be the same as the inputted. 
For instance, phrases with 心亻will not be returned.

3. Regex Search - Similar to #2, but with regular expressions.
Input: grep [亻心]
Output: Return all phrases with the radicals 亻心. The order of the outputted characters must be the same as the inputted. 
For instance, phrases with 心亻will not be returned.

Other Ideas:
-Implement number of strokes in a character?