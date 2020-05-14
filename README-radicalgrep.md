# README-radicalgrep

Radical Grep is a tool built off of the icgrep engine. It is a program that searches for given Chinese radicals, and returns the characters that corresponds to the input. Note that radicals will be processed according to the Kangxi Radical-Stroke indices.

Please refer to this webpage for the list of radicals: https://en.wikipedia.org/wiki/Kangxi_radical

## Installation

To build radical grep, the working environment needs to have all requirements of the icgrep build met. icgrep must also be built beforehand.

After the above has been done, run  `make`  and `make check` on the terminal to build the software and run the test suite.

To build only Radical Grep and it's dependencies, run `make radicaltest` and `make radicaltest check`.

After everything passes, you are ready to run Radical Grep.

## How to Run Radical Grep

To run Radical Grep, run the following commands in the bin directory:

## Iteration 1: Hard-Coding the Testcases into the Program

In the first iteration, Radical Grep takes in pre-programmed inputs and returns the phrase with the corresponding radicals.

 ## Example 1

    Input: ./radicalgrep 亻_心_ ../QA/radicaltest/testfiles/test1
    Output: 以下是一些关于部首分类的信息

## Example 2
    Input: ./radicalgrep 氵_宀 _ ../QA/radicaltest/testfiles/test1
    Output: 这是采用“两分法”对汉字进行结构分析得出的认识


## Iteration 2: Grep Implementation & Radical Count

Plans for iteration 2 include:
* Implement icgrep 
* Use the Kangxi radical indices as input 
* Output the characters instead of the sentence, with the corresponding radicals 

## Radical Count
Radical Count is a program built based off of `ucount`. Given a filepath and the index(s) of a radical, it counts the occurences of characters with the corresponding radical in the input file.

## How to Run Radical Count
Build the program by typing `make radicalcount` into the terminal. Go into the bin diretory and run the following commands.

    ./radicalcount <Radical Expression> <Path of Input File>

## Example 1
    Input: ./radicalcount 85_ ../QA/radicaltest/testfiles/test1
    Output: 3       ../QA/radicaltest/testfiles/test1

## Example 2
    Input: ./radicalcount 9_61_ ../QA/radicaltest/testfiles/test1
    Output:       6       ../QA/radicaltest/testfiles/test1
                  2       ../QA/radicaltest/testfiles/test1
    
###### *Inputs of 2 radicals returns the count of each radical, and not the number of consecutive occurences (i.e. phrases).

Future Improvements Include:

1. Perform the search using the actual radicals and not the Kangxi index.
2. Search for consecutive occurences (phrases) when two radicals are inputted.
