# README-radicalgrep

Radical Grep is a tool built off of the icgrep engine. It is a program that searches for given Chinese radicals, and returns the characters that corresponds to the input. 

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


## Iteration 2

Plans for iteration 2 include:
* Implementing UniHan scripts to "grep" characters
* Improving LLVM command line functionality
* Support regular expressions

## Radical Count
Radical Count is a program built based off of `ucount`. Given a filepath and the index of a radical, it counts the occurences of characters with the corresponding radical in the input file.

## How to Run Radical Count
Build the program by typing `make radicalcount` into the terminal. Go into the bin diretory and run the following commands.

    ./radicalcount <[1,214]> <Path of Input File>

## Example 1
    Input: ./radicalcount 85 ../QA/radicaltest/testfiles/test1
    Output: 3       ../QA/radicaltest/testfiles/test1

