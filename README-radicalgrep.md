# README-radicalgrep

Radical Grep is a tool built off of the icgrep engine. It is a program that searches for given Chinese radicals, and returns the characters that corresponds to the input. 

## Installation

To build radical grep, the working environment needs to have all requirements of the icgrep build met. icgrep must also be built beforehand.

After the above has been done, run  `make`  and `make check` on the terminal to build the software and run the test suite.

To build only Radical Grep and it's dependencies, run `make radicaltest` and `make radicaltest check`.

After everything passes, you are ready to run Radical Grep.

## How to Run Radical Grep

To run Radical Grep for iteration 1, run the following commands on your terminal:

~~~
cd QA
python greptest.py -t radicaltest/radicaltest.xml -d . ../build/bin/icgrep
~~~

## Iteration 1

In the first iteration, Radical Grep takes in pre-programmed inputs and returns the phrase with the corresponding radicals.

 ## Test 1

    Input: 亻_心_
    Output: 以下是一些关于部首分类的信息

## Test 2
    Input: 氵_宀 _
    Output: 这是采用“两分法”对汉字进行结构分析得出的认识

## Iteration 2

Plans for iteration 2 include:
* Implementing UniHan scripts to "grep" characters
* Improving LLVM command line functionality
* Support regular expressions

