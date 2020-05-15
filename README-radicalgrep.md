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


## Iteration 2: Radical Count & Grep Implement 

In the second iteration, Radical Grep takes input of the form of actual Kangxi radical (e.g. "子_" or " 氵_子 _") and returns the phrase with the correspondings radicals. The matching radicals in the phrase are highlighted in a different colour.
To implement grep, we first implement radical count:

## Radical Count

Radical Count is a program built based off of `ucount`. Given a filepath and the index(s) of a radical, it counts the occurences of characters with the corresponding radical in the input file.

## How to Run Radical Count

Build the program by typing `make radicalcount` into the terminal. Go into the build diretory and run the following commands.

    bin/radicalcount <Radical Expression> <Path of Input File>

## Example 1

    Input: bin/radicalcount 85_ ../QA/radicaltest/testfiles/test1
    Output: 3       ../QA/radicaltest/testfiles/test1

## Example 2

    Input: bin/radicalcount 9_61_ ../QA/radicaltest/testfiles/test1
    Output:       6       ../QA/radicaltest/testfiles/test1
                  2       ../QA/radicaltest/testfiles/test1
    
###### *Inputs of 2 radicals returns the count of each radical, and not the number of consecutive occurences (i.e. phrases).

## Grep Implement (Radicalgrep)

Software Log:

###### version 2.1.0 
Radical Grep takes input of the form of Kangxi radical indices (e.g. "85_85_"), and returns the phrase with the correspondings radicals. The matching radicals in the phrase are highlighted in a different colour.
###### version 2.1.1
Add the output line when there is no phrase with correspondings radicals.
###### version 2.2.0
Perform the search using the actual radicals and not the Kangxi index.
###### version 2.2.1 (iteration 2 final version)
Add the mode which can search for single radical.

## How to Run Radical Grep (version 2.2.1)

Build the program by typing `make radicalgrep` into the terminal. Go into the build diretory and run the following commands.

    bin/radicalgrep <Radical Expression> <Path of Input File>

## Example 1

    Input: bin/radicalgrep 子_ ../QA/radicaltest/testfiles/test1
    Output: 这是一个简单的例**子**
    部首分类也是使用汉**字**之文化圈少数的共通点
    部首检字也有其局限性，许多汉**字**难以归部
    
## Example 2

    Input: bin/radicalgrep 氵_子_ ../QA/radicaltest/testfiles/test1
    Output: 部首分类也是使用**汉字**之文化圈少数的共通点
    部首检字也有其局限性，许多**汉字**难以归部
   
## Example 3

    Input: bin/radicalgrep 子_子_ ../QA/radicaltest/testfiles/test1
    Output: Can not find the results!

###### ** output is in a different color. **


Future Improvements Include:

1. Can use two modes to search, one is searched by kangxi radical indices and the other is searched by actual kangxi radical.
2. Add more functions to use this radicalgrep.

