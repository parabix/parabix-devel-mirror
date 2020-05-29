# README-radicalcount

Radical Count is a program built based off of `ucount`. Given file(s) and the Kangxi radical expression, it counts the occurences of characters with the corresponding radical in the input file(s).

For more information on the Kangxi Radical System, please visit: https://en.wikipedia.org/wiki/Kangxi_radical or https://www.yellowbridge.com/chinese/radicals.php

## **Installation**

To build Radical Count, type `make radicalcount` into the terminal.

## **How to Run Radical Count**

 Go into the bin diretory and run the following command to use Radical Count.

    ./radicalcount <Radical Expression> <Path(s) of Input File(s)>

## Example 1: Single Radical Input

    Input: 氵_ ../../QA/radicaltest/testfiles/test1
    
    Output: 氵: 3       ../../QA/radicaltest/testfiles/test1

## Example 2: Double Radical Input

    Input: 氵_子_ ../QA/radicaltest/testfiles/test1
    
    Output:     氵: 3       ../../QA/radicaltest/testfiles/test1
                子: 4       ../../QA/radicaltest/testfiles/test1

## Example 3: Double Radical Input and Multiple Files

    Input: 氵_子_ ../../QA/radicaltest/testfiles/test1 ../QA/radicaltest/testfiles/test2
    
    Output: File 0:
                氵: 3       ../../QA/radicaltest/testfiles/test1
                子: 4       ../../QA/radicaltest/testfiles/test1
            File 1:
                氵: 7       ../../QA/radicaltest/testfiles/test2
                子: 7      ../ ../QA/radicaltest/testfiles/test2

            Total Count of 氵: 10
            Total Count of 子: 11

## Example 4: Radicals With the Same Index (火 and 灬)

As mentioned before, it is possible to have more than one radical with the same index. In this example, 火 and 灬 are both radical 86 in the Kangxi dictionary. Similar cases include 氵and 水, as well as 忄and 心. These just get counted together.

    Input: ./radicalcount 火_灬_ ../QA/radicaltest/testfiles/test4

    Output:     火: 2       ../../QA/radicaltest/testfiles/test4
                灬: 2       ../../QA/radicaltest/testfiles/test4

## Example 5: Words that are Radicals Themselves

Some characters such as 土, 火, 水, 木, and 金 are radicals themselves. They still fall into their respective set, and instead have no residual strokes.
    
    Input: 土_ ../../QA/radicaltest/testfiles/test5

    Output: 土: 8       ../../QA/radicaltest/testfiles/test5

## Example 8: Kangxi Radical Designation

In the Kangxi dictionary, every Chinese character only has one desginated radical. For instance 伙 is composed of 亻and 火, which are two commonly seen radicals. According to the Kangxi dictionary, the official radical of 伙 is 亻. 伙 would only be counted when 亻is being searched for, and not when 火.

    Input: 亻_ ../../QA/radicaltest/testfiles/test6

    Output: 亻: 1       ../../QA/radicaltest/testfiles/test6

## Example 9: Invalid Inputs in Index Mode

If the user enters uses Radical Grep in index mode and searchs for index 0 or any number greater than 214, they will get an error message because these sets do not exist in the Kangxi dictionary.

    Input: -i a_ ../../QA/radicaltest/testfiles/test1

    Output: LLVM ERROR: A radical set for this input does not exist.
            Enter a integer in [1,214], followed by _.

    Input: -i 1_215_ ../../QA/radicaltest/testfiles/test1

    Output: LLVM ERROR: A radical set for this input does not exist.
            Enter a integer in [1,214], followed by _.
    
###### *Inputs of 2 radicals returns the count of each radical, and not the number of consecutive occurences (i.e. phrases).

## **Future Improvements**
* Add functionality to count the number of consecutive occurences (i.e. phrases).


**Authored by Team Delta:** Anna Tang, Lexie Yu (Yu Ruo Nan),  Pan Chu Wen

**Last Updated:** 2020/05/28