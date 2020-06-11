# README-radicalgrep

Radical Grep is a tool built off of icgrep. It searches for the given Chinese radicals, and returns the line(s) that correspond with the input. Note that radicals will be processed according to the Kangxi Radical-Stroke system. **If you are using index mode or mixed mode, please refer to the "No." column of this [list](https://www.yellowbridge.com/chinese/radicals.php) for reference.**

## **Table of Contents**

* [Introduction](#Introduction)
* [Installation](#Installation)
* [How to Run Radical Grep](#How%20to%20Run%20Radical%20Grep)
* [Supported Features](#Supported%20Features)
* [Iteration 1](#Iteration%201:%20Hard-Coding%20the%20Testcases%20into%20the%20Program)
* [Iteration 2](#Iteration%202:%20Radical%20Count%20&%20Grep%20Implementation)
* [Iteration 3](#Iteration%203:%20Adding%20New%20Features%20&%20Final%20Iteration)
* [References](#References)


## **Introduction**
The 214 Kangxi radicals are sorted in increasing order by stroke count. Originally introduced in 1615, many modern Chinese dictionaries still use the Kangxi system. In our program, we used the Unihan `kRSKangxi` property to generate Unicode sets for all 214 radicals. One important key to note is that some radicals may have the same index. For instance, characters 火 and 灬 are both the 86th radical of the dictionary and would map to the same Unicode set. Thus, 灯 and 点 would be characters in the same set.

For more information on the Kangxi Radical System, please visit: https://en.wikipedia.org/wiki/Kangxi_radical or https://www.yellowbridge.com/chinese/radicals.php



## **Installation**

To build only Radical Grep and it's dependencies, run `make radicalgrep` in the `build` directory. For testfiles and `radicaltest`, run `make radicaltest` in the `build` directory.

## **How to Run Radical Grep**

To run Radical Grep, run the following commands in the build/bin directory:

    ./radicalgrep [OPTIONS] <Radical Expression> <Path(s) of Input File>

For sample testcases, please refer to [radicaltest.xml](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/blob/delta-radicalgrep/QA/radicaltest/radicaltest.xml).

## **Supported Features**

* Single Radical Expressions 
    > e.g.  亻_
* Radical Expressions of >= 2 Radicals
    > e.g.  亻_心_ , 禾_扌_子_ 
* `-i` index mode
    > e.g. 85_
* `-m` mixed mode
    > e.g. 水_143_
* `-alt` alternative character mode 
    > e.g. 水_{火/水}_
* `-c` colourization
    > Colourization is turned off by default. 
    > To turn on colourization, include `-c auto`
* `-h` manually toggle file path to be shown for each matching line
    > This is turned on automatically each time when > 1 files are inputted
* `-n` shows the line number along with each match
* Single and multi-file input

## **Iteration 1: Hard-Coding the Testcases into the Program**

In the first iteration, Radical Grep takes in pre-programmed inputs and returns the sentence(s) with the corresponding pattern.

 ## Example 1

    Input:  亻_心_ ../../QA/radicaltest/testfiles/test1
    
    Output: 以下是一些关于部首分类的信息

## Example 2

    Input:  氵_宀_ ../../QA/radicaltest/testfiles/test1
    
    Output: 这是采用“两分法”对汉字进行结构分析得出的认识


## **Iteration 2: Radical Count & Grep Implementation**

In the second iteration, Radical Grep can take Kangxi radical character(s) as input (e.g. 子_ or 氵_子_). It returns the sentence with the correspondings radicals marked in red text. Iteration 2 of Radical Grep can be run using the same input format as iteration 1.

Another program, `Radical Count` was implemented in this iteration. The documentation can be found [here](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/blob/delta-radicalgrep/README-radicalcount.md) and the program can found in parabix-devel/tools/wc/radical_count.

## Changelog

### Radical Grep Version 2.1.0 
* Radical Grep takes the Kangxi radical indices (e.g. 85_85_ as 氵_氵_) as input. 
* Radical Grep supports multi-file search.
### Radical Grep Version 2.1.1
* Output added for the case where no match is found.
### Radical Grep Version 2.2.0
* Functionality to perform the search using the actual radicals (e.g. 氵_氵_) and not the Kangxi indices is added. 
* Functionality to use traditional radicals to search in a file with simplified characters and vice versa is added.
### Radical Grep Version 2.2.1 
* Functionality for single radicals (e.g. 氵_) is now supported.

## Example 1: Single Radical Input

    Input: 子_ ../../QA/radicaltest/testfiles/test1
    
    Output: 这是一个简单的例**子**
            部首分类也是使用汉**字**之文化圈少数的共通点
            部首检字也有其局限性，许多汉**字**难以归部

## Example 2: Radical Phrase Input

    Input: 氵_子_ ../../QA/radicaltest/testfiles/test1
    
    Output: 部首分类也是使用**汉字**之文化圈少数的共通点
            部首检字也有其局限性，许多**汉字**难以归部

## Example 3: Radical Pattern Not Found in File 

    Input: 子_子_ ../../QA/radicaltest/testfiles/test1
    
    Output: Can not find the results!

## Example 4: Single Radical Input and Multiple Files

    Input: 子_ ../../QA/radicaltest/testfiles/test1 ../../QA/radicaltest/testfiles/test2
    
    Output: 这是一个简单的例**子**
            部首分类也是使用汉**字**之文化圈少数的共通点
            部首检**字**也有其局限性，许多汉**字**难以归部
            偏旁是从造**字**构形的角度定义的
    
            这是采用“两分法”对汉**字**进行结构分析得出的认识
            由于汉**字**结构复杂，许多汉**字**并不是左右结构的
            排列在一起，并把这种排**字**方法叫做“分别部居”
            每“部”第一个**字**就是“部首”
            可见，部首也是偏旁，是用来作为排列和检索汉**字**依据的特殊“偏旁”

## Example 5: Radical Phrase Input and Multiple Files

    Input: 氵_子_ ../../QA/radicaltest/testfiles/test1 ../../QA/radicaltest/testfiles/test3
    
    Output: 部首分类也是使用**汉字**之文化圈少数的共通点
            部首检字也有其局限性，许多**汉字**难以归部
    
            部首分類也是使用**漢字**之文化圈少數的共通點
            部首檢字也有其局限性，許多**漢字**難以歸部

## Example 6: Radicals With the Same Index (火 and 灬)

As mentioned before, it is possible to have more than one radical with the same index. In this example, 火 and 灬 are both radical 86 in the Kangxi dictionary. Similar cases include 氵and 水, as well as 忄and 心.

    Input: 火_灬_ ../../QA/radicaltest/testfiles/test4
    
    Output: 今天天气**炎热**
    
    Input: 火_火_ ../../QA/radicaltest/testfiles/test4
    
    Output: 今天天气**炎热**

## Example 7: Words that are Radicals Themselves

Some characters such as 土, 火, 水, 木, and 金 are radicals themselves. They still fall into their respective set, and instead have no residual strokes.

    Input: 土_ ../../QA/radicaltest/testfiles/test5
    
    Output: 古人把宇宙万物根据其特征划分成火、水、木、金、**土**五大类，统称“五行”
            **在**中医学中，五行有着特殊含义
            “金曰从革”，代表沉降、肃杀、收敛等性质，**在**人体为肺和大肠
            “水曰润下”，代表了滋润、下行、寒凉、闭藏的性质，**在**人体为肾和膀胱
            “木曰曲直”，代表生长、升发、条达、舒畅的功能，**在**人体为肝和膽
            “火曰炎上”，代表了温热、向上等性质，**在**人体为心和小肠
            “土曰稼穑”，代表了生化、承载、受纳等性质，**在**人体为脾和胃

## Example 8: Kangxi Radical Designation

In the Kangxi dictionary, every Chinese character only has one desginated radical. For instance 伙 is composed of 亻and 火, which are two commonly seen radicals. According to the Kangxi dictionary, the official radical of 伙 is 亻.

    Input: 亻_ ../../QA/radicaltest/testfiles/test6
    
    Output: **伙**
    
    Input: 火_ ../../QA/radicaltest/testfiles/test6
    
    Output: **烛**
    
    Input: 虫_ ../../QA/radicaltest/testfiles/test6
    
    Output: Can not find the results!

## Example 9: Search for Simplified Characters with Traditional Radicals (Vice Versa)

If the file to be searched is written in simplified Chinese and the radical expression is in traditional Chinese, it will still return the corresponding characters. This works the other way around too.

    Input: 言_ ../../QA/radicaltest/testfiles/test1
    
    Output: 中国大陆的国标码使用汉**语**拼音排列
            部首检字也有其局限性，**许**多汉字难以归部
    
    Input: 讠_ ../../QA/radicaltest/testfiles/test3
    
    Output: 中國大陸的國標碼使用漢**語**拼音排列
            部首檢字也有其局限性，**許**多漢字難以歸部

###### ** Output is printed in red on the terminal. Colorization was turned on for this iteration.** 

## **Iteration 3: Adding New Features & Final Iteration**
Plans for iteration 3 include:

1. ~~Implement switch between two search modes, users can choose any search mode; kangxi radical indices and actual kangxi radical.~~ **(Done)**
2. ~~Fix colorization issue.~~ **(Done)**
3. ~~Implement a search mode which allows for both index and kangxi radical. (e.g. `水_143_`)~~ **(Done)**
4. ~~Implement a search mode like this `水_{火/水}_`.~~**(Done)**
5. ~~Add support for input with >2 radicals.~~ **(Done)**
6. ~~Graphical interface~~ Time is not in our favour :( 

## Changelog

### Radical Grep Version 3.0
 * Radical Grep now supports both Kangxi radical index and radical characters for input. (i.e. 85_ and 氵_).
 * The default searching mode is with the radical characters. To use the radical indices, include the option flag `-i` in your input.
 * Functionality added to catch all incorrect input (e.g. any non-numeric input or non-existing radical set).

### Radical Grep Version 3.1
* Radical Grep's default output has colorization turned off. You can turn it back on by using the option flag `-c auto` in your input.
* Input with 3 or more radicals is now supported.

### Radical Grep Version 3.2

* Input with both radical characters and radical indices is now supported. To use this function, include the option flag `-m` in your input.

### Radical Grep Version 3.3

* `-alt` flag functionality added, for instance `亻_衣_{生/亅}_` will search for `亻_衣_生_` and `亻_衣_亅_`.

### Radical Grep Version 3.3.1

* Add two versions of `-alt` flag functionality input, one is the alternative part is at the beginning and the other is the alternative part at anywhere except the beginning and the end. 

  For instance, `{火/水/土}_曰_` will search for `火_曰_`, `水_曰_` and `土_曰_`; `水_{火/水/人}_曰_` will search for `水_火_曰_`, `水_水_曰_` and `水_人_曰_`.

* `-n` flag added. If toggled, it will indicate the line number along with the line. 

* File path is displayed for inputs containing > 1 test files.
* Error message for invalid input added.
* A link for the chart of indices is included in the `-help` menu for `-i`.

## Example 1: Kangxi Radical Index Search Mode and Colorization Turned On

    Input: -i -c auto 85_ ../../QA/radicaltest/testfiles/test1
    
    Output: 部首分类也是使用**汉**字之文化圈少数的共通点
            中国大陆的国标码使用**汉**语拼音排列
            部首检字也有其局限性，许多**汉**字难以归部

## Example 2: Incorrect Input

If the user enters uses Radical Grep in index mode and searchs for index 0 or any number greater than 214, they will get an error message because these sets do not exist in the Kangxi dictionary.

    Input: -i a_ ../../QA/radicaltest/testfiles/test1
    
    Output: LLVM ERROR: A radical set for this input does not exist.
            Enter a integer in [1,214], followed by _.
    
    Input: -i 1_215_ ../../QA/radicaltest/testfiles/test1
    
    Output: LLVM ERROR: A radical set for this input does not exist.
            Enter a integer in [1,214], followed by _.

    Input: 检_ ../../QA/radicaltest/testfiles/*

    Output: LLVM ERROR: A radical set for this input does not exist.

## Example 3: Input with more than 3 Radicals

    Input: 禾_扌_子_ ../../QA/radicaltest/testfiles/*
    
    Output: 排列在一起，并把这种排字方法叫做“分别部居”
    
    Input: -i 115_64_39_ ../../QA/radicaltest/testfiles/*
    
    Output: 排列在一起，并把这种排字方法叫做“分别部居”

## Example 4: Input with both radical characters and radical indices

    Input: -m 禾_扌_39_ ../../QA/radicaltest/testfiles/*
    
    Output: 排列在一起，并把这种排字方法叫做“分别部居”

## Example 5: Input with Alternative Characters 

The first input searches for the radical expressions 水_火_ and 水_水_, and the second input utilizes the indices for the same radicals. The third input searches for the radical expressions 火_曰_, 水_曰_ and 土_曰_. The forth input searches for the radical expressions 曰_水_曰_, 曰_火_曰_ and 曰_土_曰_.

    Input: -alt -c auto 水_{火/水}_ ../../QA/radicaltest/testfiles/*
    
    Output: 喝着**汽水**
            “水曰润下”，代表了**滋润**、下行、寒凉、闭藏的性质，在人体为肾和膀胱
            “火曰炎上”，代表了**温热**、向上等性质，在人体为心和小肠
    
    Input: -alt -c auto -m 水_{86/85}_ ../../QA/radicaltest/testfiles/*
    
    Output: 喝着**汽水**
            “水曰润下”，代表了**滋润**、下行、寒凉、闭藏的性质，在人体为肾和膀胱
            “火曰炎上”，代表了**温热**、向上等性质，在人体为心和小肠
           
    Input: -alt -c auto {火/水/土}_曰_ ../../QA/radicaltest/testfiles/*
    
    Output: “**水曰**润下”，代表了滋润、下行、寒凉、闭藏的性质，在人体为肾和膀胱
            “**火曰**炎上”，代表了温热、向上等性质，在人体为心和小肠
            “**土曰**稼穑”，代表了生化、承载、受纳等性质，在人体为脾和胃
    				
    Input: -alt -c auto 曰_{火/水/土}_水_ ../../QA/radicaltest/testfiles/*
    
    Output: 有人**曰河海**一体
            有人**曰火水**不容
            有人**曰土水**相得益彰

## Example 6: Filepaths and Line Number 

For inputs with more than one test files, filepaths are automatically shown. You can manually toggle this feature with the `-h` flag. Line number can be toggled with the `-n` flag.

    Input: -h -n 曰_ ../../QA/radicaltest/testfiles/*

    Output: ../../QA/radicaltest/testfiles/test5:4:“金曰从革”，代表沉降、肃杀、收敛等性质，在人体为肺和大肠
    ../../QA/radicaltest/testfiles/test5:5:“水曰润下”，代表了滋润、下行、寒凉、闭藏的性质，在人体为肾和膀胱
    ../../QA/radicaltest/testfiles/test5:6:“木曰曲直”，代表生长、升发、条达、舒畅的功能，在人体为肝和膽
    ../../QA/radicaltest/testfiles/test5:7:“火曰炎上”，代表了温热、向上等性质，在人体为心和小肠
    ../../QA/radicaltest/testfiles/test5:8:“土曰稼穑”，代表了生化、承载、受纳等性质，在人体为脾和胃
    ../../QA/radicaltest/testfiles/test7:2:有人曰河海一体
    ../../QA/radicaltest/testfiles/test7:3:有人曰火水不容
    ../../QA/radicaltest/testfiles/test7:4:有人曰土水相得益彰

​    
## **References**
* [Unicode Standard Annex #38: Unihan](http://www.unicode.org/reports/tr38/)
* Unihan Database (Unihan.zip)
* [UCD-Scripts](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/tree/master/UCD-scripts)
* [icgrep](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/tree/master/tools/icgrep)

**Authored by Team Delta:** Anna Tang, Lexie Yu (Yu Ruonan),  Pan Chuwen

**Last Updated:** 2020/06/11
