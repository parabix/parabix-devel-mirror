Pinyin Grep
===========
This is a grep tool, using Chinese pinyin to "grep" lines with corresponding Chinese characters/phases in a list of files.  
It can also support simple regular-expression-like features, like:
`pinyingrep <regex> <file list>`
See below for more examples.  

How to Test Pinyin Grep
------------------
To build Pinyin Grep, the working environment needs to have all requirements of icgrep build met.  
To test pinyingrep, `cd` the directory `QA` and run the following commends on your terminal:
```
python greptest.py -v -t pinyintest.xml ../build/bin/pinyingrep
```
Also, `make check` is ready for testing all modules including pinyingrep.
## Iteration History
First Iteration
---------------
In the first iteration, the pinyin grep is supposed to handle pinyin inputs of English letters with possible tones and regular-expression-like features. It is implemented in a dummy way, simply to pass all test cases.
 

Second Iteration
----------------
In the second iteration, we have implemented **pinyingrep** version 1.0 with input parsing. In this version, **pinyingrep** is supposed to handle general regex-like pinyin syllables as input. The implementation of grep functionality is based on grep engine provided by parabix framework. We specified the requirement of our second iteration as follows:
### Functionality
#### 1. Pinyin syllables without tones specified
**Pinyingrep** supports pinyin syllables in alphabetic characters without tones sepcified.
> e.g. `zhong` or `gao`
>
As an exception of alphabetic characters in pinyin representation, `ü` is also supported as non-toned characters. `v` and `ü` are considered equivalent as un-toned input.

With such a form of input syllables, pinyingrep will 'grep' lines with Chinese characters with
readings specified by the pinyin syllables in all five tones(Tone `1-4` as well as Qingsheng/Soft/Neutral)
For instance, the result of grepping `zhong` is equivalent to those of grepping `zhong0`(Qingsheng), `zhong1`, `zhong2`, `zhong3` and `zhong4`.

#### 2. Pinyin syllables with tones specified by numbers
**Pinyingrep** supports pinyin syllables with tones specified by Arabic numbers(`0-4`).
> e.g. `zhong1` or `zho1ng`
>
We define that`0` indicates the reading is in `Qingsheng`.
#### 3. Pinyin syllables with tones specified by toned characters in unicode
**Pinyingrep** supports pinyin syllables with tones specified by toned characters in unicode, 
more specifically, latin characters like `ǎ` or `ō`.
> e.g. `xuǎn` or `xiōng`
>
Toned unicode syllables like `xuǎn` is equivalent to corresponding syllables with tones specified by Arabic numbers.
#### 4. Regular-expression-like pinyin syllables with `.`
**Pinyingrep** supports regular-expression-like pinyin syllables with `.`.
> e.g. `zh.ng` or `x..ng`
>
Similar to normal regular expressions, `.` represent arbitrary **alphabetic** characters.
But only legal pinyin syllables in the database will be considered by **pinyingrep**. 
> e.g. `x..ng` to `xiong`, `xiang`, and etc.
>
#### 5. Regular-expression-like pinyin syllables with `?` after "g"
**Pinyingrep** supports pinyin syllables with `?` after alphabetic character `g`.
> e.g. `zhang?`
>
The question mark indicates that both readings with `g` and without `g` are wanted as input.

#### 6. Sequences of pinyin syllables mentioned above
**Pinyingrep** supports sequences of the above legal syllables of arbitrary length.
> e.g. `zh.ng yao4` 
>
Any sequence of Chinese characters with readings matching the input sequences will be in result.
#### 7. Grep from a list of files
**Pinyingrep** supports one or more files as source files to grep from.
### Testcases
The following testcases give example about the functionality of **pinyingrep**.
#### 1. Simple Pinyin Inputs 
* Input: "zhong yao"
* Output: "grep" the lines with "zhong yao", "重要", "中药" and other possible combination of Chinese characters, whose pinyin is "zhong yao".
```
    Test
    Input: zhong wen
    Output:
    欢迎来到中文世界。
    在这里你可以感受中文的博大精深。
```
#### 2. Pinyin with tones specified by \[1-4\]
* Input: "zhong1 yao4"
* Output: "grep" the lines with "中药" and other other possible combination of Chinese characters, whose pinyin is "zhong1 yao4".
```		
    Test
    Input: zhong1 yao4
    Output: 
    中药主要用于轻症患者。
    所以把筐子中要卖的药都烧了。
```
#### 3. Pinyin with tones specified by Latin expension.
* Input: "zhòng yào"
* Output: "grep" the lines with "重要" and other possible combination of Chinese characters, whose pinyin is "zhòng yào".
```
    Test
    Input: zhong4 yao4
    Output: 
    重要的事情说三遍：
    种药的人觉得现在卖不出好价钱，
```
#### 4. Regular expression feature -- `.`
* Input: “m.ng” .
* Equivalent to "mang|meng|ming|mong".
```
    Test
    Input: m.ng
    Output: 
    我名字叫小明,家里在上海。
    我的叔叔是盲的，今天我要忙着带他去看医生。
    明天我会去打篮球。
    我有个梦想做医生。
    看了我叔叔收到的歧视，我想帮盲人。
```
#### 5. Regular expression feature -- `?`
* Input: “qing?” Equivalent to "qin|qing".
* "?" is only for the last "g". Equivalent to "qin|qing".
```
    Test
    Input: kang?
    Output: 
    看了我叔叔收到的歧视，我想帮盲人。
    虽然他们不能康复，但可以变更好。
```

Third Iteration(WIP)
----------------
In the third iteration, we will implement **pinyingrep** version 2.0 with additional functionalities. 

## Version History

#### Version 1.1
**Pinyingrep** version 1.1 supports:
1. CommandLine flag to switch on coloring of grep results.
2. CommandLine option to choose different pinyin database.
#### Version 1.0
**Pinyingrep** version 1.0 supports:
1. Pinyin syllables without tones specified, e.g. `zhong`
2. Pinyin syllables with tones specified by numbers, e.g. `zhong1` or `zho1ng`
3. Pinyin syllables with tones specified by toned characters in unicode, e.g. `xuǎn`
4. Regular-expression-like pinyin syllables with `.` e.g. `zh.ng`
5. Regular-expression-like pinyin syllables with `?` after "g" e.g. `zhang?`
6. Sequences of pinyin syllables above, e.g. `zhong yao4`
7. Grep from a list of files

**Pinyingrep** version 1.0  removes support of:
1. Extra empty space between sequences of pinyin syllables(in this version, pinyingrep considers one or more empty space to be equivalent separator).

## Work Assignments
The following lists the work assignments of iteration 2:

Yanwen: Program Structure Design, Database Header File Generation, `PinyinValuesParser` Implementation, `Pinyingrep.cpp` Implementation

Huke: Implementing Methods of `PinyinValuesTable` in `pinyin_interface.cpp`, Testing **Pinyingrep** thoroughly

Kirby: `PinyinValuesEnumerator` Implementation in `pinyin_interface.cpp`, Adding CommandLine flags

Wendy: Generating static member initialization of `PinyinValuesTable` and `UnicodeSetTable` in `kpy_unicodeset_table.cpp` and `kxhc_unicodeset_table.cpp`, Adding CommandLine flags