# PinYin Grep
Pinyin grep is a Chinese text grep built using the icgrep engine. The program takes pinyin input as regular characters, characters followed by a number, or as a regular expression, and would return the corresponding Chinese character. 

## Installation
In order to build pinyin grep, icgrep would have to be already built.
From the `QA` directory run the following command to generate the required test files.
```
python greptest.py -t pinyintest/pinyintest.xml ../build/bin/pinyingrep
```

## Execution
From the `build` directory, run `cmake ..` followed by `make pinyin_grep` to compile the project. 
Then run the project using a command of the following format.
```
bin/pinyin_grep <pinyin input> <path of input file> 
```
After a successful run, it is complete.

## Iteration 1: Initial Prototype
The program takes a specific pinyin input and outputs the corresponding Chinese characters according to previously made test cases.
In this iteration, there is a specific range of input implemented, just so it is enough to pass specific test cases.


## Iteration 2: Enhanced Implementation 
Iteration 2 is suppose to deliver enhanced implementation. Similar to the previous iteration, the program is suppose to be able to read an input and output the corresponding lines.\
Changes made in this iteration includes:
##### 1. Tone Number
Support for pinyin input using numbers to signify tone. 
- `ni3` `hao3`
##### 1. Regular Expressions
Support for pinyin using regular expressions.
- `qing?`
- `m.ng`
##### 1. Multiple Input
Support for multiple pinyin input.
- `"ma fan"`


## Iteration 3: Final Product

---

### Testcases:
#### Testcase 1 -- *Regular Romanisation*
Inputs Latin alphabets

```
Command:
	bin/pinyin_grep wan ../QA/pinyin_test/testfiles/T1_pinyin
Input: 
	wan
Output:
	玩乐都没时间
	写完了去睡觉
```

#### Testcase 2 -- *Romanisation with Tone Numbers*
Input Latin alphabets followed by a number (`1`, `2`, `3`, `4`)

```
Command:
	bin/pinyin_grep zhong1 ../QA/pinyin_test/testfiles/T1_pinyin
Input: 
	zhong1
Output:
	喝中药一定要吃山楂饼
```

#### Testcase 3 -- *Romanisation with Tone Marks*
Input Latin alphabets marked with the tone (`ˉ`, `ˊ`, `ˇ`, `ˋ`)

```
Command:
	bin/pinyin_grep shuì ../QA/pinyin_test/testfiles/T1_pinyin
Input: 
	shuì
Output:
	写完了去睡觉
```

#### Testcase 4 -- *Regular Expression (`.`)* 
Input a regular expression
`ch.ng` corresponds to `chang`, `cheng`, `ching`, and `chong`.

```
Command:
	bin/pinyin_grep m.ng ../QA/pinyin_test/testfiles/T2_regex
Input: 
	m.ng
Output:
	这几天太忙了，
	睡眠不足，没有梦想。
	我听说明天会很晴朗，也许明天会更好。
```

#### Testcase 5 -- *Regular Expression (`?`)* 
Input a regular expression
`cheng?` corresponds to `cheng`, and `chen`.
```
Command:
	bin/pinyin_grep qing? ../QA/pinyin_test/testfiles/T2_regex
Input:
	qing?
Output:
	没时间见亲人，
	我听说明天会很晴朗，也许明天会更好。

```


#### Testcase 6 -- *Multiple Files*
Input pinyin and the intended files, (`<file path 1>` `<file path 2>`)

```
Command: 
	bin/pinyin_grep yao4 ../QA/pinyin_test/testfiles/T1_pinyin ../QA/pinyin_test/testfiles/T3_pinyin
Input: 
	yao4
Output:
	这列子可能是重要
	想要好成绩，
```
#### Testcase 7 -- *Sequence Search*
Input multiple pinyin, using quotation marks (`"<word1>` `<word2>"`)

```
Command:
	bin/pinyin_grep "shui jiao" ../QA/pinyin_test/testfiles/T1_pinyin
Input: 
	shui jiao
Output:
	写完了去睡觉
```

