# PinYin Grep
Pinyin grep is a Chinese text grep built using the icgrep engine. The program takes pinyin input as regular characters, characters followed by a number, or as a regular expression, and would return the corresponding Chinese character. 

## Installation
In order to build pinyin grep, icgrep would have to be already built.
From the `QA` directory run the following command to generate the required test files.
```
python greptest.py -t pinyin_test/pinyin_test.xml ../build/bin/pinyin_grep
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
1. Tone Number 
2. Regular Expression
3. Multiple Input Files 
4. Sequence Search

## Iteration 3: Final Product
A polished version of iteration 2 with added features. 
Features added in this iteration includes:
1. Display Error Message when Input is not in the correct format.
2. Option to Display Grep line number of matched characters
3. Display File Path for Matched Characters in Multi File Input
4. Case Insensitive 
5. Regular Expression -- `[]`
6. Loose Matching 
7. Colorization \
7.1 Places emphasis the searched word(s) by colouring it red. 
8. Indexing \
8.1 Search for a Chinese character by using the unicode value \
8.2 Option to input a Hex (`3442`) or a Dec (`13378`) match a chinese character (`仿`)
9. Database Selection \
9.1 An option to choose between databases: `kHanyuPinyin`, `kXHC1983`
10. Simplified & Traditional Variants \
10.1 Input using *simplified*, *traditional* or *both* 
11. Definitions \
11.1 Displays the definition of searched word(s) \
11.2 using database kDefinitions, it would print the line of definition after the printed matched grep lines. 

---

### Testcases:
#### Testcase 1.1 -- *Regular Romanisation*
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

#### Testcase 1.2 -- *Romanisation with Tone Numbers*
Input Latin alphabets followed by a number (`1`, `2`, `3`, `4`)

```
Command:
	bin/pinyin_grep zhong1 ../QA/pinyin_test/testfiles/T1_pinyin
Input: 
	zhong1
Output:
	喝中药一定要吃山楂饼
```

#### Testcase 1.3 -- *Romanisation with Tone Marks*
Input Latin alphabets marked with the tone (`ˉ`, `ˊ`, `ˇ`, `ˋ`)

```
Command:
	bin/pinyin_grep shuì ../QA/pinyin_test/testfiles/T1_pinyin
Input: 
	shuì
Output:
	写完了去睡觉
```

#### Testcase 1.4 -- *Regular Expression (`.`)* 
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

#### Testcase 1.5 -- *Regular Expression (`?`)* 
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

#### Testcase 2.1 -- *Multiple Files*
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
#### Testcase 2.2 -- *Sequence Search*
Input multiple pinyin, using quotation marks (`"<word1>` `<word2>"`)

```
Command:
	bin/pinyin_grep "shui jiao" ../QA/pinyin_test/testfiles/T1_pinyin
Input: 
	shui jiao
Output:
	写完了去睡觉
```
#### Testcase 2.3 -- *Mixed*
```
Command:
	bin/pinyin_grep m.ng?4 ../QA/pinyin_test/testfiles/T2_regex
Input: 
	m.ng?4
Output:
	每天慢慢做作业。
	睡眠不足，没有梦想
```
#### Testcase 3.1 -- *Display Error Message when Input is not in the Correct Format*
#### Testcase 3.2 -- *Option to Display Grep Line Number for Matched Characters*
#### Testcase 3.3 -- *Display File Path for Matched Characters in Multi File Input*
#### Testcase 3.4 -- *Case Insensitive*
`SHANG`, `Shang`, `ShAnG`, and `shang` would all have the same results
#### Testcase 3.5 -- *Loose Matching*
Allows for a more general search for pinyin pairs such as `zh` and `z`
#### Testcase 3.6 -- *Regular Expression -- `[]`*
`[zc]hang` seaches for `zhang` and `chang`
#### Testcase 3.7 -- *Colorization*
Input `-c auto` right before pinyin input, to select the option for coloured pinyin
```
Command:
	bin/pinyin_grep -c auto wán ../QA/pinyin_test/testfiles/T1_pinyin
Input: 
	(colourful) wán
Output:
	**玩**乐都没时间
	写**完**了去睡觉
```
#### Testcase 3.8 -- *Indexing*
#### Testcase 3.9 -- *Database Selection*
#### Testcase 3.10 -- *Definitions*
