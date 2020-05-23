# PinYin Grep
Pinyin grep is a Chinese text grep built using the icgrep engine. The program takes pinyin input as regular characters, characters followed by a number, or as a regular expression, and would return the corresponding Chinese character. 

## Installation
In order to build pinyin grep, icgrep would have to be already built.\
Then the following command could be used:
```
python greptest.py -t pinyin_test.xml ../build/bin/pinyin_grep
```
After a successful run, it is complete.

## Iteration 1: Initial Prototype
The program takes a specific pinyin input and outputs the corresponding Chinese characters according to previously made test cases.

### Testcase 1.1 -- *Regular Romanisation*
Inputs Latin alphabets and returns all corresponding lines of Chinese characters.

```
Input: 
	wan le ../QA/pinyin_test/testfiles/T1_pinyin
Output:
	玩乐都没时间
	写完了去睡觉
```

### Testcase 1.2 -- *Romanisation with Tone Numbers*
Input Latin alphabets followed by a number (`1`, `2`, `3`, `4`), and returns all corresponding lines of Chinese characters.

```
Input: 
	zhong1 yao4 ../QA/pinyin_test/testfiles/T1_pinyin
Output:
	喝中药一定要吃山楂饼
```

### Testcase 1.3 -- *Romanisation with Tone Marks*
Input Latin alphabets marked with the tone (`ˉ`, `ˊ`, `ˇ`, `ˋ`), and returns all corresponding lines of Chinese characters.

```
Input: 
	wán ../QA/pinyin_test/testfiles/T1_pinyin
Output:
	玩乐都没时间
	写完了去睡觉
```

### Testcase 1.4 -- *Regular Expression*
Input a regular expression, and return all corresponding lines of Chinese characters.\
`ch.ng` corresponds to `chang`, `cheng`, `ching`, and `chong`.

```
Input: 
	m.ng ../QA/pinyin_test/testfiles/T2_regex
Output:
	这几天太忙了，
	睡眠不足，没有梦想。
	我听说明天会很晴朗，也许明天会更好。
```


## Iteration 2: Enhanced Implementation 
Iteration 2 is suppose to deliver enhanced implementation. Similar to the previous iteration, the program is suppose to be able to read an input and output the corresponding lines.

### Testcase 2.1 -- *Multiple Files*
Input pinyin and the intended files, then return all corresponding lines of Chinese Characters

```
Input: 
	yao4 ../QA/pinyin_test/testfiles/T1_pinyin ../QA/pinyin_test/testfiles/T3_pinyin
Output:
	这列子可能是重要
	想要好成绩，
```


