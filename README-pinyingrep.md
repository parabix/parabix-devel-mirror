Pinyin Grep
===========
This is a grep tool, using Chinese pinyin to "grep" corresponding Chinese characters/phases.  
It can also support simple regular-expression-like features, like:
`pinyingrep <regex> <file>`
See below for more examples.  

How to Run Pinyin Grep
------------------
To build Pinyin Grep, the working environment needs to have all requirements of icgrep build met.  
To run Pinyin Grep for the first iteration, run the following commends on your terminal:
		python greptest.py -v -t pinyintest.xml ../build/bin/pinyingrep

First Iteration
---------------
In the first iteration, the pinyin grep is supposed to handle pinyin inputs of English letters with possible tones and regular-expression-like features.
#### 1. Simple Pinyin Inputs 
* Input: "zhong yao"
* Output: "grep" the lines with "zhong yao", "重要", "中药" and other possible combination of Chinese characters, whose pinyin is "zhong yao".
		Test
		Input: zhong wen
		Output: 
		欢迎来到中文世界。
		在这里你可以感受中文的博大精深。
#### 2. Pinyin with tones specified by \[1-4\]
* Input: "zhong1 yao4"
* Output: "grep" the lines with "中药" and other other possible combination of Chinese characters, whose pinyin is "zhong1 yao4".
		Test
		Input: zhong1 yao4
		Output: 
		中药主要用于轻症患者。
		所以把筐子中要卖的药都烧了。
#### 3. Pinyin with tones specified by Latin expension.
* Input: "zhòng yào"
* Output: "grep" the lines with "重要" and other possible combination of Chinese characters, whose pinyin is "zhòng yào".
		Test
		Input: zhong4 yao4
		Output: 
		重要的事情说三遍：
		种药的人觉得现在卖不出好价钱，
#### 4. Regular expression feature -- `.`
* Input: “m.ng” .
* Equivalent to "mang|meng|ming|mong".
		Test
		Input: m.ng
		Output: 
		我名字叫小明,家里在上海。
		我的叔叔是盲的，今天我要忙着带他去看医生。
		明天我会去打篮球。
		我有个梦想做医生。
		看了我叔叔收到的歧视，我想帮盲人。
#### 5. Regular expression feature -- `?`
* Input: “qing?” Equivalent to "qin|qing".
* "?" is only for the last "g". Equivalent to "qin|qing".
		Test
		Input: kang?
		Output: 
		看了我叔叔收到的歧视，我想帮盲人。
		虽然他们不能康复，但可以变更好。
#### 6. Use of space character in pinyin, " "
* eg. One space character: "zhong yao" matches "重要","中要","中药", or any other combination with pinyin of "zhong" and "yao", with no space between the characters.
* Two space characters: "zhong  yao" (two spaces between zhong and yao) matches "重 要","中 要","中 药" or any other combination with pinyin of "zhong" and "yao" and one space character between the two chinese characters.  
Second Iteration
----------------


