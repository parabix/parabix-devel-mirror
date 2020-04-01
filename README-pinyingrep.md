### Pinyin Grep
This is a grep tool, using Chinese pinyin to "grep" corresponding Chinese characters/phases.
It can also support simple regular-expression-like features, like:
`pinyingrep <regex> <file>`
See below for more examples.
#### First Iteration
In the first iteration, the pinyin grep is supposed to handle pinyin inputs of English letters with possible tones and regular-expression-like features.
1. Simple Pinyin Inputs 
* Input: "zhong yao"
* Output: "grep" the lines with "zhong yao", "重要", "中药" and other possible combination of Chinese characters, whose pinyin is "zhong yao".
2. Pinyin with tones specified by \[1-4\]
* Input: "zhong1 yao4"
* Output: "grep" the lines with "中药" and other other possible combination of Chinese characters, whose pinyin is "zhong1 yao4".
3. Pinyin with tones specified by Latin expension.
* 

4. Regular expression feature -- `.`
* Input: “m.ng” .
* Equivalent to "mang|meng|ming|mong".
5. Regular expression feature -- `?`
* Input: “qing?” Equivalent to "qin|qing".
* "?" is only for the last "g". Equivalent to "qin|qing".

6. Use of space character in pinyin, " "
* eg. One space character: "zhong yao" matches "重要","中要","中药", or any other combination with pinyin of "zhong" and "yao", with no space between the characters.
* Two space characters: "zhong  yao" (two spaces between zhong and yao) matches "重 要","中 要","中 药" or any other combination with pinyin of "zhong" and "yao" and one space character between the two chinese characters.


