## Implementation Guide
### File Structure
```shell
$ tree
.
├── CMakeLists.txt
├── kpy_unicodeset_table.cpp
├── kxhc_unicodeset_table.cpp
├── pinyingrep.cpp
├── pinyin_interface.cpp
└── pinyin_interface.h

0 directories, 6 files
```
### File Utility
The following description is the brief summaries for the utility of each source file in **pinyingrep**. For more detailed information such as data structures and algorithms, please refer to the comments in each source file.
#### 1. `pinyingrep.cpp`
The main program of pinyingrep. It runs as the following:
* Interpret the command line input with LLVM
* Interpret the pinyin syllables input with classes defined  in `pinyin_interface` 
* Generate `re::RE` according to the input syllable sequence
* Call the grep engine in Parabix to grep the result in a list of files
#### 2. `pinyin_interface.h`
Declaration of classes used to interpret the pinyin syllables, including:
* `PinyinValuesParser`
    * Separate the input string containing multiple pinyin syllables into the sequence of individual syllable strings. And for each individual syllable, it parse it into `<{Syllables}, {Tones}>` pairs. `{Syllables}` and `{Tones}` here are two vectors containing all possible valid syllables or tones corresponding to the input.
* `ParserException`
    * Utilize the syntax error detection in Parser.
* `PinyinValuesEnumerator`
    * For each individual syllable, enumerate the pair of vector mentioned above into `{<syllable, tone>,...}` a vector of pairs. Each pair corresponds to one possible combination for this input syllable.
    * Create `re::RE` according to the enumerated result.
* `PinyinValuesTable`
    * Utilize the Parser to interpret the pinyin syllable
    * Containing information of legitimate pinyin syllables and mapping between a latin character and a pair of alphabet characters and tones.
* `UnicodeSetTable`
    * Map `<syllable, tone>` pairs to corresponding `Unicodeset` defined in header files.
#### 3. `pinyin_interface.cpp`
Define the class methods of `PinyinValuesParser`, `PinyinValuesEnumerator` and `PinyinValuesTable`. Also, the static member initialization of `PinyinValuesTable`.
#### 4. `kpy_unicodeset_table.cpp` and `kxhc_unicodeset_table.cpp`
Contain the static member initialization of `UnicodeSetTable`.