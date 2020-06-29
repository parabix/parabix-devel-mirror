# Implentation Guide
This document contains a brief description of important files, functions, and their relationships.

## File Structure
```
Project (Pinyin Grep)
│   README-pinyingrep.md
│   CMakeLists.txt
│
└── tools
│   └── ... pinyin_grep
│   │       CMakeLists.txt
│   │       pinyin.h
│   │       pinyin.cpp
│   │       pinyingrep.cpp
│   │       ...
│   ...

```

## File Utility 

### 1. `pinyingrep.cpp` 
The main function performs in the following order:
1. Interpret the LLVM Command Line parameter.
1. Use the RE Parser to process the preprocessed string into an `re::RE *`
1. Call the Parabix InternalSearchEngine to search through a buffer containing `"Unihan_Readings.txt"`, for lines matching `re::RE`
1. Make the search result another  `CC` and Call the Parabix InternalSearchEngine again to search the files 
1. Get the search result

#### 1.1 Command Line Flags
- Database
    - use `-data` to search the kXHC1983 database instead of kHanyuPinyin database 
- Color
    - Set the colorization of the output
    - use `-c always` to turn on colorization when outputting to a file and terminal
    - use `-c auto` to turn on colorization only when outputting to terminal
    - use `-c never` to turn off output colorization
- LineNumberOption
    - use `-n` to display the grep line number of each matching line
- WithFilenameOption
    - use `-h` to display the file path of each matching line
- IndexSearchingOption
    - use `-i` to use hexidemical unicode index for searching



### 2. `pinyin.h`
Declaration of Namespace `PinyinPattern`, as well as its associated classes and methods:
- `Buffer`
Buffer is used to read in and store the file `"Unihan_Readings.txt"`.
- `PinyinSetAccumulator`
A subclass of `grep::MatchAccumulator`, it's purpose is to obtain the codepoints of the matched lines, store it as a `codepoint_t`(`int32`) in the UnicodeSet. \
The functions used to parse the input syllables which include:
    - `Syllable_Parse`: \
        Parses the input and returns a string vector containing correct pinyin format.

    - `Add_kHanyuPinyin_fix`: \
        Format the input to search through the `kHanyuPinyin` database.

    - `Add_kXHC1983_fix`: \
        Format the input to search through `kXHC1983` databse.

    - `All_Alpha`: \
        Determines whether the input contains purely latin alphabets.

    - `check_legel`: \
        Determines whether the input is in the correct format.

    - `find_bracket`: \
        Finds `[]` in the input to expand its content -- used in *feature 5*.

    - `trim`:\
        Deletes extra spaces both before and after the string.


### 3. `pinyin.cpp`
- Description of the function used to parse the input syllables and the possible syllable table.
