# Implentation Guide

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
1. Use the RE Parser to process the preprocceed string into an `re::RE *`
1. Call the Parabix InternalSearchEngine to search through a buffer containing `"Unihan_Readings.txt"`, for lines matching `re::RE`
1. Make the search result another  `CC` and Call the Parabix InternalSearchEngine again to search the files 
1. Get the search result


### 2. `pinyin.h`
Declaration of Namespace `PinyinPattern`, as well as its associated classes and methods:
- `Buffer`
Buffer is used to read in and store the file `"Unihan_Readings.txt"`.
- `PinyinSetAccumulator`
A subclass of `grep::MatchAccumulator`, it's purpose is to obtain the codepoints of the matched lines, store it as a `codepoint_t`(`int32`) in the UnicodeSet.

###2. `pinyin.cpp`
Description of the function used to parse the input syllables
