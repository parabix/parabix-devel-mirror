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
│   │       pinyingrep.cpp
│   │       ...
│   ...

```

##File Utility 

### 1. `pinyingrep.cpp` 
The main function performs in the following order:
1. Interpret the LLVM Command Line parameter.
1. Use the RE Parser to process the preprocceed string into an `re::RE *`
1. Call the Parabix InternalSeachEngine to search through a buffer containing `"Unihan_Readings.txt"`, for lines matching `re::RE`
1. Use `CC` to account for all occurrences of matched codepoints
1. Sum up the total `CC` generated from the input files.

### 2. `pinyin.h`
Declaration of Namespace `PinyinPattern`, as well as its associated classes and methods:
- `Buffer`
Buffer is used to read in and store the file `"Unihan_Readings.txt"`.
- `PinyinSetAccumulator`
A subclass of `grep::MatchAccumulator`, it's purpose is to obtain the codepoints of the matched lines, store it as a `codepoint_t`(`int32`) in the UnicodeSet.