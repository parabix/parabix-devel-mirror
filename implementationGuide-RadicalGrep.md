# Implementation Guide-Radical Grep 

This document provides an overview of the Radical Grep implementation. Please review [README-radicalgrep](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/blob/delta-radicalgrep/README-radicalgrep.md) before reading on.

This guide only provides a simple explaination of the key concepts and data structures used. Please refer to the source code for more details and inline documentation as well.

## **File Directory**
To find the source code for Radical Count, go into `tools/radicalgrep`. The radicalgrep folder contains all the necessary files to build the program.

```
radicalgrep
├── CMakeLists.txt
├── radicalgrep.cpp 
├── radical_interface.cpp
├── radical_interface.h
```

`radicalgrep.cpp` is the main framework for the Radical Grep program. The auxilary functions and radical-set maps can be found in `radical_interface.h`.

### **kRSKangXi.h**
Radical Grep uses the kRSKangxi property to distinguish all 214 radicals in the [Kangxi Radical System](https://en.wikipedia.org/wiki/Kangxi_radical). In `kRSKangxi.h`, we have a unicode set for each radical, where each set contains the codepoint ranges for the Chinese characters with the corresponding radical. We generated this header file by using [unihan-scripts](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/tree/delta-radicalgrep/unihan-scripts), based off of [Unihan_RadicalStrokeCounts.txt](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/blob/delta-radicalgrep/unihan-scripts/Unihan/Unihan_RadicalStrokeCounts.txt) of the Unihan database.

## **radical_interface.h & radical_interface.cpp**

The `radical_interface` files defines the namespace `BS` and the functions and variables used in Radical Grep.

Members of class `UnicodeSetTable`:

* `map<string, const UCD::UnicodeSet*>_unicodeset_radical_table`:  
This is a map that lists all 214 radicals and their corresponding Unicode set there were predefined from [kRSKangXi.h](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/blob/delta-radicalgrep/include/unicode/data/kRSKangXi.h). Th

* `map<string, const UCD::UnicodeSet*> radical_table`:
Instead of using a numeric key, the actual Kangxi radical is used and mapped to their corresponding values. Note that one unicode set may belong to different radicals (e.g. 水 and 氵both map to set 85).

* `get_uset()`:
This function maps the inputted radical to the corresponding UnicodesSet predefined in `radical_table`. If the program is in index mode (`-i`), the function looks for the requested radical in `_unicodeset_radical_table` and checks if the input is valid. In mixed mode (`-m`), the functions searches for the radical in both tables. In the case of an invalid input, an error message will appear and terminate the program.
 
Members of class `RadicalValuesEnumerator`:
* `parse_input()`:
This function parses the inputted radical expression (e.g. 氵_ or 氵_子_ ) and stores it in a vector `radical_list`. In alt mode (`-alt`), further parsing must be done. Radicals that are not bounded by parentheses are put in storage buffer vectors `zi` and `zi2`. The radicals in the parentheses are sent to `reParse()` for further processing.

* `reParse()`:
This function in alt mode, and tokenizes the radicals that are bounded by the parenthesis. When given a radical expression of `X_Y_{A/B}_`, `reParse()` tokenizes {A/B} with '\' as the delimiter. `A` and `B` are pushed into the vector `reTemp`.

* `createREs()`:
This function takes the radicals that have been parsed from `parse_input()` and `reParse()`, and returns a vector `REs`. `REs` is a regular expression that represents the inputted radical expression, and contains "alt" nodes of each radical character that were retrieved from `radical_list`, `reTemp`, `zi` and `zi2`.

## **radicalgrep.cpp**

This file is the main framework of Radical Grep. The LLVM input parser takes in two arguments; the radical expression and the filepaths(s).

### The Running Process
    1. Get the input and the file(s) to be searched. 
    2. Analyze the input to get the corresponding radical Unicode set(s).  
    3. Search each file. 
    4. If colourization is on, recolour the matching words in the sentence and output the result. Else, just return the matching sentences as-is.   

### Variables
* `input_radical`: The inputted radical expression.

* `inputfiles`:  The file(s) that will be searched. 

* `allfiles`: Stores the filepaths. When a file has finished being processed, it gets popped from the vector so that a new file can be looked at. 

* `radicalREs`: Stores the return value of `generateREs()`.

* `matchFound`: Indicates if matching line(s) have been found in the files.

### Command Line Flags

* `indexMode`: Indicates if radical indices are being used.

* `mixMode`: Indicates if radical indices and radical characters are being used in conjunction.

* `altMode`: Indicates if alternative character options are provided.

* `Color`: Command options for colourization.

* `LineNumberOption`: Prints out the line count of the file for each outputted line.

* `WithFilenameOption`: Prints out file name for each outputted line.

* `CLKCountingOption`: Prints out the runtime of the search.

### Functions
* `generateREs(std::string input_radical)`: This function parse the input and returns the regular expression of `input_radical`.

* `setColoring()`: Defined in file `grep_engine.cpp`; it changes the terminal's text colour to red for the characters with the corresponding radicals.

* `initFileResult(allfiles)`: Defined in file `grep_engine.cpp`;  this is a construction and initialization function. Like the name suggests, it initializes the input paths, path size and so on.  

* `initREs(radicalREs) `: Defined in file `grep_engine.cpp`; this is also a construction and initialization function. It takes care of all Unicode related tasks of the regular expression provided by `generateREs()`. 

* `grepCodeGen()`: Defined in file `grep_engine.cpp`; this is the main function of Radical Grep. It generates the grep pipeline.

* `searchAllFiles()`: Defined in file `grep_engine.cpp`; this function searches all the files for matches at the same time. If there results have been found, it returns true. Else, it returns false.

**Authored by Team Delta:** Anna Tang, Lexie Yu (Yu Ruonan),  Pan Chuwen

**Last Updated:** 2020/06/12
