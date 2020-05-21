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
The Radical Count program uses the kRSKangxi property to distinguish all 214 radicals in the [Kangxi Radical System](https://en.wikipedia.org/wiki/Kangxi_radical). In `kRSKangxi.h`, we have a unicode set for each radical, where each set contains the codepoint ranges for the Chinese characters with the corresponding radical. We generated this header file by using [unihan-scripts](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/tree/delta-radicalgrep/unihan-scripts), based off of [Unihan_RadicalStrokeCounts.txt](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/blob/delta-radicalgrep/unihan-scripts/Unihan/Unihan_RadicalStrokeCounts.txt) of the Unihan database.

## **radical_interface.h & radical_interface.cpp**

The `radical_interface` files defines the namespace `BS` and the corresponding functions and variables related to radical grep.

Members of class `UnicodeSetTable`:

* `map<string, const UCD::UnicodeSet*>_unicodeset_radical_table`:  
This is a map that lists all 214 radicals and their corresponding Unicode set there were predefined from [kRSKangXi.h](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/blob/delta-radicalgrep/include/unicode/data/kRSKangXi.h). This is not used in the current iteration, but will be implemented later on.    

* `map<string, const UCD::UnicodeSet*> radical_table`:
Instead of using a numeric key, the actual Kangxi radical is used and mapped to their corresponding values. Note that one unicode set may belong to different radicals (e.g. 水 and 氵both map to set 85).

* `get_uset()`:
This function maps the inputted radical to the corresponding UnicodesSet predefined in [kRSKangXi.h](https://cs-git-research.cs.surrey.sfu.ca/cameron/parabix-devel/blob/delta-radicalgrep/include/unicode/data/kRSKangXi.h).
 
Members of class `RadicalValuesEnumerator`:
* `parse_input()`:
This function parses the inputted radical expression (e.g. 氵_ or 氵_子_ ) and stores it in a vector `radical_list`. The variable `radical_num` Store the number of inputted radical(s).

* `createREs()`:
This function finds the inputted radical from `radical_list`, and searches for it by invoking `get_uset()`.

## **radicalgrep.cpp**

This file is the main framework of Radical Grep. The LLVM input parser takes in two arguments; the radical expression and the filepaths(s).

### The Running Process
    1. Get the input and the file(s) to be searched. 
    2. Analyze the input to get the corresponding radical Unicode set(s).  
    3. Search each file. 
    4. Recolour the matching words in the sentence and output the result.     

### Variables
* `input_radical`: The inputted radical expression.

* `inputfiles`:  The file(s) that will be searched. 

* `allfiles`: Stores the filepaths. When a file has finished being processed, it gets popped from the vector so that a new file can be looked at. 

* `radicalREs`: Stores the return value of `generateREs()`.

### Functions
* `generateREs(std::string input_radical)`: This function parse the input and gets the REs by invoking `createREs()`.

* `setColoring()`: Defined in file `grep_engine.cpp`; it changes the terminal's text colour to red for the characters with the corresponding radicals.

* `initFileResult(allfiles)`: Defined in file `grep_engine.cpp`;  this is a construction and initialization function. Like the name suggests, it initializes the input paths, path size and so on.  

* `initREs(radicalREs) `: Defined in file `grep_engine.cpp`; this is also a construction and initialization function. It takes care of all Unicode related tasks of the REs provided by `generateREs()`. 

* `grepCodeGen()`: Defined in file `grep_engine.cpp`; this is the main function of Radical Grep. It is a code generation function; which returns the number of equivalent characters found.

* `searchAllFiles()`: Defined in file `grep_engine.cpp`; this function searches all the files for matches at the same time. If there results have been found, it returns true. Else, it returns false.


**Authored by Team Delta:** Anna Tang, Lexie Yu (Yu Ruo Nan),  Pan Chu Wen

**Last Updated:** 2020/05/20
