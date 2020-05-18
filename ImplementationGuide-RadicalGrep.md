# Radical Grep Implementation Guide
## All Source Files
#### FilePath
              parabix-devel/tools/radicalgrep
#### Files
		radical_interface.h
		radical_interface.cpp
		radicalgrep.cpp
		CMakeLists.txt
## Important Data Structure and Algorithm
### radical_interface.h & radical_interface.cpp
**namespace**:  `BS`  
It is used to define the corresponding functions and variables related to radical grep.


**class**:  `UnicodeSetTable`, `RadicalValuesEnumerator`


* **UnicodeSetTable**:  

**Data Structure**   
static map&lt;string, const UCD::UnicodeSet*&gt; radical_table  
The map list all kinds of radicals and their corresponding UnicodeSet prodefined in kRSKangXi.h  

 **Function**  
const UCD::UnicodeSet&& get_uset(string radical)   
Map the input radical to the corresponding UnicodeSet predefined in kRSKangXi.h
 
* **RadicalValuesEnumerator**:  

**Data Structure**  
1. std::vector&lt;<string&gt;> radical_list    
   Store the input radical(s)  
2. int radical_num 
  Store the number of input radical(s)  

**Function**  
1.   void parse_input(string input_radical)  
Parse the input "r1_r2_" or "r0_", disassemble the input radical(s) and store it (them) in vector  
2. std::vector&lt;re::RE*&gt; createREs()  
Search for the results

### radicalgrep.cpp  

#### This file is the top-level running file, the running process is:  
    Get the input and the file you want to search ->
    Analyze the input to get the corresponding radical UnicodeSet -> 
    Search the result in each file -> 
    Output and highlight the result     

**Data Structure**  
1. static cl::OptionCategory radicalgrepFlags("Command Flags", "radicalgrep options")  
The command line  
2. static cl::opt&lt;std::string&gt; input_radical(cl::Positional, cl::desc("&lt;Radical Index&gt;"), cl::Required, cl::cat(radicalgrepFlags))  
The input  radical(s)  
3. static cl::list&lt;std::string&gt; inputfiles(cl::Positional, cl::desc("&lt;Input File&gt;"), cl::OneOrMore, cl::cat(radicalgrepFlags))  
The files you want to search   
4. std::vector&lt;fs::path&gt; allfiles  
Store all path of files  

**Function**  
1. std::vector&lt;re::RE*&gt; generateREs(std::string input_radical)   
This function parse the input and get the results  
2. setColoring()  
Defined in file grep_engine, Get result highlight  
3. initFileResult(allfiles)  
Defined in file grep_engine, Initialize results of each file  
4. initREs(radicalREs)  
Defined in file grep_engine, Initialize the output  
5. grepCodeGen()  
Return the number of the result  
6. searchAllFiles()  
Return if there have found any result, if yes, return true, else return false  
