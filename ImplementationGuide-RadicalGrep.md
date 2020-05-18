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

&ensp;&ensp;&ensp;&ensp;&ensp;It is used to define the corresponding functions and variables related to radical grep.


**class**:  `UnicodeSetTable`, `RadicalValuesEnumerator`


* UnicodeSetTable:   
**Data Structure**   
&ensp;&ensp;&ensp;static map<string, const UCD::UnicodeSet*> radical_table 
&ensp;&ensp;&ensp;The map list all kinds of radicals and their corresponding UnicodeSet prodefined in kRSKangXi.h  
 **Function**  
&ensp;&ensp;&ensp;const UCD::UnicodeSet&& get_uset(string radical) `
&ensp;&ensp;&ensp;Map the input radical to the corresponding UnicodeSet predefined in kRSKangXi.h
 
* RadicalValuesEnumerator:  
**Data Structure** 
  1. std::vector<string> radical_list    
   Store the input radical(s)
   2. int radical_num 
  Store the number of input radical(s)  

	**Function**  
&ensp;&ensp; 1.   void parse_input(string input_radical)
&ensp;&ensp;&ensp;&ensp;&ensp;Parse the input "r1_r2_" or "r0_", disassemble the input radical(s) and store it (them) in vector  
&ensp;&ensp;	2. std::vector<re::RE*> createREs()  
&ensp;&ensp;&ensp;&ensp;&ensp;Search for the results

###radicalgrep.cpp
&ensp;&ensp;&ensp;This file is the top-level running file, the running process is:  

&ensp;&ensp;&ensp;Get the input and the file you want to search -> Analyze the input to get the corresponding radical UnicodeSet -> Search the result in each file -> Output and highlight the result   

&ensp;&ensp;&ensp;**Data Structure**  
 &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. static cl::OptionCategory radicalgrepFlags("Command Flags", "radicalgrep options")  
 &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;The command line  
 &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. static cl::opt<std::string> input_radical(cl::Positional, cl::desc("<Radical Index>"), cl::Required, cl::cat(radicalgrepFlags))  
  &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;The input  radical(s)  
 &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;3. static cl::list<std::string> inputfiles(cl::Positional, cl::desc("<Input File>"), cl::OneOrMore, cl::cat(radicalgrepFlags))  
  &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;The files you want to search   
  &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;4. std::vector<fs::path> allfiles  
  &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Store all path of files  
&ensp;&ensp;&ensp;**Function**   
  &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;1. `std::vector<re::RE*> generateREs(std::string input_radical)  
    &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;This function parse the input and get the results  
    &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;2. setColoring() 
    &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Defined in file grep_engine, Get result highlight  
   &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;3. initFileResult(allfiles) 
    &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Defined in file grep_engine, Initialize results of each file  
   &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;4. grep->initREs(radicalREs) 
    &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Defined in file grep_engine, Initialize the output  
    &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;5. grepCodeGen() 
    &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Return the number of the result  
    &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;6. searchAllFiles()
    &ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;Return if there have found any result, if yes, return true, else return false  
