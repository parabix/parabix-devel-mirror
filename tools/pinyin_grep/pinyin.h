#ifndef _PINYIN_INTERFACE_
#define _PINYIN_INTERFACE_

#include <vector>
#include <string>
#include <array>
#include <map>
#include <utility>
#include <set>
#include <regex>
#include <iostream>
#include <fstream>
#include <sstream>
#include <grep/grep_engine.h>
#include <grep/grep_kernel.h>
#include <llvm/ADT/STLExtras.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/Signals.h>
#include <llvm/Support/ManagedStatic.h>
#include <llvm/Support/raw_ostream.h>
enum ColoringType {alwaysColor, autoColor, neverColor};
extern ColoringType ColorFlag;

namespace PinyinPattern{
    using namespace std;
    
    class Buffer{
        size_t rmd;
        size_t size, size32, diff;
        string str, name;
        string fstring;
    public:
        Buffer()
        {
            name = "../tools/pinyin_grep/Unihan_Readings.txt";
            ifstream file(name);
            if(file) {
                str.assign(istreambuf_iterator<char>(file), istreambuf_iterator<char>());
                //test if it read the file
                //std::cout<<str<<std::endl;
            }
            else{
                cout << "Fatal Error, cannot open the file"<<endl;
                cout << name << endl;
            }
            size = find_size(name);
            size32 = set_size(name, size);
            diff = size32 - size;
            fstring = str;
        }
        size_t find_size (string filename){
            ifstream f(filename, ios::binary);
            f.seekg(0, ios::end);
            return int(f.tellg());
        }
        size_t set_size(string filename, int s){
            rmd = s % 32;
            if (!rmd){
                return s;
            }
            return s - rmd + 32;
        }
        size_t R_size()
        {
            return size;
        }
        size_t R_size32()
        {
            return size32;
        }
        size_t R_diff()
        {
            return diff;
        }
        string R_fstring()
        {
            return fstring;
        }
    };
    
    class PinyinSetAccumulator : public grep::MatchAccumulator {
        UCD::UnicodeSet mAccumSet;
        unsigned int parsed_codepoint;
        string strcodepoint;
        unsigned int conv_int(string h){
            stringstream s;
            unsigned int ret;
            s << hex << h;
            s >> ret;
            return ret;
        }
    public:
        PinyinSetAccumulator() {}
        UCD::UnicodeSet && getAccumulatedSet() {
            return move(mAccumSet);
        }
        void accumulate_match(const size_t pos, char * start, char * end) override {
            // add codepoint to UnicodeSet
            int i = 2;
            strcodepoint.clear();
            while(*(start+i) != '\t'){
                strcodepoint += *(start+i);
                i++;
                // llvm::errs() << "strcodepoint " << strcodepoint << "\n";
            }
            // llvm::errs() << "strcodepoint " << strcodepoint << "\n";
            parsed_codepoint = conv_int(strcodepoint);
            // llvm::errs() << "parsed_codepoint " << parsed_codepoint << "\n";
            mAccumSet.insert(parsed_codepoint);
        }
    };
    vector <vector<string> > Syllable_Parse(string Pinyin_syllables, bool database);
    string& trim(string &s);
    string Add_kHanyuPinyin_fix(string to_add);
    string Add_kXHC1983_fix(string to_add);
    bool All_Alpha(string word);
    
}
#endif
