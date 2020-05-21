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
#include <unicode/data/KHanyuPinyin.h>
#include "../../unihan-scripts/Unihan/Unihan_Readings.txt"

namespace PinyinPattern{
    using namespace std;
    class InputChecker{
        bool is_check;
        vector<string> first_check;
        vector <pair<string, int> >final;
        public:
        InputChecker(){is_check = false;first_check.clear();}
        InputChecker(string s)
        {
            InputCheck(s);
            is_check = true;
        }
        void InputCheck(string to_check){
            MultiInput_Check(to_check,first_check);
            Divide();
            is_check = true;
        }
        void MultiInput_Check(string to_check, vector<string> &multiCheck_store);
        string& trim(string &s) 
        {
            if (s.empty()) 
            {
                return s;
            }
            s.erase(0,s.find_first_not_of(" "));
            s.erase(s.find_last_not_of(" ") + 1);
            return s;
        }
        void Divide();
    };

    class Pattern_Enumerate{

    };

    class Buffer{
        int rmd, fsize;
        string str, name;
        name = "Unihan_Readings.txt";
        int find_size (string filename){
            ifstream f(filename, ios::binary);
            f.seekg(0, ios::end);
            return int(f.tellg());
        }
        int set_size(string filename, int s){
                rmd = s % 32;
                if (!rmd){
                    return s;
                }
            return s - rmd + 32;
        }
        ifstream file(name);
        if (file) {
            str.assign(istreambuf_iterator<char>(file), istreambuf_iterator<char>());
        }
        
        public:
        const auto size, size32, diff;
        string fstring = str;
        size = find_size(name);
        size32 = set_size(name, size);
        diff = size32 - size;
    };
}
#endif