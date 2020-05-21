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
    std::string& trim(std::string &s)
    {
        if(s.empty())
        {
            return s;
        }
        
        s.erase(0,s.find_first_not_of(" "));
        s.erase(s.find_last_not_of(" ") + 1);
        return s;
    }
    std::string Add_Search_Prefix(string to_add)
    {
        std::string temp = to_add;
        std::string prefix = "kHanyuPinyin.*";
        temp = prefix + temp;
        return temp;
    }
    std::vector<std::string> Before_Search(std::string Pinyin_syllables)
    {
        int pos = 0;
        Pinyin_syllables = trim(Pinyin_syllables);
        std::vector<std::string> Divided;
        while(pos!=-1)
        {
            string temp;
            pos = Pinyin_syllables.find_first_of(' ',pos);
            temp = Pinyin_syllables.substr(0,pos);
            Pinyin_syllables.erase(0,pos);
            temp = trim(temp);
            Divided.push_back(temp);
            Pinyin_syllables = trim(Pinyin_syllables);
        }
        if(Pinyin_syllables!="")
        {
            Divided.push_back(Pinyin_syllables);
        }
        for(std::vector<std::string>::iterator iter = Divided.begin();iter!=Divided.end();iter++)
        {
            *iter = Add_Search_Prefix(*iter);
        }
        return Divided;
    }
    class Buffer{
        int rmd;
        int size, size32, diff;
        string str, name;
        string fstring;
    public:
        Buffer()
        {
            name = "Unihan_Readings.txt";
            ifstream file(name);
            if(file) {
                str.assign(istreambuf_iterator<char>(file), istreambuf_iterator<char>());
            }
            else{
                std::cout << "Fatal Error, cannot open the file"<<endl;
            }
            size = find_size(name);
            size32 = set_size(name, size);
            diff = size32 - size;
            fstring = str;
        }
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
        int R_size()
        {
            return size;
        }
        int R_size32()
        {
            return size32;
        }
        int R_diff()
        {
            return diff;
        }
        string R_fstring()
        {
            return fstring;
        }
    };

    class MatchAccumulator{
        vector<string> parsedvector;
        public:
        MatchAccumulator(vector<string> values):parsedvector(values) {};
        void accumulate_match(int pos, char * start, char * end){
            // add codepoint to UnicodeSet
            parsedvector.emplace_back(start, end);
        }
    };
}
#endif
