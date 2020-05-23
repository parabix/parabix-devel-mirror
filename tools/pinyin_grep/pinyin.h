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
    string& trim(string &s)
    {
        if(s.empty())
        {
            return s;
        }
        
        s.erase(0,s.find_first_not_of(" "));
        s.erase(s.find_last_not_of(" ") + 1);
        return s;
    }
    string Add_Search_Prefix(string to_add)
    {
        string temp = to_add;
        string prefix = "kHanyuPinyin.*";
        temp = prefix + temp;
        return temp;
    }
    vector<string> Before_Search(string Pinyin_syllables)
    {
        int pos = 0;
        Pinyin_syllables = trim(Pinyin_syllables);
        vector<string> Divided;
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
        for(vector<string>::iterator iter = Divided.begin(); iter!=Divided.end(); iter++)
        {
            *iter = Add_Search_Prefix(*iter);
        }
        return Divided;
    }
    class Buffer{
        size_t rmd;
        size_t size, size32, diff;
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
                cout << "Fatal Error, cannot open the file"<<endl;
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
            int i = 3;
            strcodepoint.clear();
            while(*(start+i) != '\t'){
                strcodepoint += *(start+i);
                i++;
            }
            parsed_codepoint = conv_int(strcodepoint);
            mAccumSet.insert(parsed_codepoint);
        }
    };
}
#endif