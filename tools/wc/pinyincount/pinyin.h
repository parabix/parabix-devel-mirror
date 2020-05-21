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
#include <unicode/data/KHanyuPinyin.h>

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
    class Pattern_Parse{

    };

    class Pattern_Enumerate{

    };

}
#endif
