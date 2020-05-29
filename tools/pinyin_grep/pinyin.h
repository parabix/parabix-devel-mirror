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
    //this function should be rewriten totally inorder to deal with multiple syllables since it has no space between two syllables
    //should find a new way to divided the syllables by not using the space
    bool all_alpha(string word)
    {
        for(int a=0; a<word.length(); a++){
            if (!isalpha(word[a])){
                return false;
            }
        }
        return true;
    }
    vector<string> Before_Search(string Pinyin_syllables)
    {
        int pos = 0;
        Pinyin_syllables = trim(Pinyin_syllables);
        vector<string> Divided, FinalVec;
        string temp, temp2;
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
        //Task1
        //one function need to be here to modify all the syllables as a normal form as hōng
        //for example hong1 to hōng and hong to hōng......etc

        vector <vector<string>> syl { {"a", "ā", "á", "ǎ", "à"}, 
                                    {"o", "ō", "ó", "ǒ", "ò"},
                                    {"e", "ē", "é", "ě", "è"},
                                    {"i", "ī", "í", "ǐ", "ì"},
                                    {"u", "ū", "ú", "ǔ", "ù"},
                                    {"ê", "ê̄ ", "ế ", "ê̌ ", "ề "},
                                    {"m", "m̄", "ḿ", "m̀"},
                                    {"n", "ń", "ň", "ǹ"} };
        for(int i=0; i<Divided.size(); i++){
            string word = Divided[i];
            if (word.find('.') != string::npos) // found regex . 
            {   
                for (int j=0; j<6; j++){
                    temp = word;
                    temp.replace(word.find('.'), 1, syl[j][0]);
                    Divided.push_back(temp);
                }
                Divided.erase(Divided.begin()+i);
                i--;
            }
            else if (word.find('?') != string::npos) // found regex ?
            {
                temp = temp2 = word;
                temp.replace(word.find('?'), 1, "");
                Divided.push_back(temp);
                temp2.replace(word.find('?')-1, 2, "");
                Divided.push_back(temp2);
            }
            else if (word.find('1') != string::npos || word.find('2') != string::npos|| word.find('3') != string::npos|| word.find('4') != string::npos)
            {
                int tone;
                if (word.find('1') != string::npos)
                {
                    tone = 1;
                }
                else if (word.find('2') != string::npos)
                {
                    tone = 2;
                }
                else if (word.find('3') != string::npos)
                {
                    tone = 3;
                }
                else if (word.find('4') != string::npos)
                {
                    tone = 4;
                }
                word.replace(word.find(to_string(tone)), 1, "");
                for (int j=0; j<syl.size(); j++){
                    if (word.find(syl[j][0]) != string::npos){
                        word.replace(word.find(syl[j][0]), 1, syl[j][tone]);
                        FinalVec.push_back(word);
                        break;
                    }
                }
            }
            else if (all_alpha(word))
            {
                for (int j=0; j<syl.size(); j++)
                {
                    if (Divided[i].find(syl[j][0]) != string::npos){
                        for (int k=1; k<syl[j].size(); k++){
                            temp = word;
                            temp.replace(word.find(syl[j][0]), 1, syl[j][k]);
                            FinalVec.push_back(temp);;
                        }
                        break;
                    }
                }
            }
            else
            {
                FinalVec.push_back(word);
            }
            
        }
        Divided = vector <string>(); //deallocate unused vector 

        //adding the kHanyuPinyin.* to all the strings
        for(vector<string>::iterator iter = FinalVec.begin(); iter!=FinalVec.end(); iter++)
        {
            *iter = Add_Search_Prefix(*iter);
        }
        return FinalVec;
    }
    class Buffer{
        size_t rmd;
        size_t size, size32, diff;
        string str, name;
        string fstring;
    public:
        Buffer()
        {
            name = "../../unihan-scripts/Unihan/Unihan_Readings.txt";
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
