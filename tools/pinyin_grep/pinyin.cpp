#include"pinyin.h"
namespace PinyinPattern{
/*  
    Before Search works as the following three steps:
    1. divide the input into single or multi syllables1
    2. parse those syllables to transfer them to formal format for searching
    3. add search prefix and suffix
*/
    vector <vector<string> > Before_Search(string Pinyin_syllables)
    {
        vector<string> Divided;
        vector <vector <string> > FinalVec;
        int pos = 0;
        Pinyin_syllables = trim(Pinyin_syllables);
        
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
        

        vector <vector<string>> syl { {"a", "ā", "á", "ǎ", "à"},
            {"o", "ō", "ó", "ǒ", "ò"},
            {"e", "ē", "é", "ě", "è"},
            {"i", "ī", "í", "ǐ", "ì"},
            {"u", "ū", "ú", "ǔ", "ù"},
            {"ê", "ê̄ ", "ế ", "ê̌ ", "ề "},
            {"m", "m̄", "ḿ", "m̀"},
            {"n", "ń", "ň", "ǹ"} };
        for(size_t i=0; i<Divided.size(); i++){
            string word = Divided[i];
            vector<string> temp_vec;
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
                int tone = 0;
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
                for (size_t j=0; j<syl.size(); j++){
                    if (word.find(syl[j][0]) != string::npos){
                        word.replace(word.find(syl[j][0]), 1, syl[j][tone]);
                        temp_vec.push_back(word);
                        break;
                    }
                }
            }
            else if (All_Alpha(word))
            {
                for (size_t j=0; j<syl.size(); j++)
                {
                    if (Divided[i].find(syl[j][0]) != string::npos){
                        for (size_t k=1; k<syl[j].size(); k++){
                            temp = word;
                            temp.replace(word.find(syl[j][0]), 1, syl[j][k]);
                            temp_vec.push_back(temp);;
                        }
                        break;
                    }
                }
            }
            else
            {
                temp_vec.push_back(word);
            }
            
            //adding the prefix and suffix to all the strings
            for(vector<string>::iterator iter = temp_vec.begin(); iter!=temp_vec.end(); iter++)
            {
                *iter = Add_Search_Prefix(*iter);
            }
            // for (auto x: FinalVec){
            //     llvm::errs() << x << " ";
            // }
            // llvm::errs() << "\n";
            
            FinalVec.push_back(temp_vec);
        }
        Divided = vector <string>(); //deallocate unused vector
        
        
        return FinalVec;
    }
    // trim erases the extra space from both head and tail of a std::string
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
    // Add_Search_Prefix adds both the prefix and suffix of the search module to search in the grep engine
    string Add_Search_Prefix(string to_add)
    {
        string temp = to_add;
        string prefix = "(kHanyuPinyin.*"; 
        string suffix = ")(,|$)";
        temp = prefix + temp + suffix;
        return temp;
    }
    

    bool All_Alpha(string word)
    {
        for(size_t a=0; a<word.length(); a++){
            if (!isalpha(word[a])){
                return false;
            }
        }
        return true;
    }
}
