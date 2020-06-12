#include"pinyin.h"
namespace PinyinPattern{
    //the program will not support for syllable ê anymore because it can be replace by ei
    vector <vector<string> > syl { {"a", "ā", "á", "ǎ", "à"},
        {"o", "ō", "ó", "ǒ", "ò"},
        {"e", "ē", "é", "ě", "è"},
        {"i", "ī", "í", "ǐ", "ì"},
        {"u", "ū", "ú", "ǔ", "ù"},
        {"ü", "ǖ", "ǘ", "ǚ", "ǜ"}
        //        {"ê", "ê̄ ", "ế ", "ê̌ ", "ề "},
        //        {"m", "m̄", "ḿ", "m̀"},
        //        {"n", "ń", "ň", "ǹ"}
    };
    vector <string> consonant {"b","c","d","f","g","h","j","k","l","m","n","p","q",
        "r","s","t","v","w","x","y","z","ch","sh","zh"
    };
    vector <string> vowel{"a", "ā", "á", "ǎ", "à",
        "ai", "āi", "ái", "ǎi", "ài",
        "ao", "āo", "áo", "ǎo", "ào",
        "an", "ān", "án", "ǎn","àn",
        "ang", "āng", "áng", "ǎng", "àng",
        "ian", "iān", "ián", "iǎn","iàn",
        "iao", "iāo", "iáo", "iǎo", "iào",
        "iang", "iāng", "iáng", "iǎng","iàng",
        "ia", "iā", "iá", "iǎ", "ià",
        "ua", "uā", "uá", "uǎ", "uà",
        "uan", "uān", "uán", "uǎn","uàn",
        "uai", "uāi", "uái", "uǎi", "uài",
        "uang", "uāng", "uáng", "uǎng","uàng",
        "o", "ō", "ó", "ǒ", "ò",
        "ou", "ōu", "óu", "ǒu", "òu",
        "uo", "uō", "uó", "uǒ", "uò",
        "ong", "ōng", "óng", "ǒng", "òng",
        "iong", "iōng", "ióng", "iǒng", "iòng",
        "e", "ē", "é", "ě", "è",
        "ei", "ēi", "éi", "ěi", "èi",
        "en", "ēn", "én", "ěn", "èn",
        "ie", "iē", "ié", "iě", "iè",
        "eng", "ēng", "éng", "ěng", "èng",
        "er", "ēr", "ér", "ěr", "èr",
        "üe", "üē", "üé", "üě", "üè",
        "ue", "uē", "ué", "uě", "uè",
        "i", "ī", "í", "ǐ", "ì",
        "ui", "uī", "uí", "uǐ", "uì",
        "in", "īn", "ín", "ǐn", "ìn",
        "ing", "īng", "íng", "ǐng", "ìng",
        "u", "ū", "ú", "ǔ", "ù",
        "iu", "iū", "iú", "iǔ", "iù",
        "un", "ūn", "ún", "ǔn", "ùn",
        "ü", "ǖ", "ǘ", "ǚ", "ǜ"
        //                          "ê", "ê̄", "ế", "ê̌", "ề"
    };
/*
    Before Search works as the following three steps:
    1. divide the input into single or multi syllables1
    2. parse those syllables to transfer them to formal format for searching
    3. add search prefix and suffix
*/
    vector <vector<string> > Syllable_Parse(string Pinyin_syllables, bool database)
    {
        vector<string> Divided;
        vector <vector <string> > FinalVec;
        bool legal = true;
        bool vague_input = false;
        int pos = 0;
        Pinyin_syllables = trim(Pinyin_syllables);
        transform(Pinyin_syllables.begin(),Pinyin_syllables.end(),Pinyin_syllables.begin(),::tolower);
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
        


        for(size_t i=0; i<Divided.size(); i++){
            string word = Divided[i];
            vector<string> temp_vec;
            vector<string> tone_parse;
            /*if find regex . , store all possible syllables without tone in the tone_parse*/
            if (word.find('.') != string::npos)
            {
                for (int j=0; j<6; j++){
                    temp = word;
                    temp.replace(word.find('.'), 1, syl[j][0]);
                    tone_parse.push_back(temp);
                }
            }
            /*if find regex ? , store all possible syllables without tone in the tone_parse*/
            else if (word.find('?') != string::npos)
            {
                temp = temp2 = word;
                temp.replace(word.find('?'), 1, "");
                tone_parse.push_back(temp);
                temp2.replace(word.find('?')-1, 2, "");
                tone_parse.push_back(temp2);
            }
            /*if find regex [ ] , store all possible syllables without tone in the tone_parse*/
            else if (find_bracket(word))
            {
                int pos1,pos2;
                pos1 = word.find_first_of('[');
                pos2 = word.find_first_of(']');
                vector <string>  vague_syntax;
                for(int i = pos1+1;i!=pos2;i++)
                {
                    vague_syntax.push_back(word.substr(i,1));
                }
                vector <string>::iterator iter;
                iter = vague_syntax.begin();
                for(int i = 1;i<pos2-pos1;i++)
                {
                    
                    string tmp_str;
                    tmp_str = *iter;
                    temp = word;
                    temp.erase(pos1,pos2-pos1+1);
                    temp.insert(pos1,tmp_str);
                    tone_parse.push_back(temp);
                    iter++;
                }
            }
            /*if the word has digital tone, replace it with the regex one*/
            else if (word.find('1') != string::npos || word.find('2') != string::npos|| word.find('3') != string::npos|| word.find('4') != string::npos)
            {
                int tone=0;
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
            /*deal with the example for mang*/
            else if (All_Alpha(word))
            {
                temp_vec.push_back(word);
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
            /*the default one is the one has regex tone */
            else
            {
                temp_vec.push_back(word);
            }
            //parse the tone_parse
            for(vector<string>::iterator temp_iter = tone_parse.begin();temp_iter!=tone_parse.end();temp_iter++)
            {
                string parse_tone = *temp_iter;
                if(All_Alpha(parse_tone))
                {
                    temp_vec.push_back(parse_tone);
                    for (size_t j=0; j<syl.size(); j++)
                    {
                        if (parse_tone.find(syl[j][0]) != string::npos){
                            for (size_t k=1; k<syl[j].size(); k++){
                                temp = parse_tone;
                                temp.replace(parse_tone.find(syl[j][0]), 1, syl[j][k]);
                                temp_vec.push_back(temp);;
                            }
                            break;
                        }
                    }
                }
                else
                {
                    temp_vec.push_back(parse_tone);
                }
            }
            //check the legality
            legal = check_legel(temp_vec);
            if(vague_input)
                legal = true;
            if(legal)
            {
                //select the database
                if(!database)
                {
                    //adding the prefix and suffix to all the strings
                    
                    for(vector<string>::iterator iter = temp_vec.begin(); iter!=temp_vec.end(); iter++)
                    {
                        *iter = Add_kHanyuPinyin_fix(*iter);
                    }
                }
                else
                {
                    for(vector<string>::iterator iter = temp_vec.begin(); iter!=temp_vec.end(); iter++)
                    {
                        *iter = Add_kXHC1983_fix(*iter);
                    }
                }
                // for (auto x: FinalVec){
                //     llvm::errs() << x << " ";
                // }
                // llvm::errs() << "\n";
                //test the parsing
                //for(vector<string>::iterator iter = temp_vec.begin(); iter!=temp_vec.end(); iter++)
                //{
                //    cout<<*iter<<endl;
                //}
                //
                if(temp_vec.size()!=0)
                {
                    FinalVec.push_back(temp_vec);
                }
            }
            else
            {
                FinalVec.clear();
                return FinalVec;
            }
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
    string Add_kHanyuPinyin_fix(string to_add)
    {
        string temp = to_add;
        string prefix = "(kHanyuPinyin.*"; 
        string suffix = ")(,|$)";
        temp = prefix + temp + suffix;
        return temp;
    }
    string Add_kXHC1983_fix(string to_add)
    {
        string temp = to_add;
        string prefix = "(kXHC1983.*";
        string suffix = ")(,|$| )";
        temp = prefix + temp + suffix;
        return temp;
    }
    //check if the string is all English words
    bool All_Alpha(string word)
    {
        for(size_t a=0; a<word.length(); a++){
            if (!isalpha(word[a])){
                return false;
            }
        }
        return true;
    }
    bool check_legel(vector<string> to_check)
    {
        vector<string>::iterator consonant_iter,vowel_iter,str_iter;
        for(str_iter = to_check.begin();str_iter != to_check.end();str_iter++)
        {
            bool check_flag = false;
            for(consonant_iter = consonant.begin();consonant_iter != consonant.end();consonant_iter++)
            {
                for(vowel_iter = vowel.begin();vowel_iter != vowel.end();vowel_iter++)
                {
                    string temp_str = *consonant_iter+*vowel_iter;
                    if(*str_iter==temp_str)
                    {
                        check_flag = true;
                    }
                }
            }
            if(!check_flag)
            {
                cout<<"Please enter the correct format of syllables, listed in the README-pinyingrep.md, for example 'ming'. "<<endl;
                return false;
            }
        }
        return true;
    }
    bool find_bracket(string to_find)
    {
        bool find_flag = false;
        int pos1,pos2;
        pos1 = pos2 = -1;
        pos1 = to_find.find_first_of('[');
        pos2 = to_find.find_first_of(']');
        if(pos1!=-1&&pos2>pos1)
        {
            find_flag = true;
        }
        return find_flag;
    }
}
