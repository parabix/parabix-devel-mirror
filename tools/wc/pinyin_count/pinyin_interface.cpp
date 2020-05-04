/*
 *  Copyright Group ALpha
 *  in Software Engineering(2020 Spring), ZJU
 *  Advisor: Prof. Cameron
 */

#include "pinyin_interface.h"
#define DEBUG 0

using namespace std;
namespace PY{
    // Method of Parser
    // Parse multiple pinyin regexes into a list based on ' ' space
    // with no extra space in both ends
    void PinyinValuesParser::_parse_multi_syllable(string s, vector<string>& list){
        int start = s.find_first_not_of(' '), end;
        while(start != s.npos){
            s = s.substr(start);        // replace prefix empty space
            end = s.find_first_of(' '); // find the end of the current syllable 
            list.push_back(s.substr(start, end - start));

            // erase the current syllable
            s = s.substr(end);  
            start = s.find_first_not_of(' ');
        }
        #if DEBUG
            cout<<s<<" -- Parse Multi syllables Result:"<<endl;
            for(auto iter = list.begin(); iter != list.end(); iter++){
                cout<<*iter<<endl;
            }
        #endif
    }

    // interpret
    std::pair<vector<string>, vector<int>>
    PinyinValuesParser::_interpret_regex(string s){
        std::pair<vector<string>, vector<int>> resolved;
        PinyinValuesTable table;

        // resloved tones specified 
        regex pattern(".*([0-4]).*");
        smatch match_result;
        if(regex_search(s, match_result, pattern)){
            string tone = match_result.str();
            if(tone.length() > 1) throw ParserException("Invalid Syntax -- Too many numbers to specify tones");
            s = regex_replace(s, pattern, ""); // erase the tone number
            resolved.second.push_back(*tone.begin() - '0'); // convert char to int
        }
        else if(table.is_toned(s)){
            // resolve unicode tones
            int tone = table.replace_tone(s);
            resolved.second.push_back(tone);
        }
        else{
            // no tone
            table.replace_equivalence(s);
            resolved.second = vector<int>{0, 1, 2, 3, 4};
        }
        // resolve regex '?'
        int qmark_index = s.find('?');
        if(qmark_index != s.npos){
            if(s.find("g?") == s.npos) 
                throw ParserException("Invalid Syntax -- only support ? after \'g\'");
            // erase ?
            s = s.substr(0, qmark_index);
            resolved.first.push_back(s.substr(s.find_last_of('g'))); // push_back the syllable without g
        }
        resolved.first.push_back(s); // push s into possibly final result
        
        // resolve regex '.'
        if(s.find('.') != s.npos){
            int initial_size = resolved.first.size();
            for(int i = 0; i < initial_size; i++){
                pattern = regex(resolved.first[0]);
                for(auto iter = table.legal_begin(); iter != table.legal_end(); iter++){
                    if(regex_match(*iter, pattern)){
                        resolved.first.push_back(*iter);
                    }
                }
                resolved.first.erase(resolved.first.begin()); // erase the regex with .
            }
        }
        
        #if DEBUG
            cout<<s<<" -- Interpret Result:"<<endl;
            for(auto iter = resolved.first.begin(); iter != resolved.first.end(); iter++)
                cout<<"* "<<*iter<<endl;
            cout<<"=============="<<endl;
            for(auto iter = resolved.second.begin(); iter != resolved.second.end(); iter++)
                cout<<"* "<<*iter<<endl;
            cout<<"=============="<<endl;
        #endif
        return resolved;
    }

    void PinyinValuesParser::parse(string toparse){
        vector<string> list;
        _parse_multi_syllable(toparse, list);
        try{
            for(auto iter = list.begin(); iter != list.end(); iter++){
                _parsed_syllable_tone.push_back(_interpret_regex(*iter));
            }
            _parsed = true;
            
        }catch(ParserException& e){
            cout<<"ParserException:";
            cout<<e.what()<<endl;
        }catch(std::exception& e){
            cout<<"Unexpected Exception!"<<endl;
            cout<<e.what()<<endl;
        }
    }

    // elimiate extra space in both ends of the string
    void PinyinValuesParser::eliminate_space(string& s){
        int first_index = s.find_first_not_of(' ');
        int len = s.find_last_not_of(' ')+1 - first_index;
        s = s.substr(first_index, len);
    }



    // Methods in PinyinValuesEnumerator
    void PinyinValuesEnumerator::enumerate(PinyinValuesParser& parser){
        std::vector<vector<pair<string,int>>> temp_enumerated; //temporary vector of all pairs of parsed inputs
        for(auto first_syl=parser._parsed_syllable_tone.begin();first_syl!=parser._parsed_syllable_tone.end();first_syl++){
            vector<pair<string,int>> temp;
            for(auto syl=first_syl->first.begin();syl!=first_syl->first.end();syl++){
                for(auto tone=first_syl->second.begin();tone!=first_syl->second.end();tone++){
                    temp.push_back(make_pair(*syl,*tone));
                }
            }            
            temp_enumerated.push_back(temp);
        }
        vector<int> indices(temp_enumerated.size());
        int i=temp_enumerated.size()-1;
        while(i>=0){
            vector<pair<string,int>> T; //temporary vector of pairs
            for(int k=0;k<temp_enumerated.size();k++){
                T.push_back(temp_enumerated[k][indices[k]]); //build current combination
            }
            _enumerated_list.push_back(T); //add to final vector
            i=temp_enumerated.size()-1;
            while(i>=0&&++indices[i]==temp_enumerated[i].size()){ 
                indices[i--]=0; //reset indices to 0;
            }
        }
    }

    // Methods in PinyinValuesTable
    
    // Method: get_initial
    // Get the initial part of the syllable
    // if no initial part(e.g. "an" ), simply return ""
    string PinyinValuesTable::get_intial(string s){
        int len = (int)s.length();
        for(int i = 1;i <= len;++i){
            string initial_part = s.substr(0, i);
            if(_initial_syllable_set.find(initial_part) != _initial_syllable_set.end()){
                return initial_part;
            }else if(_toned_character_table.find(initial_part) != _toned_character_table.end()){
                replace_tone(initial_part);
                if(_initial_syllable_set.find(initial_part) != _initial_syllable_set.end()) return initial_part;
            }//for m/n
        }
        return "";
    }
    // Method: get_final
    // Get the final part of the syllable
    // if toned, the tone will be replaced
    string PinyinValuesTable::get_final(string s){
        int len = (int)s.length();
        int i;
        for(i = 1;i <= len;++i){
            string initial_part = s.substr(0, i);
            if(_initial_syllable_set.find(initial_part) != _initial_syllable_set.end()){
                break;
            }else if(_toned_character_table.find(initial_part) != _toned_character_table.end()){
                replace_tone(initial_part);
                if(_initial_syllable_set.find(initial_part) != _initial_syllable_set.end()) break;
            }
        }
        if(i > len){
            string final_part = s;
        }else{
            string final_part = s.substr(i);
        }
        for(i = 0;i < final_part.lengh();++i){
            string toned_char = final_part.substr(i,2);
            if(_toned_character_table.find(toned_char) != _toned_character_table.end()){
                replace_tone(toned_char);
                final_part = final_part.replace(i,2,toned_char);
                if(_final_syllable_set.find(final_part) != _final_syllable_set.end()) return final_part;
            }else if(_final_syllable_set.find(final_part) != _final_syllable_set.end()){
                return final_part;
            }
        }
        return "";
    }
    // Method: is_legal
    // Check whether the syllable is legal or not
    string PinyinValuesTable::is_legal(string s){
        return _legal_syllables_set.find(get_initial(s) + get_final(s)) != _legal_syllables_set.end();
    }
    // Method: is_toned
    // Check whether the syllable has toned final
    string PinyinValuesTable::is_toned(string s){
        int len = (int)s.length();
        for(int i = 0;i < len;++i){
            string final_part = s.substr(i,2);
            if(_toned_character_table.find(final_part) != _toned_character_table.end()){
                return true;
            }
        }
        return false;
    }
    // Method: get_tone
    // get the tone of the syllable
    // returning 0 as neutral tone
    string PinyinValuesTable::get_tone(string s){
        int len = (int)s.length();
        for(int i = 0; i < len; ++i){
            string final_part = s.substr(i,2);
            if(_toned_character_table.find(final_part) != _toned_character_table.end()){
                return _toned_character_table[final_part].second;
            }
        }
        return 0;
    }
    // Method: replace_tone
    // replace the toned part of syllable with non-toned
    // return the tone
    string PinyinValuesTable::replace_tone(string& toned){
        std::pair<string, int> final_part_and_tone = _toned_character_table[toned];
        toned = final_part_and_tone.first;
        return final_part_and_tone.second;
    }

    // Method: replace_equivalence
    // replave unicode v and e_hat
    void PinyinValuesTable::replace_equivalence(string& s){
        for(auto iter = _equivalence_table.begin(); iter != _equivalence_table.end(); iter++){
            int index = s.find(iter->first);
            if(index != s.npos){
                s = s.replace(index, iter->first.length(), iter->second);
            }
        }
    }

    map<string, string> PinyinValuesTable::_equivalence_table{
        make_pair("ü", "v"), make_pair("ê", "e_hat")
    };
    set<string> PinyinValuesTable::_initial_syllable_set{"b","p","m","f","d","t","n","l","g","k","h","j","q","x","zh","ch","sh","r","z","c","s","y","w"
    };
    set<string> PinyinValuesTable::_final_syllable_set{"i","a","o","e","ai","ei","e_hat","ao","ou","an","en","ang","eng","ong","er","i","ia","ie","iao","iu","ian","in","ing","iang","iong","u","ua",
    "uo","uai","ui","uan","un","uang","ueng","v","ve","van","vn"
    };
    set<string> PinyinValuesTable::_legal_syllables_set{
    "a", "o", "e", "ai", "ei", "ao", "ou", "an", "en", "ang", "eng", "er", "bi", "ba", "bo", "bai", "bei", "bao", "ban", "ben", "bang", "beng", "bi", "bie",
    "biao", "bian", "bin", "bing", "bu", "pi", "pa", "po", "pai", "pei", "pao", "pou", "pan", "pen", "pang", "peng", "pi", "pie", "piao", "pian", "pin", "ping", "pu", "m",
    "mi", "ma", "mo", "me", "mai", "mei", "mao", "mou", "man", "men", "mang", "meng", "mi", "mie", "miao", "miu", "mian", "min", "ming", "mu", "fa", "fo", "fei", "fou",
    "fan", "fen", "fang", "feng", "fu", "di", "da", "de", "dai", "dei", "dao", "dou", "dan", "den", "dang", "deng", "dong", "di", "dia", "die", "diao", "diu", "dian", "din",
    "ding", "du", "duo", "dui", "duan", "dun", "e_hat", "ti", "ta", "te", "tai", "tao", "tou", "tan", "tang", "teng", "tong", "ti", "tie", "tiao", "tian", "ting", "tu", "tuo",
    "tui", "tuan", "tun", "n", "ni", "na", "ne", "nai", "nei", "nao", "nou", "nan", "nen", "nang", "neng", "nong", "ni", "nia", "nie", "niao", "niu", "nian", "nin", "ning",
    "niang", "nu", "nuo", "nuan", "nun", "nv", "nve", "li", "la", "lo", "le", "lai", "lei", "lao", "lou", "lan", "len", "lang", "leng", "long", "li", "lia", "lie", "liao",
    "liu", "lian", "lin", "ling", "liang", "lu", "luo", "luan", "lun", "lv", "lve", "ga", "ge", "gai", "gei", "gao", "gou", "gan", "gen", "gang", "geng", "gong", "gu", "gua",
    "guo", "guai", "gui", "guan", "gun", "guang", "ka", "ke", "kai", "kei", "kao", "kou", "kan", "ken", "kang", "keng", "kong", "ku", "kua", "kuo", "kuai", "kui", "kuan", "kun",
    "kuang", "ha", "he", "hai", "hei", "hao", "hou", "han", "hen", "hang", "heng", "hong", "hu", "hua", "huo", "huai", "hui", "huan", "hun", "huang", "ji", "ji", "jia", "jie",
    "jiao", "jiu", "jian", "jin", "jing", "jiang", "jiong", "ju", "juan", "jun", "qi", "qi", "qia", "qie", "qiao", "qiu", "qian", "qin", "qing", "qiang", "qiong", "qu", "quan", "qun",
    "xi", "xi", "xia", "xie", "xiao", "xiu", "xian", "xin", "xing", "xiang", "xiong", "xu", "xuan", "xun", "zhi", "zha", "zhe", "zhai", "zhei", "zhao", "zhou", "zhan", "zhen", "zhang",
    "zheng", "zhong", "zhi", "zhu", "zhua", "zhuo", "zhuai", "zhui", "zhuan", "zhun", "zhuang", "chi", "cha", "che", "chai", "chao", "chou", "chan", "chen", "chang", "cheng", "chong", "chi", "chu",
    "chua", "chuo", "chuai", "chui", "chuan", "chun", "chuang", "shi", "sha", "she", "shai", "shei", "shao", "shou", "shan", "shen", "shang", "sheng", "shi", "shu", "shua", "shuo", "shuai", "shui",
    "shuan", "shun", "shuang", "ri", "re", "rao", "rou", "ran", "ren", "rang", "reng", "rong", "ri", "ru", "rua", "ruo", "rui", "ruan", "run", "zi", "za", "ze", "zai", "zei",
    "zao", "zou", "zan", "zen", "zang", "zeng", "zong", "zi", "zu", "zuo", "zui", "zuan", "zun", "ci", "ca", "ce", "cai", "cao", "cou", "can", "cen", "cang", "ceng", "cong",
    "ci", "cu", "cuo", "cui", "cuan", "cun", "si", "sa", "se", "sai", "sao", "sou", "san", "sen", "sang", "seng", "song", "si", "su", "suo", "sui", "suan", "sun", "yi",
    "ya", "yo", "ye", "yao", "you", "yan", "yang", "yong", "yi", "yin", "ying", "yu", "yuan", "yun", "wa", "wo", "wai", "wei", "wan", "wen", "wang", "weng", "wong", "wu"
    };
    map<string, std::pair<string, int>> PinyinValuesTable::_toned_character_table{ 
    {"ā",make_pair("a",1)},{"á",make_pair("a",2)},{"ǎ",make_pair("a",3)},{"à",make_pair("a",4)},
    {"ī",make_pair("i",1)},{"í",make_pair("i",2)},{"ǐ",make_pair("i",3)},{"ì",make_pair("i",4)},    {"ū",make_pair("u",1)},{"ú",make_pair("u",2)},{"ǔ",make_pair("u",3)},{"ù",make_pair("u",4)},
    {"ē",make_pair("e",1)},{"é",make_pair("e",2)},{"ě",make_pair("e",3)},{"è",make_pair("e",4)},    {"ō",make_pair("o",1)},{"ó",make_pair("o",2)},{"ǒ",make_pair("o",3)},{"ò",make_pair("o",4)},
    {"ǜ",make_pair("v",1)},{"ǘ",make_pair("v",2)},{"ǚ",make_pair("v",3)},{"ǜ",make_pair("v",4)},    {"m̄",make_pair("m",1)},{"ḿ",make_pair("m",2)},{"m̀",make_pair("m",4)},
    {"ń",make_pair("n",2)},{"ň",make_pair("n",3)},{"ǹ",make_pair("n",4)},                           {"ê̄",make_pair("e_hat",1)},{"ế",make_pair("e_hat",2)},{"ê̌",make_pair("e_hat",3)},{"ề",make_pair("e_hat",4)}
    }; 
    map<std::pair<string, int>, UCD::UnicodeSet*> UnicodeSetTable:: _unicodeset_table{ 
    {make_pair("a",0),&a_Set[0]},           {make_pair("a",1),&a_Set[1]},           {make_pair("a",2),&a_Set[2]},           {make_pair("a",3),&a_Set[3]},           {make_pair("a",4),&a_Set[4]},           {make_pair("o",0),&o_Set[0]},           
    {make_pair("o",1),&o_Set[1]},           {make_pair("o",2),&o_Set[2]},           {make_pair("e",0),&e_Set[0]},           {make_pair("e",1),&e_Set[1]},           {make_pair("e",2),&e_Set[2]},           {make_pair("e",3),&e_Set[3]},           
    {make_pair("e",4),&e_Set[4]},           {make_pair("e_hat",1),&e_hat_Set[1]},   {make_pair("e_hat",2),&e_hat_Set[2]},   {make_pair("e_hat",3),&e_hat_Set[3]},   {make_pair("e_hat",4),&e_hat_Set[4]},   {make_pair("ai",1),&ai_Set[1]},         
    {make_pair("ai",2),&ai_Set[2]},         {make_pair("ai",3),&ai_Set[3]},         {make_pair("ai",4),&ai_Set[4]},         {make_pair("ei",2),&ei_Set[2]},         {make_pair("ei",3),&ei_Set[3]},         {make_pair("ei",4),&ei_Set[4]},         
    {make_pair("ao",1),&ao_Set[1]},         {make_pair("ao",2),&ao_Set[2]},         {make_pair("ao",3),&ao_Set[3]},         {make_pair("ao",4),&ao_Set[4]},         {make_pair("ou",0),&ou_Set[0]},         {make_pair("ou",1),&ou_Set[1]},         
    {make_pair("ou",2),&ou_Set[2]},         {make_pair("ou",3),&ou_Set[3]},         {make_pair("ou",4),&ou_Set[4]},         {make_pair("an",1),&an_Set[1]},         {make_pair("an",2),&an_Set[2]},         {make_pair("an",3),&an_Set[3]},         
    {make_pair("an",4),&an_Set[4]},         {make_pair("en",1),&en_Set[1]},         {make_pair("en",3),&en_Set[3]},         {make_pair("en",4),&en_Set[4]},         {make_pair("ang",1),&ang_Set[1]},       {make_pair("ang",2),&ang_Set[2]},       
    {make_pair("ang",3),&ang_Set[3]},       {make_pair("ang",4),&ang_Set[4]},       {make_pair("eng",1),&eng_Set[1]},       {make_pair("er",2),&er_Set[2]},         {make_pair("er",3),&er_Set[3]},         {make_pair("er",4),&er_Set[4]},         
    {make_pair("bi",1),&bi_Set[1]},         {make_pair("bi",2),&bi_Set[2]},         {make_pair("bi",3),&bi_Set[3]},         {make_pair("bi",4),&bi_Set[4]},         {make_pair("ba",0),&ba_Set[0]},         {make_pair("ba",1),&ba_Set[1]},         
    {make_pair("ba",2),&ba_Set[2]},         {make_pair("ba",3),&ba_Set[3]},         {make_pair("ba",4),&ba_Set[4]},         {make_pair("bo",0),&bo_Set[0]},         {make_pair("bo",1),&bo_Set[1]},         {make_pair("bo",2),&bo_Set[2]},         
    {make_pair("bo",3),&bo_Set[3]},         {make_pair("bo",4),&bo_Set[4]},         {make_pair("bai",0),&bai_Set[0]},       {make_pair("bai",1),&bai_Set[1]},       {make_pair("bai",2),&bai_Set[2]},       {make_pair("bai",3),&bai_Set[3]},       
    {make_pair("bai",4),&bai_Set[4]},       {make_pair("bei",0),&bei_Set[0]},       {make_pair("bei",1),&bei_Set[1]},       {make_pair("bei",3),&bei_Set[3]},       {make_pair("bei",4),&bei_Set[4]},       {make_pair("bao",1),&bao_Set[1]},       
    {make_pair("bao",2),&bao_Set[2]},       {make_pair("bao",3),&bao_Set[3]},       {make_pair("bao",4),&bao_Set[4]},       {make_pair("ban",1),&ban_Set[1]},       {make_pair("ban",3),&ban_Set[3]},       {make_pair("ban",4),&ban_Set[4]},       
    {make_pair("ben",1),&ben_Set[1]},       {make_pair("ben",3),&ben_Set[3]},       {make_pair("ben",4),&ben_Set[4]},       {make_pair("bang",1),&bang_Set[1]},     {make_pair("bang",3),&bang_Set[3]},     {make_pair("bang",4),&bang_Set[4]},     
    {make_pair("beng",1),&beng_Set[1]},     {make_pair("beng",2),&beng_Set[2]},     {make_pair("beng",3),&beng_Set[3]},     {make_pair("beng",4),&beng_Set[4]},     {make_pair("bi",1),&bi_Set[1]},         {make_pair("bi",2),&bi_Set[2]},         
    {make_pair("bi",3),&bi_Set[3]},         {make_pair("bi",4),&bi_Set[4]},         {make_pair("bie",1),&bie_Set[1]},       {make_pair("bie",2),&bie_Set[2]},       {make_pair("bie",3),&bie_Set[3]},       {make_pair("bie",4),&bie_Set[4]},       
    {make_pair("biao",1),&biao_Set[1]},     {make_pair("biao",3),&biao_Set[3]},     {make_pair("biao",4),&biao_Set[4]},     {make_pair("bian",1),&bian_Set[1]},     {make_pair("bian",3),&bian_Set[3]},     {make_pair("bian",4),&bian_Set[4]},     
    {make_pair("bin",1),&bin_Set[1]},       {make_pair("bin",3),&bin_Set[3]},       {make_pair("bin",4),&bin_Set[4]},       {make_pair("bing",1),&bing_Set[1]},     {make_pair("bing",3),&bing_Set[3]},     {make_pair("bing",4),&bing_Set[4]},     
    {make_pair("bu",1),&bu_Set[1]},         {make_pair("bu",2),&bu_Set[2]},         {make_pair("bu",3),&bu_Set[3]},         {make_pair("bu",4),&bu_Set[4]},         {make_pair("pi",1),&pi_Set[1]},         {make_pair("pi",2),&pi_Set[2]},         
    {make_pair("pi",3),&pi_Set[3]},         {make_pair("pi",4),&pi_Set[4]},         {make_pair("pa",1),&pa_Set[1]},         {make_pair("pa",2),&pa_Set[2]},         {make_pair("pa",3),&pa_Set[3]},         {make_pair("pa",4),&pa_Set[4]},         
    {make_pair("po",1),&po_Set[1]},         {make_pair("po",2),&po_Set[2]},         {make_pair("po",3),&po_Set[3]},         {make_pair("po",4),&po_Set[4]},         {make_pair("pai",1),&pai_Set[1]},       {make_pair("pai",2),&pai_Set[2]},       
    {make_pair("pai",3),&pai_Set[3]},       {make_pair("pai",4),&pai_Set[4]},       {make_pair("pei",1),&pei_Set[1]},       {make_pair("pei",2),&pei_Set[2]},       {make_pair("pei",3),&pei_Set[3]},       {make_pair("pei",4),&pei_Set[4]},       
    {make_pair("pao",1),&pao_Set[1]},       {make_pair("pao",2),&pao_Set[2]},       {make_pair("pao",3),&pao_Set[3]},       {make_pair("pao",4),&pao_Set[4]},       {make_pair("pou",1),&pou_Set[1]},       {make_pair("pou",2),&pou_Set[2]},       
    {make_pair("pou",3),&pou_Set[3]},       {make_pair("pou",4),&pou_Set[4]},       {make_pair("pan",1),&pan_Set[1]},       {make_pair("pan",2),&pan_Set[2]},       {make_pair("pan",3),&pan_Set[3]},       {make_pair("pan",4),&pan_Set[4]},       
    {make_pair("pen",1),&pen_Set[1]},       {make_pair("pen",2),&pen_Set[2]},       {make_pair("pen",3),&pen_Set[3]},       {make_pair("pen",4),&pen_Set[4]},       {make_pair("pang",1),&pang_Set[1]},     {make_pair("pang",2),&pang_Set[2]},     
    {make_pair("pang",3),&pang_Set[3]},     {make_pair("pang",4),&pang_Set[4]},     {make_pair("peng",1),&peng_Set[1]},     {make_pair("peng",2),&peng_Set[2]},     {make_pair("peng",3),&peng_Set[3]},     {make_pair("peng",4),&peng_Set[4]},     
    {make_pair("pi",1),&pi_Set[1]},         {make_pair("pi",2),&pi_Set[2]},         {make_pair("pi",3),&pi_Set[3]},         {make_pair("pi",4),&pi_Set[4]},         {make_pair("pie",1),&pie_Set[1]},       {make_pair("pie",3),&pie_Set[3]},       
    {make_pair("pie",4),&pie_Set[4]},       {make_pair("piao",1),&piao_Set[1]},     {make_pair("piao",2),&piao_Set[2]},     {make_pair("piao",3),&piao_Set[3]},     {make_pair("piao",4),&piao_Set[4]},     {make_pair("pian",1),&pian_Set[1]},     
    {make_pair("pian",2),&pian_Set[2]},     {make_pair("pian",3),&pian_Set[3]},     {make_pair("pian",4),&pian_Set[4]},     {make_pair("pin",1),&pin_Set[1]},       {make_pair("pin",2),&pin_Set[2]},       {make_pair("pin",3),&pin_Set[3]},       
    {make_pair("pin",4),&pin_Set[4]},       {make_pair("ping",1),&ping_Set[1]},     {make_pair("ping",2),&ping_Set[2]},     {make_pair("ping",3),&ping_Set[3]},     {make_pair("ping",4),&ping_Set[4]},     {make_pair("pu",1),&pu_Set[1]},         
    {make_pair("pu",2),&pu_Set[2]},         {make_pair("pu",3),&pu_Set[3]},         {make_pair("pu",4),&pu_Set[4]},         {make_pair("m",1),&m_Set[1]},           {make_pair("m",2),&m_Set[2]},           {make_pair("m",4),&m_Set[4]},           
    {make_pair("mi",1),&mi_Set[1]},         {make_pair("mi",2),&mi_Set[2]},         {make_pair("mi",3),&mi_Set[3]},         {make_pair("mi",4),&mi_Set[4]},         {make_pair("ma",0),&ma_Set[0]},         {make_pair("ma",1),&ma_Set[1]},         
    {make_pair("ma",2),&ma_Set[2]},         {make_pair("ma",3),&ma_Set[3]},         {make_pair("ma",4),&ma_Set[4]},         {make_pair("mo",1),&mo_Set[1]},         {make_pair("mo",2),&mo_Set[2]},         {make_pair("mo",3),&mo_Set[3]},         
    {make_pair("mo",4),&mo_Set[4]},         {make_pair("me",0),&me_Set[0]},         {make_pair("mai",2),&mai_Set[2]},       {make_pair("mai",3),&mai_Set[3]},       {make_pair("mai",4),&mai_Set[4]},       {make_pair("mei",2),&mei_Set[2]},       
    {make_pair("mei",3),&mei_Set[3]},       {make_pair("mei",4),&mei_Set[4]},       {make_pair("mao",1),&mao_Set[1]},       {make_pair("mao",2),&mao_Set[2]},       {make_pair("mao",3),&mao_Set[3]},       {make_pair("mao",4),&mao_Set[4]},       
    {make_pair("mou",1),&mou_Set[1]},       {make_pair("mou",2),&mou_Set[2]},       {make_pair("mou",3),&mou_Set[3]},       {make_pair("mou",4),&mou_Set[4]},       {make_pair("man",2),&man_Set[2]},       {make_pair("man",3),&man_Set[3]},       
    {make_pair("man",4),&man_Set[4]},       {make_pair("men",1),&men_Set[1]},       {make_pair("men",2),&men_Set[2]},       {make_pair("men",4),&men_Set[4]},       {make_pair("mang",1),&mang_Set[1]},     {make_pair("mang",2),&mang_Set[2]},     
    {make_pair("mang",3),&mang_Set[3]},     {make_pair("mang",4),&mang_Set[4]},     {make_pair("meng",1),&meng_Set[1]},     {make_pair("meng",2),&meng_Set[2]},     {make_pair("meng",3),&meng_Set[3]},     {make_pair("meng",4),&meng_Set[4]},     
    {make_pair("mi",1),&mi_Set[1]},         {make_pair("mi",2),&mi_Set[2]},         {make_pair("mi",3),&mi_Set[3]},         {make_pair("mi",4),&mi_Set[4]},         {make_pair("mie",0),&mie_Set[0]},       {make_pair("mie",1),&mie_Set[1]},       
    {make_pair("mie",2),&mie_Set[2]},       {make_pair("mie",4),&mie_Set[4]},       {make_pair("miao",1),&miao_Set[1]},     {make_pair("miao",2),&miao_Set[2]},     {make_pair("miao",3),&miao_Set[3]},     {make_pair("miao",4),&miao_Set[4]},     
    {make_pair("miu",3),&miu_Set[3]},       {make_pair("miu",4),&miu_Set[4]},       {make_pair("mian",2),&mian_Set[2]},     {make_pair("mian",3),&mian_Set[3]},     {make_pair("mian",4),&mian_Set[4]},     {make_pair("min",0),&min_Set[0]},       
    {make_pair("min",2),&min_Set[2]},       {make_pair("min",3),&min_Set[3]},       {make_pair("ming",2),&ming_Set[2]},     {make_pair("ming",3),&ming_Set[3]},     {make_pair("ming",4),&ming_Set[4]},     {make_pair("mu",2),&mu_Set[2]},         
    {make_pair("mu",3),&mu_Set[3]},         {make_pair("mu",4),&mu_Set[4]},         {make_pair("fa",1),&fa_Set[1]},         {make_pair("fa",2),&fa_Set[2]},         {make_pair("fa",3),&fa_Set[3]},         {make_pair("fa",4),&fa_Set[4]},         
    {make_pair("fo",2),&fo_Set[2]},         {make_pair("fei",1),&fei_Set[1]},       {make_pair("fei",2),&fei_Set[2]},       {make_pair("fei",3),&fei_Set[3]},       {make_pair("fei",4),&fei_Set[4]},       {make_pair("fou",1),&fou_Set[1]},       
    {make_pair("fou",2),&fou_Set[2]},       {make_pair("fou",3),&fou_Set[3]},       {make_pair("fan",1),&fan_Set[1]},       {make_pair("fan",2),&fan_Set[2]},       {make_pair("fan",3),&fan_Set[3]},       {make_pair("fan",4),&fan_Set[4]},       
    {make_pair("fen",1),&fen_Set[1]},       {make_pair("fen",2),&fen_Set[2]},       {make_pair("fen",3),&fen_Set[3]},       {make_pair("fen",4),&fen_Set[4]},       {make_pair("fang",1),&fang_Set[1]},     {make_pair("fang",2),&fang_Set[2]},     
    {make_pair("fang",3),&fang_Set[3]},     {make_pair("fang",4),&fang_Set[4]},     {make_pair("feng",1),&feng_Set[1]},     {make_pair("feng",2),&feng_Set[2]},     {make_pair("feng",3),&feng_Set[3]},     {make_pair("feng",4),&feng_Set[4]},     
    {make_pair("fu",1),&fu_Set[1]},         {make_pair("fu",2),&fu_Set[2]},         {make_pair("fu",3),&fu_Set[3]},         {make_pair("fu",4),&fu_Set[4]},         {make_pair("di",1),&di_Set[1]},         {make_pair("di",2),&di_Set[2]},         
    {make_pair("di",3),&di_Set[3]},         {make_pair("di",4),&di_Set[4]},         {make_pair("da",0),&da_Set[0]},         {make_pair("da",1),&da_Set[1]},         {make_pair("da",2),&da_Set[2]},         {make_pair("da",3),&da_Set[3]},         
    {make_pair("da",4),&da_Set[4]},         {make_pair("de",0),&de_Set[0]},         {make_pair("de",1),&de_Set[1]},         {make_pair("de",2),&de_Set[2]},         {make_pair("dai",1),&dai_Set[1]},       {make_pair("dai",3),&dai_Set[3]},       
    {make_pair("dai",4),&dai_Set[4]},       {make_pair("dei",3),&dei_Set[3]},       {make_pair("dao",1),&dao_Set[1]},       {make_pair("dao",2),&dao_Set[2]},       {make_pair("dao",3),&dao_Set[3]},       {make_pair("dao",4),&dao_Set[4]},       
    {make_pair("dou",1),&dou_Set[1]},       {make_pair("dou",3),&dou_Set[3]},       {make_pair("dou",4),&dou_Set[4]},       {make_pair("dan",1),&dan_Set[1]},       {make_pair("dan",3),&dan_Set[3]},       {make_pair("dan",4),&dan_Set[4]},       
    {make_pair("den",4),&den_Set[4]},       {make_pair("dang",0),&dang_Set[0]},     {make_pair("dang",1),&dang_Set[1]},     {make_pair("dang",3),&dang_Set[3]},     {make_pair("dang",4),&dang_Set[4]},     {make_pair("deng",1),&deng_Set[1]},     
    {make_pair("deng",3),&deng_Set[3]},     {make_pair("deng",4),&deng_Set[4]},     {make_pair("dong",1),&dong_Set[1]},     {make_pair("dong",3),&dong_Set[3]},     {make_pair("dong",4),&dong_Set[4]},     {make_pair("di",1),&di_Set[1]},         
    {make_pair("di",2),&di_Set[2]},         {make_pair("di",3),&di_Set[3]},         {make_pair("di",4),&di_Set[4]},         {make_pair("dia",3),&dia_Set[3]},       {make_pair("die",1),&die_Set[1]},       {make_pair("die",2),&die_Set[2]},       
    {make_pair("die",3),&die_Set[3]},       {make_pair("die",4),&die_Set[4]},       {make_pair("diao",1),&diao_Set[1]},     {make_pair("diao",3),&diao_Set[3]},     {make_pair("diao",4),&diao_Set[4]},     {make_pair("diu",1),&diu_Set[1]},       
    {make_pair("dian",1),&dian_Set[1]},     {make_pair("dian",2),&dian_Set[2]},     {make_pair("dian",3),&dian_Set[3]},     {make_pair("dian",4),&dian_Set[4]},     {make_pair("din",4),&din_Set[4]},       {make_pair("ding",1),&ding_Set[1]},     
    {make_pair("ding",3),&ding_Set[3]},     {make_pair("ding",4),&ding_Set[4]},     {make_pair("du",1),&du_Set[1]},         {make_pair("du",2),&du_Set[2]},         {make_pair("du",3),&du_Set[3]},         {make_pair("du",4),&du_Set[4]},         
    {make_pair("duo",0),&duo_Set[0]},       {make_pair("duo",1),&duo_Set[1]},       {make_pair("duo",2),&duo_Set[2]},       {make_pair("duo",3),&duo_Set[3]},       {make_pair("duo",4),&duo_Set[4]},       {make_pair("dui",1),&dui_Set[1]},       
    {make_pair("dui",3),&dui_Set[3]},       {make_pair("dui",4),&dui_Set[4]},       {make_pair("duan",1),&duan_Set[1]},     {make_pair("duan",3),&duan_Set[3]},     {make_pair("duan",4),&duan_Set[4]},     {make_pair("dun",1),&dun_Set[1]},       
    {make_pair("dun",3),&dun_Set[3]},       {make_pair("dun",4),&dun_Set[4]},       {make_pair("ti",1),&ti_Set[1]},         {make_pair("ti",2),&ti_Set[2]},         {make_pair("ti",3),&ti_Set[3]},         {make_pair("ti",4),&ti_Set[4]},         
    {make_pair("ta",1),&ta_Set[1]},         {make_pair("ta",2),&ta_Set[2]},         {make_pair("ta",3),&ta_Set[3]},         {make_pair("ta",4),&ta_Set[4]},         {make_pair("te",4),&te_Set[4]},         {make_pair("tai",1),&tai_Set[1]},       
    {make_pair("tai",2),&tai_Set[2]},       {make_pair("tai",3),&tai_Set[3]},       {make_pair("tai",4),&tai_Set[4]},       {make_pair("tao",1),&tao_Set[1]},       {make_pair("tao",2),&tao_Set[2]},       {make_pair("tao",3),&tao_Set[3]},       
    {make_pair("tao",4),&tao_Set[4]},       {make_pair("tou",0),&tou_Set[0]},       {make_pair("tou",1),&tou_Set[1]},       {make_pair("tou",2),&tou_Set[2]},       {make_pair("tou",3),&tou_Set[3]},       {make_pair("tou",4),&tou_Set[4]},       
    {make_pair("tan",1),&tan_Set[1]},       {make_pair("tan",2),&tan_Set[2]},       {make_pair("tan",3),&tan_Set[3]},       {make_pair("tan",4),&tan_Set[4]},       {make_pair("tang",1),&tang_Set[1]},     {make_pair("tang",2),&tang_Set[2]},     
    {make_pair("tang",3),&tang_Set[3]},     {make_pair("tang",4),&tang_Set[4]},     {make_pair("teng",1),&teng_Set[1]},     {make_pair("teng",2),&teng_Set[2]},     {make_pair("teng",4),&teng_Set[4]},     {make_pair("tong",1),&tong_Set[1]},     
    {make_pair("tong",2),&tong_Set[2]},     {make_pair("tong",3),&tong_Set[3]},     {make_pair("tong",4),&tong_Set[4]},     {make_pair("ti",1),&ti_Set[1]},         {make_pair("ti",2),&ti_Set[2]},         {make_pair("ti",3),&ti_Set[3]},         
    {make_pair("ti",4),&ti_Set[4]},         {make_pair("tie",1),&tie_Set[1]},       {make_pair("tie",2),&tie_Set[2]},       {make_pair("tie",3),&tie_Set[3]},       {make_pair("tie",4),&tie_Set[4]},       {make_pair("tiao",0),&tiao_Set[0]},     
    {make_pair("tiao",1),&tiao_Set[1]},     {make_pair("tiao",2),&tiao_Set[2]},     {make_pair("tiao",3),&tiao_Set[3]},     {make_pair("tiao",4),&tiao_Set[4]},     {make_pair("tian",1),&tian_Set[1]},     {make_pair("tian",2),&tian_Set[2]},     
    {make_pair("tian",3),&tian_Set[3]},     {make_pair("tian",4),&tian_Set[4]},     {make_pair("ting",1),&ting_Set[1]},     {make_pair("ting",2),&ting_Set[2]},     {make_pair("ting",3),&ting_Set[3]},     {make_pair("ting",4),&ting_Set[4]},     
    {make_pair("tu",1),&tu_Set[1]},         {make_pair("tu",2),&tu_Set[2]},         {make_pair("tu",3),&tu_Set[3]},         {make_pair("tu",4),&tu_Set[4]},         {make_pair("tuo",1),&tuo_Set[1]},       {make_pair("tuo",2),&tuo_Set[2]},       
    {make_pair("tuo",3),&tuo_Set[3]},       {make_pair("tuo",4),&tuo_Set[4]},       {make_pair("tui",1),&tui_Set[1]},       {make_pair("tui",2),&tui_Set[2]},       {make_pair("tui",3),&tui_Set[3]},       {make_pair("tui",4),&tui_Set[4]},       
    {make_pair("tuan",1),&tuan_Set[1]},     {make_pair("tuan",2),&tuan_Set[2]},     {make_pair("tuan",3),&tuan_Set[3]},     {make_pair("tuan",4),&tuan_Set[4]},     {make_pair("tun",1),&tun_Set[1]},       {make_pair("tun",2),&tun_Set[2]},       
    {make_pair("tun",3),&tun_Set[3]},       {make_pair("tun",4),&tun_Set[4]},       {make_pair("n",0),&n_Set[0]},           {make_pair("n",2),&n_Set[2]},           {make_pair("n",3),&n_Set[3]},           {make_pair("n",4),&n_Set[4]},           
    {make_pair("ni",1),&ni_Set[1]},         {make_pair("ni",2),&ni_Set[2]},         {make_pair("ni",3),&ni_Set[3]},         {make_pair("ni",4),&ni_Set[4]},         {make_pair("na",0),&na_Set[0]},         {make_pair("na",1),&na_Set[1]},         
    {make_pair("na",2),&na_Set[2]},         {make_pair("na",3),&na_Set[3]},         {make_pair("na",4),&na_Set[4]},         {make_pair("ne",0),&ne_Set[0]},         {make_pair("ne",2),&ne_Set[2]},         {make_pair("ne",4),&ne_Set[4]},         
    {make_pair("nai",2),&nai_Set[2]},       {make_pair("nai",3),&nai_Set[3]},       {make_pair("nai",4),&nai_Set[4]},       {make_pair("nei",2),&nei_Set[2]},       {make_pair("nei",3),&nei_Set[3]},       {make_pair("nei",4),&nei_Set[4]},       
    {make_pair("nao",1),&nao_Set[1]},       {make_pair("nao",2),&nao_Set[2]},       {make_pair("nao",3),&nao_Set[3]},       {make_pair("nao",4),&nao_Set[4]},       {make_pair("nou",2),&nou_Set[2]},       {make_pair("nou",3),&nou_Set[3]},       
    {make_pair("nou",4),&nou_Set[4]},       {make_pair("nan",1),&nan_Set[1]},       {make_pair("nan",2),&nan_Set[2]},       {make_pair("nan",3),&nan_Set[3]},       {make_pair("nan",4),&nan_Set[4]},       {make_pair("nen",4),&nen_Set[4]},       
    {make_pair("nang",0),&nang_Set[0]},     {make_pair("nang",1),&nang_Set[1]},     {make_pair("nang",2),&nang_Set[2]},     {make_pair("nang",3),&nang_Set[3]},     {make_pair("nang",4),&nang_Set[4]},     {make_pair("neng",2),&neng_Set[2]},     
    {make_pair("neng",3),&neng_Set[3]},     {make_pair("neng",4),&neng_Set[4]},     {make_pair("nong",2),&nong_Set[2]},     {make_pair("nong",3),&nong_Set[3]},     {make_pair("nong",4),&nong_Set[4]},     {make_pair("ni",1),&ni_Set[1]},         
    {make_pair("ni",2),&ni_Set[2]},         {make_pair("ni",3),&ni_Set[3]},         {make_pair("ni",4),&ni_Set[4]},         {make_pair("nia",1),&nia_Set[1]},       {make_pair("nie",1),&nie_Set[1]},       {make_pair("nie",2),&nie_Set[2]},       
    {make_pair("nie",3),&nie_Set[3]},       {make_pair("nie",4),&nie_Set[4]},       {make_pair("niao",3),&niao_Set[3]},     {make_pair("niao",4),&niao_Set[4]},     {make_pair("niu",1),&niu_Set[1]},       {make_pair("niu",2),&niu_Set[2]},       
    {make_pair("niu",3),&niu_Set[3]},       {make_pair("niu",4),&niu_Set[4]},       {make_pair("nian",1),&nian_Set[1]},     {make_pair("nian",2),&nian_Set[2]},     {make_pair("nian",3),&nian_Set[3]},     {make_pair("nian",4),&nian_Set[4]},     
    {make_pair("nin",2),&nin_Set[2]},       {make_pair("nin",3),&nin_Set[3]},       {make_pair("ning",2),&ning_Set[2]},     {make_pair("ning",3),&ning_Set[3]},     {make_pair("ning",4),&ning_Set[4]},     {make_pair("niang",2),&niang_Set[2]},   
    {make_pair("niang",3),&niang_Set[3]},   {make_pair("niang",4),&niang_Set[4]},   {make_pair("nu",2),&nu_Set[2]},         {make_pair("nu",3),&nu_Set[3]},         {make_pair("nu",4),&nu_Set[4]},         {make_pair("nuo",2),&nuo_Set[2]},       
    {make_pair("nuo",3),&nuo_Set[3]},       {make_pair("nuo",4),&nuo_Set[4]},       {make_pair("nuan",2),&nuan_Set[2]},     {make_pair("nuan",3),&nuan_Set[3]},     {make_pair("nuan",4),&nuan_Set[4]},     {make_pair("nun",2),&nun_Set[2]},       
    {make_pair("nun",4),&nun_Set[4]},       {make_pair("nv",1),&nv_Set[1]},         {make_pair("nv",2),&nv_Set[2]},         {make_pair("nv",3),&nv_Set[3]},         {make_pair("nve",4),&nve_Set[4]},       {make_pair("li",0),&li_Set[0]},         
    {make_pair("li",2),&li_Set[2]},         {make_pair("li",3),&li_Set[3]},         {make_pair("li",4),&li_Set[4]},         {make_pair("la",0),&la_Set[0]},         {make_pair("la",1),&la_Set[1]},         {make_pair("la",2),&la_Set[2]},         
    {make_pair("la",3),&la_Set[3]},         {make_pair("la",4),&la_Set[4]},         {make_pair("lo",0),&lo_Set[0]},         {make_pair("le",0),&le_Set[0]},         {make_pair("le",1),&le_Set[1]},         {make_pair("le",4),&le_Set[4]},         
    {make_pair("lai",2),&lai_Set[2]},       {make_pair("lai",3),&lai_Set[3]},       {make_pair("lai",4),&lai_Set[4]},       {make_pair("lei",0),&lei_Set[0]},       {make_pair("lei",1),&lei_Set[1]},       {make_pair("lei",2),&lei_Set[2]},       
    {make_pair("lei",3),&lei_Set[3]},       {make_pair("lei",4),&lei_Set[4]},       {make_pair("lao",1),&lao_Set[1]},       {make_pair("lao",2),&lao_Set[2]},       {make_pair("lao",3),&lao_Set[3]},       {make_pair("lao",4),&lao_Set[4]},       
    {make_pair("lou",0),&lou_Set[0]},       {make_pair("lou",1),&lou_Set[1]},       {make_pair("lou",2),&lou_Set[2]},       {make_pair("lou",3),&lou_Set[3]},       {make_pair("lou",4),&lou_Set[4]},       {make_pair("lan",2),&lan_Set[2]},       
    {make_pair("lan",3),&lan_Set[3]},       {make_pair("lan",4),&lan_Set[4]},       {make_pair("len",4),&len_Set[4]},       {make_pair("lang",1),&lang_Set[1]},     {make_pair("lang",2),&lang_Set[2]},     {make_pair("lang",3),&lang_Set[3]},     
    {make_pair("lang",4),&lang_Set[4]},     {make_pair("leng",1),&leng_Set[1]},     {make_pair("leng",2),&leng_Set[2]},     {make_pair("leng",3),&leng_Set[3]},     {make_pair("leng",4),&leng_Set[4]},     {make_pair("long",2),&long_Set[2]},     
    {make_pair("long",3),&long_Set[3]},     {make_pair("long",4),&long_Set[4]},     {make_pair("li",0),&li_Set[0]},         {make_pair("li",2),&li_Set[2]},         {make_pair("li",3),&li_Set[3]},         {make_pair("li",4),&li_Set[4]},         
    {make_pair("lia",3),&lia_Set[3]},       {make_pair("lie",0),&lie_Set[0]},       {make_pair("lie",1),&lie_Set[1]},       {make_pair("lie",2),&lie_Set[2]},       {make_pair("lie",3),&lie_Set[3]},       {make_pair("lie",4),&lie_Set[4]},       
    {make_pair("liao",1),&liao_Set[1]},     {make_pair("liao",2),&liao_Set[2]},     {make_pair("liao",3),&liao_Set[3]},     {make_pair("liao",4),&liao_Set[4]},     {make_pair("liu",1),&liu_Set[1]},       {make_pair("liu",2),&liu_Set[2]},       
    {make_pair("liu",3),&liu_Set[3]},       {make_pair("liu",4),&liu_Set[4]},       {make_pair("lian",2),&lian_Set[2]},     {make_pair("lian",3),&lian_Set[3]},     {make_pair("lian",4),&lian_Set[4]},     {make_pair("lin",2),&lin_Set[2]},       
    {make_pair("lin",3),&lin_Set[3]},       {make_pair("lin",4),&lin_Set[4]},       {make_pair("ling",1),&ling_Set[1]},     {make_pair("ling",2),&ling_Set[2]},     {make_pair("ling",3),&ling_Set[3]},     {make_pair("ling",4),&ling_Set[4]},     
    {make_pair("liang",2),&liang_Set[2]},   {make_pair("liang",3),&liang_Set[3]},   {make_pair("liang",4),&liang_Set[4]},   {make_pair("lu",1),&lu_Set[1]},         {make_pair("lu",2),&lu_Set[2]},         {make_pair("lu",3),&lu_Set[3]},         
    {make_pair("lu",4),&lu_Set[4]},         {make_pair("luo",0),&luo_Set[0]},       {make_pair("luo",1),&luo_Set[1]},       {make_pair("luo",2),&luo_Set[2]},       {make_pair("luo",3),&luo_Set[3]},       {make_pair("luo",4),&luo_Set[4]},       
    {make_pair("luan",2),&luan_Set[2]},     {make_pair("luan",3),&luan_Set[3]},     {make_pair("luan",4),&luan_Set[4]},     {make_pair("lun",1),&lun_Set[1]},       {make_pair("lun",2),&lun_Set[2]},       {make_pair("lun",3),&lun_Set[3]},       
    {make_pair("lun",4),&lun_Set[4]},       {make_pair("lv",1),&lv_Set[1]},         {make_pair("lv",2),&lv_Set[2]},         {make_pair("lv",3),&lv_Set[3]},         {make_pair("lve",3),&lve_Set[3]},       {make_pair("lve",4),&lve_Set[4]},       
    {make_pair("ga",1),&ga_Set[1]},         {make_pair("ga",2),&ga_Set[2]},         {make_pair("ga",3),&ga_Set[3]},         {make_pair("ga",4),&ga_Set[4]},         {make_pair("ge",1),&ge_Set[1]},         {make_pair("ge",2),&ge_Set[2]},         
    {make_pair("ge",3),&ge_Set[3]},         {make_pair("ge",4),&ge_Set[4]},         {make_pair("gai",1),&gai_Set[1]},       {make_pair("gai",3),&gai_Set[3]},       {make_pair("gai",4),&gai_Set[4]},       {make_pair("gei",3),&gei_Set[3]},       
    {make_pair("gao",1),&gao_Set[1]},       {make_pair("gao",3),&gao_Set[3]},       {make_pair("gao",4),&gao_Set[4]},       {make_pair("gou",1),&gou_Set[1]},       {make_pair("gou",3),&gou_Set[3]},       {make_pair("gou",4),&gou_Set[4]},       
    {make_pair("gan",1),&gan_Set[1]},       {make_pair("gan",3),&gan_Set[3]},       {make_pair("gan",4),&gan_Set[4]},       {make_pair("gen",1),&gen_Set[1]},       {make_pair("gen",2),&gen_Set[2]},       {make_pair("gen",3),&gen_Set[3]},       
    {make_pair("gen",4),&gen_Set[4]},       {make_pair("gang",1),&gang_Set[1]},     {make_pair("gang",3),&gang_Set[3]},     {make_pair("gang",4),&gang_Set[4]},     {make_pair("geng",1),&geng_Set[1]},     {make_pair("geng",3),&geng_Set[3]},     
    {make_pair("geng",4),&geng_Set[4]},     {make_pair("gong",1),&gong_Set[1]},     {make_pair("gong",3),&gong_Set[3]},     {make_pair("gong",4),&gong_Set[4]},     {make_pair("gu",0),&gu_Set[0]},         {make_pair("gu",1),&gu_Set[1]},         
    {make_pair("gu",2),&gu_Set[2]},         {make_pair("gu",3),&gu_Set[3]},         {make_pair("gu",4),&gu_Set[4]},         {make_pair("gua",1),&gua_Set[1]},       {make_pair("gua",2),&gua_Set[2]},       {make_pair("gua",3),&gua_Set[3]},       
    {make_pair("gua",4),&gua_Set[4]},       {make_pair("guo",0),&guo_Set[0]},       {make_pair("guo",1),&guo_Set[1]},       {make_pair("guo",2),&guo_Set[2]},       {make_pair("guo",3),&guo_Set[3]},       {make_pair("guo",4),&guo_Set[4]},       
    {make_pair("guai",1),&guai_Set[1]},     {make_pair("guai",3),&guai_Set[3]},     {make_pair("guai",4),&guai_Set[4]},     {make_pair("gui",1),&gui_Set[1]},       {make_pair("gui",3),&gui_Set[3]},       {make_pair("gui",4),&gui_Set[4]},       
    {make_pair("guan",1),&guan_Set[1]},     {make_pair("guan",3),&guan_Set[3]},     {make_pair("guan",4),&guan_Set[4]},     {make_pair("gun",3),&gun_Set[3]},       {make_pair("gun",4),&gun_Set[4]},       {make_pair("guang",1),&guang_Set[1]},   
    {make_pair("guang",3),&guang_Set[3]},   {make_pair("guang",4),&guang_Set[4]},   {make_pair("ka",1),&ka_Set[1]},         {make_pair("ka",3),&ka_Set[3]},         {make_pair("ke",0),&ke_Set[0]},         {make_pair("ke",1),&ke_Set[1]},         
    {make_pair("ke",2),&ke_Set[2]},         {make_pair("ke",3),&ke_Set[3]},         {make_pair("ke",4),&ke_Set[4]},         {make_pair("kai",1),&kai_Set[1]},       {make_pair("kai",3),&kai_Set[3]},       {make_pair("kai",4),&kai_Set[4]},       
    {make_pair("kei",1),&kei_Set[1]},       {make_pair("kao",1),&kao_Set[1]},       {make_pair("kao",3),&kao_Set[3]},       {make_pair("kao",4),&kao_Set[4]},       {make_pair("kou",1),&kou_Set[1]},       {make_pair("kou",3),&kou_Set[3]},       
    {make_pair("kou",4),&kou_Set[4]},       {make_pair("kan",1),&kan_Set[1]},       {make_pair("kan",3),&kan_Set[3]},       {make_pair("kan",4),&kan_Set[4]},       {make_pair("ken",1),&ken_Set[1]},       {make_pair("ken",3),&ken_Set[3]},       
    {make_pair("ken",4),&ken_Set[4]},       {make_pair("kang",1),&kang_Set[1]},     {make_pair("kang",2),&kang_Set[2]},     {make_pair("kang",3),&kang_Set[3]},     {make_pair("kang",4),&kang_Set[4]},     {make_pair("keng",1),&keng_Set[1]},     
    {make_pair("keng",3),&keng_Set[3]},     {make_pair("kong",1),&kong_Set[1]},     {make_pair("kong",3),&kong_Set[3]},     {make_pair("kong",4),&kong_Set[4]},     {make_pair("ku",1),&ku_Set[1]},         {make_pair("ku",2),&ku_Set[2]},         
    {make_pair("ku",3),&ku_Set[3]},         {make_pair("ku",4),&ku_Set[4]},         {make_pair("kua",1),&kua_Set[1]},       {make_pair("kua",3),&kua_Set[3]},       {make_pair("kua",4),&kua_Set[4]},       {make_pair("kuo",4),&kuo_Set[4]},       
    {make_pair("kuai",3),&kuai_Set[3]},     {make_pair("kuai",4),&kuai_Set[4]},     {make_pair("kui",1),&kui_Set[1]},       {make_pair("kui",2),&kui_Set[2]},       {make_pair("kui",3),&kui_Set[3]},       {make_pair("kui",4),&kui_Set[4]},       
    {make_pair("kuan",1),&kuan_Set[1]},     {make_pair("kuan",3),&kuan_Set[3]},     {make_pair("kun",1),&kun_Set[1]},       {make_pair("kun",3),&kun_Set[3]},       {make_pair("kun",4),&kun_Set[4]},       {make_pair("kuang",1),&kuang_Set[1]},   
    {make_pair("kuang",2),&kuang_Set[2]},   {make_pair("kuang",3),&kuang_Set[3]},   {make_pair("kuang",4),&kuang_Set[4]},   {make_pair("ha",1),&ha_Set[1]},         {make_pair("ha",2),&ha_Set[2]},         {make_pair("ha",3),&ha_Set[3]},         
    {make_pair("ha",4),&ha_Set[4]},         {make_pair("he",1),&he_Set[1]},         {make_pair("he",2),&he_Set[2]},         {make_pair("he",3),&he_Set[3]},         {make_pair("he",4),&he_Set[4]},         {make_pair("hai",1),&hai_Set[1]},       
    {make_pair("hai",2),&hai_Set[2]},       {make_pair("hai",3),&hai_Set[3]},       {make_pair("hai",4),&hai_Set[4]},       {make_pair("hei",1),&hei_Set[1]},       {make_pair("hao",1),&hao_Set[1]},       {make_pair("hao",2),&hao_Set[2]},       
    {make_pair("hao",3),&hao_Set[3]},       {make_pair("hao",4),&hao_Set[4]},       {make_pair("hou",1),&hou_Set[1]},       {make_pair("hou",2),&hou_Set[2]},       {make_pair("hou",3),&hou_Set[3]},       {make_pair("hou",4),&hou_Set[4]},       
    {make_pair("han",1),&han_Set[1]},       {make_pair("han",2),&han_Set[2]},       {make_pair("han",3),&han_Set[3]},       {make_pair("han",4),&han_Set[4]},       {make_pair("hen",1),&hen_Set[1]},       {make_pair("hen",2),&hen_Set[2]},       
    {make_pair("hen",3),&hen_Set[3]},       {make_pair("hen",4),&hen_Set[4]},       {make_pair("hang",1),&hang_Set[1]},     {make_pair("hang",2),&hang_Set[2]},     {make_pair("hang",3),&hang_Set[3]},     {make_pair("hang",4),&hang_Set[4]},     
    {make_pair("heng",1),&heng_Set[1]},     {make_pair("heng",2),&heng_Set[2]},     {make_pair("heng",4),&heng_Set[4]},     {make_pair("hong",1),&hong_Set[1]},     {make_pair("hong",2),&hong_Set[2]},     {make_pair("hong",3),&hong_Set[3]},     
    {make_pair("hong",4),&hong_Set[4]},     {make_pair("hu",1),&hu_Set[1]},         {make_pair("hu",2),&hu_Set[2]},         {make_pair("hu",3),&hu_Set[3]},         {make_pair("hu",4),&hu_Set[4]},         {make_pair("hua",1),&hua_Set[1]},       
    {make_pair("hua",2),&hua_Set[2]},       {make_pair("hua",4),&hua_Set[4]},       {make_pair("huo",0),&huo_Set[0]},       {make_pair("huo",1),&huo_Set[1]},       {make_pair("huo",2),&huo_Set[2]},       {make_pair("huo",3),&huo_Set[3]},       
    {make_pair("huo",4),&huo_Set[4]},       {make_pair("huai",0),&huai_Set[0]},     {make_pair("huai",2),&huai_Set[2]},     {make_pair("huai",4),&huai_Set[4]},     {make_pair("hui",0),&hui_Set[0]},       {make_pair("hui",1),&hui_Set[1]},       
    {make_pair("hui",2),&hui_Set[2]},       {make_pair("hui",3),&hui_Set[3]},       {make_pair("hui",4),&hui_Set[4]},       {make_pair("huan",1),&huan_Set[1]},     {make_pair("huan",2),&huan_Set[2]},     {make_pair("huan",3),&huan_Set[3]},     
    {make_pair("huan",4),&huan_Set[4]},     {make_pair("hun",1),&hun_Set[1]},       {make_pair("hun",2),&hun_Set[2]},       {make_pair("hun",3),&hun_Set[3]},       {make_pair("hun",4),&hun_Set[4]},       {make_pair("huang",0),&huang_Set[0]},   
    {make_pair("huang",1),&huang_Set[1]},   {make_pair("huang",2),&huang_Set[2]},   {make_pair("huang",3),&huang_Set[3]},   {make_pair("huang",4),&huang_Set[4]},   {make_pair("ji",1),&ji_Set[1]},         {make_pair("ji",2),&ji_Set[2]},         
    {make_pair("ji",3),&ji_Set[3]},         {make_pair("ji",4),&ji_Set[4]},         {make_pair("ji",1),&ji_Set[1]},         {make_pair("ji",2),&ji_Set[2]},         {make_pair("ji",3),&ji_Set[3]},         {make_pair("ji",4),&ji_Set[4]},         
    {make_pair("jia",0),&jia_Set[0]},       {make_pair("jia",1),&jia_Set[1]},       {make_pair("jia",2),&jia_Set[2]},       {make_pair("jia",3),&jia_Set[3]},       {make_pair("jia",4),&jia_Set[4]},       {make_pair("jie",0),&jie_Set[0]},       
    {make_pair("jie",1),&jie_Set[1]},       {make_pair("jie",2),&jie_Set[2]},       {make_pair("jie",3),&jie_Set[3]},       {make_pair("jie",4),&jie_Set[4]},       {make_pair("jiao",1),&jiao_Set[1]},     {make_pair("jiao",2),&jiao_Set[2]},     
    {make_pair("jiao",3),&jiao_Set[3]},     {make_pair("jiao",4),&jiao_Set[4]},     {make_pair("jiu",1),&jiu_Set[1]},       {make_pair("jiu",3),&jiu_Set[3]},       {make_pair("jiu",4),&jiu_Set[4]},       {make_pair("jian",1),&jian_Set[1]},     
    {make_pair("jian",3),&jian_Set[3]},     {make_pair("jian",4),&jian_Set[4]},     {make_pair("jin",1),&jin_Set[1]},       {make_pair("jin",3),&jin_Set[3]},       {make_pair("jin",4),&jin_Set[4]},       {make_pair("jing",1),&jing_Set[1]},     
    {make_pair("jing",3),&jing_Set[3]},     {make_pair("jing",4),&jing_Set[4]},     {make_pair("jiang",1),&jiang_Set[1]},   {make_pair("jiang",3),&jiang_Set[3]},   {make_pair("jiang",4),&jiang_Set[4]},   {make_pair("jiong",1),&jiong_Set[1]},   
    {make_pair("jiong",3),&jiong_Set[3]},   {make_pair("jiong",4),&jiong_Set[4]},   {make_pair("ju",1),&ju_Set[1]},         {make_pair("ju",2),&ju_Set[2]},         {make_pair("ju",3),&ju_Set[3]},         {make_pair("ju",4),&ju_Set[4]},         
    {make_pair("juan",1),&juan_Set[1]},     {make_pair("juan",3),&juan_Set[3]},     {make_pair("juan",4),&juan_Set[4]},     {make_pair("jun",1),&jun_Set[1]},       {make_pair("jun",3),&jun_Set[3]},       {make_pair("jun",4),&jun_Set[4]},       
    {make_pair("qi",1),&qi_Set[1]},         {make_pair("qi",2),&qi_Set[2]},         {make_pair("qi",3),&qi_Set[3]},         {make_pair("qi",4),&qi_Set[4]},         {make_pair("qi",1),&qi_Set[1]},         {make_pair("qi",2),&qi_Set[2]},         
    {make_pair("qi",3),&qi_Set[3]},         {make_pair("qi",4),&qi_Set[4]},         {make_pair("qia",1),&qia_Set[1]},       {make_pair("qia",2),&qia_Set[2]},       {make_pair("qia",3),&qia_Set[3]},       {make_pair("qia",4),&qia_Set[4]},       
    {make_pair("qie",1),&qie_Set[1]},       {make_pair("qie",2),&qie_Set[2]},       {make_pair("qie",3),&qie_Set[3]},       {make_pair("qie",4),&qie_Set[4]},       {make_pair("qiao",1),&qiao_Set[1]},     {make_pair("qiao",2),&qiao_Set[2]},     
    {make_pair("qiao",3),&qiao_Set[3]},     {make_pair("qiao",4),&qiao_Set[4]},     {make_pair("qiu",1),&qiu_Set[1]},       {make_pair("qiu",2),&qiu_Set[2]},       {make_pair("qiu",3),&qiu_Set[3]},       {make_pair("qiu",4),&qiu_Set[4]},       
    {make_pair("qian",1),&qian_Set[1]},     {make_pair("qian",2),&qian_Set[2]},     {make_pair("qian",3),&qian_Set[3]},     {make_pair("qian",4),&qian_Set[4]},     {make_pair("qin",1),&qin_Set[1]},       {make_pair("qin",2),&qin_Set[2]},       
    {make_pair("qin",3),&qin_Set[3]},       {make_pair("qin",4),&qin_Set[4]},       {make_pair("qing",1),&qing_Set[1]},     {make_pair("qing",2),&qing_Set[2]},     {make_pair("qing",3),&qing_Set[3]},     {make_pair("qing",4),&qing_Set[4]},     
    {make_pair("qiang",1),&qiang_Set[1]},   {make_pair("qiang",2),&qiang_Set[2]},   {make_pair("qiang",3),&qiang_Set[3]},   {make_pair("qiang",4),&qiang_Set[4]},   {make_pair("qiong",1),&qiong_Set[1]},   {make_pair("qiong",2),&qiong_Set[2]},   
    {make_pair("qiong",4),&qiong_Set[4]},   {make_pair("qu",0),&qu_Set[0]},         {make_pair("qu",1),&qu_Set[1]},         {make_pair("qu",2),&qu_Set[2]},         {make_pair("qu",3),&qu_Set[3]},         {make_pair("qu",4),&qu_Set[4]},         
    {make_pair("quan",1),&quan_Set[1]},     {make_pair("quan",2),&quan_Set[2]},     {make_pair("quan",3),&quan_Set[3]},     {make_pair("quan",4),&quan_Set[4]},     {make_pair("qun",1),&qun_Set[1]},       {make_pair("qun",2),&qun_Set[2]},       
    {make_pair("qun",3),&qun_Set[3]},       {make_pair("xi",1),&xi_Set[1]},         {make_pair("xi",2),&xi_Set[2]},         {make_pair("xi",3),&xi_Set[3]},         {make_pair("xi",4),&xi_Set[4]},         {make_pair("xi",1),&xi_Set[1]},         
    {make_pair("xi",2),&xi_Set[2]},         {make_pair("xi",3),&xi_Set[3]},         {make_pair("xi",4),&xi_Set[4]},         {make_pair("xia",1),&xia_Set[1]},       {make_pair("xia",2),&xia_Set[2]},       {make_pair("xia",3),&xia_Set[3]},       
    {make_pair("xia",4),&xia_Set[4]},       {make_pair("xie",1),&xie_Set[1]},       {make_pair("xie",2),&xie_Set[2]},       {make_pair("xie",3),&xie_Set[3]},       {make_pair("xie",4),&xie_Set[4]},       {make_pair("xiao",1),&xiao_Set[1]},     
    {make_pair("xiao",2),&xiao_Set[2]},     {make_pair("xiao",3),&xiao_Set[3]},     {make_pair("xiao",4),&xiao_Set[4]},     {make_pair("xiu",1),&xiu_Set[1]},       {make_pair("xiu",2),&xiu_Set[2]},       {make_pair("xiu",3),&xiu_Set[3]},       
    {make_pair("xiu",4),&xiu_Set[4]},       {make_pair("xian",1),&xian_Set[1]},     {make_pair("xian",2),&xian_Set[2]},     {make_pair("xian",3),&xian_Set[3]},     {make_pair("xian",4),&xian_Set[4]},     {make_pair("xin",1),&xin_Set[1]},       
    {make_pair("xin",2),&xin_Set[2]},       {make_pair("xin",3),&xin_Set[3]},       {make_pair("xin",4),&xin_Set[4]},       {make_pair("xing",1),&xing_Set[1]},     {make_pair("xing",2),&xing_Set[2]},     {make_pair("xing",3),&xing_Set[3]},     
    {make_pair("xing",4),&xing_Set[4]},     {make_pair("xiang",1),&xiang_Set[1]},   {make_pair("xiang",2),&xiang_Set[2]},   {make_pair("xiang",3),&xiang_Set[3]},   {make_pair("xiang",4),&xiang_Set[4]},   {make_pair("xiong",1),&xiong_Set[1]},   
    {make_pair("xiong",2),&xiong_Set[2]},   {make_pair("xiong",3),&xiong_Set[3]},   {make_pair("xiong",4),&xiong_Set[4]},   {make_pair("xu",0),&xu_Set[0]},         {make_pair("xu",1),&xu_Set[1]},         {make_pair("xu",2),&xu_Set[2]},         
    {make_pair("xu",3),&xu_Set[3]},         {make_pair("xu",4),&xu_Set[4]},         {make_pair("xuan",1),&xuan_Set[1]},     {make_pair("xuan",2),&xuan_Set[2]},     {make_pair("xuan",3),&xuan_Set[3]},     {make_pair("xuan",4),&xuan_Set[4]},     
    {make_pair("xun",1),&xun_Set[1]},       {make_pair("xun",2),&xun_Set[2]},       {make_pair("xun",4),&xun_Set[4]},       {make_pair("zhi",1),&zhi_Set[1]},       {make_pair("zhi",2),&zhi_Set[2]},       {make_pair("zhi",3),&zhi_Set[3]},       
    {make_pair("zhi",4),&zhi_Set[4]},       {make_pair("zha",0),&zha_Set[0]},       {make_pair("zha",1),&zha_Set[1]},       {make_pair("zha",2),&zha_Set[2]},       {make_pair("zha",3),&zha_Set[3]},       {make_pair("zha",4),&zha_Set[4]},       
    {make_pair("zhe",0),&zhe_Set[0]},       {make_pair("zhe",1),&zhe_Set[1]},       {make_pair("zhe",2),&zhe_Set[2]},       {make_pair("zhe",3),&zhe_Set[3]},       {make_pair("zhe",4),&zhe_Set[4]},       {make_pair("zhai",1),&zhai_Set[1]},     
    {make_pair("zhai",2),&zhai_Set[2]},     {make_pair("zhai",3),&zhai_Set[3]},     {make_pair("zhai",4),&zhai_Set[4]},     {make_pair("zhei",4),&zhei_Set[4]},     {make_pair("zhao",1),&zhao_Set[1]},     {make_pair("zhao",2),&zhao_Set[2]},     
    {make_pair("zhao",3),&zhao_Set[3]},     {make_pair("zhao",4),&zhao_Set[4]},     {make_pair("zhou",1),&zhou_Set[1]},     {make_pair("zhou",2),&zhou_Set[2]},     {make_pair("zhou",3),&zhou_Set[3]},     {make_pair("zhou",4),&zhou_Set[4]},     
    {make_pair("zhan",1),&zhan_Set[1]},     {make_pair("zhan",2),&zhan_Set[2]},     {make_pair("zhan",3),&zhan_Set[3]},     {make_pair("zhan",4),&zhan_Set[4]},     {make_pair("zhen",1),&zhen_Set[1]},     {make_pair("zhen",2),&zhen_Set[2]},     
    {make_pair("zhen",3),&zhen_Set[3]},     {make_pair("zhen",4),&zhen_Set[4]},     {make_pair("zhang",1),&zhang_Set[1]},   {make_pair("zhang",3),&zhang_Set[3]},   {make_pair("zhang",4),&zhang_Set[4]},   {make_pair("zheng",1),&zheng_Set[1]},   
    {make_pair("zheng",3),&zheng_Set[3]},   {make_pair("zheng",4),&zheng_Set[4]},   {make_pair("zhong",1),&zhong_Set[1]},   {make_pair("zhong",3),&zhong_Set[3]},   {make_pair("zhong",4),&zhong_Set[4]},   {make_pair("zhi",1),&zhi_Set[1]},       
    {make_pair("zhi",2),&zhi_Set[2]},       {make_pair("zhi",3),&zhi_Set[3]},       {make_pair("zhi",4),&zhi_Set[4]},       {make_pair("zhu",1),&zhu_Set[1]},       {make_pair("zhu",2),&zhu_Set[2]},       {make_pair("zhu",3),&zhu_Set[3]},       
    {make_pair("zhu",4),&zhu_Set[4]},       {make_pair("zhua",1),&zhua_Set[1]},     {make_pair("zhua",3),&zhua_Set[3]},     {make_pair("zhuo",1),&zhuo_Set[1]},     {make_pair("zhuo",2),&zhuo_Set[2]},     {make_pair("zhuo",4),&zhuo_Set[4]},     
    {make_pair("zhuai",1),&zhuai_Set[1]},   {make_pair("zhuai",3),&zhuai_Set[3]},   {make_pair("zhuai",4),&zhuai_Set[4]},   {make_pair("zhui",1),&zhui_Set[1]},     {make_pair("zhui",3),&zhui_Set[3]},     {make_pair("zhui",4),&zhui_Set[4]},     
    {make_pair("zhuan",1),&zhuan_Set[1]},   {make_pair("zhuan",3),&zhuan_Set[3]},   {make_pair("zhuan",4),&zhuan_Set[4]},   {make_pair("zhun",1),&zhun_Set[1]},     {make_pair("zhun",3),&zhun_Set[3]},     {make_pair("zhun",4),&zhun_Set[4]},     
    {make_pair("zhuang",1),&zhuang_Set[1]}, {make_pair("zhuang",3),&zhuang_Set[3]}, {make_pair("zhuang",4),&zhuang_Set[4]}, {make_pair("chi",1),&chi_Set[1]},       {make_pair("chi",2),&chi_Set[2]},       {make_pair("chi",3),&chi_Set[3]},       
    {make_pair("chi",4),&chi_Set[4]},       {make_pair("cha",1),&cha_Set[1]},       {make_pair("cha",2),&cha_Set[2]},       {make_pair("cha",3),&cha_Set[3]},       {make_pair("cha",4),&cha_Set[4]},       {make_pair("che",1),&che_Set[1]},       
    {make_pair("che",2),&che_Set[2]},       {make_pair("che",3),&che_Set[3]},       {make_pair("che",4),&che_Set[4]},       {make_pair("chai",1),&chai_Set[1]},     {make_pair("chai",2),&chai_Set[2]},     {make_pair("chai",3),&chai_Set[3]},     
    {make_pair("chai",4),&chai_Set[4]},     {make_pair("chao",1),&chao_Set[1]},     {make_pair("chao",2),&chao_Set[2]},     {make_pair("chao",3),&chao_Set[3]},     {make_pair("chao",4),&chao_Set[4]},     {make_pair("chou",1),&chou_Set[1]},     
    {make_pair("chou",2),&chou_Set[2]},     {make_pair("chou",3),&chou_Set[3]},     {make_pair("chou",4),&chou_Set[4]},     {make_pair("chan",1),&chan_Set[1]},     {make_pair("chan",2),&chan_Set[2]},     {make_pair("chan",3),&chan_Set[3]},     
    {make_pair("chan",4),&chan_Set[4]},     {make_pair("chen",0),&chen_Set[0]},     {make_pair("chen",1),&chen_Set[1]},     {make_pair("chen",2),&chen_Set[2]},     {make_pair("chen",3),&chen_Set[3]},     {make_pair("chen",4),&chen_Set[4]},     
    {make_pair("chang",1),&chang_Set[1]},   {make_pair("chang",2),&chang_Set[2]},   {make_pair("chang",3),&chang_Set[3]},   {make_pair("chang",4),&chang_Set[4]},   {make_pair("cheng",1),&cheng_Set[1]},   {make_pair("cheng",2),&cheng_Set[2]},   
    {make_pair("cheng",3),&cheng_Set[3]},   {make_pair("cheng",4),&cheng_Set[4]},   {make_pair("chong",1),&chong_Set[1]},   {make_pair("chong",2),&chong_Set[2]},   {make_pair("chong",3),&chong_Set[3]},   {make_pair("chong",4),&chong_Set[4]},   
    {make_pair("chi",1),&chi_Set[1]},       {make_pair("chi",2),&chi_Set[2]},       {make_pair("chi",3),&chi_Set[3]},       {make_pair("chi",4),&chi_Set[4]},       {make_pair("chu",1),&chu_Set[1]},       {make_pair("chu",2),&chu_Set[2]},       
    {make_pair("chu",3),&chu_Set[3]},       {make_pair("chu",4),&chu_Set[4]},       {make_pair("chua",1),&chua_Set[1]},     {make_pair("chua",3),&chua_Set[3]},     {make_pair("chua",4),&chua_Set[4]},     {make_pair("chuo",1),&chuo_Set[1]},     
    {make_pair("chuo",4),&chuo_Set[4]},     {make_pair("chuai",1),&chuai_Set[1]},   {make_pair("chuai",2),&chuai_Set[2]},   {make_pair("chuai",3),&chuai_Set[3]},   {make_pair("chuai",4),&chuai_Set[4]},   {make_pair("chui",1),&chui_Set[1]},     
    {make_pair("chui",2),&chui_Set[2]},     {make_pair("chui",3),&chui_Set[3]},     {make_pair("chui",4),&chui_Set[4]},     {make_pair("chuan",1),&chuan_Set[1]},   {make_pair("chuan",2),&chuan_Set[2]},   {make_pair("chuan",3),&chuan_Set[3]},   
    {make_pair("chuan",4),&chuan_Set[4]},   {make_pair("chun",1),&chun_Set[1]},     {make_pair("chun",2),&chun_Set[2]},     {make_pair("chun",3),&chun_Set[3]},     {make_pair("chuang",1),&chuang_Set[1]}, {make_pair("chuang",2),&chuang_Set[2]}, 
    {make_pair("chuang",3),&chuang_Set[3]}, {make_pair("chuang",4),&chuang_Set[4]}, {make_pair("shi",0),&shi_Set[0]},       {make_pair("shi",1),&shi_Set[1]},       {make_pair("shi",2),&shi_Set[2]},       {make_pair("shi",3),&shi_Set[3]},       
    {make_pair("shi",4),&shi_Set[4]},       {make_pair("sha",1),&sha_Set[1]},       {make_pair("sha",3),&sha_Set[3]},       {make_pair("sha",4),&sha_Set[4]},       {make_pair("she",1),&she_Set[1]},       {make_pair("she",2),&she_Set[2]},       
    {make_pair("she",3),&she_Set[3]},       {make_pair("she",4),&she_Set[4]},       {make_pair("shai",1),&shai_Set[1]},     {make_pair("shai",3),&shai_Set[3]},     {make_pair("shai",4),&shai_Set[4]},     {make_pair("shei",2),&shei_Set[2]},     
    {make_pair("shao",1),&shao_Set[1]},     {make_pair("shao",2),&shao_Set[2]},     {make_pair("shao",3),&shao_Set[3]},     {make_pair("shao",4),&shao_Set[4]},     {make_pair("shou",1),&shou_Set[1]},     {make_pair("shou",2),&shou_Set[2]},     
    {make_pair("shou",3),&shou_Set[3]},     {make_pair("shou",4),&shou_Set[4]},     {make_pair("shan",1),&shan_Set[1]},     {make_pair("shan",2),&shan_Set[2]},     {make_pair("shan",3),&shan_Set[3]},     {make_pair("shan",4),&shan_Set[4]},     
    {make_pair("shen",1),&shen_Set[1]},     {make_pair("shen",2),&shen_Set[2]},     {make_pair("shen",3),&shen_Set[3]},     {make_pair("shen",4),&shen_Set[4]},     {make_pair("shang",1),&shang_Set[1]},   {make_pair("shang",3),&shang_Set[3]},   
    {make_pair("shang",4),&shang_Set[4]},   {make_pair("sheng",1),&sheng_Set[1]},   {make_pair("sheng",2),&sheng_Set[2]},   {make_pair("sheng",3),&sheng_Set[3]},   {make_pair("sheng",4),&sheng_Set[4]},   {make_pair("shi",0),&shi_Set[0]},       
    {make_pair("shi",1),&shi_Set[1]},       {make_pair("shi",2),&shi_Set[2]},       {make_pair("shi",3),&shi_Set[3]},       {make_pair("shi",4),&shi_Set[4]},       {make_pair("shu",1),&shu_Set[1]},       {make_pair("shu",2),&shu_Set[2]},       
    {make_pair("shu",3),&shu_Set[3]},       {make_pair("shu",4),&shu_Set[4]},       {make_pair("shua",1),&shua_Set[1]},     {make_pair("shua",3),&shua_Set[3]},     {make_pair("shua",4),&shua_Set[4]},     {make_pair("shuo",1),&shuo_Set[1]},     
    {make_pair("shuo",2),&shuo_Set[2]},     {make_pair("shuo",4),&shuo_Set[4]},     {make_pair("shuai",1),&shuai_Set[1]},   {make_pair("shuai",3),&shuai_Set[3]},   {make_pair("shuai",4),&shuai_Set[4]},   {make_pair("shui",2),&shui_Set[2]},     
    {make_pair("shui",3),&shui_Set[3]},     {make_pair("shui",4),&shui_Set[4]},     {make_pair("shuan",1),&shuan_Set[1]},   {make_pair("shuan",4),&shuan_Set[4]},   {make_pair("shun",3),&shun_Set[3]},     {make_pair("shun",4),&shun_Set[4]},     
    {make_pair("shuang",1),&shuang_Set[1]}, {make_pair("shuang",3),&shuang_Set[3]}, {make_pair("shuang",4),&shuang_Set[4]}, {make_pair("ri",4),&ri_Set[4]},         {make_pair("re",2),&re_Set[2]},         {make_pair("re",3),&re_Set[3]},         
    {make_pair("re",4),&re_Set[4]},         {make_pair("rao",2),&rao_Set[2]},       {make_pair("rao",3),&rao_Set[3]},       {make_pair("rao",4),&rao_Set[4]},       {make_pair("rou",2),&rou_Set[2]},       {make_pair("rou",3),&rou_Set[3]},       
    {make_pair("rou",4),&rou_Set[4]},       {make_pair("ran",2),&ran_Set[2]},       {make_pair("ran",3),&ran_Set[3]},       {make_pair("ran",4),&ran_Set[4]},       {make_pair("ren",2),&ren_Set[2]},       {make_pair("ren",3),&ren_Set[3]},       
    {make_pair("ren",4),&ren_Set[4]},       {make_pair("rang",1),&rang_Set[1]},     {make_pair("rang",2),&rang_Set[2]},     {make_pair("rang",3),&rang_Set[3]},     {make_pair("rang",4),&rang_Set[4]},     {make_pair("reng",1),&reng_Set[1]},     
    {make_pair("reng",2),&reng_Set[2]},     {make_pair("reng",4),&reng_Set[4]},     {make_pair("rong",2),&rong_Set[2]},     {make_pair("rong",3),&rong_Set[3]},     {make_pair("rong",4),&rong_Set[4]},     {make_pair("ri",4),&ri_Set[4]},         
    {make_pair("ru",2),&ru_Set[2]},         {make_pair("ru",3),&ru_Set[3]},         {make_pair("ru",4),&ru_Set[4]},         {make_pair("rua",2),&rua_Set[2]},       {make_pair("ruo",2),&ruo_Set[2]},       {make_pair("ruo",4),&ruo_Set[4]},       
    {make_pair("rui",2),&rui_Set[2]},       {make_pair("rui",3),&rui_Set[3]},       {make_pair("rui",4),&rui_Set[4]},       {make_pair("ruan",2),&ruan_Set[2]},     {make_pair("ruan",3),&ruan_Set[3]},     {make_pair("ruan",4),&ruan_Set[4]},     
    {make_pair("run",2),&run_Set[2]},       {make_pair("run",3),&run_Set[3]},       {make_pair("run",4),&run_Set[4]},       {make_pair("zi",0),&zi_Set[0]},         {make_pair("zi",1),&zi_Set[1]},         {make_pair("zi",2),&zi_Set[2]},         
    {make_pair("zi",3),&zi_Set[3]},         {make_pair("zi",4),&zi_Set[4]},         {make_pair("za",1),&za_Set[1]},         {make_pair("za",2),&za_Set[2]},         {make_pair("za",3),&za_Set[3]},         {make_pair("za",4),&za_Set[4]},         
    {make_pair("ze",2),&ze_Set[2]},         {make_pair("ze",4),&ze_Set[4]},         {make_pair("zai",1),&zai_Set[1]},       {make_pair("zai",3),&zai_Set[3]},       {make_pair("zai",4),&zai_Set[4]},       {make_pair("zei",2),&zei_Set[2]},       
    {make_pair("zao",1),&zao_Set[1]},       {make_pair("zao",2),&zao_Set[2]},       {make_pair("zao",3),&zao_Set[3]},       {make_pair("zao",4),&zao_Set[4]},       {make_pair("zou",1),&zou_Set[1]},       {make_pair("zou",3),&zou_Set[3]},       
    {make_pair("zou",4),&zou_Set[4]},       {make_pair("zan",0),&zan_Set[0]},       {make_pair("zan",1),&zan_Set[1]},       {make_pair("zan",2),&zan_Set[2]},       {make_pair("zan",3),&zan_Set[3]},       {make_pair("zan",4),&zan_Set[4]},       
    {make_pair("zen",1),&zen_Set[1]},       {make_pair("zen",3),&zen_Set[3]},       {make_pair("zen",4),&zen_Set[4]},       {make_pair("zang",1),&zang_Set[1]},     {make_pair("zang",3),&zang_Set[3]},     {make_pair("zang",4),&zang_Set[4]},     
    {make_pair("zeng",1),&zeng_Set[1]},     {make_pair("zeng",3),&zeng_Set[3]},     {make_pair("zeng",4),&zeng_Set[4]},     {make_pair("zong",1),&zong_Set[1]},     {make_pair("zong",3),&zong_Set[3]},     {make_pair("zong",4),&zong_Set[4]},     
    {make_pair("zi",0),&zi_Set[0]},         {make_pair("zi",1),&zi_Set[1]},         {make_pair("zi",2),&zi_Set[2]},         {make_pair("zi",3),&zi_Set[3]},         {make_pair("zi",4),&zi_Set[4]},         {make_pair("zu",1),&zu_Set[1]},         
    {make_pair("zu",2),&zu_Set[2]},         {make_pair("zu",3),&zu_Set[3]},         {make_pair("zu",4),&zu_Set[4]},         {make_pair("zuo",0),&zuo_Set[0]},       {make_pair("zuo",1),&zuo_Set[1]},       {make_pair("zuo",2),&zuo_Set[2]},       
    {make_pair("zuo",3),&zuo_Set[3]},       {make_pair("zuo",4),&zuo_Set[4]},       {make_pair("zui",1),&zui_Set[1]},       {make_pair("zui",2),&zui_Set[2]},       {make_pair("zui",3),&zui_Set[3]},       {make_pair("zui",4),&zui_Set[4]},       
    {make_pair("zuan",1),&zuan_Set[1]},     {make_pair("zuan",3),&zuan_Set[3]},     {make_pair("zuan",4),&zuan_Set[4]},     {make_pair("zun",1),&zun_Set[1]},       {make_pair("zun",2),&zun_Set[2]},       {make_pair("zun",3),&zun_Set[3]},       
    {make_pair("zun",4),&zun_Set[4]},       {make_pair("ci",1),&ci_Set[1]},         {make_pair("ci",2),&ci_Set[2]},         {make_pair("ci",3),&ci_Set[3]},         {make_pair("ci",4),&ci_Set[4]},         {make_pair("ca",1),&ca_Set[1]},         
    {make_pair("ca",3),&ca_Set[3]},         {make_pair("ca",4),&ca_Set[4]},         {make_pair("ce",4),&ce_Set[4]},         {make_pair("cai",1),&cai_Set[1]},       {make_pair("cai",2),&cai_Set[2]},       {make_pair("cai",3),&cai_Set[3]},       
    {make_pair("cai",4),&cai_Set[4]},       {make_pair("cao",1),&cao_Set[1]},       {make_pair("cao",2),&cao_Set[2]},       {make_pair("cao",3),&cao_Set[3]},       {make_pair("cao",4),&cao_Set[4]},       {make_pair("cou",1),&cou_Set[1]},       
    {make_pair("cou",2),&cou_Set[2]},       {make_pair("cou",3),&cou_Set[3]},       {make_pair("cou",4),&cou_Set[4]},       {make_pair("can",1),&can_Set[1]},       {make_pair("can",2),&can_Set[2]},       {make_pair("can",3),&can_Set[3]},       
    {make_pair("can",4),&can_Set[4]},       {make_pair("cen",1),&cen_Set[1]},       {make_pair("cen",2),&cen_Set[2]},       {make_pair("cang",1),&cang_Set[1]},     {make_pair("cang",2),&cang_Set[2]},     {make_pair("cang",3),&cang_Set[3]},     
    {make_pair("cang",4),&cang_Set[4]},     {make_pair("ceng",1),&ceng_Set[1]},     {make_pair("ceng",2),&ceng_Set[2]},     {make_pair("ceng",4),&ceng_Set[4]},     {make_pair("cong",1),&cong_Set[1]},     {make_pair("cong",2),&cong_Set[2]},     
    {make_pair("cong",3),&cong_Set[3]},     {make_pair("cong",4),&cong_Set[4]},     {make_pair("ci",1),&ci_Set[1]},         {make_pair("ci",2),&ci_Set[2]},         {make_pair("ci",3),&ci_Set[3]},         {make_pair("ci",4),&ci_Set[4]},         
    {make_pair("cu",1),&cu_Set[1]},         {make_pair("cu",2),&cu_Set[2]},         {make_pair("cu",3),&cu_Set[3]},         {make_pair("cu",4),&cu_Set[4]},         {make_pair("cuo",1),&cuo_Set[1]},       {make_pair("cuo",2),&cuo_Set[2]},       
    {make_pair("cuo",3),&cuo_Set[3]},       {make_pair("cuo",4),&cuo_Set[4]},       {make_pair("cui",1),&cui_Set[1]},       {make_pair("cui",3),&cui_Set[3]},       {make_pair("cui",4),&cui_Set[4]},       {make_pair("cuan",1),&cuan_Set[1]},     
    {make_pair("cuan",2),&cuan_Set[2]},     {make_pair("cuan",4),&cuan_Set[4]},     {make_pair("cun",1),&cun_Set[1]},       {make_pair("cun",2),&cun_Set[2]},       {make_pair("cun",3),&cun_Set[3]},       {make_pair("cun",4),&cun_Set[4]},       
    {make_pair("si",0),&si_Set[0]},         {make_pair("si",1),&si_Set[1]},         {make_pair("si",2),&si_Set[2]},         {make_pair("si",3),&si_Set[3]},         {make_pair("si",4),&si_Set[4]},         {make_pair("sa",0),&sa_Set[0]},         
    {make_pair("sa",1),&sa_Set[1]},         {make_pair("sa",3),&sa_Set[3]},         {make_pair("sa",4),&sa_Set[4]},         {make_pair("se",4),&se_Set[4]},         {make_pair("sai",1),&sai_Set[1]},       {make_pair("sai",3),&sai_Set[3]},       
    {make_pair("sai",4),&sai_Set[4]},       {make_pair("sao",1),&sao_Set[1]},       {make_pair("sao",3),&sao_Set[3]},       {make_pair("sao",4),&sao_Set[4]},       {make_pair("sou",1),&sou_Set[1]},       {make_pair("sou",3),&sou_Set[3]},       
    {make_pair("sou",4),&sou_Set[4]},       {make_pair("san",0),&san_Set[0]},       {make_pair("san",1),&san_Set[1]},       {make_pair("san",3),&san_Set[3]},       {make_pair("san",4),&san_Set[4]},       {make_pair("sen",1),&sen_Set[1]},       
    {make_pair("sen",3),&sen_Set[3]},       {make_pair("sang",1),&sang_Set[1]},     {make_pair("sang",3),&sang_Set[3]},     {make_pair("sang",4),&sang_Set[4]},     {make_pair("seng",1),&seng_Set[1]},     {make_pair("seng",4),&seng_Set[4]},     
    {make_pair("song",1),&song_Set[1]},     {make_pair("song",2),&song_Set[2]},     {make_pair("song",3),&song_Set[3]},     {make_pair("song",4),&song_Set[4]},     {make_pair("si",0),&si_Set[0]},         {make_pair("si",1),&si_Set[1]},         
    {make_pair("si",2),&si_Set[2]},         {make_pair("si",3),&si_Set[3]},         {make_pair("si",4),&si_Set[4]},         {make_pair("su",1),&su_Set[1]},         {make_pair("su",2),&su_Set[2]},         {make_pair("su",3),&su_Set[3]},         
    {make_pair("su",4),&su_Set[4]},         {make_pair("suo",1),&suo_Set[1]},       {make_pair("suo",2),&suo_Set[2]},       {make_pair("suo",3),&suo_Set[3]},       {make_pair("suo",4),&suo_Set[4]},       {make_pair("sui",1),&sui_Set[1]},       
    {make_pair("sui",2),&sui_Set[2]},       {make_pair("sui",3),&sui_Set[3]},       {make_pair("sui",4),&sui_Set[4]},       {make_pair("suan",1),&suan_Set[1]},     {make_pair("suan",3),&suan_Set[3]},     {make_pair("suan",4),&suan_Set[4]},     
    {make_pair("sun",1),&sun_Set[1]},       {make_pair("sun",3),&sun_Set[3]},       {make_pair("sun",4),&sun_Set[4]},       {make_pair("yi",1),&yi_Set[1]},         {make_pair("yi",2),&yi_Set[2]},         {make_pair("yi",3),&yi_Set[3]},         
    {make_pair("yi",4),&yi_Set[4]},         {make_pair("ya",0),&ya_Set[0]},         {make_pair("ya",1),&ya_Set[1]},         {make_pair("ya",2),&ya_Set[2]},         {make_pair("ya",3),&ya_Set[3]},         {make_pair("ya",4),&ya_Set[4]},         
    {make_pair("yo",0),&yo_Set[0]},         {make_pair("yo",1),&yo_Set[1]},         {make_pair("ye",1),&ye_Set[1]},         {make_pair("ye",2),&ye_Set[2]},         {make_pair("ye",3),&ye_Set[3]},         {make_pair("ye",4),&ye_Set[4]},         
    {make_pair("yao",1),&yao_Set[1]},       {make_pair("yao",2),&yao_Set[2]},       {make_pair("yao",3),&yao_Set[3]},       {make_pair("yao",4),&yao_Set[4]},       {make_pair("you",1),&you_Set[1]},       {make_pair("you",2),&you_Set[2]},       
    {make_pair("you",3),&you_Set[3]},       {make_pair("you",4),&you_Set[4]},       {make_pair("yan",1),&yan_Set[1]},       {make_pair("yan",2),&yan_Set[2]},       {make_pair("yan",3),&yan_Set[3]},       {make_pair("yan",4),&yan_Set[4]},       
    {make_pair("yang",1),&yang_Set[1]},     {make_pair("yang",2),&yang_Set[2]},     {make_pair("yang",3),&yang_Set[3]},     {make_pair("yang",4),&yang_Set[4]},     {make_pair("yong",1),&yong_Set[1]},     {make_pair("yong",2),&yong_Set[2]},     
    {make_pair("yong",3),&yong_Set[3]},     {make_pair("yong",4),&yong_Set[4]},     {make_pair("yi",1),&yi_Set[1]},         {make_pair("yi",2),&yi_Set[2]},         {make_pair("yi",3),&yi_Set[3]},         {make_pair("yi",4),&yi_Set[4]},         
    {make_pair("yin",1),&yin_Set[1]},       {make_pair("yin",2),&yin_Set[2]},       {make_pair("yin",3),&yin_Set[3]},       {make_pair("yin",4),&yin_Set[4]},       {make_pair("ying",1),&ying_Set[1]},     {make_pair("ying",2),&ying_Set[2]},     
    {make_pair("ying",3),&ying_Set[3]},     {make_pair("ying",4),&ying_Set[4]},     {make_pair("yu",1),&yu_Set[1]},         {make_pair("yu",2),&yu_Set[2]},         {make_pair("yu",3),&yu_Set[3]},         {make_pair("yu",4),&yu_Set[4]},         
    {make_pair("yuan",1),&yuan_Set[1]},     {make_pair("yuan",2),&yuan_Set[2]},     {make_pair("yuan",3),&yuan_Set[3]},     {make_pair("yuan",4),&yuan_Set[4]},     {make_pair("yun",1),&yun_Set[1]},       {make_pair("yun",2),&yun_Set[2]},       
    {make_pair("yun",3),&yun_Set[3]},       {make_pair("yun",4),&yun_Set[4]},       {make_pair("wa",0),&wa_Set[0]},         {make_pair("wa",1),&wa_Set[1]},         {make_pair("wa",2),&wa_Set[2]},         {make_pair("wa",3),&wa_Set[3]},         
    {make_pair("wa",4),&wa_Set[4]},         {make_pair("wo",1),&wo_Set[1]},         {make_pair("wo",3),&wo_Set[3]},         {make_pair("wo",4),&wo_Set[4]},         {make_pair("wai",0),&wai_Set[0]},       {make_pair("wai",1),&wai_Set[1]},       
    {make_pair("wai",3),&wai_Set[3]},       {make_pair("wai",4),&wai_Set[4]},       {make_pair("wei",1),&wei_Set[1]},       {make_pair("wei",2),&wei_Set[2]},       {make_pair("wei",3),&wei_Set[3]},       {make_pair("wei",4),&wei_Set[4]},       
    {make_pair("wan",1),&wan_Set[1]},       {make_pair("wan",2),&wan_Set[2]},       {make_pair("wan",3),&wan_Set[3]},       {make_pair("wan",4),&wan_Set[4]},       {make_pair("wen",1),&wen_Set[1]},       {make_pair("wen",2),&wen_Set[2]},       
    {make_pair("wen",3),&wen_Set[3]},       {make_pair("wen",4),&wen_Set[4]},       {make_pair("wang",1),&wang_Set[1]},     {make_pair("wang",2),&wang_Set[2]},     {make_pair("wang",3),&wang_Set[3]},     {make_pair("wang",4),&wang_Set[4]},     
    {make_pair("weng",1),&weng_Set[1]},     {make_pair("weng",3),&weng_Set[3]},     {make_pair("weng",4),&weng_Set[4]},     {make_pair("wong",4),&wong_Set[4]},     {make_pair("wu",1),&wu_Set[1]},         {make_pair("wu",2),&wu_Set[2]},         
    {make_pair("wu",3),&wu_Set[3]},         {make_pair("wu",4),&wu_Set[4]}
    };
}