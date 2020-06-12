/*
 *  Copyright Group ALpha
 *  in Software Engineering(2020 Spring), ZJU
 *  Advisor: Prof. Cameron
 */

#include "pinyin_interface.h"

using namespace std;

using UST = PY::UnicodeSetTable;
namespace PY{
    // Method of Parser
    // Parse multiple pinyin regexes into a list based on ' ' space
    // with no extra space in both ends
    void PinyinValuesParser::_parse_multi_syllable(string s, vector<string>& list){
        std::size_t start = s.find_first_not_of(' '), end;
        while(start != s.npos){
            #if DEBUG
                cout<<"start:"<<start<<endl;
            #endif
            s = s.substr(start);        // replace prefix empty space
            #if DEBUG
                cout<<"after s=s.substr(start): "<<s<<endl;
            #endif
            end = s.find_first_of(' '); // find the end of the current syllable 
            if(end != s.npos){
                #if DEBUG
                    cout<<"end:"<<end<<endl;
                #endif
                list.push_back(s.substr(0, end));
                // erase the current syllable
                #if DEBUG
                    cout<<"s.substr(0, end): "<<s.substr(0, end)<<endl;
                #endif
                s = s.substr(end);  
                start = s.find_first_not_of(' ');
            }
            else{
                list.push_back(s); break;
            }
            
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
            string tone = match_result[1].str();
            #if DEBUG
                cout<<"tone:"<<tone<<endl;
            #endif
            if(tone.length() > 1) throw ParserException("Invalid Syntax -- Too many numbers to specify tones");
            s = regex_replace(s, regex("[0-4]"), ""); // erase the tone number
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
        //resolve capital letters
        for(unsigned i=0; i < s.size(); i++){
            if(s[i] >= 'A' && s[i] <= 'Z')
                s[i] |= 0x20; // make lower case
        }
            // resolve regex '?'
        std::size_t qmark_index = s.find('?');
        if(qmark_index != s.npos){
            if(qmark_index == 0) 
                throw ParserException("Invalid Syntax -- only support ? after something'");
            // erase ?
            s.erase(qmark_index,1);
	        string tmps = s;
            resolved.first.push_back(tmps.erase(qmark_index-1,1));
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
    void PinyinValuesEnumerator::enumerate(vector<std::pair<vector<string>, vector<int>>> parsed){
        
        #if LAZY_ENUM
            _parsed = parsed;
        #else
            for(auto first_syl=parsed.begin();first_syl!=parsed.end();first_syl++){
                vector<pair<string,int>> temp;
                for(auto syl=first_syl->first.begin();syl!=first_syl->first.end();syl++){
                    for(auto tone=first_syl->second.begin();tone!=first_syl->second.end();tone++){
                        temp.push_back(make_pair(*syl,*tone));
                    }
                }            
                _half_enumerated_list.push_back(temp);
            }
            #if DEBUG
            std::cout<<"Half Enumerated Result: "<<std::endl;
            for(auto iter = _half_enumerated_list.begin(); iter != _half_enumerated_list.end(); iter++){
                for(auto inner = iter->begin(); inner != iter->end(); inner++){
                    std::cout<<"* "<<inner->first<<" "<<inner->second<<std::endl;
                }
            }
            #endif
            if(_fully_enumerate){
                vector<int> indices(_half_enumerated_list.size());
                int i=_half_enumerated_list.size()-1;
                while(i>=0){
                    vector<pair<string,int>> T; //temporary vector of pairs
                    for(int k=0;k<_half_enumerated_list.size();k++){
                        T.push_back(_half_enumerated_list[k][indices[k]]); //build current combination
                    }
                    _enumerated_list.push_back(T); //add to final vector
                    i=_half_enumerated_list.size()-1;
                    while(i>=0&&++indices[i]==_half_enumerated_list[i].size()){ 
                        indices[i--]=0; //reset indices to 0;
                    }
                }
            }
        #endif
        
        _enumerated = true;    
    }

    re::CC* PinyinValuesEnumerator::_intersect_character_type(const UCD::UnicodeSet&& uset, OptTraditional opt){
        static map<OptTraditional, OptTraditionalFunctionType> fmap{
            //All, Traditional, Simplified, TraditionalOnly, SimplifiedOnly
            {All, 
            [](const UCD::UnicodeSet&& us){return re::makeCC(UCD::UnicodeSet(us));}},
            {Traditional, 
            [](const UCD::UnicodeSet&& us){
                return re::subtractCC(
                    re::makeCC(UCD::UnicodeSet(us)),
                    re::makeCC(UCD::UnicodeSet(UST::get_simplified_only()))
                    );
                }
            },
            {Simplified, 
            [](const UCD::UnicodeSet&& us){
                return re::subtractCC(
                    re::makeCC(UCD::UnicodeSet(us)),
                    re::makeCC(UCD::UnicodeSet(UST::get_traditional_only()))
                    );        
                }
            },
            {TraditionalOnly, 
            [](const UCD::UnicodeSet&& us){
                return re::intersectCC(
                    re::makeCC(UCD::UnicodeSet(us)),
                    re::makeCC(UCD::UnicodeSet(UST::get_traditional_only()))
                    ); 
                }
            },
            {SimplifiedOnly, 
            [](const UCD::UnicodeSet&& us){
                return re::intersectCC(
                    re::makeCC(UCD::UnicodeSet(us)),
                    re::makeCC(UCD::UnicodeSet(UST::get_simplified_only()))
                    );
                }
            }
        };
        return fmap[opt](std::move(uset));
    }

    std::vector<re::RE*> PinyinValuesEnumerator::createREs(int database, OptTraditional opt){ //database=1 for kpy, database=0 for xhc
        std::vector<re::RE*> REs;
        #if LAZY_ENUM
            for(auto iter = _parsed.begin(); iter != _parsed.end(); iter++){
                std::vector<re::RE*> components;
                for(auto syllable_iter = iter->first.begin(); 
                syllable_iter != iter->first.end(); syllable_iter++)
                    for(auto tone_iter = iter->second.begin();
                    tone_iter != iter->second.end(); tone_iter++){
                        if (database==1)
                            components.push_back(
                                _intersect_character_type
                                    (UST::get_KPY(*syllable_iter, *tone_iter), opt)
                                );
                        else
                            components.push_back(
                                _intersect_character_type
                                    (UST::get_XHC(*syllable_iter, *tone_iter), opt)
                                );
                    }   
                REs.push_back(re::makeAlt(components.begin(), components.end())); 
            }
        #else
            for(auto iter = _half_enumerated_list.begin(); iter != _half_enumerated_list.end(); iter++){
                std::vector<re::RE*> components;
                for(auto inner = iter->begin(); inner != iter->end(); inner++){
                    if (database==1){
                        components.push_back(
                            _intersect_character_type(
                                UST::get_KPY(inner->first, inner->second), opt
                                )
                            );
                    }
                    else{
                        components.push_back(
                            _intersect_character_type(
                                UST::get_XHC(inner->first, inner->second), opt
                            )
                        );
                    }
                }
                REs.push_back(re::makeAlt(components.begin(), components.end()));
            }
        #endif
        return std::vector<re::RE*>(1, re::makeSeq(REs.begin(), REs.end()));
    }


    // Methods in PinyinValuesTable
    
    // Method: get_initial
    // Get the initial part of the syllable
    // if no initial part(e.g. "an" ), simply return ""
    string PinyinValuesTable::get_initial(string s){
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
        string final_part;
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
            final_part = s;
        }else{
            final_part = s.substr(i);
        }
        for(int charlen=1;charlen<5;charlen++){
            if(charlen > (int)final_part.length()) break;
            for(i = 0;i <= (int)final_part.length()-charlen;++i){
                string toned_char = final_part.substr(i,charlen);
                if(_toned_character_table.find(toned_char) != _toned_character_table.end()){
                    replace_tone(toned_char);
                    final_part = final_part.replace(i,charlen,toned_char);
                    if(_final_syllable_set.find(final_part) != _final_syllable_set.end()) return final_part;
                }else if(_final_syllable_set.find(final_part) != _final_syllable_set.end()){
                    return final_part;
                }
            }
        }
        return "";
    }
    // Method: is_legal
    // Check whether the syllable is legal or not
    bool PinyinValuesTable::is_legal(string s){
        return _legal_syllables_set.find(get_initial(s) + get_final(s)) != _legal_syllables_set.end();
    }
    // Method: is_toned
    // Check whether the syllable has toned final
    bool PinyinValuesTable::is_toned(string s){
        int len = (int)s.length();
        for(int charlen=1;charlen<5;charlen++){
            if(charlen > len) break;
            for(int i = 0;i <= len-charlen;++i){
                string final_part = s.substr(i,charlen);
                if(_toned_character_table.find(final_part) != _toned_character_table.end()){
                    return true;
                }
            }
        }
        return false;
    }
    // Method: get_tone
    // get the tone of the syllable
    // returning 0 as neutral tone
    int PinyinValuesTable::get_tone(string s){
        int len = (int)s.length();
        for(int charlen=1;charlen<5;charlen++){
            if(charlen > len) break;
            for(int i = 0; i <= len-charlen; ++i){
                string final_part = s.substr(i,charlen);
                if(_toned_character_table.find(final_part) != _toned_character_table.end()){
                    return _toned_character_table[final_part].second;
                }
            }
        }
        return 0;
    }
    // Method: replace_tone
    // replace the toned part of syllable with non-toned
    // return the tone
    int PinyinValuesTable::replace_tone(string& toned){
        int len = (int)toned.length();
        string tonedchar;
        for(int charlen=1;charlen<5;charlen++){
            if(charlen > len) break;
            for(int i = 0;i <= len-charlen;++i){
                string check_part = toned.substr(i,charlen);
                if(_toned_character_table.find(check_part) != _toned_character_table.end()){
                    std::pair<string, int> final_part_and_tone = _toned_character_table[check_part];
                    tonedchar = final_part_and_tone.first;
                    toned = toned.replace(i,charlen,tonedchar);
                    return final_part_and_tone.second;
                }
            }
        }
        return 0;
    }
    // Method: replace_equivalence
    // replave unicode v and e_hat
    void PinyinValuesTable::replace_equivalence(string& s){
        for(auto iter = _equivalence_table.begin(); iter != _equivalence_table.end(); iter++){
            auto index = s.find(iter->first);
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
    { "Ā",make_pair("a", 1)}, { "Á",make_pair("a", 2)}, { "Ǎ",make_pair("a", 3)}, { "À",make_pair("a", 4)}, 
    { "Ō",make_pair("o", 1)}, { "Ó",make_pair("o", 2)}, { "Ǒ",make_pair("o", 3)}, { "Ò",make_pair("o", 4)}, 
    { "Ē",make_pair("e", 1)}, { "É",make_pair("e", 2)}, { "Ě",make_pair("e", 3)}, { "È",make_pair("e", 4)}, 
    { "Ī",make_pair("i", 1)}, { "Í",make_pair("i", 2)}, { "Ǐ",make_pair("i", 3)}, { "Ì",make_pair("i", 4)}, 
    { "Ū",make_pair("u", 1)}, { "Ú",make_pair("u", 2)}, { "Ǔ",make_pair("u", 3)}, { "Ù",make_pair("u", 4)}, 
    { "Ǖ",make_pair("v", 1)}, { "Ǘ",make_pair("v", 2)}, { "Ǚ",make_pair("v", 3)}, { "Ǜ",make_pair("v", 4)}, 

    {"ā",make_pair("a",1)},{"á",make_pair("a",2)},{"ǎ",make_pair("a",3)},{"à",make_pair("a",4)},
    {"ī",make_pair("i",1)},{"í",make_pair("i",2)},{"ǐ",make_pair("i",3)},{"ì",make_pair("i",4)},    {"ū",make_pair("u",1)},{"ú",make_pair("u",2)},{"ǔ",make_pair("u",3)},{"ù",make_pair("u",4)},
    {"ē",make_pair("e",1)},{"é",make_pair("e",2)},{"ě",make_pair("e",3)},{"è",make_pair("e",4)},    {"ō",make_pair("o",1)},{"ó",make_pair("o",2)},{"ǒ",make_pair("o",3)},{"ò",make_pair("o",4)},
    {"ǜ",make_pair("v",1)},{"ǘ",make_pair("v",2)},{"ǚ",make_pair("v",3)},{"ǜ",make_pair("v",4)},    {"m̄",make_pair("m",1)},{"ḿ",make_pair("m",2)},{"m̀",make_pair("m",4)},
    {"ń",make_pair("n",2)},{"ň",make_pair("n",3)},{"ǹ",make_pair("n",4)},                           {"ê̄",make_pair("e_hat",1)},{"ế",make_pair("e_hat",2)},{"ê̌",make_pair("e_hat",3)},{"ề",make_pair("e_hat",4)}
    }; 

}
