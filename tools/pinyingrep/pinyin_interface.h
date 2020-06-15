/*
 *  Copyright Group ALpha
 *  in Software Engineering(2020 Spring), ZJU
 *  Advisor: Prof. Cameron
 */

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
#include <re/adt/re_alt.h>
#include <re/adt/re_seq.h>
#include <re/adt/re_start.h>
#include <re/adt/re_end.h>
#include <re/adt/re_utility.h>
#include <re/parse/parser.h>
#include <re/toolchain/toolchain.h>
#include <unicode/data/KHanyuPinyin.h>
#include <unicode/data/KXHC1983.h>
#include <unicode/data/KTraditional.h>
#define LAZY_ENUM 1
#define DEBUG 0

enum OptTraditional {
  All, Traditional, Simplified, TraditionalOnly, SimplifiedOnly
};
enum Database {
    XHC, KPY
};
typedef re::CC* (*OptTraditionalFunctionType)(const UCD::UnicodeSet&& );
namespace PY{
    using std::vector;
    using std::array;
    using std::string;
    // using std::wstring;
    using std::map;
    using std::set;


    // Pinyin Values Parser Class
    // takes in input string(to do: take in unicode input)
    // parse the input into possible syllables and tones    
    class PinyinValuesParser{
    public:
        // Constructor
        PinyinValuesParser(string input): _parsed(false), _warning(false){
            parse(input);
        }
        PinyinValuesParser():_parsed(false), _warning(false) {}

        // Core Method

        // Method: parse
        // takes in input string(to do: take in unicode input)
        // parse the input into two part:
        // 1. possible syllables in an vector _parsed[syllable]
        // 2, possible tones in an vector _parsed[tone]
        void parse(string to_parse);

        // return whether the parser has generated
        // valid _parsed_syllable_tone vectors
        bool is_parsed(){
            return _parsed;
        }

        void set_warning(){
            _warning = true;
        }

        // return _parsed_syllable_tone(copy)
        vector<std::pair<vector<string>, vector<int>>>
        get_parsed() { return _parsed_syllable_tone; }

    private:
        vector<std::pair<vector<string>, vector<int>>> _parsed_syllable_tone; // record the possible syllables and tones
        // e.g. (<,> for pair, and {} for vector)
        // { <{jin},{0,1,2,3,4}>, <{rong},{0,1,2,3,4}>} for input "jin rong"

        bool _parsed; // whether the parser has done the parsing or not
        bool _warning; // whether display warning message

        // Private Method

        // Parse multiple pinyin regexes into a list based on ' ' space
        void _parse_multi_syllable(string s, vector<string>& list);

        // interpret pinyin regex
        std::pair<vector<string>, vector<int>> _interpret_regex(string s);

        // elimiate extra space in both ends of the string
        void eliminate_space(string& s);

    };

    class ParserException : public std::exception{
    public:
        ParserException(const char* c): message(c) {}
        const char* what() throw(){
            return message.c_str();
        }
    private:
        string message;
    };

    

    // Pinyin Values Enumerator
    // takes in a reference to a PinyinValuesParser Object 
    // enumerate the parsed values into all possible combinations
    // of syllables and tones
    class PinyinValuesEnumerator{
    public:
        PinyinValuesEnumerator(bool flag = false)
        : _enumerated(false), _fully_enumerate(flag), _warning(false) {}
        // Core Method

        // Method: emumerate
        // takes the reference of a parser
        // , generate _half_enumerated_list
        // and generate _enumerated_list if _fully_enumerate is true
        // which contains all possible values of <syllable,tone> pairs
        void enumerate(vector<std::pair<vector<string>, vector<int>>> parsed);

        // Method: createREs(Database, ChineseCharacterType) 
        // Database set to xhc-database as default. Can be switched to kpy-database.
        // ChineseCharacterType options includes: traditional, simplified, trad-only, and simp-only
        // create a vector of RE* for grep engine
        std::vector<re::RE*> createREs(Database database, OptTraditional opt);

        void set_warning(){
            _warning = true;
        }
    private:
        // private method
        re::CC* _intersect_character_type(const UCD::UnicodeSet&& uset, OptTraditional opt);

        // private member
        vector<std::pair<vector<string>, vector<int>>> _parsed; // parsed list from parser

        vector<vector<std::pair<string, int>>> _enumerated_list;
        // e.g.  (<,> for pair, and {} for vector)
	    // { {<jin, 0>, <rong, 0>}, {<jin,0>, <rong, 1>}, ... } for input ""jin rong"

        std::vector<vector<std::pair<string,int>>> _half_enumerated_list;
        // e.g.
        // { {<jin, 0>, <jin, 1>,...}, {<rong, 0>, <rong, 1>, ...}}

        bool _enumerated;
        bool _fully_enumerate; // whether to enumerate fully
        bool _warning; // whether display warning message
    };

    // PinyinValuesTable
    // Mapping Unicode toned syllables to syllable without tones 
    // and separate tones as integers
    // This is to facilitate the parsing of PinyinValuesParser
    class PinyinValuesTable{
    public:
        // Core Method

        // Method: get_initial
        // Get the initial part of the syllable
        // if no initial part(e.g. "an" ), simply return ""
        string get_initial(string s);

        // Method: get_final
        // Get the final part of the syllable
        // if toned, the tone will be replaced
        string get_final(string s);

        // Method: is_legal
        // Check whether the syllable is legal or not
        bool is_legal(string s);

        // Method: is_toned
        // Check whether the syllable has toned final
        bool is_toned(string s);

        // Method: get_tone
        // get the tone of the syllable
        // returning 0 as neutral tone
        int get_tone(string s);

        // Method: replace_tone
        // replace the toned part of syllable with non-toned
        // return the tone
        int replace_tone(string& toned);

        // Method: replace_equivalence
        // replave unicode v and e_hat
        void replace_equivalence(string& s);

        // utility to iterate through _legal_syllables_table
        set<string>::const_iterator legal_begin(){
            return _legal_syllables_set.cbegin();
        }
        const set<string>::const_iterator legal_end(){
            return _legal_syllables_set.cend();
        }
    private:
        static set<string> _initial_syllable_set; // storing all valid initial parts of syllables
        static set<string> _final_syllable_set;   // storing all valid final parts of syllables
        static set<string> _legal_syllables_set;      // storing all valid combinations of initials and finals 
        static map<string, std::pair<string, int>> _toned_character_table; 
        static map<string, string> _equivalence_table;
        
    };

    // UnicodeSetTable
    // mapping pairs of a syllable and its tone
    // to corresponding UnicodeSet predefined in KHanyuPinyin.h
    class UnicodeSetTable{
    private:
        static map<std::pair<string, int>, const UCD::UnicodeSet*> _kpy_unicodeset_table;
        static map<std::pair<string, int>, const UCD::UnicodeSet*> _kxhc_unicodeset_table;
    public:
        static const UCD::UnicodeSet&& get_XHC(string syllable, int tone){
            if(_kxhc_unicodeset_table.find(make_pair(syllable, tone)) != _kxhc_unicodeset_table.end()) 
                return std::move(*_kxhc_unicodeset_table[make_pair(syllable, tone)]);
            else 
                return std::move(UCD::UnicodeSet());
        }
        static const UCD::UnicodeSet&& get_KPY(string syllable, int tone){
            if(_kpy_unicodeset_table.find(make_pair(syllable, tone)) != _kpy_unicodeset_table.end()) 
                return std::move(*_kpy_unicodeset_table[make_pair(syllable, tone)]);
            else 
                return std::move(UCD::UnicodeSet());
        }
        static const UCD::UnicodeSet&& get_traditional_only(){
            return std::move(UCD::KTRD_ns::trd_only_Set);
        }
        static const UCD::UnicodeSet&& get_simplified_only(){
            return std::move(UCD::KTRD_ns::sim_only_Set);
        }
    };
}


#endif

