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
#include <unicode/data/KHanyuPinyin.h>

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
        // Constructor
        PinyinValuesParser(string input): _parsed(false){
            parse(input);
        }
        PinyinValuesParser():_parsed(false) {}

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

        friend class PinyinValuesEnumerator;
    private:
        vector<std::pair<vector<string>, vector<int>>> _parsed_syllable_tone; // record the possible syllables and tones
        // e.g. (<,> for pair, and {} for vector)
        // { <{jin},{0,1,2,3,4}>, <{rong},{0,1,2,3,4}>} for input "jin rong"

        bool _parsed; // whether the parser has done the parsing or not

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
        PinyinValuesEnumerator(): _enumerated(false) {}
        // Core Method

        // Method: emumerate
        // takes the reference of a parser
        // and generate _enumerated_list
        // which contains all possible values of <syllable,tone> pairs
        void enumerate(PinyinValuesParser& parser);
    private:
        vector<vector<std::pair<string, int>>> _enumerated_list;
        // e.g.  (<,> for pair, and {} for vector)
	// { {<jin, 0>, <rong, 0>}, {<jin,0>, <rong, 1>}, ... } for input ""jin rong"
        bool _enumerated;
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
        const UCD::UnicodeSet&& get_uset(string syllable, int tone){
            if(_unicodeset_table.find(make_pair(syllable, tone)) != _unicodeset_table.end()) 
                return std::move(*_unicodeset_table[make_pair(syllable, tone)]);
            else 
                return std::move(UCD::UnicodeSet());
        }
    private:
        static map<std::pair<string, int>, const UCD::UnicodeSet*> _unicodeset_table;
    };
}


#endif

