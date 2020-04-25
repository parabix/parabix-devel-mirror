/*
 *  Copyright Group ALpha
 *  in 2020 Software Engineering, ZJU
 *  Advisor: Prof. Cameron
 */

#ifndef _PINYIN_INTERFACE_
#define _PINYIN_INTERFACE_

#include <vector>
#include <string>
#include <array>
#include <map>
#include <utility>
#include <pinyin/KHanyuPinyin.h>

namespace PY{
    using std::vector;
    using std::array;
    using std::string;
    // using std::wstring;
    using std::map;

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

    private:
        std::pair<vector<string>, vector<int>> _parsed_syllable_tone; // record the possible syllables and tones

        bool _parsed; // whether the parser has done the parsing or not
    };

    

    // Pinyin Values Enumerator
    // takes in a reference to a PinyinValuesParser Object 
    // enumerate the parsed values into all possible combinations
    // of syllables and tones
    class PinyinValuesEnumerator{
        PinyinValuesEnumerator(): _enumerated(false) {}
        // Core Method

        // Method: emumerate
        // takes the reference of a parser
        // and generate _enumerated_list
        // which contains all possible values of <syllable,tone> pairs
        void enumerate(PinyinValuesParser& parser);
    private:
        vector<std::pair<string, int>> _enumerated_list;
        bool _enumerated;
    };

    // PinyinValuesTable
    // Mapping Unicode toned syllables to syllable without tones 
    // and separate tones as integers
    // This is to facilitate the parsing of PinyinValuesParser
    class PinyinValuesTable{
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
    private:
        static vector<string> _initial_syllable_list;
        static vector<string> _final_syllable_list;
        static map<string, std::pair<string, int>> _toned_character_table;
        
    };

    // UnicodeSetTable
    // mapping pairs of a syllable and its tone
    // to corresponding UnicodeSet predefined in KHanyuPinyin.h
    class UnicodeSetTable{
        UnicodeSet&& get_uset(string syllable, int tone){
            if(_unicodeset_table.find(make_pair(syllable, pair)) != _unicodeset_table.end()) 
                return std::move(*_unicodeset_table[make_pair(syllable, tone)]);
            else 
                return std::move(UCD::UnicodeSet());
        }
    private:
        static map<std::pair<string, int>, UCD::UnicodeSet* uc_ptr> _unicodeset_table;
    };
}


#endif