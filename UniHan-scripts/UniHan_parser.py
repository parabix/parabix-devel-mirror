#
#  Copyright Group ALpha
#  in Software Engineering(2020 Spring), ZJU
#  Advisor: Prof. Cameron
#

from unicode_set import *
from syllable_util import *
import UniHan_config
import re

def process_syllable(syllable):
    first = None
    final = None
    if(syllable.find('h') != -1):
        # double first
        first = syllable[   : 2]
        final = syllable [ 2 :  ]
    else:
        # single or no first
        if(is_first(syllable[0])):
            first = syllable[0]
            final = syllable[1:]
            if(final == ''):
                final = None
        else: # no first
            final = syllable
    
    return (first, final)
def process_tone(pinyin):
    # 0 represent no tone
    processed = pinyin
    tone = 0
    found = False
    toned_map = get_toned_map()

    for key in toned_map.keys():
        value_list = toned_map[key]
        for i in range(len(value_list)):
            to_search = value_list[i]
            if(pinyin.find(to_search) != -1): # found correctly toned
                processed = pinyin.replace(to_search,key)
                tone = i+1  # started from 1 to 4
                found = True
                break
        if(found):
            break
    return processed, tone
    
def parse_pinyins(pinyins):
    # return a list of map, where each map contains 
    # {syllable":...(str), "first":...(str), "final":...(str), "tone":...(int)}
    pinyin_list = []
    preprocessed = pinyins.split(',')
    preprocessed = filter(None,preprocessed)
    for toned_syllable in preprocessed:
        
        syllable, tone = process_tone(toned_syllable)
        syllable = replace_equivalence(syllable)
        first, final = process_syllable(syllable)
        pinyin_list.append(
            {
                "syllable": syllable,
                "first":    first,
                "final":    final,
                "tone":     tone
            }
        )
    return pinyin_list   

Field_Pattern = re.compile(r"\d*\.\d*:(.*?)\s")
def parse_fields(fields, field_name):
    fields = fields + " " # make re work properly for all fields
    pinyin_list = []
    if(field_name == "kHanyuPinyin" or field_name == "kXHC1983"):
        field_list = Field_Pattern.findall(fields)
        for field in field_list:
            pinyin_list.extend(parse_pinyins(field))
    else:
        # kMandarin
        pinyin_list.extend(parse_pinyins(fields.strip().split(" ")[0])) 
    # remove duplicated ones
    res_list = []
    for syllable_dict in pinyin_list:
        if(res_list.count(syllable_dict)==0):
            res_list.append(syllable_dict)
    return res_list

KPY_Pattern = re.compile(r"U\+(\w+?)\s+(kHanyuPinyin)\s+(.+)")
KXHC_Pattern = re.compile(r"U\+(\w+?)\s+(kXHC1983)\s+(.+)")
KMandarin_Pattern = re.compile(r"U\+(\w+?)\s+(kMandarin)\s+(.+)")

def parse_Reading_txt(f, property_code):
    # replace the utility of property object
    # return value (prop_values, independent_prop_values, value_map)
    prop_values = []
    independent_prop_values = None
    value_map = {} # value_map is from value name to corresponding unicode set
                   # each key match a value or value array
    lines = f.readlines()
    if(property_code == 'kpy' or property_code == 'kxhc'):
        # parse KHanyuPinyin
        # prop_values contains all syllable without tones
        # value map use syllable without tones as keys, 
        # and corresponding values are 5-element lists from 0(no tone) to 4
        for line in lines:
            match_obj = None
            if(property_code == 'kpy'):
                match_obj = KPY_Pattern.match(line)
                # kHanyuPinyin need to work with kMandarin
                if(match_obj is None):
                    match_obj = KMandarin_Pattern.match(line)
            else:
                match_obj = KXHC_Pattern.match(line)
            if(match_obj is not None):
                parsed_property = {}
                codepoint = match_obj.group(1)
                field_name = match_obj.group(2)
                fields = match_obj.group(3)
                pinyin_list = parse_fields(fields, field_name)

                parsed_property["codepoint"] = codepoint
                parsed_property["pinyin"] = pinyin_list

                for pinyin in pinyin_list:
                    syllable = pinyin["syllable"]
                    tone = pinyin["tone"]
                    if(prop_values.count(syllable) == 0):
                        # new syllable
                        prop_values.append(syllable)
                        value_map[syllable] = [None, None, None, None, None] # 0 means no tone
                        value_map[syllable][tone] = singleton_uset(int(codepoint,16))
                    elif(value_map[syllable][tone] == None):
                        # new tone in this syllable
                        value_map[syllable][tone] = singleton_uset(int(codepoint, 16))
                    else:
                        value_map[syllable][tone] = uset_union(value_map[syllable][tone],
                                                              singleton_uset(int(codepoint, 16)))

    independent_prop_values = len(prop_values)
    return (prop_values, independent_prop_values, value_map)

KTraditionalVariant_Pattern = re.compile(r"U\+(\w+?)\s+(kTraditionalVariant)\s+(.+)")
KSimplifiedVariant_Pattern = re.compile(r"U\+(\w+?)\s+(kSimplifiedVariant)\s+(.+)")
def parse_Variant_txt(f, property_code):
     # replace the utility of property object
    # return value (prop_values, independent_prop_values, value_map)
    prop_values = []
    independent_prop_values = None
    value_map = {} # value_map is from value name to corresponding unicode set
                   # each key match a value or value array
    lines = f.readlines()
    if(property_code == 'ktrd'):
        have_traditional = empty_uset()
        have_simplified = empty_uset()
        prop_values.append('trd_only')
        prop_values.append('sim_only')
        independent_prop_values = 2
        for line in lines:
            match_obj = KTraditionalVariant_Pattern.match(line)
            if(match_obj is not None):
                codepoint = match_obj.group(1)
                have_traditional = uset_union(have_traditional, singleton_uset(int(codepoint, 16)))

            match_obj = KSimplifiedVariant_Pattern.match(line)
            if(match_obj is not None):
                codepoint = match_obj.group(1)
                have_simplified = uset_union(have_simplified, singleton_uset(int(codepoint, 16)))
        value_map['sim_only'] = uset_intersection(uset_complement(have_simplified), have_traditional)
        value_map['trd_only'] = uset_intersection(uset_complement(have_traditional), have_simplified)
    return (prop_values, independent_prop_values, value_map)


def parse_property_file(filename_root, property_code):
    # replace the utility of property object
    # return value (prop_values, independent_prop_values, value_map)
    prop_values = None
    independent_prop_values = None
    value_map = {} # value_map is from value name to corresponding unicode set
                   # each key match a value or value array
    f = open(UniHan_config.UniHan_src_dir + '/' + filename_root + '.txt')

    if(property_code == "kpy" or property_code == "kxhc"):
        prop_values, independent_prop_values, value_map = parse_Reading_txt(f, property_code)
    elif(property_code == "ktrd"):
        prop_values, independent_prop_values, value_map = parse_Variant_txt(f, property_code)
    
    return (prop_values, independent_prop_values, value_map)


if __name__ == "__main__":
    # parse_property_file("Unihan_Readings", "kpy")
    # print(process_tone("tiàn"))
    fields = ("32140.110:āi,ǎi,xiè,ế,éi,ê̌,ěi,ề,èi,ê̄")
    print(replace_equivalence(fields))
    print(fields)
    print(parse_fields(replace_equivalence(fields)))

    fields2 = "53449.060:xuǎn,jiōng 74379.120:jiǒng,xiàn,xuǎn,jiōng"
    fields2 = replace_equivalence(fields2)
    print(fields2)
    print(parse_fields(fields2))

    fields3 = "10684.020:fǔ,m̄"
    fields3 = replace_equivalence(fields3)
    print(fields3)
    print(parse_fields(fields3))
    
