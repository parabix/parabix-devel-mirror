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
        toned_syllable = replace_equivalence(toned_syllable)
        syllable, tone = process_tone(toned_syllable)
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
def parse_fields(fields):
    fields = fields + " " # make re work properly for all fields
    pinyin_list = []
    field_list = Field_Pattern.findall(fields)
    for field in field_list:
        pinyin_list.extend(parse_pinyins(field))

    # remove duplicated ones
    res_list = []
    for syllable_dict in pinyin_list:
        if(res_list.count(syllable_dict)==0):
            res_list.append(syllable_dict)
    return res_list

KPY_Pattern = re.compile(r"U\+(\w+?)\s+kHanyuPinyin\s+(.+)")

def parse_Reading_txt(f, property_code):
    # replace the utility of property object
    # return value (prop_values, independent_prop_values, value_map)
    prop_values = []
    independent_prop_values = None
    value_map = {} # value_map is from value name to corresponding unicode set
                   # each key match a value or value array
    lines = f.readlines()
    if(property_code == 'kpy'):
        # parse KHanyuPinyin
        # prop_values contains all syllable without tones
        # value map use syllable without tones as keys, 
        # and corresponding values are 5-element lists from 0(no tone) to 4
        for line in lines:
            match_obj = KPY_Pattern.match(line)
            if(match_obj is not None):
                parsed_property = {}
                codepoint = match_obj.group(1)
                fields = match_obj.group(2)
                pinyin_list = parse_fields(fields)

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

def parse_property_file(filename_root, property_code):
    # replace the utility of property object
    # return value (prop_values, independent_prop_values, value_map)
    prop_values = None
    independent_prop_values = None
    value_map = {} # value_map is from value name to corresponding unicode set
                   # each key match a value or value array
    f = open(UniHan_config.UniHan_src_dir + '/' + filename_root + '.txt')

    if(property_code == "kpy"):
        prop_values, independent_prop_values, value_map = parse_Reading_txt(f, property_code)
    
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
    