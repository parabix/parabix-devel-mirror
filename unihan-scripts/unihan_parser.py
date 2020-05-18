# parses to generate corresponding unicode

import re
import unihan_config
from unicode_set import *

def phono(syl):
    cons = 'b p m f d t n l g k h j q x r z c s y w'
    res = cons.split().count(syl)
    return res != 0

def get_tones():
    dict_tones = {
        'a' : ['ā','á','ǎ','à'],
        'o' : ['ō','ó','ǒ','ò'],
        'e' : ['ē','é','ě','è'],
        'i' : ['ī','í','ǐ','ì'],
        'u' : ['ū','ú','ǔ','ù'],
        'v' : ['ǜ','ǘ','ǚ','ǜ'],
        'm' : ['m̄','ḿ','None','m̀'],
        'n' : ['None','ń','ň','ǹ']
    }
    return dict_tones

def det_syl(syl):
    s1 = None
    s2 = None
    if 'h' in syl:
        s1 = syl[:2]
        s2 = syl[2:]
    else:
        if phono(syl[0]):
            s1 = syl[0]
            s2 = syl[1:]
            if not s2 : s2 = None
        else:
            s2 = syl
    return s1, s2

def det_tone(word):
    flag = 0
    py = word
    d_tones = get_tones()
    for key in d_tones.keys():
        pos_tones = d_tones[key]
        t_num = 0
        for num in pos_tones:
            t_num += 1
            if num in word:
                flag = 1
                py = word.replace(num, key)
                break
        if flag == 1 : break
    return py, t_num


def parse_pinyin(word):
    pinyin_list = []
    pinyin = filter(None, word.split(','))
    for pos in pinyin:
        syl, tone = det_tone(pos)
        if 'ü' in syl:
            syl = syl.replace('ü', 'v')
        s1, s2 = det_syl(syl)
        pinyin_list.append(
            {
                "syllable" : syl,
                "syl_1" : s1,
                "syl_2" : s2,
                "tone" :  tone
            }
        )
    return pinyin_list

f_pattern = re.compile(r"\d*\.\d*:(.*?)\s")
khy_pattern = re.compile(r"U\+(\w+?)\s+kHanyuPinyin\s+(.+)")

def parse_fields(f):
    f += ' '
    f_list = f_pattern.findall(f)
    pinyin_list = []
    for i_f in f_list:
        pinyin_list.extend(parse_pinyin(i_f))
    return [x for y, x in enumerate(pinyin_list) if x not in pinyin_list[:y]]

def parse_pinyin_txt(file, property_code):
    val = []
    i_val = None
    val_map = {}
    lines = file.readlines()
    if property_code  == 'kpy':
        for word in lines:
            obj = khy_pattern.match(word)
            if obj is not None:
                codepoint = obj.group(1)
                field = obj.group(2)
                py_list = parse_fields(field)
                for ele in py_list:
                    syl = ele["syllable"]
                    tone = ele["tone"]
                    if syl not in val:
                        val.append(syl)
                        val_map[syl] = [None, None, None, None, None]
                        val_map[syl][tone] = singleton_uset(int(codepoint, 16))
                    elif val_map[syl][tone] == None:
                        val_map[syl][tone] = singleton_uset(int(codepoint, 16))
                    else:
                        val_map[syl][tone] = uset_union(val_map[syl][tone], singleton_uset(int(codepoint, 16))) 
    i_val = len(val)
    return val, i_val, val_map

def parse_property_file(filename, property_code):
    val = None
    i_val = None
    val_map = {}
    file = open(unihan_config.unihan_src_dir + '/' + filename + '.txt')
    if property_code == 'kpy':
        val, i_val, val_map = parse_pinyin_txt(file, property_code)
    return val, i_val, val_map



