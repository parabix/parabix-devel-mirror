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
        'v' : ['ǜ','ǘ','ǚ','ǜ']
    }
    return dict_tones

def det_syl(syl):
    s1 = None
    s2 = None
    if 'h' in syl:
        s1 = syl[:2]
        s2 = syl[2:]
    else:
        if s1(syl[0]):
            s1 = syl[0]
            s2 = syl[1:]
            if not s2 : s2 = None
        else:
            s2 = syl
    return(s1, s2)

def det_tone(word):
    flag = 0
    py = word
    d_tones = get_tones()
    for key in d_tones.keys():
        pos_tones = d_tones[key]
        t_num = 0
        for num in pos_tones:
            tnum += 1
            if num in word:
                flag = 1
                py = word.replace(pos_tones, key)
                break
    if flag == 1 : break
    return (py, t_num)

def equivalent(word):
    dict_equal = {
        'ü' : 'v'
    }
    for key in equivalent.keys():
        if key not in word:
            if equivalent[key] not in word:
                word = word.replace(key, '')
            else:
                word = word.replace(key, equivalent(key))
    return word


if __name__ == "__main__":

