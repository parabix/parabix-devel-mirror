#
#  Copyright Group ALpha
#  in Software Engineering(Spring 2020), ZJU
#  Advisor: Prof. Cameron
#
def replace_equivalence(syllable):
    equivalence_map = {
        # 'ê̄':'ēi',
        # 'ế':'éi',
        # 'ê̌':'ěi',
        # 'ề':'èi',
        'ü':'v',
    }
    for key in equivalence_map.keys():
        if(syllable.find(key) != -1):
            if(syllable.find(equivalence_map[key]) != -1): # already have
                syllable = syllable.replace(key,"")
            else:
                syllable = syllable.replace(key, equivalence_map[key])

    return syllable
def get_toned_map():
    map_tone_unicode = {'a': ['ā','á','ǎ','à'],
                        'e': ['ē','é','ě','è'],
                        'i': ['ī','í','ǐ','ì'],
                        'o': ['ō','ó','ǒ','ò'],
                        'u': ['ū','ú','ǔ','ù'],
                        'v': ['ǜ','ǘ','ǚ','ǜ'],
                        'ê': ['ê̄','ế','ê̌','ề'],
                        'm': ['m̄','ḿ','None','m̀'], # None will not be in the syllables
                        'n': ['None','ń','ň','ǹ'],
                        }
    return map_tone_unicode

def is_first(s):
    first_str = 'b p m f d t n l g k h j q x r z c s y w'
    first_list = first_str.split(' ') 
    return first_list.count(s) != 0

if __name__ == "__main__":
    print(is_first('p'))
    print(is_first('w'))
    print(is_first('ch'))
    print("ǒ" == "ǒ")
