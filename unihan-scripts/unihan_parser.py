#
# unihan_parser.py generates an expression that requests an unicode set with a specific radical
#

from unicode_set import *
import unihan_config
import re

fields_pattern = re.compile(r"\d*\.\d*")
def parse_fields(fields):
    radical_list = []
    radicalpoint = fields.split('.')
    radical_list.append(radicalpoint[0])
    result_list = []
    for radical_dict in radical_list:
        if(result_list.count(radical_dict)==0):
            result_list.append(radical_dict)
    return result_list

krs_Pattern = re.compile(r"U\+(\w+?)\s+kRSKangXi\s+(.+)")
#Regular expression in format U+????    kRSKangXi      ???.???
def parse_radicals_txt(f, property_code):
    prop_values = []
    independent_prop_values = None
    value_map = {}
    #prop_values - all the value (radicals) in this property
    #independent_prop_values - the number of value (radicals)
    #value_map - dict map value to UnicodeSet
    line = f.readlines()
    if(property_code == 'krs'):
        for info in line:
            wether_match = krs_Pattern.match(info)
            if(wether_match is not None):
                    codepoint = whether_match.group(1) #U+(????)
                    fields = whether_match.group(2) #???.???
                    radical_list = parse_fields(fields)

                    for radicals in radical_list:
                        if(prop_values.count(radicals) == 0):
                        # new radical
                            prop_values.append(radicals)
                            value_map[radicals]=singleton_uset(int(codepoint,16))
                        else:
                            value_map[radicals]=uset_union(value_map[radicals],singleton_uset(int(codepoint, 16)))

        independent_prop_values = len(prop_values)
        return (prop_values, independent_prop_values, value_map)

def parse_property_file(filename_root, property_code):
    prop_values = None
    independent_prop_values = None
    value_map = {}
    f = open(unihan_config.UniHan_src_dir + '/' + filename_root + '.txt')

    if(property_code == "krs"):
        prop_values, independent_prop_values, value_map = parse_radicals_txt(f, property_code)
    return (prop_values, independent_prop_values, value_map)

    

    
