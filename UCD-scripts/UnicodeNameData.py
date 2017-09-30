#
# UnicodeNameData.py 
#
# Robert D. Cameron
# March 2, 2016
#
# Licensed under Open Software License 3.0.
#
#
import re, string, os.path, cformat
import UCD_config
from UCD_parser import *


UnicodeNameData_cpp_template = r"""
#include "UnicodeNameData.h"
const int Unamesize = %s;
char __attribute__ ((aligned (32))) Unamedata[Unamesize + %s] = R"___(%s)___";

char * getUnicodeNameDataPtr() {
  return Unamedata;
}
int getUnicodeNameDataSize() {
  return Unamesize-1;
}
"""

NonName_regexp = re.compile("<[^>]*>")

def genUnicodeNameData():
    (parsed_data, ranges) = parse_UnicodeData_txt()
    name_data_string = ""
    name_data_len = 0
    for record in parsed_data:
        (cp, name, gc, ccc, bidic, decomp, decval, digitval, numval, bidim, uc, lc, tc) = record
        if NonName_regexp.match(name): continue   # Skip codepoints whose name field is not actually a name.
        name_data_string += cp + ";" + name + "\n"
    # for range_record in ranges:
    #     (lo_cp, hi_cp, range_name, gc, ccc, bidic, decomp, decval, digitval, numval, bidim, uc, lc, tc) = range_record
    #     print(lo_cp, hi_cp, range_name)
    #     if range_name[:13] == "CJK Ideograph":
    #         for cp in range(int(lo_cp,16), int(hi_cp,16)):
    #             name_data_string += "%04X;CJK UNIFIED IDEOGRAPH-%04X\n" % (cp, cp)
    #     elif range_name[:16] == "Tangut Ideograph":
    #         for cp in range(int(lo_cp,16), int(hi_cp,16)):
    #             name_data_string += "%04X;TANGUT IDEOGRAPH-%04X\n" % (cp, cp)
    #     elif range_name[:5] == "Nushu":
    #         for cp in range(int(lo_cp,16), int(hi_cp,16)):
    #             name_data_string += "%04X;NUSHU CHARACTER-%04X\n" % (cp, cp)
    name_data_len = len(name_data_string)
    f = open(UCD_config.UCD_output_dir + '/UnicodeNameData.cpp', 'w')
    f.write(UnicodeNameData_cpp_template % (name_data_len + 1, 255 - (name_data_len % 256), name_data_string))
    f.close()

if __name__ == "__main__":
  genUnicodeNameData()
