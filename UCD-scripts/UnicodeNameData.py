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
char __attribute__ ((aligned (32))) Unamedata[Unamesize + %s] = "%s";

char * getUnicodeNameDataPtr() {
  return Unamedata;
}
int getUnicodeNameDataSize() {
  return Unamesize-1;
}
"""


def genUnicodeNameData():
    parsed_data = parse_UnicodeData_txt()
    name_data_string = ""
    name_data_len = 0
    for record in parsed_data:
        (cp, name, gc, ccc, bidic, decomp, decval, digitval, numval, bidim, uc, lc, tc) = record
        name_data_string += cp + ";" + name + "\\n"
        name_data_len += len(cp) + len(name) + 2
    f = open(UCD_config.UCD_output_dir + '/UnicodeNameData.cpp', 'w')
    f.write(UnicodeNameData_cpp_template % (name_data_len + 1, 255 - (name_data_len % 256), name_data_string))
    f.close()

if __name__ == "__main__":
  genUnicodeNameData()
