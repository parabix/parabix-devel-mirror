#
# Unihan_parser.py - parsing Unihan files
#
# Robert D. Cameron
# Febrary 25, 2021
#
# Licensed under Open Software License 3.0.
#
#
import UCD_config, re, cformat
from unicode_set import *

#
# Unihan entries are triples, consisting of a codepoint expression, a Unihan field name and a value part.
# The value part encodes one or more values separated by spaces.
# Values have their own syntax depending on the field name.
#
Unihan_entry_regexp = re.compile("^U\+([A-F0-9]{4,6})\t([a-zA-Z_]+)\t(.*)$")
Unihan_skip = re.compile("^#.*$|^\s*$")

def parse_Unihan_file(unihan_file):
    f = open(UCD_config.UCD_src_dir + "/Unihan/" + unihan_file)

    lines = f.readlines()
    f.close()
    Unihan_map = {}
    for t in lines:
        if  Unihan_skip.match(t):
            continue
        m = Unihan_entry_regexp.match(t)
        if not m: raise Exception("Unexpected Unihan database syntax: %s" % t)
        cp = int(m.group(1), 16)
        field_name = m.group(2)
        value_part = m.group(3).split(" ")
        if not field_name in Unihan_map:
            Unihan_map[field_name] = {}
        Unihan_map[field_name][cp] = value_part
    return Unihan_map

kRSUnicode_regexp = re.compile("^([0-9]{1,3}'?)\.(-?[0-9]+)$")

def generate_Radical_sets():
    Unihan_prop_map = parse_Unihan_file("Unihan_IRGSources.txt")
    kRSUnicode_map = Unihan_prop_map["kRSUnicode"]
    radical_map = {}
    for cp in kRSUnicode_map.keys():
        cp_set = singleton_uset(cp)
        values = kRSUnicode_map[cp]
        for v in values:
            m = kRSUnicode_regexp.match(v)
            if not m: raise Exception("Unexpected kRSUnicode value: '%s'" % v)
            radical = m.group(1)
            if not radical in radical_map.keys():
                radical_map[radical] = cp_set
            else:
                radical_map[radical] = uset_union(radical_map[radical], cp_set)
    return radical_map

CJKradicals_regexp = re.compile("^([0-9]{1,3}'?);\s+([0-9A-F]{4,6});\s+([0-9A-F]{4,6})\s*$")


def radicalIdent(radicalCode):
    radical_id = radicalCode
    if radicalCode[-1] == "'":
        radical_id = radical_id[:-1] + "s"
    return "kangXi_" + radical_id

def parse_CJK_Radicals_txt():
    f = open(UCD_config.UCD_src_dir + "/CJKRadicals.txt")
    lines = f.readlines()
    f.close()
    radical_map = {}
    ideograph_map = {}
    for t in lines:
        if  Unihan_skip.match(t):
            continue
        m = CJKradicals_regexp.match(t)
        if not m: raise Exception("Unexpected CJKRadicals syntax: %s" % t)
        radical = m.group(1)
        rad_cp = int(m.group(2), 16)
        ideograph_cp = int(m.group(3), 16)
        radical_map[radical] = rad_cp
        ideograph_map[radical] = ideograph_cp
    return (radical_map, ideograph_map)

def generate_Radical_Set_Map():
    (radical_map, ideograph_map) = parse_CJK_Radicals_txt()
    gen = ""
    pending = "\n    {"
    for radical_code in radical_map.keys():
        cps = [radical_map[radical_code], ideograph_map[radical_code]]
        r = "&" + radicalIdent(radical_code)
        for cp in cps:
            gen += pending + "{0x%x, %s}" % (cp, r)
            pending = ",\n    "
    gen += "};\n"
    return gen

RadicalSets_cpp_template = r"""#include <map>
#include <unicode/data/RadicalSets.h>
#include <unicode/core/unicode_set.h>

namespace UCD {

%s

static std::map<codepoint_t, const UnicodeSet *> RadicalSetMap = %s;

const UnicodeSet * getRadicalSet(codepoint_t radical_cp) {
    auto f = RadicalSetMap.find(radical_cp);
    if (f == RadicalSetMap.end()) return nullptr;
    return f->second;
}
}
"""

def generate_RadicalSets_cpp():
    f = cformat.open_cpp_file_for_write('RadicalSets')
    radical_sets = generate_Radical_sets()
    set_decls = ""
    for radical in sorted(radical_sets.keys()):
        set_decls += radical_sets[radical].generate(radicalIdent(radical))
    radical_set_map = generate_Radical_Set_Map()
    f.write(RadicalSets_cpp_template % (set_decls, radical_set_map))
    f.close()

def Unihan_main():
    generate_RadicalSets_cpp()

if __name__ == "__main__":
  Unihan_main()
