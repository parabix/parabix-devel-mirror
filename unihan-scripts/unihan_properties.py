# main

import string, os.path
from unihan_parser import *
from unihan_config import *

def emit_enumerated_property(f, property_code, independent_prop_values, prop_values, value_map, type):
    f.write("  namespace %s_ns {\n" % property_code.upper())
    f.write("    const unsigned independent_prop_values = %s;\n" % independent_prop_values)
    for v in prop_values:
        if type == 'reg':
            f.write("    /** Code Point Ranges for %s\n    " % v)
            f.write(cformat.multiline_fill(['[%04x, %04x]' % (lo, hi) for (lo, hi) in uset_to_range_list(value_map[v])], ',', 4))
            f.write("**/\n\n")
            f.write(value_map[v].generate(v.lower() + "_Set", 4))
        elif type == 'arr':
            for i in range(len(value_map[v])):
                if value_map[v][i] is None:
                    continue
                f.write("    /** Code Point Ranges for %s%d\n    " % (v, i))
                f.write(cformat.multiline_fill(['[%04x, %04x]' % (lo, hi) for (lo, hi) in uset_to_range_list(value_map[v][i])], ',', 4))
                f.write("**/\n\n")
                f.write(value_map[v][i].generate(v.lower() + "%d_Set" % i, 4, "arr"))
            if array:
                f.write(arr_format(v.lower(), value_map[v], 4))
    if not independant:
        set_list = ['&%s_Set' % (v.lower(), i) for v in prop_values for i in range(5)]
        f.write("    static EnumeratedPropertyObject property_object\n")
        f.write("        {%s,\n" % property_code)
        f.write("        %s_ns::independent_prop_values,\n" % property_code.upper())
        f.write("        std::move(%s_ns::enum_names),\n" % property_code.upper())
        f.write("        std::move(%s_ns::value_names),\n" % property_code.upper())
        f.write("        std::move(%s_ns::aliases_only_map),{\n" % property_code.upper())
        f.write("        " + cformat.multiline_fill(set_list, ',', 8))
        f.write("\n        }};")
    f.write("\n    }\n")
                



def arr_format(pr_val, arr, indent):
    R_arr = ' ' * indent + "\n\n" + ' ' * indent + "const static std::array<UnicodeSet, 5> %s_Set = {\n" % pr_val
    for pos, n in enumerate(arr):
        if pos != 0:
            R_arr += ",\n"
        if n is not None:
            R_arr += (' ' * 2 * indent) + "UnicodeSet(const_cast<UnicodeSet::run_t*>(__%s_runs), %i, 0, const_cast<UnicodeSet::bitquad_t *>(__%s_quads), %i, 0)" % (pr_val + "%d_Set" % pos, len(n.runs), pr_val + "%d_Set" % pos, len(n.quads))
        else:
            R_arr += (' ' * 2 * indent) + "UnicodeSet()"
    R_arr += "\n" + (' ' * 2 * indent) + '};\n\n'
    return R_arr

def sum_bytes(value_map, type):
    sum = 0
    if type == 'reg':
        for i in value_map.keys():
            sum += value_map[i].bytes()
    elif type == 'arr':
        for i in value_map.keys():
            for j in range(len(value_map[i])):
                if value_map[i][j] is not None:
                    sum += value_map[i][j].bytes()
    return sum

def get_property_full_name(property_code):
    name = None
    if property_code == "kpy":
        name = "KHanyuPinyin"
    return name 
class unihan_generator():
    def __init__(self):
        self.parsed_map = []
        self.supported_props = []
        self.property_data_headers = []

    def emit_property(self, f, property_code, prop_values, independent_prop_values, value_map):
        full_name = get_property_full_name(property_code)
        if property_code == 'kpy':
            emit_enumerated_property(f, property_code, independent_prop_values, prop_values, value_map, 'arr')
            print("%s: %s bytes" % (full_name, sum_bytes(value_map, 'arr')))      
        else:
            emit_enumerated_property(f, property_code, independent_prop_values, prop_values, value_map, 'reg')
            print("%s: %s bytes" % (full_name, sum_bytes(value_map, 'reg')))        
        self.supported_props.append(property_code)

    def generate_property_value_file(self, filename_root, property_code):
        prop_values, independent_prop_values, value_map = parse_property_file(filename_root, property_code)
        prop_name = get_property_full_name(property_code)
        f = cformat.open_header_file_for_write(prop_name)
        cformat.write_imports(f, ['<array>', '"PropertyAliases.h"', '"PropertyObjects.h"', '"PropertyValueAliases.h"', '"unicode_set.h"'])
        f.write("\nnamespace UCD {\n")
        self.emit_property(f, property_code, prop_values, independent_prop_values, value_map)
        f.write("}\n")
        cformat.close_header_file(f)
        self.property_data_headers.append(prop_name)

def unihan_main():
    unihan = unihan_generator()
    unihan.generate_property_value_file('Unihan_Readings', 'kpy')

if __name__ == "__main__":
  unihan_main()
