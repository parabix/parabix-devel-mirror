import os
import string, os.path
from UniHan_parser import *

# configuration for whether using array_format in the .h header file
from UniHan_config import array_format, independent_header

def is_property_array(property_code):
    is_property_array_map = {
        "kpy":  True, 
        "kxhc": True,
    }
    if(property_code in is_property_array_map):
        return is_property_array_map[property_code]
    else:
        raise ValueError("Invalid Property Code:" + property_code)

def sum_bytes(value_map, property_type = "single"):
    bytes_sum = 0
    if(property_type == "single"):
        bytes_sum = sum([value_map[v].bytes() for v in value_map.keys()])
    elif(property_type == "array"):
        for v in value_map.keys():
            for i in range(len(value_map[v])):
                if(value_map[v][i] is not None):
                    bytes_sum += value_map[v][i].bytes()
    else:
        raise ValueError("Invalid Property Type: " + property_type)
    return bytes_sum
def get_property_full_name(property_code):
    # return Property Full Name of the property code
    property_full_name = None
    if(property_code == "kpy"):
        property_full_name = "KHanyuPinyin"
    if(property_code == "kxhc"):
        property_full_name = "KXHC1983"
    return property_full_name

def write_array_format(propertyValue, uset_array, indent = 4 ):
    # propertyValue is without "_Set" postfix
    str = (" " * indent) + "\n\n" + \
                (" " * indent) + \
                "const static std::array<UnicodeSet, 5> %s_Set = {\n" % propertyValue
    for index, uset in enumerate(uset_array):
        if(index != 0):
            str += ',\n'
        if(uset != None):
            str += (" " * (2*indent)) + "UnicodeSet(const_cast<UnicodeSet::run_t *>(__%s_runs), %i, 0, " \
                    "const_cast<UnicodeSet::bitquad_t *>(__%s_quads), %i, 0)" \
                    % (propertyValue+'%d_Set'%index, len(uset.runs),
                    propertyValue+'%d_Set'%index, len(uset.quads))
        else:
            str += (" " * (2*indent)) + "UnicodeSet()"
    str += '\n' + (2*indent)*" " + '};\n\n'
    return str

def emit_enumerated_property(f, property_code, independent_prop_values, prop_values, value_map, property_type = "single"):
    f.write("  namespace %s_ns {\n" % property_code.upper())
    f.write("    const unsigned independent_prop_values = %s;\n" % independent_prop_values)
    for v in prop_values:
        # to modify
        if(property_type == "single"):
            f.write("    /** Code Point Ranges for %s\n    " % v)
            f.write(cformat.multiline_fill(['[%04x, %04x]' % (lo, hi) for (lo, hi) in uset_to_range_list(value_map[v])], ',', 4))
            f.write("**/\n\n")
            f.write(value_map[v].generate(v.lower() + "_Set", 4))
        elif(property_type == "array"):
            for i in range(len(value_map[v])):
                if(value_map[v][i] is None):
                    continue
                f.write("    /** Code Point Ranges for %s%d\n    " % (v, i))
                f.write(cformat.multiline_fill(['[%04x, %04x]' % (lo, hi) for (lo, hi) in uset_to_range_list(value_map[v][i])], ',', 4))
                f.write("**/\n\n")
                f.write(value_map[v][i].generate(v.lower() + "%d_Set"%i, 4, "array"))
            if(array_format == True):
                f.write(write_array_format(v.lower(), value_map[v], 4))
        else:
            raise ValueError("Invalide Property Type " + property_type)
    if(not independent_header):
        set_list = ['&%s_Set[%d]' % (v.lower(), i) for v in prop_values for i in range(5)]
        f.write("    static EnumeratedPropertyObject property_object\n")
        f.write("        {%s,\n" % property_code)
        f.write("        %s_ns::independent_prop_values,\n" % property_code.upper())
        f.write("        std::move(%s_ns::enum_names),\n" % property_code.upper())
        f.write("        std::move(%s_ns::value_names),\n" % property_code.upper())
        f.write("        std::move(%s_ns::aliases_only_map),{\n" % property_code.upper())
        f.write("        " + cformat.multiline_fill(set_list, ',', 8))
        f.write("\n        }};")

    f.write("\n    }\n") # end } for namespace _ns

class UniHan_generator():
    def __init__(self):
        self.parsed_map = []
        self.supported_props = []
        self.property_data_headers = []
        
    def emit_property(self, f, property_code, prop_values, independent_prop_values, value_map):
        prop_full_name = get_property_full_name(property_code)
        
        if(is_property_array(property_code)):
            emit_enumerated_property(f, property_code, independent_prop_values, prop_values, value_map, "array")
            print("%s: %s bytes" % (prop_full_name, sum_bytes(value_map,"array")))
        else:
            emit_enumerated_property(f, property_code, independent_prop_values, prop_values, value_map)
            print("%s: %s bytes" % (prop_full_name, sum_bytes(value_map)))
        
        self.supported_props.append(property_code)
    
    def generate_property_value_file(self, filename_root, property_code):
        prop_values, independent_prop_values, value_map = parse_property_file(filename_root, property_code)
        property_name = get_property_full_name(property_code)
        f = cformat.open_header_file_for_write(property_name)
        cformat.write_imports(f, ['<array>','"PropertyAliases.h"', '"PropertyObjects.h"', '"PropertyValueAliases.h"', '<unicode/core/unicode_set.h>'])
        f.write("\nnamespace UCD {\n")
        self.emit_property(f, property_code, prop_values, independent_prop_values, value_map)
        f.write("}\n")
        cformat.close_header_file(f)
        self.property_data_headers.append(property_name)

def UniHan_main():
    UniHan = UniHan_generator()
    #UniHan.generate_property_value_file('Unihan_Readings','kpy')
    UniHan.generate_property_value_file('Unihan_Readings','kxhc')

if __name__ == "__main__":
    UniHan_main()