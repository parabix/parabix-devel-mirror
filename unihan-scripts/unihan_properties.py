#
# unihan_properties.py - parsing Unihan Database (UCD) files
# and generating C headers for property data using a compact bitset
# representation.
#
import string, os.path, os
from unihan_parser import *

def emit_enumerated_property(f, property_code, independent_prop_values, prop_values, value_map):
    f.write("  namespace %s_ns {\n" % property_code.upper())
    f.write("    const unsigned independent_prop_values = %s;\n" % independent_prop_values)
    for v in prop_values:
        f.write("    /** Code Point Ranges for %s\n    " % v)
        f.write(cformat.multiline_fill(['[%04x, %04x]' % (lo, hi) for (lo, hi) in uset_to_range_list(value_map[v])], ',', 4))
        f.write("**/\n\n")
        f.write(value_map[v].generate(v.lower() + "_Set", 4))
    f.write("\n    }\n")
 
def sum_bytes(value_map):
    byte_sum = 0
    byte_sum = sum([value_map[v].bytes() for v in value_map.keys()])
    return byte_sum
    
def get_property_full_name(property_code):
    property_name = None
    if(property_code == "krs"):
        property_name = "kRSKangXi"
    return property_name
    
class unihan_generator():
    def __init__(self):
        self.parsed_map = []
        self.supported_props = []
        self.property_data_headers = []

    def emit_property(self, f, property_code, prop_values, independent_prop_values, value_map):
    
        prop_full_name = get_property_full_name(property_code)
        
        emit_enumerated_property(f, property_code, independent_prop_values, prop_values, value_map)
        print("%s: %s bytes" % (prop_full_name, sum_bytes(value_map)))
        
        self.supported_props.append(property_code)
    
    def generate_property_value_file(self, filename_root, property_code):
        prop_values, independent_prop_values, value_map = parse_property_file(filename_root, property_code)
        property_name = get_property_full_name(property_code)
        f = cformat.open_header_file_for_write(property_name)
        cformat.write_imports(f, ['"PropertyAliases.h"', '"PropertyObjects.h"', '"PropertyValueAliases.h"', '"unicode_set.h"'])
        f.write("\nnamespace UCD {\n")
        self.emit_property(f, property_code, prop_values, independent_prop_values, value_map)
        f.write("}\n")
        cformat.close_header_file(f)
        self.property_data_headers.append(property_name)

def unihan_main():
    unihan = unihan_generator()
    unihan.generate_property_value_file('Unihan_RadicalStrokeCounts','krs')

if __name__ == "__main__":
    unihan_main()

