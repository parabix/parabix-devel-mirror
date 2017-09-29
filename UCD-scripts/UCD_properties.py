#
# UCD_properties.py - parsing Unicode Character Database (UCD) files
# and generating C headers for property data using a compact bitset
# representation.
#
# Robert D. Cameron
# January 2, 2015
#
# Licensed under Open Software License 3.0.
#
#
import re, string, os.path, cformat, UCD_config
from unicode_set import *
from UCD_parser import *

PropertyAliases_template = r"""
namespace UCD {
    enum property_t {
        %s};
    const static std::vector<std::string> property_enum_name = {
        %s};
    const static std::vector<std::string> property_full_name = {
        %s};
    static std::unordered_map<std::string, int> alias_map {{
        %s}};
}
"""

EnumeratedProperty_template = r"""
    namespace %s_ns {
        enum value_t {
            %s};
        const static std::vector<std::string> enum_names = {
            %s};
        const static std::vector<std::string> value_names = {
            %s};
        static std::unordered_map<std::string, int> aliases_only_map {{
            %s}};
    }
"""

CodepointProperties = ['scf', 'slc', 'suc', 'stc']

class UCD_generator():
    def __init__(self):
        self.supported_props = []
        self.property_data_headers = []
        self.missing_specs = {}
        self.binary_properties = {}

    def load_property_name_info(self):
        (self.property_enum_name_list, self.full_name_map, self.property_lookup_map, self.property_kind_map) = parse_PropertyAlias_txt()

    def generate_PropertyAliases_h(self):
        f = cformat.open_header_file_for_write('PropertyAliases')
        cformat.write_imports(f, ["<string>", "<unordered_map>", "<vector>"])
        enum_text = cformat.multiline_fill(self.property_enum_name_list, ',', 8)
        enum_text2 = cformat.multiline_fill(['"%s"' % e for e in self.property_enum_name_list], ',', 8)
        full_name_text = cformat.multiline_fill(['"%s"' % self.full_name_map[e] for e in self.property_enum_name_list], ',', 8)
        map_text = cformat.multiline_fill(['{"%s", %s}' % (k, self.property_lookup_map[k]) for k in sorted(self.property_lookup_map.keys())], ',', 8)
        f.write(PropertyAliases_template % (enum_text, enum_text2, full_name_text, map_text))
        cformat.close_header_file(f)

    def load_property_value_info(self):
        (self.property_value_list, self.property_value_enum_integer, self.property_value_full_name_map, self.property_value_lookup_map, self.missing_specs) = parse_PropertyValueAlias_txt(self.property_lookup_map)


    def generate_PropertyValueAliases_h(self):
        f = cformat.open_header_file_for_write('PropertyValueAliases')
        cformat.write_imports(f, ['"PropertyAliases.h"', "<vector>", "<unordered_map>", "<string>"])
        f.write("namespace UCD {\n")
        #  Generate the aliases for all Binary properties.
        enum_text = cformat.multiline_fill(['N', 'Y'], ',', 12)
        enum_names = cformat.multiline_fill(['"N"', '"Y"'], ',', 12)
        full_name_text = cformat.multiline_fill(['"No"', '"Yes"'], ',', 12)
        binary_properties = ['{"n", N}', '{"y", Y}', '{"no", N}', '{"yes", Y}', '{"f", N}', '{"t", Y}', '{"false", N}', '{"true", Y}']
        binary_map_text = cformat.multiline_fill(binary_properties, ',', 12)
        f.write(EnumeratedProperty_template % ('Binary', enum_text, enum_names, full_name_text, binary_map_text))
        #
        for p in self.property_enum_name_list:
           if p in self.property_value_list:
              if not self.property_kind_map[p] == 'Binary':
                  enum_text = cformat.multiline_fill(self.property_value_list[p], ',', 12)
                  enum_names = cformat.multiline_fill(['"%s"' % s for s in self.property_value_list[p]], ',', 12)
                  if p == 'ccc': # Special case: add numeric value information for ccc.
                      enum_text += r"""
        };
        const uint16_t enum_val[] = {
    """
                      enum_text += "      " + cformat.multiline_fill(["%s" % (self.property_value_enum_integer[p][e]) for e in self.property_value_list['ccc']], ',', 12)
                  full_names = [self.property_value_full_name_map[p][e] for e in self.property_value_list[p]]
                  full_name_text = cformat.multiline_fill(['"%s"' % name for name in full_names], ',', 12)
                  canon_full_names = [canonicalize(name) for name in full_names]
                  canon_enums = [canonicalize(e) for e in self.property_value_list[p]]
                  canon_keys = [canonicalize(k) for k in self.property_value_lookup_map[p].keys()]
                  aliases_only = [k for k in canon_keys if not k in canon_enums + canon_full_names]
                  map_text = cformat.multiline_fill(['{"%s", %s_ns::%s}' % (k, p.upper(), self.property_value_lookup_map[p][k]) for k in sorted(aliases_only)], ',', 12)
                  f.write(EnumeratedProperty_template % (p.upper(), enum_text, enum_names, full_name_text, map_text))
        f.write("}\n")
        cformat.close_header_file(f)

    def generate_property_value_file(self, filename_root, property_code):
        vlist = self.property_value_list[property_code]
        canon_map = self.property_value_lookup_map[property_code]
        (prop_values, value_map) = parse_UCD_enumerated_property_map(property_code, vlist, canon_map, filename_root + '.txt')
        canon_map = self.property_value_lookup_map[property_code]
        if property_code in self.missing_specs:
            default_value = canon_map[canonicalize(self.missing_specs[property_code])]
            value_map = add_Default_Values(value_map, 0, 0x10FFFF, default_value)
        independent_prop_values = len(prop_values)
        for v in vlist:
            if not v in prop_values:
                #raise Exception("Property %s value %s missing" % (self.full_name_map[property_code], v))
                print("Warning: property %s has no instance of value %s" % (property_code, v))
                prop_values.append(v)
        # 
        self.property_value_list[property_code] = prop_values
        if property_code == 'gc':
            # special logic for derived categories
            value_map['LC'] = union_of_all([value_map[v] for v in ['Lu', 'Ll', 'Lt']])
            value_map['L'] = union_of_all([value_map[v] for v in ['Lu', 'Ll', 'Lt', 'Lm', 'Lo']])
            value_map['M'] = union_of_all([value_map[v] for v in ['Mn', 'Mc', 'Me']])
            value_map['N'] = union_of_all([value_map[v] for v in ['Nd', 'Nl', 'No']])
            value_map['P'] = union_of_all([value_map[v] for v in ['Pc', 'Pd', 'Ps', 'Pe', 'Pi', 'Pf', 'Po']])
            value_map['S'] = union_of_all([value_map[v] for v in ['Sm', 'Sc', 'Sk', 'So']])
            value_map['Z'] = union_of_all([value_map[v] for v in ['Zs', 'Zl', 'Zp']])
            value_map['C'] = union_of_all([value_map[v] for v in ['Cc', 'Cf', 'Cs', 'Co', 'Cn']])
        basename = os.path.basename(filename_root)
        f = cformat.open_header_file_for_write(os.path.basename(filename_root))
        cformat.write_imports(f, ['"PropertyObjects.h"', '"PropertyValueAliases.h"', '"unicode_set.h"'])
        f.write("\nnamespace UCD {\n")
        f.write("  namespace %s_ns {\n" % property_code.upper())
        f.write("    const unsigned independent_prop_values = %s;\n" % independent_prop_values)
        for v in prop_values:
            f.write("    /** Code Point Ranges for %s\n    " % v)
            f.write(cformat.multiline_fill(['[%04x, %04x]' % (lo, hi) for (lo, hi) in uset_to_range_list(value_map[v])], ',', 4))
            f.write("**/\n")
            f.write("    const UnicodeSet %s_Set \n" % v.lower())
            f.write(value_map[v].showC(8) + ";\n")
        print("%s: %s bytes" % (basename, sum([value_map[v].bytes() for v in value_map.keys()])))
        set_list = ['&%s_Set' % v.lower() for v in prop_values]
        f.write("    static EnumeratedPropertyObject property_object\n")
        f.write("        {%s,\n" % property_code)
        f.write("         %s_ns::independent_prop_values,\n" % property_code.upper())
        f.write("         %s_ns::enum_names,\n" % property_code.upper())
        f.write("         %s_ns::value_names,\n" % property_code.upper())
        f.write("         %s_ns::aliases_only_map,\n" % property_code.upper())
        f.write("         {")
        f.write(cformat.multiline_fill(set_list, ',', 8))
        f.write("\n         }};\n    }\n}\n")
        cformat.close_header_file(f)
        self.supported_props.append(property_code)
        self.property_data_headers.append(basename)

    def generate_ScriptExtensions_h(self):
        filename_root = 'ScriptExtensions'
        property_code = 'scx'
        (prop_values, value_map) = parse_ScriptExtensions_txt(self.property_value_list['sc'], self.property_value_lookup_map['sc'])
        basename = os.path.basename(filename_root)
        f = cformat.open_header_file_for_write(basename)
        cformat.write_imports(f, ['"PropertyObjects.h"', '"PropertyValueAliases.h"', '"unicode_set.h"'])
        f.write("\nnamespace UCD {\n")
        f.write("    namespace SCX_ns {\n")
        for v in self.property_value_list['sc']:
            f.write("        /** Code Point Ranges for %s\n        " % v)
            f.write(cformat.multiline_fill(['[%s, %s]' % (lo, hi) for (lo, hi) in uset_to_range_list(value_map[v])], ',', 8))
            f.write("**/\n")
            f.write("        const UnicodeSet %s_Ext \n" % v.lower())
            f.write(value_map[v].showC(12) + ";\n")
        set_list = ['&%s_Ext' % v.lower() for v in self.property_value_list['sc']]
        f.write("        static ExtensionPropertyObject property_object\n")
        f.write("       {%s,\n" % property_code)
        f.write("        UCD::sc,\n")
        f.write("       {")
        f.write(cformat.multiline_fill(set_list, ',', 8))
        f.write("\n        }};\n    }\n}\n")
        cformat.close_header_file(f)
        print("%s: %s bytes" % (basename, sum([value_map[v].bytes() for v in value_map.keys()])))
        self.supported_props.append(property_code)
        self.property_data_headers.append(basename)

    def generate_binary_properties_file(self, filename_root):
        (props, prop_map) = parse_UCD_codepoint_name_map(filename_root + '.txt', self.property_lookup_map)
        basename = os.path.basename(filename_root)
        f = cformat.open_header_file_for_write(basename)
        cformat.write_imports(f, ['"PropertyAliases.h"', '"unicode_set.h"', "<vector>"])
        f.write("\nnamespace UCD {\n")
        for p in sorted(props):
            # f.write("  namespace %s_ns {\n    const UnicodeSet codepoint_set \n" % p.upper())
            # f.write(prop_map[p].showC(12) + ";\n")
            # f.write("    static BinaryPropertyObject property_object{%s, codepoint_set};\n  }\n" % p)
            f.write("    namespace %s_ns {\n" % p.upper())
            f.write("        /** Code Point Ranges for %s\n        " % p)
            f.write(cformat.multiline_fill(['[%s, %s]' % (lo, hi) for (lo, hi) in uset_to_range_list(prop_map[p])], ',', 8))
            f.write("**/\n")
            f.write("        const UnicodeSet codepoint_set \n")
            f.write(prop_map[p].showC(12) + ";\n")
            f.write("        static BinaryPropertyObject property_object{%s, codepoint_set};\n    }\n" % p)
        f.write("}\n\n")
        cformat.close_header_file(f)
        print("%s: %s bytes" % (basename, sum([prop_map[p].bytes() for p in prop_map.keys()])))
        self.supported_props += props
        for p in prop_map.keys(): self.binary_properties[p] = prop_map[p]
        self.property_data_headers.append(basename)

    def generate_PropertyObjectTable_h(self):
        f = cformat.open_header_file_for_write('PropertyObjectTable')
        cformat.write_imports(f, ['"PropertyObjects.h"', '"PropertyAliases.h"', '<array>'])
        cformat.write_imports(f, ['"%s.h"' % fname for fname in self.property_data_headers])
        f.write("\nnamespace UCD {\n")
        f.write("   const std::string UnicodeVersion = \"%s\";\n" % UCD_config.version)
        objlist = []
        for p in self.property_enum_name_list:
            k = self.property_kind_map[p]
            if p in self.supported_props:
                objlist.append("&%s_ns::property_object" % p.upper())
            elif k == 'String':
                if p in CodepointProperties:
                    objlist.append("new UnsupportedPropertyObject(%s, PropertyObject::ClassTypeId::CodepointProperty)" % p)
                else:
                    objlist.append("new UnsupportedPropertyObject(%s, PropertyObject::ClassTypeId::StringProperty)" % p)
            else:
                objlist.append("new UnsupportedPropertyObject(%s, PropertyObject::ClassTypeId::%sProperty)" % (p, k))
        f.write("\n  const std::array<PropertyObject *, %i> property_object_table = {{\n    " % len(objlist))
        f.write(",\n    ".join(objlist) + '  }};\n}\n')
        cformat.close_header_file(f)



def UCD_main():
    setVersionfromReadMe_txt()
    
    ucd = UCD_generator()

    # First parse all property names and their aliases
    ucd.load_property_name_info()
    #
    # Generate the PropertyAliases.h file to define all the Unicode property_t enum
    # and the basic property information.
    ucd.generate_PropertyAliases_h()
    #
    # Next parse all property value names and their aliases.  Generate the data.
    ucd.load_property_value_info()
    #
    # The Age property
    ucd.generate_property_value_file('DerivedAge', 'age')
    #
    # The Block property
    ucd.generate_property_value_file('Blocks', 'blk')
    #
    # Scripts
    ucd.generate_property_value_file('Scripts', 'sc')
    #
    # Script Extensions
    ucd.generate_ScriptExtensions_h()
    #
    # General Category
    ucd.generate_property_value_file('extracted/DerivedGeneralCategory', 'gc')
    #
    # Binary properties from PropList.txt
    ucd.generate_binary_properties_file('PropList')
    #
    # Binary properties from DerivedCoreProperties.txt
    ucd.generate_binary_properties_file('DerivedCoreProperties')
    #
    #
    # LineBreak types
    #ucd.generate_property_value_file('extracted/DerivedLineBreak', 'lb')
    ucd.generate_property_value_file('LineBreak', 'lb')
    #
    # Grapheme Cluster Break property
    ucd.generate_property_value_file('auxiliary/GraphemeBreakProperty', 'GCB')
    #
    # Sentence Break property
    ucd.generate_property_value_file('auxiliary/SentenceBreakProperty', 'SB')
    #
    # Word Break property
    ucd.generate_property_value_file('auxiliary/WordBreakProperty', 'WB')
    #
    # East Asian Width
    ucd.generate_property_value_file('EastAsianWidth', 'ea')
    #ucd.generate_property_value_file('extracted/DerivedEastAsianWidth', 'ea')
    #
    # Hangul Syllable Type
    ucd.generate_property_value_file('HangulSyllableType', 'hst')
    #
    # Bidi Mirroroing from DerivedCoreProperties.txt
    ucd.generate_binary_properties_file('extracted/DerivedBinaryProperties')
    #
    # Canonical_Combining_Class
    ucd.generate_property_value_file('extracted/DerivedCombiningClass', 'ccc')
    #
    # Decomposition Type
    ucd.generate_property_value_file('extracted/DerivedDecompositionType', 'dt')
    #
    # Joining Group and Type
    ucd.generate_property_value_file('extracted/DerivedJoiningGroup', 'jg')
    ucd.generate_property_value_file('extracted/DerivedJoiningType', 'jt')
    #
    # Numeric Type and Value
    ucd.generate_property_value_file('extracted/DerivedNumericType', 'nt')
    #ucd.generate_property_value_file('extracted/DerivedNumericValue', 'nv')
    #
    # Binary normalization properties.
    ucd.generate_binary_properties_file('DerivedNormalizationProps')
    #
    # Bidi_Class
    ucd.generate_property_value_file('extracted/DerivedBidiClass', 'bc')

    #
    # Jamo Short Name - AAARGH - property value for 110B is an empty string!!!!!  - Not in PropertyValueAliases.txt
    # ucd.generate_property_value_file('Jamo', 'jsn')
    #
    # 
    #
    ucd.generate_PropertyValueAliases_h()

    ucd.generate_PropertyObjectTable_h()

if __name__ == "__main__":
  UCD_main()
