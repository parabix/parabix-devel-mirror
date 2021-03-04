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
import string, os.path
from UCD_parser import *
from UCD_property_objects import *

PropertyAliases_h_template = r"""
namespace UCD {
    enum property_t : int {
        %s,
        Undefined = -1};
    const std::string & getPropertyEnumName(const property_t);
    const std::string & getPropertyFullName(const property_t);
    property_t getPropertyCode(std::string & propertyIdent);
}
"""

PropertyAliases_cpp_template = r"""
namespace UCD {
    const static std::vector<std::string> property_enum_name = {
        %s};
    const static std::vector<std::string> property_full_name = {
        %s};
    static std::unordered_map<std::string, int> alias_map {{
        %s}};
    const std::string & getPropertyEnumName(const property_t p) {
        return property_enum_name[p];
    }
    const std::string & getPropertyFullName(const property_t p) {
        return property_full_name[p];
    }
    property_t getPropertyCode(std::string & propertyIdent) {
        auto propit = alias_map.find(propertyIdent);
        if (propit == alias_map.end()) return Undefined;
        return static_cast<property_t>(propit->second);
    }
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

def emit_string_property(f, property_code, null_set, reflexive_set, cp_value_map):
    s = string.Template(r"""    namespace ${prop_enum_up}_ns {
        /** Code Point Ranges for ${prop_enum} mapping to <none>
        ${null_set_ranges}**/

        ${null_set_value}

        /** Code Point Ranges for ${prop_enum} mapping to <codepoint>
        ${reflexive_set_ranges}**/

        ${reflexive_set_value}

        const static std::vector<unsigned> buffer_offsets = {
        ${buffer_offsets}};
        const static char string_buffer LLVM_ALIGNAS(64) [${allocation_length}] = u8R"__(${string_buffer})__";

        const static std::vector<codepoint_t> defined_cps{
        ${explicitly_defined_cps}};
        static StringPropertyObject property_object(${prop_enum},
                                                    std::move(null_codepoint_set),
                                                    std::move(reflexive_set),
                                                    static_cast<const char *>(string_buffer),
                                                    std::move(buffer_offsets),
                                                    std::move(defined_cps));
    }
""")
    cps = sorted(cp_value_map.keys())
    string_buffer = ""
    buffer_offsets = [0]
    for cp in cps:
        string_buffer += cp_value_map[cp] + "\n"
        buffer_offsets.append(len(string_buffer.encode("utf-8")))
    buffer_length = buffer_offsets[-1]
    f.write(s.substitute(prop_enum = property_code,
                         prop_enum_up = property_code.upper(),
                         string_buffer = string_buffer,
                         buffer_offsets = cformat.multiline_fill(['%i' % o for o in buffer_offsets], ',', 8),
                         allocation_length = (buffer_length + 255) & -256,
                         null_set_ranges = cformat.multiline_fill(['[%04x, %04x]' % (lo, hi) for (lo, hi) in uset_to_range_list(null_set)], ',', 8),
                         null_set_value = null_set.generate("null_codepoint_set", 8),
                         reflexive_set_ranges = cformat.multiline_fill(['[%04x, %04x]' % (lo, hi) for (lo, hi) in uset_to_range_list(reflexive_set)], ',', 8),
                         reflexive_set_value = reflexive_set.generate("reflexive_set", 8),
                         explicitly_defined_cp_count = len(cps),
                         explicitly_defined_cps = cformat.multiline_fill(['0x%04x' % cp for cp in cps], ',', 8)
                         ))

def emit_string_override_property(f, property_code, overridden_code, override_set, cp_value_map):
    s = string.Template(r"""    namespace ${prop_enum_up}_ns {
        /** Code Point Ranges for ${prop_enum} (possibly overriding values from ${overridden})
        ${overridden_set_ranges}**/

        ${overridden_set_value}

        const static std::vector<unsigned> buffer_offsets = {
        ${buffer_offsets}};
        const static char string_buffer LLVM_ALIGNAS(64) [${allocation_length}] = u8R"__(${string_buffer})__";

        const static std::vector<codepoint_t> defined_cps{
        ${explicitly_defined_cps}};
        static StringOverridePropertyObject property_object(${prop_enum},
                                                    ${overridden},
                                                    std::move(explicitly_defined_set),
                                                    static_cast<const char *>(string_buffer),
                                                    std::move(buffer_offsets),
                                                    std::move(defined_cps));
    }
""")
    cps = sorted(cp_value_map.keys())
    string_buffer = ""
    buffer_offsets = [0]
    for cp in cps:
        string_buffer += cp_value_map[cp] + "\n"
        buffer_offsets.append(len(string_buffer.encode("utf-8")))
    buffer_length = buffer_offsets[-1]
    f.write(s.substitute(prop_enum = property_code,
                         prop_enum_up = property_code.upper(),
                         overridden = overridden_code,
                         string_buffer = string_buffer,
                         buffer_offsets = cformat.multiline_fill(['%i' % o for o in buffer_offsets], ',', 8),
                         allocation_length = (buffer_length + 255) & -256,
                         overridden_set_ranges = cformat.multiline_fill(['[%04x, %04x]' % (lo, hi) for (lo, hi) in uset_to_range_list(override_set)], ',', 8),
                         overridden_set_value = override_set.generate("explicitly_defined_set", 8),
                         explicitly_defined_cp_count = len(cps),
                         explicitly_defined_cps = cformat.multiline_fill(['0x%04x' % cp for cp in cps], ',', 8)
                         ))

def emit_numeric_property(f, property_code, NaN_set, cp_value_map):
    s = string.Template(r"""    namespace ${prop_enum_up}_ns {
        /** Code Point Ranges for ${prop_enum} mapping to NaN
        ${NaN_set_ranges}**/

        ${NaN_set_value}

        const unsigned buffer_length = ${buffer_length};
        const static char string_buffer LLVM_ALIGNAS(64) [${allocation_length}] = u8R"__(${string_buffer})__";

        const static std::vector<codepoint_t> defined_cps = {
        ${explicitly_defined_cps}};
        static NumericPropertyObject property_object(${prop_enum},
                                                    std::move(NaN_set),
                                                    static_cast<const char *>(string_buffer),
                                                    buffer_length,
                                                    std::move(defined_cps));
    }
""")
    cps = sorted(cp_value_map.keys())
    string_buffer = ""
    for cp in cps:
        string_buffer += cp_value_map[cp] + "\n"
    string_buffer += "NaN\n"  # This inserts the standard default value for strings as the last entry
    buffer_length = len(string_buffer.encode("utf-8"))
    f.write(s.substitute(prop_enum = property_code,
                         prop_enum_up = property_code.upper(),
                         string_buffer = string_buffer,
                         buffer_length = buffer_length,
                         allocation_length = (buffer_length + 255) & -256,
                         NaN_set_ranges = cformat.multiline_fill(['[%04x, %04x]' % (lo, hi) for (lo, hi) in uset_to_range_list(NaN_set)], ',', 8),
                         NaN_set_value = NaN_set.generate("NaN_set", 8),
                         explicitly_defined_cp_count = len(cps),
                         explicitly_defined_cps = cformat.multiline_fill(['0x%04x' % cp for cp in cps], ',', 8)
                         ))


def emit_binary_property(f, property_code, property_set):
    f.write("    namespace %s_ns {\n" % property_code.upper())
    f.write("        /** Code Point Ranges for %s\n        " % property_code)
    f.write(cformat.multiline_fill(['[%04x, %04x]' % (lo, hi) for (lo, hi) in uset_to_range_list(property_set)], ',', 8))
    f.write("**/\n\n")
    f.write(property_set.generate("codepoint_set", 8))
    f.write("        static BinaryPropertyObject property_object{%s, std::move(codepoint_set)};\n    }\n" % property_code)

def emit_enumerated_property(f, property_code, independent_prop_values, prop_values, value_map):
    f.write("  namespace %s_ns {\n" % property_code.upper())
    f.write("    const unsigned independent_prop_values = %s;\n" % independent_prop_values)
    for v in prop_values:
        f.write("    /** Code Point Ranges for %s\n    " % v)
        f.write(cformat.multiline_fill(['[%04x, %04x]' % (lo, hi) for (lo, hi) in uset_to_range_list(value_map[v])], ',', 4))
        f.write("**/\n\n")
        f.write(value_map[v].generate(v.lower() + "_Set", 4))
    set_list = ['&%s_Set' % v.lower() for v in prop_values]
    f.write("    static EnumeratedPropertyObject property_object\n")
    f.write("        {%s,\n" % property_code)
    f.write("        %s_ns::independent_prop_values,\n" % property_code.upper())
    f.write("        std::move(%s_ns::enum_names),\n" % property_code.upper())
    f.write("        std::move(%s_ns::value_names),\n" % property_code.upper())
    f.write("        std::move(%s_ns::aliases_only_map),{\n" % property_code.upper())
    f.write("        " + cformat.multiline_fill(set_list, ',', 8))
    f.write("\n        }};"
            "\n    }\n")

def emit_Obsolete_property(f, property_code):
    s = string.Template(r"""    namespace ${prop_enum_up}_ns {
        static ObsoletePropertyObject property_object(${prop_enum});
    }
""")
    f.write(s.substitute(prop_enum = property_code, prop_enum_up = property_code.upper()))


def simple_CaseClosure_map(fold_data):
   simpleFoldMap = {}
   for k in fold_data['S'].keys(): simpleFoldMap[k] = int(fold_data['S'][k], 16)
   for k in fold_data['C'].keys(): simpleFoldMap[k] = int(fold_data['C'][k], 16)
   cl_map = {}
   for k in simpleFoldMap.keys():
      v = simpleFoldMap[k]
      if not v in cl_map: cl_map[v] = [k]
      else: cl_map[v].append(k)
      if not k in cl_map: cl_map[k] = [v]
      else: cl_map[k].append(v)
   newEntries = True
   while newEntries:
      newEntries = False
      for k in cl_map.keys():
         vlist = cl_map[k]
         for v in vlist:
            for w in cl_map[v]:
               if k != w and not k in cl_map[w]:
                  cl_map[w].append(k)
                  newEntries = True
   return cl_map

#
# Simple case fold map.
# The simple case fold map is an ordered list of fold entries each of
# the form (lo_codepoint, hicodepoint, offset).  Each entry describes
# the case fold that applies for the consecutive entries in the given
# codepoint range, according to the following equations.
# casefold(x) = x + offset, if ((x - low_codepoint) div offset) mod 2 = 0
#             = x - offset, if ((x - low_codepoint) div offset) mod 2 = 1
#
#
def caseFoldRangeMap(casemap):
   foldable = sorted(casemap.keys())
   entries = []
   cp = foldable[0]
   open_entries = [(cp, f - cp) for f in casemap[cp]]
   last_cp = cp
   for cp in foldable[1:]:
      if cp != last_cp + 1:
         # Close the pending range entries
         for (cp0, offset) in open_entries:
            entries.append((cp0, last_cp, offset))
         open_entries = [(cp, f - cp) for f in casemap[cp]]
      else:
         new_open = []
         projected = []
         for (cp0, offset) in open_entries:
            even_odd_offset_group = int(abs(cp - cp0)/ abs(offset)) & 1
            if even_odd_offset_group == 0:
               projected_foldcp = cp + offset
            else: projected_foldcp = cp - offset
            if not projected_foldcp in casemap[cp]:
               entries.append((cp0, last_cp, offset))
            else:
               new_open.append((cp0, offset))
               projected.append(projected_foldcp)
         open_entries = new_open
         for f in casemap[cp]:
            if not f in projected:
               open_entries.append((cp, f-cp))
      last_cp = cp
   # Close the final entries.
   for (cp0, offset) in open_entries:
      entries.append((cp0, last_cp, offset))
   return entries



def genFoldEntryData(casemap):
   rMap = caseFoldRangeMap(casemap)
   individuals = [(m[0],m[0]+m[2]) for m in rMap if m[0] == m[1]]
   ranges = [m for m in rMap if m[0] != m[1]]
   last_hi = -1
   generated = "const FoldEntry foldTable[foldTableSize] = {\n"
   foldTableSize = 0
   for (lo, hi, offset) in ranges:
      if lo != last_hi + 1:
         pairs = ["{0x%x, 0x%x}" % (m[0], m[1]) for m in individuals if m[0]>last_hi and m[0]< lo]
         generated += "  {0x%x, 0, {" % (last_hi + 1) + cformat.multiline_fill(pairs) + "}},\n"
         foldTableSize += 1
      last_hi = hi
      pairs = ["{0x%x, 0x%x}" % (m[0], m[1]) for m in individuals if m[0]>=lo and m[0]<= hi]
      generated += "  {0x%x, %i, {" % (lo, offset) + cformat.multiline_fill(pairs) + "}},\n"
      foldTableSize += 1
   if last_hi != 0x10FFFF:
      pairs = ["{0x%x, 0x%x}" % (m[0], m[1]) for m in individuals if m[0]>last_hi]
      generated += "  {0x%x, 0, {" % (last_hi + 1) + cformat.multiline_fill(pairs) + "}},\n"
      foldTableSize += 1
   generated += "  {0x110000, 0, {}}};"
   foldTableSize += 1
   generated = "\nconst int foldTableSize = %s;\n\n" % foldTableSize  + generated
   return generated

foldDeclarations = r"""
struct FoldEntry {
    const UCD::codepoint_t range_lo;
    const int fold_offset;
    const std::vector<UCD::interval_t> fold_pairs;
};

UCD::UnicodeSet caseInsensitize(const UCD::UnicodeSet & cc);

"""


class UCD_generator():
    def __init__(self):
        self.supported_props = []
        self.property_data_headers = []
        self.missing_specs = {}
        self.binary_properties = {}

    def load_property_name_info(self):
        (self.property_enum_name_list, self.property_object_map) = parse_PropertyAlias_txt()
        self.property_lookup_map = getPropertyLookupMap(self.property_object_map)
        self.full_name_map = {}
        for p in self.property_enum_name_list:
            self.full_name_map[p] = self.property_object_map[p].getPropertyFullName()


    def generate_PropertyAliases(self):
        f = cformat.open_header_file_for_write('PropertyAliases')
        cformat.write_imports(f, ["<string>"])
        enum_text = cformat.multiline_fill(self.property_enum_name_list, ',', 8)
        enum_text2 = cformat.multiline_fill(['"%s"' % e for e in self.property_enum_name_list], ',', 8)
        full_name_text = cformat.multiline_fill(['"%s"' % self.full_name_map[e] for e in self.property_enum_name_list], ',', 8)
        map_text = cformat.multiline_fill(['{"%s", %s}' % (k, self.property_lookup_map[k]) for k in sorted(self.property_lookup_map.keys())], ',', 8)
        f.write(PropertyAliases_h_template % (enum_text))
        cformat.close_header_file(f)
        f = cformat.open_cpp_file_for_write('PropertyAliases')
        cformat.write_imports(f, ["<string>", "<unordered_map>", "<vector>", '<unicode/data/PropertyAliases.h>'])
        f.write(PropertyAliases_cpp_template % (enum_text2, full_name_text, map_text))
        cformat.close_cpp_file(f)

    def load_property_value_info(self):
        initializePropertyValues(self.property_object_map, self.property_lookup_map)

    def generate_PropertyValueAliases(self):
        f = cformat.open_header_file_for_write('PropertyValueAliases')
        cformat.write_imports(f, ['<unicode/data/PropertyAliases.h>', "<vector>", "<unordered_map>", "<string>"])
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
            po = self.property_object_map[p]
            if isinstance(po, EnumeratedPropertyObject):
                ordered_enum_list = po.property_value_list
                enum_text = cformat.multiline_fill(ordered_enum_list, ',', 12)
                enum_names = cformat.multiline_fill(['"%s"' % s for s in ordered_enum_list], ',', 12)
                if p == 'ccc': # Special case: add numeric value information for ccc.
                    enum_text += r"""
        };
        const uint16_t enum_val[] = {
        """
                    enum_text += "      " + cformat.multiline_fill(["%s" % (po.property_value_enum_integer[e]) for e in ordered_enum_list], ',', 12)
                full_names = [po.property_value_full_name_map[e] for e in ordered_enum_list]
                full_name_text = cformat.multiline_fill(['"%s"' % name for name in full_names], ',', 12)
                canon_full_names = [canonicalize(name) for name in full_names]
                canon_enums = [canonicalize(e) for e in ordered_enum_list]
                canon_keys = [canonicalize(k) for k in po.property_value_lookup_map.keys()]
                aliases_only = []
                for k in canon_keys:
                    if k in canon_enums: continue
                    if k in canon_full_names: continue
                    if k in aliases_only: continue
                    aliases_only.append(k)
                map_text = cformat.multiline_fill(['{"%s", %s_ns::%s}' % (k, p.upper(), po.property_value_lookup_map[k]) for k in sorted(aliases_only)], ',', 12)
                f.write(EnumeratedProperty_template % (p.upper(), enum_text, enum_names, full_name_text, map_text))
        f.write("}\n")
        cformat.close_header_file(f)

    def emit_property(self, f, property_code):
        property_object = self.property_object_map[property_code]
        if isinstance(property_object, BinaryPropertyObject):
            emit_binary_property(f, property_code, property_object.value_map['Y'])
            print("%s: %s bytes" % (property_object.getPropertyFullName(), property_object.value_map['Y'].bytes()))
        elif isinstance(property_object, EnumeratedPropertyObject):
            prop_values = property_object.name_list_order
            independent_prop_values = property_object.independent_prop_values
            emit_enumerated_property(f, property_code, independent_prop_values, prop_values, property_object.value_map)
            print("%s: %s bytes" % (property_object.getPropertyFullName(), sum([property_object.value_map[v].bytes() for v in property_object.value_map.keys()])))
        elif isinstance(property_object, StringPropertyObject):
            emit_string_property(f, property_code, property_object.null_str_set, property_object.reflexive_set, property_object.cp_value_map)
        elif isinstance(property_object, StringOverridePropertyObject):
            emit_string_override_property(f, property_code, property_object.overridden_code, property_object.overridden_set, property_object.cp_value_map)
        elif isinstance(property_object, NumericPropertyObject):
            emit_numeric_property(f, property_code, property_object.NaN_set, property_object.cp_value_map)
        elif isinstance(property_object, ObsoletePropertyObject):
            emit_Obsolete_property(f, property_code)
        else:
            print("%s: unsupported property.")
            return
        p = property_code.upper()
        f.write("PropertyObject * get_%s_PropertyObject() {  return & %s_ns::property_object; }\n" % (p, p))
        self.supported_props.append(property_code)


    def generate_property_value_file(self, filename_root, property_code):
        if not property_code in self.property_object_map.keys():
            print("Property code %s not in property_object_map" % property_code)
            return
        property_object = self.property_object_map[property_code]
        parse_property_data(self.property_object_map[property_code], filename_root + '.txt')
        basename = os.path.basename(filename_root)
        f = cformat.open_cpp_file_for_write(basename)
        cformat.write_imports(f, ['<unicode/data/PropertyAliases.h>', '<unicode/data/PropertyObjects.h>', '<unicode/data/PropertyValueAliases.h>', '<unicode/core/unicode_set.h>'])
        f.write("\nnamespace UCD {\n")
        self.emit_property(f, property_code)
        f.write("}\n")
        cformat.close_cpp_file(f)
        self.property_data_headers.append(basename)

    def generate_multisection_properties_file(self, filename_root):
        props = parse_multisection_property_data(filename_root + '.txt', self.property_object_map, self.property_lookup_map)
        #(props, prop_map) = parse_UCD_codepoint_name_map(filename_root + '.txt', self.property_lookup_map)
        basename = os.path.basename(filename_root)
        f = cformat.open_cpp_file_for_write(basename)
        cformat.write_imports(f, ['<unicode/data/PropertyAliases.h>', '<unicode/data/PropertyObjects.h>', '<unicode/data/PropertyValueAliases.h>', '<unicode/core/unicode_set.h>'])
        f.write("\nnamespace UCD {\n")
        for p in sorted(props):
            self.emit_property(f, p)
            property_object = self.property_object_map[p]
        f.write("}\n\n")
        cformat.close_cpp_file(f)
        self.property_data_headers.append(basename)

    def generate_multicolumn_properties_file(self, filename_root, prop_code_list):
        props = parse_multicolumn_property_data(filename_root + '.txt', self.property_object_map, self.property_lookup_map, prop_code_list)
        #(props, prop_map) = parse_UCD_codepoint_name_map(filename_root + '.txt', self.property_lookup_map)
        basename = os.path.basename(filename_root)
        f = cformat.open_cpp_file_for_write(basename)
        cformat.write_imports(f, ['<unicode/data/PropertyAliases.h>', '<unicode/data/PropertyObjects.h>', '<unicode/data/PropertyValueAliases.h>', '<unicode/core/unicode_set.h>'])
        f.write("\nnamespace UCD {\n")
        for p in prop_code_list:
            if p in self.property_object_map: self.emit_property(f, p)
        f.write("}\n\n")
        cformat.close_cpp_file(f)
        self.property_data_headers.append(basename)

    def generate_UnicodeData(self):
        basename = 'UnicodeData'
        parse_UnicodeData_txt(self.property_object_map)
        f = cformat.open_cpp_file_for_write(basename)
        cformat.write_imports(f, ['<unicode/data/PropertyAliases.h>', '<unicode/data/PropertyObjects.h>', '<unicode/data/PropertyValueAliases.h>', '<unicode/core/unicode_set.h>'])
        prop_code_list = ['na', 'dm', 'suc', 'slc', 'stc', 'na1', 'isc', 'nv']
        f.write("\nnamespace UCD {\n")
        for p in prop_code_list:
            self.emit_property(f, p)
            property_object = self.property_object_map[p]
        f.write("}\n\n")
        cformat.close_cpp_file(f)
        self.property_data_headers.append(basename)

    def generate_SpecialCasing(self):
        basename = 'SpecialCasing'
        parse_SpecialCasing_txt(self.property_object_map)
        f = cformat.open_cpp_file_for_write(basename)
        cformat.write_imports(f, ['<unicode/data/PropertyAliases.h>', '<unicode/data/PropertyObjects.h>', '<unicode/data/PropertyValueAliases.h>', '<unicode/core/unicode_set.h>'])
        f.write("\nnamespace UCD {\n")
        for p in ['lc', 'uc', 'tc']:
            self.emit_property(f, p)
            property_object = self.property_object_map[p]
        f.write("}\n\n")
        cformat.close_cpp_file(f)
        self.property_data_headers.append(basename)

    def generate_ScriptExtensions(self):
        filename_root = 'ScriptExtensions'
        property_code = 'scx'
        extension_object = self.property_object_map['scx']
        extension_object.setBaseProperty(self.property_object_map['sc'])
        parse_property_data(extension_object, filename_root+'.txt')
        basename = os.path.basename(filename_root)
        f = cformat.open_cpp_file_for_write(basename)
        cformat.write_imports(f, ['<unicode/data/PropertyAliases.h>', '<unicode/data/PropertyObjects.h>', '<unicode/data/PropertyValueAliases.h>', '<unicode/core/unicode_set.h>'])
        prop_list = self.property_object_map['sc'].name_list_order
        value_map = extension_object.value_map
        f.write("\nnamespace UCD {\n")
        f.write("    namespace SCX_ns {\n")
        for v in prop_list:
            f.write("        /** Code Point Ranges for %s\n        " % v)
            f.write(cformat.multiline_fill(['[%04x, %04x]' % (lo, hi) for (lo, hi) in uset_to_range_list(value_map[v])], ',', 8))
            f.write("**/\n")
            f.write(value_map[v].generate(v.lower() + "_Ext", 8))
        set_list = ['&%s_Ext' % v.lower() for v in prop_list]
        f.write("        static ExtensionPropertyObject property_object\n")
        f.write("       {%s,\n" % property_code)
        f.write("        UCD::sc,\n")
        f.write("       {")
        f.write(cformat.multiline_fill(set_list, ',', 8))
        f.write("\n        }};\n    }\n\n")
        f.write("PropertyObject * get_SCX_PropertyObject() {  return & SCX_ns::property_object; }\n}\n")
        cformat.close_cpp_file(f)
        print("%s: %s bytes" % (basename, sum([value_map[v].bytes() for v in value_map.keys()])))
        self.supported_props.append(property_code)
        self.property_data_headers.append(basename)

    def generate_CompatibilityProperties(self):
        basename = 'CompatibilityProperties'
        # alnum
        alnum = self.property_object_map["alnum"]
        alpha = self.property_object_map["Alpha"].value_map['Y']
        digit = self.property_object_map["gc"].value_map['Nd']
        alnum.fromUnicodeSet(uset_union(alpha, digit))
        # xdigit
        xdigit = self.property_object_map["xdigit"]
        hexdigit = self.property_object_map["Hex"].value_map['Y']
        xdigit.fromUnicodeSet(uset_union(hexdigit, digit))
        #blank
        blank = self.property_object_map["blank"]
        space_sep = self.property_object_map["gc"].value_map['Zs']
        blank.fromUnicodeSet(uset_union(space_sep, singleton_uset(9)))
        #graph
        graph = self.property_object_map["graph"]
        control = self.property_object_map["gc"].value_map['Cc']
        surrogate = self.property_object_map["gc"].value_map['Cs']
        unassigned = self.property_object_map["gc"].value_map['Cn']
        space = self.property_object_map["WSpace"].value_map['Y']
        graph.fromUnicodeSet(uset_complement(union_of_all([control, surrogate, unassigned, space])))
        #print
        printp = self.property_object_map["print"]
        printp.fromUnicodeSet(uset_difference(uset_union(graph.value_map['Y'], blank.value_map['Y']), control))
        #word
        word = self.property_object_map["word"]
        connector = self.property_object_map["gc"].value_map['Pc']
        mark = self.property_object_map["gc"].value_map['M']
        join_c = self.property_object_map["Join_C"].value_map['Y']
        word.fromUnicodeSet(union_of_all([alpha, digit, connector, mark, join_c]))

        f = cformat.open_cpp_file_for_write(basename)
        cformat.write_imports(f, ['<unicode/data/PropertyAliases.h>', '<unicode/data/PropertyObjects.h>', '<unicode/data/PropertyValueAliases.h>', '<unicode/core/unicode_set.h>'])
        f.write("\nnamespace UCD {\n")
        for p in Compatibility_Properties:
            if p in self.property_object_map: self.emit_property(f, p)
        f.write("}\n\n")
        cformat.close_cpp_file(f)
        self.property_data_headers.append(basename)


    def generate_PropertyObjectTable(self):
        f = cformat.open_header_file_for_write('PropertyObjectTable')
        cformat.write_imports(f, ['<unicode/data/PropertyObjects.h>', '"PropertyAliases.h"'])
        f.write("\nnamespace UCD {\n")
        for p in self.property_enum_name_list:
            if p in self.supported_props:
                f.write("PropertyObject * get_%s_PropertyObject();\n" % p.upper())
        f.write("\nPropertyObject * getPropertyObject(property_t property_code);\n}\n")
        cformat.close_header_file(f)
        f = cformat.open_cpp_file_for_write('PropertyObjectTable')
        cformat.write_imports(f, ['<unicode/data/PropertyObjectTable.h>', '<array>'])
        f.write("\nnamespace UCD {\n")
        objlist = []
        for p in self.property_enum_name_list:
            k = self.property_object_map[p].getPropertyKind()
            if p in self.supported_props:
                objlist.append("get_%s_PropertyObject()" % p.upper())
            else:
                objlist.append("new UnsupportedPropertyObject(%s, PropertyObject::ClassTypeId::%sProperty)" % (p, k))
        f.write("\n  const std::array<PropertyObject *, %i> property_object_table = {{\n    " % len(objlist))
        f.write(",\n    ".join(objlist) + '  }};\n\n')
        f.write("PropertyObject * getPropertyObject(property_t property_code) {\n")
        f.write("    return property_object_table[property_code];\n}\n}")

        cformat.close_cpp_file(f)




    def generate_UCD_Config(self):
        f = cformat.open_header_file_for_write('UCD_Config')
        f.write("#include <utility>\n")
        f.write("namespace UCD {\n")
        f.write("\tconst auto UnicodeVersion = \"%s\";\n" % UCD_config.UCD_version_string)
        f.write("\tusing codepoint_t = unsigned;\n")
        f.write("\tenum : codepoint_t { UNICODE_MAX = %s };\n" % UCD_config.UCD_max_code_point)
        f.write("\tusing interval_t = std::pair<codepoint_t, codepoint_t>;\n")
        f.write("}\n")
        cformat.close_header_file(f)


    def genCaseFolding(self):
        basename = 'CaseFolding'
        fold_data = parse_CaseFolding_txt(self.property_object_map)
        cm = simple_CaseClosure_map(fold_data)
        f = cformat.open_cpp_file_for_write(basename)
        cformat.write_imports(f, ['<unicode/data/PropertyAliases.h>', '<unicode/data/PropertyObjects.h>', '<unicode/data/PropertyValueAliases.h>', '<unicode/core/unicode_set.h>', '<vector>'])
        f.write(foldDeclarations)
        f.write(genFoldEntryData(cm))
        f.write("\nnamespace UCD {\n")
        self.emit_property(f, 'scf')
        self.emit_property(f, 'cf')
        f.write("}\n")
        cformat.close_cpp_file(f)
        self.supported_props.append(['scf', 'cf'])
        self.property_data_headers.append(basename)



def UCD_main():
    cformat.open_output_directory()
    setVersionfromReadMe_txt()
    if UCD_config.version != UCD_config.UCD_version_string:
        print("UCD version mismatch %s vs %s" % (UCD_config.version, UCD_config.UCD_version_string))
    ucd = UCD_generator()

    # First parse all property names and their aliases
    ucd.load_property_name_info()
    #
    # Generate the PropertyAliases.h file to define all the Unicode property_t enum
    # and the basic property information.
    ucd.generate_PropertyAliases()
    #
    # Next parse all property value names and their aliases.  Generate the data.
    ucd.load_property_value_info()

    ucd.generate_UnicodeData()

    ucd.generate_SpecialCasing()

    ucd.genCaseFolding()

    ucd.generate_multicolumn_properties_file('NameAliases', ['Name_Alias', 'Alias_Kind'])

    #
    # The Age property
    ucd.generate_property_value_file('DerivedAge', 'age')
    #
    # The Block property
    ucd.generate_property_value_file('Blocks', 'blk')

    # Scripts
    ucd.generate_property_value_file('Scripts', 'sc')
    #
    # # Script Extensions
    ucd.generate_ScriptExtensions()
    # #
    # General Category
    ucd.generate_property_value_file('extracted/DerivedGeneralCategory', 'gc')

    # Binary properties from PropList.txt
    ucd.generate_multisection_properties_file('PropList')

    # Binary properties from DerivedCoreProperties.txt
    ucd.generate_multisection_properties_file('DerivedCoreProperties')
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
    # Vertical orientation property
    ucd.generate_property_value_file('VerticalOrientation', 'vo')

    # East Asian Width - can use either source
    ucd.generate_property_value_file('EastAsianWidth', 'ea')
    #ucd.generate_property_value_file('extracted/DerivedEastAsianWidth', 'ea')
    #
    # Hangul Syllable Type
    ucd.generate_property_value_file('HangulSyllableType', 'hst')
    #
    ucd.generate_multisection_properties_file('extracted/DerivedBinaryProperties')
    # #
    # # Canonical_Combining_Class
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
    # Normalization properties.
    ucd.generate_multisection_properties_file('DerivedNormalizationProps')
    #
    # Bidirectional properties
    ucd.generate_property_value_file('extracted/DerivedBidiClass', 'bc')
    ucd.generate_multicolumn_properties_file('BidiBrackets', ['bpb', 'bpt'])
    ucd.generate_property_value_file('BidiMirroring', 'bmg')

    # Indic properties
    ucd.generate_property_value_file('IndicPositionalCategory', 'InPC')
    ucd.generate_property_value_file('IndicSyllabicCategory', 'InSC')

    ucd.generate_property_value_file('CompositionExclusions', 'CE')
    #
    ucd.generate_property_value_file('Jamo', 'JSN')
    #
    ucd.generate_CompatibilityProperties()
    #
        # Binary properties from PropList.txt
    ucd.generate_multisection_properties_file('emoji/emoji-data')

    #
    ucd.generate_PropertyValueAliases()

    ucd.generate_PropertyObjectTable()

    ucd.generate_UCD_Config()

if __name__ == "__main__":
  UCD_main()
