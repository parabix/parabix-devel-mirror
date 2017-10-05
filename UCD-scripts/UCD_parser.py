#
# UCD_parser.py - parsing Unicode Character Database (UCD) files
#
# Robert D. Cameron
# December 28, 2014
#
# Licensed under Open Software License 3.0.
#
#
import re, string, os.path
import UCD_config
from unicode_set import *
from UCD_property_objects import *

version_regexp = re.compile(".*Version\s+([0-9.]*)\s+of the Unicode Standard.*")

def setVersionfromReadMe_txt():
    f = open(UCD_config.UCD_src_dir + "/" + 'ReadMe.txt')
    lines = f.readlines()
    for t in lines:
        m = version_regexp.match(t)
        if m: 
            UCD_config.version = m.group(1)
            print("Version %s" % m.group(1))

trivial_name_char_re = re.compile('[-_\s]')
def canonicalize(property_string):
    return trivial_name_char_re.sub('', property_string.lower())

#
#  Processing files of the UCD
#
#  General format for skippable comments, blank lines
UCD_skip = re.compile("^#.*$|^\s*$")

#
#  UCD Property File Format 1: property aliases
#  PropertyAliases.txt
#
UCD_property_section_regexp = re.compile("^#\s*([-A-Za-z_0-9]+)\s*Properties\s*$")
UCD_property_alias_regexp = re.compile("^([-A-Za-z_0-9]+)\s*;\s*([-A-Za-z_0-9]+)([^#]*)")

# Section 2.3.3 of UAX $44 
Obsolete_Properties = ["na1", "Gr_Link", "Hyphen", "isc", "XO_NFC", "XO_NFD", "XO_NFKC", "XO_NFKD" ,"FC_NFKC"]

def parse_PropertyAlias_txt():
    property_object_map = {}
    property_enum_name_list = []
    f = open(UCD_config.UCD_src_dir + "/" + 'PropertyAliases.txt')
    lines = f.readlines()
    for t in lines:
        m = UCD_property_section_regexp.match(t)
        if m:
            property_kind = m.group(1)
        if UCD_skip.match(t): continue  # skip comment and blank lines
        m = UCD_property_alias_regexp.match(t)
        if not m: raise Exception("Unknown property alias syntax: %s" % t)
        (property_code, prop_preferred_full_name, prop_extra) = (m.group(1), m.group(2), m.group(3))
        property_enum_name_list.append(property_code)
        if property_code in Obsolete_Properties:
            property_object_map[property_code] = ObsoletePropertyObject()
        elif property_kind == "Binary":
            property_object_map[property_code] = BinaryPropertyObject()
        elif property_kind == "Enumerated":
            property_object_map[property_code] = EnumeratedPropertyObject()
        elif property_kind == "Catalog":   # Age, Block, Script 
            property_object_map[property_code] = EnumeratedPropertyObject()
        elif property_kind == "String":
            property_object_map[property_code] = StringPropertyObject()
        elif property_kind == "Numeric":
            property_object_map[property_code] = NumericPropertyObject()
        else:  # Miscellaneous properties
            if property_code == "scx":
                property_object_map[property_code] = ExtensionPropertyObject()
            else:
                # All other Miscellaneous properties have string values
                property_object_map[property_code] = StringPropertyObject()
        property_object_map[property_code].setID(property_code, prop_preferred_full_name)
        prop_aliases = re.findall("[-A-Za-z_0-9]+", prop_extra)
        property_object_map[property_code].setAliases(prop_aliases)
    return (property_enum_name_list, property_object_map)


#
#  Property Default Value Specifications
#
#  THe UCD uses special comment lines ("@missing specifications") to declare default 
#  values for properties.   Examples showing the two common formats are:
#  (1)  Blocks.txt                    # @missing: 0000..10FFFF; No_Block
#  (2)  PropertyValueAliases.txt      # @missing: 0000..10FFFF; Case_Folding; <code point>
#  The general format gives a range of codepoints (generally 0000..10FFFF),
#  an optional property name (if the file containing the specification defines
#  many different properties), and the default value.
#
#  There are some important default values for different property types:
#  <codepoint>:  This is a default value for certain String properties,
#                indicating the default for a codepoint under the given property
#                is to map to itself.
#  <none>:       This is a default for certain String properties indicating that
#                the default value for a code point is the empty string.
#  <script>:     The default value for the ScriptExtnesions property is the
#                value of the Script property.
#  NaN           The default value for numeric property is the NaN (not a number) value.
#

#  Given a line known to contain such a @missing specification, 
#  parse_missing_spec(data_line) returns a (cp_lo, cp_hi, fields) triple.
#  Generally, cp_lo = 0 and cp_hi = 0x10FFFF
#  The list of fields contains one or two entries: an optional
#  property name and the default value specified for the range.
#  @missing specifications generally omit the property name when
#  the file being processed is defined for a single property only.
#
UCD_missing_check = re.compile("^#\s*@missing:.*")
UCD_missing_regexp = re.compile("^#\s*@missing:\s*([0-9A-F]{4,6})[.][.]([0-9A-F]{4,6})\s*;\s*([^#]*)(?:#|$)")

def parse_missing_spec(data_line):
    m = UCD_missing_regexp.match(data_line)
    if not m: raise Exception("UCD missing spec parsing error: " + data_line)
    cp_lo = int(m.group(1), 16)
    cp_hi = int(m.group(2), 16)
    # We may have to restructure in the event that missing specs do not cover the full Unicode range.
    if cp_lo != 0 or cp_hi != 0x10FFFF: raise Exception("Unexpected range error in missing spec: " + data_line)
    field_data = m.group(3)
    fields = field_data.split(';')
    fields = [f.lstrip().rstrip() for f in fields]
    return (cp_lo, cp_hi, fields)

#
#  Missing specifications and other types of UCD data records often produce
#  a list of one or two fields which indicate a property and a value.
#
#  parse_property_and_value(fields, property_lookup_map) checks that 
#  first of the given fields is indeed a property identifier identified
#  in the given lookup map, and returns a pair consisting of the 
#  unique property code for the property, plus a corresponding value
#  (or None, if only one field was given).
#  
def parse_property_and_value(fields, property_lookup_map):
    if len(fields) > 2: raise Exception("Too many fields")
    if len(fields) == 0: raise Exception("Expecting at least 1 field")
    canon = canonicalize(fields[0])
    if not canon in property_lookup_map: raise Exception("Unexpected name: " + name_str)
    pcode = property_lookup_map[canon]
    if len(fields) == 1: return (pcode, None)
    else: return (pcode, fields[1])

#
#  UCD Property File Format 2: property value aliases
#  PropertyValueAliases.txt
#
#  This file records value aliases for property values for
#  each enumerated property, with the following additional notes:
#  (1) The corresponding integer value of the enum constant is
#      also specified for ccc (second field).
#  (2) The Age property is a numeric type which has decimal float
#      values as the enum constants: these won't be legal in enum syntax.
#  (3) Binary properties also have enumerated values and aliases listed,
#      although this is redundant, because all binary properties have the
#      same value space.
#  (4) @missing lines provide default value information, primarily for some
#      non-enumerated types


def initializePropertyValues(property_object_map, property_lookup_map):
    UCD_property_value_missing_regexp = re.compile("^#\s*@missing:\s*([0-9A-F]{4,6})[.][.]([0-9A-F]{4,6})\s*;\s*([-A-Za-z_0-9.]+)\s*;\s*([-A-Za-z_0-9.<> ]+)\s*([^#]*)")
    UCD_property_value_alias_regexp = re.compile("^([-A-Za-z_0-9.]+)\s*;\s*([-A-Za-z_0-9.]+)\s*;\s*([-A-Za-z_0-9.]+)([^#]*)")
    missing_specs = {}
    f = open(UCD_config.UCD_src_dir + "/" + 'PropertyValueAliases.txt')
    lines = f.readlines()
    for t in lines:
        if UCD_skip.match(t):
            if UCD_missing_check.match(t):
                (cp_lo, cp_hi, fields) = parse_missing_spec(t)
                (property_code, default_value) = parse_property_and_value(fields, property_lookup_map)
                property_object_map[property_code].setDefaultValue(default_value)
            continue  # skip comment and blank lines
        m = UCD_property_value_alias_regexp.match(t)
        if not m: raise Exception("Unknown property value alias syntax: %s" % t)
        prop_code = canonicalize(m.group(1))
        if not prop_code in property_lookup_map: raise Exception("Property code: '%s' is unknown" % prop_code)
        else: prop_code = property_lookup_map[prop_code]
        if not prop_code in property_object_map: raise Exception("Property object: '%s' is uninitialized" % prop_code)
        po = property_object_map[prop_code]
        # Special case for ccc: second field is enum integer value
        if prop_code == 'ccc':
            value_enum = m.group(3)
            extra = m.group(4)
            extra_list = re.findall("[-A-Za-z_0-9.]+", extra)
            value_preferred_full_name = extra_list[0]
            # Treat integer string as an alias
            value_aliases = [m.group(2)] + extra_list[1:]
        # Special case for age: second field is numeric, third field is enum
        # treat numeric value as an alias string
        elif prop_code == 'age':
            value_enum = m.group(3)
            value_preferred_full_name = m.group(3)
            extra = m.group(4)
            value_aliases = [m.group(2)] + re.findall("[-A-Za-z_0-9]+", extra)
        else:
            value_enum = m.group(2)
            value_preferred_full_name = m.group(3)
            extra = m.group(4)
            value_aliases = re.findall("[-A-Za-z_0-9]+", extra)
        if not isinstance(po, EnumeratedPropertyObject): continue
        po.addPropertyValue(value_enum, value_preferred_full_name, value_aliases)


#
#  UCD Property File Format 3:  codepoint/range -> data record maps
#  Many files have data records consisting of a codepoint or codepoint range
#  followed by fields separated by semicolons.
# 

UCD_point_regexp = re.compile("^([0-9A-F]{4,6})([^0-9A-F.#][^#]*)(?:#|$)")
UCD_range_regexp = re.compile("^([0-9A-F]{4,6})[.][.]([0-9A-F]{4,6})([^#]*)(?:#|$)")

#
# parse_data_record is a generic parser for most of the UCD data files.
# Given a data_line beginning with a codepoint or codepoint range,
# this function returns a (cp_lo, cp_hi, fields) triple givnig the
# low and high codepoints of the range (these values may be equal in
# the case of a single codepoint), as well as a list of fields.
# The semicolon separators are removed as well as leading or trailing
# whitespace for each field value.

def parse_data_record(data_line):
    m = UCD_point_regexp.match(data_line)
    if m:
        cp_lo = int(m.group(1), 16)
        cp_hi = cp_lo
        field_data = m.group(2)
    else:
        m = UCD_range_regexp.match(data_line)
        if not m: raise Exception("UCD data record parsing error: " + data_line)
        cp_lo = int(m.group(1), 16)
        cp_hi = int(m.group(2), 16)
        field_data = m.group(3)
    field_data = field_data.lstrip().rstrip()
    if field_data == '': 
        fields = []
    else:
        if field_data[0] != ';': 
            raise Exception("Field data syntax: " + field_data)
        fields = field_data[1:].split(';')
    fields = [f.lstrip().rstrip() for f in fields]
    return (cp_lo, cp_hi, fields)


#  parse_multisection_property_data parses such a file and populates
#  the property objects for each property through successive calls to
#  the corresponding addDataRecord method.
#
def parse_multisection_property_data(pfile, property_object_map, property_lookup_map):
    f = open(UCD_config.UCD_src_dir + "/" + pfile)
    props = []
    lines = f.readlines()
    for t in lines:
        if UCD_missing_regexp.match(t):
            (cp_lo, cp_hi, fields) = parse_missing_spec(t)
            (prop_code, dflt) = parse_property_and_value(fields, property_lookup_map)
            property_object_map[prop_code].setDefaultValue(dflt)
            if not prop_code in props: props.append(prop_code)
        elif UCD_skip.match(t):
            continue
        else:
            (cp_lo, cp_hi, fields) = parse_data_record(t)
            (prop_code, v) = parse_property_and_value(fields, property_lookup_map)
            if not prop_code in props: props.append(prop_code)
            property_object_map[prop_code].addDataRecord(cp_lo, cp_hi, v)
    for p in props:
        property_object_map[p].finalizeProperty()
    return props


#
#   Some UCD files are defined for a single property.   
#   parse_property_data deals with such a file, given the property
#   object to populate and the file root.
#

def parse_property_data(property_object, pfile):
    f = open(UCD_config.UCD_src_dir + "/" + pfile)
    lines = f.readlines()
    for t in lines:
        if UCD_missing_regexp.match(t):
            (cp_lo, cp_hi, fields) = parse_missing_spec(t)
            if len(fields) != 1: raise Exception("Expecting exactly 1 field")
            property_object.setDefaultValue(fields[0])
        elif UCD_skip.match(t):
            continue
        else:
            (cp_lo, cp_hi, fields) = parse_data_record(t)
            if isinstance(property_object, BinaryPropertyObject) and len(fields) == 0:
                property_object.addDataRecord(cp_lo, cp_hi, None)
            else:
                property_object.addDataRecord(cp_lo, cp_hi, fields[0])
    property_object.finalizeProperty()


#
#   Some UCD files are organized to support multiple properties with one
#   property per column.
#   parse_multicolumn_property_data deals with such files given a list of
#   property codes.
#

def parse_multicolumn_property_data(pfile, property_object_map, property_lookup_map, prop_code_list):
    f = open(UCD_config.UCD_src_dir + "/" + pfile)
    props = []
    lines = f.readlines()
    for t in lines:
        if UCD_skip.match(t):
            continue
        else:
            (cp_lo, cp_hi, fields) = parse_data_record(t)
            if len(fields) != len(prop_code_list): raise Exception("Mutlicolumn field count mismatch, expecting %i: " % len(prop_code_list) + t)
            for i in range(len(fields)):
                if fields[i] != '':
                    property_object_map[prop_code_list[i]].addDataRecord(cp_lo, cp_hi, fields[i])
    for p in prop_code_list:
        property_object_map[p].finalizeProperty()

UnicodeData_txt_regexp = re.compile("^([0-9A-F]{4,6});([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);(.*)$")

NonName_regexp = re.compile("<([^>]*)>")
NameRange_regexp = re.compile("<([^,]*), (First|Last)>")

#  Parse a decomposition mapping field in one of two forms:
#  (a) compatibility mappings:  "<" decomp_type:[A-Za-z]* ">" {codepoint}
#  (b) canonical mappings:  {codepoint}  
compatibility_regexp = re.compile("^<([^>]*)>\s*([0-9A-F ]*)$")
def parse_decomposition(s):
    m = compatibility_regexp.match(s)
    if m: 
        decomp_type = m.group(1)
        mapping = m.group(2)
    else:
        decomp_type = "Canonical"
        mapping = s
    return (decomp_type, mapping)

def parse_UnicodeData_txt(property_object_map):
    data_records = []
    range_records = []
    name_range_starts = {}
    f = open(UCD_config.UCD_src_dir + "/UnicodeData.txt")
    lines = f.readlines()
    for t in lines:
        if UCD_skip.match(t):
            continue  # skip comment and blank lines
        m = UnicodeData_txt_regexp.match(t)
        if not m: raise Exception("Unknown syntax: %s" % t)
        (cp, name, gc) = (int(m.group(1), 16), m.group(2), m.group(3))
        (ccc, bidic, decomp, bidim) = (m.group(4), m.group(5), m.group(6), m.group(10))
        (decval, digitval, numval) = (m.group(7), m.group(8), m.group(9))
        (na1, isc) = (m.group(10), m.group(11))
        (suc, slc, stc) = (m.group(13), m.group(14), m.group(15))
        rangeMatch = NameRange_regexp.match(name)
        if rangeMatch:
            rangeName = rangeMatch.group(1)
            print(rangeName, rangeMatch.group(2))
            if rangeMatch.group(2) == 'First': name_range_starts[rangeName] = cp
            if rangeMatch.group(2) == 'Last': 
                if not rangeName in name_range_starts: raise Exception("UnicodeData range end encountered without prior range start: %s" % t)
                range_records.append((name_range_starts[rangeName], cp, rangeName, gc))
        if not NonName_regexp.match(name):
            property_object_map['na'].addDataRecord(cp, cp, name)
        if not decomp == '':
            (decomp_type, mapping) = parse_decomposition(decomp)
            property_object_map['dm'].addDataRecord(cp, cp, mapping)
        if not na1 == '':
            property_object_map['na1'].addDataRecord(cp, cp, na1)
        if not suc == '':
            property_object_map['suc'].addDataRecord(cp, cp, suc)
            if stc == '':
                property_object_map['stc'].addDataRecord(cp, cp, uc)
        if not slc == '':
            property_object_map['slc'].addDataRecord(cp, cp, slc)
        if not stc == '':
            property_object_map['stc'].addDataRecord(cp, cp, stc)
    property_object_map['na'].finalizeProperty()
    property_object_map['na1'].finalizeProperty()
    property_object_map['isc'].finalizeProperty()
    property_object_map['dm'].finalizeProperty()
    property_object_map['slc'].finalizeProperty()
    property_object_map['suc'].finalizeProperty()
    property_object_map['stc'].finalizeProperty()

