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
        if property_kind == "Binary":
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
            m = UCD_property_value_missing_regexp.match(t)
            if m:
                if m.group(1) != '0000' or m.group(2) != '10FFFF': raise Exception("Bad missing spec: " + s)
                cname = canonicalize(m.group(3))
                if not cname in property_lookup_map: raise Exception("Bad missing property: " + s)
                property_object_map[property_lookup_map[cname]].setDefaultValue(m.group(4))
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

UCD_missing_regexp = re.compile("^#\s*@missing:\s*([0-9A-F]{4,6})[.][.]([0-9A-F]{4,6})\s*;\s*([^#]*)(?:#|$)")

def parse_missing_spec(data_line):
    m = UCD_missing_regexp.match(data_line)
    if not m: raise Exception("UCD missing spec parsing error: " + data_line)
    cp_lo = int(m.group(1), 16)
    cp_hi = int(m.group(2), 16)
    field_data = m.group(3)
    fields = field_data.split(';')
    fields = [f.lstrip().rstrip() for f in fields]
    return (cp_lo, cp_hi, fields)

def parse_property_and_value(fields, property_lookup_map):
    if len(fields) > 2: raise Exception("Too many fields")
    if len(fields) == 0: raise Exception("Expecting at least 1 field")
    canon = canonicalize(fields[0])
    if not canon in property_lookup_map: raise Exception("Unexpected name: " + name_str)
    pcode = property_lookup_map[canon]
    if len(fields) == 1: return (pcode, None)
    else: return (pcode, fields[1])

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
            if v == None:  # binary property
                property_object_map[prop_code].addDataRecord(cp_lo, cp_hi)
            else:
                property_object_map[prop_code].addDataRecord(cp_lo, cp_hi, v)
    for p in props:
        property_object_map[p].finalizeProperty()
    return props

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
                property_object.addDataRecord(cp_lo, cp_hi)
            else:
                property_object.addDataRecord(cp_lo, cp_hi, fields[0])
    property_object.finalizeProperty()

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

def parse_ScriptExtensions_txt(script_property_object):
    filename_root = 'ScriptExtensions'
    parse_property_data(script_property_object, filename_root + '.txt')

UnicodeData_txt_regexp = re.compile("^([0-9A-F]{4,6});([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);(.*)$")

NonNameRange_regexp = re.compile("<([^>]*)>")
NameRange_regexp = re.compile("<([^,]*), (First|Last)>")

def parse_UnicodeData_txt():
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
        (cp, name, gc) = (m.group(1), m.group(2), m.group(3))
        (ccc, bidic, decomp, bidim) = (m.group(4), m.group(5), m.group(6), m.group(10))
        (decval, digitval, numval) = (m.group(7), m.group(8), m.group(9))
        # Unicode 1 name and ISO comment are obolete 
        (uc, lc, tc) = (m.group(13), m.group(14), m.group(15))
        nonNameMatch = NonNameRange_regexp.match(name)
        if nonNameMatch:
            rangeMatch = NameRange_regexp.match(name)
            if rangeMatch:
                rangeName = rangeMatch.group(1)
                print(rangeName, rangeMatch.group(2))
                if rangeMatch.group(2) == 'First': name_range_starts[rangeName] = cp
                if rangeMatch.group(2) == 'Last': 
                    if not rangeName in name_range_starts: raise Exception("UnicodeData range end encountered without prior range start: %s" % t)
                    range_records.append((name_range_starts[rangeName], cp, rangeName, gc, ccc, bidic, decomp, decval, digitval, numval, bidim, uc, lc, tc))
            continue
        data_records.append((cp, name, gc, ccc, bidic, decomp, decval, digitval, numval, bidim, uc, lc, tc))
    return (data_records, range_records)

#  Parse a decomposition mapping field in one of two forms:
#  (a) compatibility mappings:  "<" decomp_type:[A-Za-z]* ">" {codepoint}
#  (b) canonical mappings:  {codepoint}  
compatibility_regexp = re.compile("^<([^>]*)>\s*([0-9A-F ]*)$")
codepoints_regexp = re.compile("^[0-9A-F]{4,6}(?: +[0-9A-F]{4,6})*$")
def parse_decomposition(s):
    m = compatibility_regexp.match(s)
    if m: 
        decomp_type = m.group(1)
        mapping = m.group(2)
    else:
        decomp_type = "Canonical"
        mapping = s
    m = codepoints_regexp.match(mapping)
    if not m: raise Exception("Bad codepoint string syntax in parse_decomposition: %s" % mapping)
    cps = [int(x, 16) for x in mapping.split(" ")]
    return (decomp_type, cps)

