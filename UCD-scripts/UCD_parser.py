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
    property_enum_name_list = []
    full_name_map = {}
    property_lookup_map = {}
    property_kind_map = {}
    property_kind = "unspecified"
    f = open(UCD_config.UCD_src_dir + "/" + 'PropertyAliases.txt')
    lines = f.readlines()
    for t in lines:
        m = UCD_property_section_regexp.match(t)
        if m:
            property_kind = m.group(1)
        if UCD_skip.match(t): continue  # skip comment and blank lines
        m = UCD_property_alias_regexp.match(t)
        if not m: raise Exception("Unknown property alias syntax: %s" % t)
        (prop_enum, prop_preferred_full_name, prop_extra) = (m.group(1), m.group(2), m.group(3))
        prop_aliases = re.findall("[-A-Za-z_0-9]+", prop_extra)
        property_enum_name_list.append(prop_enum)
        full_name_map[prop_enum] = prop_preferred_full_name
        property_lookup_map[canonicalize(prop_enum)] = prop_enum
        property_lookup_map[canonicalize(prop_preferred_full_name)] = prop_enum
        for a in prop_aliases: property_lookup_map[canonicalize(a)] = prop_enum
        property_kind_map[prop_enum] = property_kind
    #
    # Override the property kind for scx
    property_kind_map['scx'] = 'Extension'
    return (property_enum_name_list, full_name_map, property_lookup_map, property_kind_map)


UCD_property_value_missing_regexp = re.compile("^#\s*@missing:\s*([0-9A-F]{4,6})[.][.]([0-9A-F]{4,6})\s*;\s*([-A-Za-z_0-9.]+)\s*;\s*([-A-Za-z_0-9.<> ]+)\s*([^#]*)")
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

def parse_PropertyValueAlias_txt(property_lookup_map):
    UCD_property_value_alias_regexp = re.compile("^([-A-Za-z_0-9.]+)\s*;\s*([-A-Za-z_0-9.]+)\s*;\s*([-A-Za-z_0-9.]+)([^#]*)")
    property_value_list = {}
    property_value_enum_integer = {}
    property_value_full_name_map = {}
    property_value_lookup_map = {}
    missing_specs = {}
    f = open(UCD_config.UCD_src_dir + "/" + 'PropertyValueAliases.txt')
    lines = f.readlines()
    for t in lines:
        if UCD_skip.match(t):
            m = UCD_property_value_missing_regexp.match(t)
            if m:
                if m.group(1) != '0000' or m.group(2) != '10FFFF': raise Exception("Bad missing spec: " + s)
                cname = canonicalize(m.group(3))
                if not property_lookup_map.has_key(cname): raise Exception("Bad missing property: " + s)
                missing_specs[property_lookup_map[cname]] = m.group(4)
            continue  # skip comment and blank lines
        m = UCD_property_value_alias_regexp.match(t)
        if not m: raise Exception("Unknown property value alias syntax: %s" % t)
        prop_code = canonicalize(m.group(1))
        if not property_lookup_map.has_key(prop_code): raise Exception("Property code: '%s' is unknown" % prop_code)
        else: prop_code = property_lookup_map[prop_code]
        if not property_value_list.has_key(prop_code):
            property_value_list[prop_code] = []
            property_value_enum_integer[prop_code] = {}
            property_value_full_name_map[prop_code] = {}
            property_value_lookup_map[prop_code] = {}
            enum_integer = 0
        # Special case for ccc: second field is enum integer value
        if prop_code == 'ccc':
            enum_integer = int(m.group(2))
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
        property_value_list[prop_code].append(value_enum)
        property_value_enum_integer[prop_code][value_enum] = enum_integer
        enum_integer += 1
        property_value_full_name_map[prop_code][value_enum] = value_preferred_full_name
        property_value_lookup_map[prop_code][value_enum] = value_enum
        property_value_lookup_map[prop_code][canonicalize(value_enum)] = value_enum
        property_value_lookup_map[prop_code][canonicalize(value_preferred_full_name)] = value_enum
        for a in value_aliases: property_value_lookup_map[prop_code][canonicalize(a)] = value_enum
    # Special case for scx:
    property_value_list['scx'] = property_value_list['sc']
    property_value_enum_integer['scx'] = property_value_enum_integer['sc']
    property_value_full_name_map['scx'] = property_value_full_name_map['sc']
    property_value_lookup_map['scx'] = property_value_lookup_map['sc']
    return (property_value_list, property_value_enum_integer, property_value_full_name_map, property_value_lookup_map, missing_specs)



#
#  Union of a list of sets
#
def union_of_all(uset_list):
    if uset_list == []: return empty_uset()
    else:
        accum_set = uset_list[0]
        for s in uset_list[1:]:
            accum_set = uset_union(accum_set, s)
        return accum_set

#
#  UCD Property File Format 3:  codepoint -> name maps
# 
UCD_skip = re.compile("^#.*$|^\s*$")
UCD_missing_regexp1 = re.compile("^#\s*@missing:\s*([0-9A-F]{4,6})[.][.]([0-9A-F]{4,6})\s*;\s*([-A-Za-z0-9_]+)\s*(?:[;#]|$)")
UCD_point_name_regexp = re.compile("^([0-9A-F]{4,6})\s*;\s*((?:[-A-Za-z0-9_.]+\s+)*[-A-Za-z0-9_.]+)\s*(?:[;#]|$)")
UCD_range_name_regexp = re.compile("^([0-9A-F]{4,6})[.][.]([0-9A-F]{4,6})\s*;\s*((?:[-A-Za-z0-9_.]+\s+)*[-A-Za-z0-9_.]+)\s*(?:[;#]|$)")

#
# Parse a file defining the enumerated property values for a given enumerated property,
# returning the list of independent property values found, as well as the value map.
# Ensure that the default value for the property is first in the list of property values,
# and that all codepoints not explicitly identified in the file are mapped to this default.
def parse_UCD_enumerated_property_map(property_code, vlist, canon_map, mapfile, default_value = None):
    value_map = {}
    for v in vlist: value_map[v] = empty_uset()
    if default_value == None:
        name_list_order = []
    else:
        # Default value must always be first in the final enumeration order.
        name_list_order = [default_value]
    f = open(UCD_config.UCD_src_dir + "/" + mapfile)
    lines = f.readlines()
    for t in lines:
        if UCD_skip.match(t):
            m = UCD_missing_regexp1.match(t)
            if m:
                if default_value != None:
                    raise Exception("Default value already specified, extraneous @missing spec: %s" % t)
                (missing_lo, missing_hi, default_value) = (int(m.group(1), 16), int(m.group(2), 16), m.group(3))
                default_value = canonicalize(default_value)
                if not canon_map.has_key(default_value):  raise Exception("Unknown default property value name '%s'" % default_value)
                if missing_lo != 0 or missing_hi != 0x10FFFF: raise Exception("Unexpected missing data range '%x, %x'" % (missing_lo, missing_hi))
                default_value = canon_map[default_value]
                #print "Property %s: setting default_value  %s" % (property_code, default_value)
                # Default value must always be first in the final enumeration order.
                if default_value in name_list_order: name_list_order.remove(default_value)
                name_list_order = [default_value] + name_list_order
            continue  # skip comment and blank lines
        m = UCD_point_name_regexp.match(t)
        if m:
            (codepoint, name) = (int(m.group(1), 16), m.group(2))
            newset = singleton_uset(codepoint)
        else:
            m = UCD_range_name_regexp.match(t)
            if not m: raise Exception("Unknown syntax: %s" % t)
            (cp_lo, cp_hi, name) = (int(m.group(1), 16), int(m.group(2), 16), m.group(3))
            newset = range_uset(cp_lo, cp_hi)
        cname = canonicalize(name)
        if not canon_map.has_key(cname):  raise Exception("Unknown property or property value name '%s'" % cname)
        name = canon_map[cname]
        if not name in name_list_order:
            name_list_order.append(name)
        value_map[name] = uset_union(value_map[name], newset)
    explicitly_defined_cps = empty_uset()
    for k in value_map.keys(): explicitly_defined_cps = uset_union(explicitly_defined_cps, value_map[k])
    need_default_value = uset_complement(explicitly_defined_cps)
    if default_value != None:
        value_map[default_value] = uset_union(value_map[default_value], need_default_value)
    elif uset_popcount(need_default_value) > 0:
        print "Warning no default value, but %i codepoints not specified" % uset_popcount(need_default_value)
    return (name_list_order, value_map)

def parse_ScriptExtensions_txt(scripts, canon_map):
    filename_root = 'ScriptExtensions'
    property_code = 'scx'
    (scriptlist, script_map) = parse_UCD_enumerated_property_map('sc', scripts, canon_map, 'Scripts.txt')
    (scx_sets, scx_set_map) = parse_UCD_codepoint_name_map('ScriptExtensions.txt')
    value_map = {}
    explicitly_defined_set = empty_uset()
    for scx_list in scx_sets:
        scx_items = scx_list.split(" ")
        for scx in scx_items:
            # sc = canonical_property_value_map[canonicalize(scx)]
            sc = scx
            if value_map.has_key(sc):
                value_map[sc] = uset_union(value_map[sc], scx_set_map[scx_list])
            else: value_map[sc] = scx_set_map[scx_list]
        explicitly_defined_set = uset_union(explicitly_defined_set, scx_set_map[scx_list])
    for v in scripts:
        if value_map.has_key(v):
            value_map[v] = uset_union(value_map[v], uset_difference(script_map[v], explicitly_defined_set))
        elif script_map.has_key(v):
            value_map[v] = script_map[v]
        else: value_map[v] = empty_uset()
    return (scripts, value_map)


def parse_UCD_codepoint_name_map(mapfile, canon_map = None):
    value_map = {}
    name_list_order = []
    f = open(UCD_config.UCD_src_dir + "/" + mapfile)
    lines = f.readlines()
    for t in lines:
        if UCD_skip.match(t):
            continue  # skip comment and blank lines
        m = UCD_point_name_regexp.match(t)
        if m:
            (codepoint, name) = (int(m.group(1), 16), m.group(2))
            newset = singleton_uset(codepoint)
        else:
            m = UCD_range_name_regexp.match(t)
            if not m: raise Exception("Unknown syntax: %s" % t)
            (cp_lo, cp_hi, name) = (int(m.group(1), 16), int(m.group(2), 16), m.group(3))
            newset = range_uset(cp_lo, cp_hi)
        if not canon_map == None:
            cname = canonicalize(name)
            if not canon_map.has_key(cname):
                raise Exception("Unknown property or property value name '%s'" % cname)
            name = canon_map[cname]
        if not value_map.has_key(name):
            value_map[name] = newset
            name_list_order.append(name)
        else: value_map[name] = uset_union(value_map[name], newset)
    return (name_list_order, value_map)


UnicodeData_txt_regexp = re.compile("^([0-9A-F]{4,6});([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);(.*)$")

def parse_UnicodeData_txt():
   data_records = []
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
      data_records.append((cp, name, gc, ccc, bidic, decomp, decval, digitval, numval, bidim, uc, lc, tc))
   return data_records

