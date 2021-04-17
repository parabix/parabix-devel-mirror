from unicode_set import *
trivial_name_char_re = re.compile('[-_\s]')
def canonicalize(property_string):
    return trivial_name_char_re.sub('', property_string.lower())

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


class PropertyObject():
    def __init__(self):
        self.default_value = None
        self.aliases = []
    def setID(self, prop_code, long_name):
        self.property_code = prop_code
        self.full_name = long_name
    def setAliases(self, aliases):
        self.aliases = aliases
    def setSource(self, file_root):
        self.source_file = file_root
    def setDefaultValue(self, default):
        if self.default_value != None and self.default_value != default:
            raise Exception("Conflicting default specification")
        self.default_value = default
    def addDataRecord(self, cp_lo, cp_hi, v):
        pass
    def finalizeProperty(self):
        pass
    def getPropertyCode(self):
        return self.property_code
    def getPropertyFullName(self):
        return self.full_name
    def getAliases(self):
        return self.aliases
    def getFileSource(self):
        return self.source_file
    def getAllCanonicalNames(self):
        return [canonicalize(self.property_code), canonicalize(self.full_name)] + [canonicalize(x) for x in self.aliases]

class EnumeratedPropertyObject(PropertyObject):
    def __init__(self):
        PropertyObject.__init__(self)
        self.property_value_list = []
        self.property_value_enum_integer = {}
        self.property_value_full_name_map = {}
        self.property_value_lookup_map = {}
        self.enum_integer = 0
        self.value_map = {}
        self.default_value = None
        self.name_list_order = []
        self.independent_prop_values = 0

    def getPropertyKind(self): return "Enumerated"

    def addPropertyValue(self, value_enum, value_preferred_full_name, aliases):
        if value_enum in self.property_value_list: raise Exception("Duplicate entry " + value_enum)
        self.property_value_list.append(value_enum)
        if self.property_code == "ccc":
            self.property_value_enum_integer[value_enum] = int(aliases[0])
        else:
            self.property_value_enum_integer[value_enum] = self.enum_integer
            self.enum_integer += 1
        self.property_value_full_name_map[value_enum] = value_preferred_full_name
        for name in [value_enum, value_preferred_full_name] + aliases:
            self.property_value_lookup_map[name] = value_enum
            self.property_value_lookup_map[canonicalize(name)] = value_enum
        self.value_map[value_enum] = empty_uset()

    def setDefaultValue(self, default):
        if not default in self.property_value_lookup_map:
            raise Exception("Erroneous default value %s for property %s" % (default, self.full_name))
        dflt = self.property_value_lookup_map[default]
        if not dflt in self.name_list_order: self.name_list_order = [dflt] + self.name_list_order
        PropertyObject.setDefaultValue(self, dflt)

    def finalizeProperty(self):
        explicitly_defined_cps = empty_uset()
        for k in self.value_map.keys():
            explicitly_defined_cps = uset_union(explicitly_defined_cps, self.value_map[k])
        need_default_value = uset_complement(explicitly_defined_cps)
        dflt = self.default_value
        if not dflt == None:
            if dflt in self.value_map:
                self.value_map[dflt] = uset_union(self.value_map[dflt], need_default_value)
            else:
                self.value_map[dflt] = need_default_value
        self.independent_prop_values = len(self.name_list_order)
        for v in self.property_value_list:
            if not v in self.name_list_order:
                self.name_list_order.append(v)
        self.property_value_list = self.name_list_order
        if self.property_code == 'gc':
            # special logic for derived categories
            self.value_map['LC'] = union_of_all([self.value_map[v] for v in ['Lu', 'Ll', 'Lt']])
            self.value_map['L'] = union_of_all([self.value_map[v] for v in ['Lu', 'Ll', 'Lt', 'Lm', 'Lo']])
            self.value_map['M'] = union_of_all([self.value_map[v] for v in ['Mn', 'Mc', 'Me']])
            self.value_map['N'] = union_of_all([self.value_map[v] for v in ['Nd', 'Nl', 'No']])
            self.value_map['P'] = union_of_all([self.value_map[v] for v in ['Pc', 'Pd', 'Ps', 'Pe', 'Pi', 'Pf', 'Po']])
            self.value_map['S'] = union_of_all([self.value_map[v] for v in ['Sm', 'Sc', 'Sk', 'So']])
            self.value_map['Z'] = union_of_all([self.value_map[v] for v in ['Zs', 'Zl', 'Zp']])
            self.value_map['C'] = union_of_all([self.value_map[v] for v in ['Cc', 'Cf', 'Cs', 'Co', 'Cn']])



    def addDataRecord(self, cp_lo, cp_hi, v):
        canon = canonicalize(v)
        if not canon in self.property_value_lookup_map:
            raise Exception("Unknown enumeration value for %s: %s" % (self.full_name, v))
        enum_code = self.property_value_lookup_map[canon]
        self.value_map[enum_code] = uset_union(self.value_map[enum_code], range_uset(cp_lo, cp_hi))
        if not enum_code in self.name_list_order: self.name_list_order.append(enum_code)

class BinaryPropertyObject(PropertyObject):
    def __init__(self):
        PropertyObject.__init__(self)
        self.empty_regexp = re.compile("\s+")
        self.property_value_full_name_map = {"N" : "No", "Y" : "Yes"}
        self.name_list_order = ['N', 'Y']
        self.property_value_lookup_map = {"n" : "N", "N" : "N", "no" : "N", "f" : "N", "false" : "N",
        "y" : "Y", "Y" : "Y", "yes" : "Y", "t" : "Y", "true" : "Y"}
        self.default_value = "N"
        self.value_map = {"N" : empty_uset(), "Y" : empty_uset()}

    def getPropertyKind(self): return "Binary"

    def addDataRecord(self, cp_lo, cp_hi, v):
        if v==None or v in self.property_value_lookup_map[v] == 'Y':
            self.value_map['Y'] = uset_union(self.value_map['Y'], range_uset(cp_lo, cp_hi))
        else:
            self.value_map['Y'] = uset_difference(self.value_map['Y'], range_uset(cp_lo, cp_hi))

    def setDefaultValue(self, default):
        dflt = canonicalize(default)
        if not dflt in self.property_value_lookup_map:
            raise Exception("Erroneous default value %s for property %s" % (default, self.full_name))
        dflt = self.property_value_lookup_map[dflt]
        if dflt != "N":
            raise Exception("Binary properties must have default value No")

    def fromUnicodeSet(self, theUset):
        self.value_map['Y'] = theUset

class NumericPropertyObject(PropertyObject):
    def __init__(self):
        PropertyObject.__init__(self)
        self.cp_value_map = {}
        self.NaN_set = empty_uset()

    def getPropertyKind(self): return "Numeric"

    def addDataRecord(self, cp_lo, cp_hi, stringValue):
        if stringValue == '':
            self.NaN_set = uset_union(self.NaN_set, range_uset(cp_lo, cp_hi))
        else:
            for cp in range(cp_lo, cp_hi+1):
                self.cp_value_map[cp] = stringValue

    def finalizeProperty(self):
        explicitly_defined_cps = empty_uset()
        for cp in self.cp_value_map.keys():
            explicitly_defined_cps = uset_union(explicitly_defined_cps, singleton_uset(cp))
        # set NaN default
        self.NaN_set = uset_union(self.NaN_set, uset_complement(explicitly_defined_cps))


class ExtensionPropertyObject(PropertyObject):
    def __init__(self):
        PropertyObject.__init__(self)
        self.value_map = {}

    def setBaseProperty(self, property_obj):
        self.base_property = property_obj
        for p in property_obj.property_value_list:
            self.value_map[p] = empty_uset()

    def getPropertyKind(self): return "Extension"

    def addDataRecord(self, cp_lo, cp_hi, base_item_list):
        newset = range_uset(cp_lo, cp_hi)
        base_items = base_item_list.split()
        for e in base_items:
            self.value_map[e] = uset_union(self.value_map[e], newset)

    def finalizeProperty(self):
        explicitly_defined_cps = empty_uset()
        for k in self.value_map.keys():
            explicitly_defined_cps = uset_union(explicitly_defined_cps, self.value_map[k])
        # set <script> default
        for k in self.base_property.value_map.keys():
            base_set = self.base_property.value_map[k]
            if k in ['Zzzz', 'Zyyy', 'Zinh']: # Unknown, Common, Inherited not included if the set is explicitly defined UAX #24
                self.value_map[k] = uset_union(self.value_map[k], uset_difference(base_set, explicitly_defined_cps))
            else:
                self.value_map[k] = uset_union(self.value_map[k], base_set)

codepoint_String_regexp = re.compile("^[A-F0-9]{4,6}(?: [A-F0-9]{4,6})*$")
class StringPropertyObject(PropertyObject):
    def __init__(self):
        PropertyObject.__init__(self)
        self.cp_value_map = {}
        self.null_str_set = empty_uset()
        self.reflexive_set = empty_uset()

    def getPropertyKind(self):
        return "String"

    def addDataRecord(self, cp_lo, cp_hi, stringValue):
        if stringValue == '':
            self.null_str_set = uset_union(self.null_str_set, range_uset(cp_lo, cp_hi))
        else:
            if codepoint_String_regexp.match(stringValue):
                s = ""
                for cp in [int(x, 16) for x in stringValue.split(' ')]:
                    s += chr(cp)
                stringValue = s
            for cp in range(cp_lo, cp_hi+1):
                if len(stringValue) == 1 and ord(stringValue[0]) == cp:
                    #print("Found reflexive entry for %s: %s" % (self.property_code, stringValue))
                    self.reflexive_set = uset_union(self.reflexive_set, singleton_uset(ord(stringValue[0])))
                else:
                    self.cp_value_map[cp] = stringValue

    def finalizeProperty(self):
        explicitly_defined_cps = empty_uset()
        for cp in self.cp_value_map.keys():
            explicitly_defined_cps = uset_union(explicitly_defined_cps, singleton_uset(cp))
        # set <script> default
        if self.default_value == "<code point>":
            self.reflexive_set = uset_union(self.reflexive_set, uset_complement(uset_union(explicitly_defined_cps, self.null_str_set)))
        else:
            self.null_str_set = uset_union(self.null_str_set, uset_complement(uset_union(explicitly_defined_cps, self.reflexive_set)))

    def getStringValue(self, cp):
        if (cp in self.cp_value_map): return self.cp_value_map[cp]
        if self.default_value == "<code point>": return chr(cp)
        return ""


class StringOverridePropertyObject(PropertyObject):
    def __init__(self, overridden_code):
        PropertyObject.__init__(self)
        self.cp_value_map = {}
        self.overridden_code = overridden_code
        self.overridden_set = empty_uset()

    def setBaseObject(self, base_object):
        self.base_object = base_object

    def getPropertyKind(self):
        return "StringOverride"

    def addDataRecord(self, cp_lo, cp_hi, stringValue):
        if codepoint_String_regexp.match(stringValue):
            s = ""
            for cp in [int(x, 16) for x in stringValue.split(' ')]:
                s += chr(cp)
            stringValue = s
        else:
            raise Exception("Expecting codepoint string, but got " + stringValue)
        for cp in range(cp_lo, cp_hi+1):
            if stringValue != self.base_object.getStringValue(cp):
                self.cp_value_map[cp] = stringValue

    def finalizeProperty(self):
        explicitly_defined_cps = empty_uset()
        for cp in self.cp_value_map.keys():
            explicitly_defined_cps = uset_union(explicitly_defined_cps, singleton_uset(cp))
        self.overridden_set = explicitly_defined_cps

class BoundaryPropertyObject(PropertyObject):
    def __init__(self):
        PropertyObject.__init__(self)

    def getPropertyKind(self): return "Boundary"


class ObsoletePropertyObject(PropertyObject):
    def __init__(self):
        PropertyObject.__init__(self)

    def getPropertyKind(self): return "Obsolete"


def getPropertyLookupMap(property_object_map):
    property_lookup_map = {}
    for k in property_object_map.keys():
        po = property_object_map[k]
        names = po.getAllCanonicalNames()
        for n in names:
            property_lookup_map[n] = k
    return property_lookup_map

