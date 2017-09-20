#
# generate_UCD_tests.py -
# generating Python pablo functions for various Unicode properties
#
# Robert D. Cameron
# January 31, 2015
#
# Licensed under Open Software License 3.0.
#
#
import re, string, os.path, cformat
from random import randint
from unicode_set import *
from UCD_parser import *
from string import Template

class UCD_test_generator():
    def __init__(self): 
        self.enum_value_map = {}
        self.binary_value_map = {}
        self.all_good_set = uset_union(range_uset(0x20, 0xD7FF), range_uset(0xE000,0x10FFFF))
        self.all_good_set = uset_difference(self.all_good_set, singleton_uset(0x85))
        self.all_good_set = uset_difference(self.all_good_set, range_uset(0x2028,0x2029))

    def load_property_name_info(self):
        (self.property_enum_name_list, self.full_name_map, self.property_lookup_map, self.property_kind_map) = parse_PropertyAlias_txt()

    def load_property_value_info(self):
        (self.property_value_list, self.property_value_enum_integer, self.property_value_full_name_map, self.property_value_lookup_map, self.missing_specs) = parse_PropertyValueAlias_txt(self.property_lookup_map)

    def load_enumerated_property_data(self, filename_root, property_code):
        vlist = self.property_value_list[property_code]
        canon_map = self.property_value_lookup_map[property_code]
        (prop_values, value_map) = parse_UCD_enumerated_property_map(property_code, vlist, canon_map, filename_root + '.txt')
        self.enum_value_map[property_code] = value_map

    def load_ScriptExtensions_data(self):
        filename_root = 'ScriptExtensions'
        property_code = 'scx'
        vlist = self.property_value_list['sc']
        (prop_values, value_map) = parse_ScriptExtensions_txt(vlist, self.property_value_lookup_map['sc'])
        self.enum_value_map['scx'] = value_map
       
    def load_binary_properties_data(self, filename_root):
        (props, prop_map) = parse_UCD_codepoint_name_map(filename_root + '.txt', self.property_lookup_map)
        for p in props:
            self.binary_value_map[p] = prop_map[p]

    def load_others(self):
        self.others = ['Alphabetic', 'Uppercase', 'Lowercase', 'White_Space', 'Noncharacter_Code_Point', 'Default_Ignorable_Code_Point', 'ANY', 'ASCII', 'ASSIGNED']
        self.binary_value_map['ANY'] = range_uset(0, 0x10FFFF)
        self.binary_value_map['ASCII'] = range_uset(0, 0x7F)
        self.binary_value_map['ASSIGNED'] = uset_complement(self.enum_value_map['gc']['Cn'])      
        self.binary_value_map['White_Space'] = self.binary_value_map['WSpace']
        self.binary_value_map['Uppercase'] = self.binary_value_map['Upper']
        self.binary_value_map['Lowercase'] = self.binary_value_map['Lower']
        self.binary_value_map['Alphabetic'] = self.binary_value_map['Alpha']
        self.binary_value_map['Noncharacter_Code_Point'] = self.binary_value_map['NChar']
        self.binary_value_map['Default_Ignorable_Code_Point'] = self.binary_value_map['DI']

    def load_all(self):
        # First parse all property names and their aliases
        self.load_property_name_info()
        #
        # Next parse all property value names and their aliases.  Generate the data.
        self.load_property_value_info()
        #
        # The Block property
        self.load_enumerated_property_data('Blocks', 'blk')
        #
        # Scripts
        self.load_enumerated_property_data('Scripts', 'sc')
        #
        # Script Extensions
        self.load_ScriptExtensions_data()
        #
        # General Category
        self.load_enumerated_property_data('extracted/DerivedGeneralCategory', 'gc')
        #
        # Core Properties
        self.load_binary_properties_data('DerivedCoreProperties')
        #
        self.load_binary_properties_data('PropList')
        self.load_others()

    def generate_level_1_property_terms(self, negated_per_10 = 5, propgroups=['others', 'sc', 'scx', 'gc']):
        template = r"""<grepcase regexp="^\%s{%s}$" datafile="All_good" grepcount="%i"/>"""
        terms = []
        if 'others' in propgroups:
            for p in self.others:
                s = self.binary_value_map[p]
                lbl = 'p'
                if randint(1,10) <= negated_per_10:
                    s = uset_complement(s)
                    lbl = 'P'
                terms.append(template % (lbl, p, uset_popcount(uset_intersection(self.all_good_set, s))))
        if 'gc' in propgroups:
            for v in self.property_value_list['gc']:
                s = self.enum_value_map['gc'][v]
                lbl = 'p'
                if randint(1,10) <= negated_per_10:
                    s = uset_complement(s)
                    lbl = 'P'
                terms.append(template % (lbl, v, uset_popcount(uset_intersection(self.all_good_set, s))))
        if 'sc' in propgroups:
            for v in self.property_value_list['sc']:
                s = self.enum_value_map['sc'][v]
                vname = self.property_value_full_name_map['sc'][v]
                lbl = 'p'
                if randint(1,10) <= negated_per_10:
                    s = uset_complement(s)
                    lbl = 'P'
                terms.append(template % (lbl, vname, uset_popcount(uset_intersection(self.all_good_set, s))))
        if 'scx' in propgroups:
            for v in self.property_value_list['sc']:
                s = self.enum_value_map['scx'][v]
                vname = self.property_value_full_name_map['sc'][v]
                lbl = 'p'
                if randint(1,10) <= negated_per_10:
                    s = uset_complement(s)
                    lbl = 'P'
                terms.append(template % (lbl, "scx=" + vname, uset_popcount(uset_intersection(self.all_good_set, s))))
        return terms

    def random_binary(self, a1, a2, useLookbehindAssertions = False):
        (p1, t1) = a1
        (p2, t2) = a2
        op = randint(0,2)
        s1 = self.enum_value_map[p1][t1]
        if p2 == 'others':
            s2 = self.binary_value_map[t2]
        else: s2 = self.enum_value_map[p2][t2]
        if op == 0: s3 = uset_intersection(s1, s2)
        elif op == 1: s3 = uset_difference(s1, s2)
        elif op == 2: s3 = uset_union(s1, s2)
        s3 = uset_intersection(s3, self.all_good_set)
        if p1 == 'sc' or p1 == 'scx': t1 = self.property_value_full_name_map['sc'][t1]
        if p2 == 'sc' or p2 == 'scx': t2 = self.property_value_full_name_map['sc'][t2]
        if p1 == 'scx': t1 = 'scx=' + t1
        if p2 == 'scx': t2 = 'scx=' + t2
        v1 = "\\p{%s}" % (t1)
        v2 = "\\p{%s}" % (t2)
        if not useLookbehindAssertions:
            opr = ["&amp;&amp;", "--", ""][op]
            return r"""<grepcase regexp="^[%s%s%s]$" datafile="All_good" grepcount="%i"/>""" % (v1, opr, v2, uset_popcount(s3))
        if op == 0:
            return r"""<grepcase regexp="^%s(?&lt;=%s)$" datafile="All_good" grepcount="%i"/>""" % (v1, v2, uset_popcount(s3))
        elif op == 1:
            return r"""<grepcase regexp="^%s(?&lt;!%s)$" datafile="All_good" grepcount="%i"/>""" % (v1, v2, uset_popcount(s3))
        else:
            return r"""<grepcase regexp="^[%s%s]$" datafile="All_good" grepcount="%i"/>""" % (v1, v2, uset_popcount(s3))

    def generate_random_property_expressions(self, useLookbehindAssertions = False):
        gc = self.property_value_list['gc']
        sc = self.property_value_list['sc']
        others = ['Alphabetic', 'Uppercase', 'Lowercase', 'White_Space', 'Noncharacter_Code_Point', 'Default_Ignorable_Code_Point', 'ANY', 'ASCII', 'ASSIGNED']
        exprs = []
        for p in gc:
           s = sc[randint(0, len(sc)-1)]
           exprs.append(self.random_binary(('gc', p), ('sc', s), useLookbehindAssertions))           
           #sx = sc[randint(0, len(sc)-1)]
           #exprs.append(self.random_binary(('gc', p), ('scx', sx), useLookbehindAssertions))
           #othr = others[randint(0, len(others)-1)]
           #exprs.append(self.random_binary(('gc', p), ('others', othr), useLookbehindAssertions))
        for p in sc:
           g = gc[randint(0, len(gc)-1)]
           exprs.append(self.random_binary(('sc', p), ('gc', g), useLookbehindAssertions))           
           #sx = sc[randint(0, len(sc)-1)]
           #exprs.append(self.random_binary(('sc', p), ('scx', sx), useLookbehindAssertions))
           #othr = others[randint(0, len(others)-1)]
           #exprs.append(self.random_binary(('sc', p), ('others', othr), useLookbehindAssertions))
        #for p in others:
           #s = sc[randint(0, len(sc)-1)]
           #exprs.append(self.random_binary(('sc', s), ('others', p), useLookbehindAssertions))
           #sx = sc[randint(0, len(sc)-1)]
           #exprs.append(self.random_binary(('scx', sx), ('others', p), useLookbehindAssertions))
        return exprs

def UCD_main():
    ucd = UCD_test_generator()
    ucd.load_all()
    print "<greptest>"
    for t in ucd.generate_level_1_property_terms(1, ['sc', 'gc']):
        print t
    for p in ucd.generate_random_property_expressions(True):
        print p
    print "</greptest>"

if __name__ == "__main__":
    UCD_main()
