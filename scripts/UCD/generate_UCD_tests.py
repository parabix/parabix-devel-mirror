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
from random import randint

from UCD_parser import *


class UCD_test_generator():
    def __init__(self): 
        self.enum_value_map = {}
        self.binary_value_map = {}
        self.all_good_set = uset_union(range_uset(0x20, 0xD7FF), range_uset(0xE000,0x10FFFF))
        self.all_good_set = uset_difference(self.all_good_set, singleton_uset(0x85))
        self.all_good_set = uset_difference(self.all_good_set, range_uset(0x2028,0x2029))

    def load_property_name_info(self):
        (self.property_enum_name_list, self.property_object_map) = parse_PropertyAlias_txt()
        self.property_lookup_map = getPropertyLookupMap(self.property_object_map)
        self.full_name_map = {}
        for p in self.property_enum_name_list:
            self.full_name_map[p] = self.property_object_map[p].getPropertyFullName()

    def load_property_value_info(self):
        initializePropertyValues(self.property_object_map, self.property_lookup_map)

    def load_property_value_file(self, filename_root, property_code):
        property_object = self.property_object_map[property_code]
        parse_property_data(self.property_object_map[property_code], filename_root + '.txt')

    def load_ScriptExtensions_data(self):
        property_code = 'scx'
        extension_object = self.property_object_map['scx']
        extension_object.setBaseProperty(self.property_object_map['sc'])
        parse_property_data(extension_object, 'ScriptExtensions.txt')
       
    def load_multisection_properties_file(self, filename_root):
        props = parse_multisection_property_data(filename_root + '.txt', self.property_object_map, self.property_lookup_map)
        for p in sorted(props):
            property_object = self.property_object_map[p]

    def load_others(self):
        self.others = ['Alphabetic', 'Uppercase', 'Lowercase', 'White_Space', 'Noncharacter_Code_Point', 'Default_Ignorable_Code_Point', 'ANY', 'ASCII', 'ASSIGNED']
        self.binary_value_map['ANY'] = range_uset(0, 0x10FFFF)
        self.binary_value_map['ASCII'] = range_uset(0, 0x7F)
        self.binary_value_map['ASSIGNED'] = uset_complement(self.property_object_map['gc'].value_map['Cn'])      
        self.binary_value_map['White_Space'] = self.property_object_map['WSpace'].value_map['Y']
        self.binary_value_map['Uppercase'] = self.property_object_map['Upper'].value_map['Y']
        self.binary_value_map['Lowercase'] = self.property_object_map['Lower'].value_map['Y']
        self.binary_value_map['Alphabetic'] = self.property_object_map['Alpha'].value_map['Y']
        self.binary_value_map['Noncharacter_Code_Point'] = self.property_object_map['NChar'].value_map['Y']
        self.binary_value_map['Default_Ignorable_Code_Point'] = self.property_object_map['DI'].value_map['Y']

    def load_all(self):
        # First parse all property names and their aliases
        self.load_property_name_info()
        #
        # Next parse all property value names and their aliases.  Generate the data.
        self.load_property_value_info()
        #
        # The Block property
        self.load_property_value_file('Blocks', 'blk')
        #
        # Scripts
        self.load_property_value_file('Scripts', 'sc')
        #
        # Script Extensions
        self.load_ScriptExtensions_data()
        #
        # General Category
        self.load_property_value_file('extracted/DerivedGeneralCategory', 'gc')
        #
        # Core Properties
        self.load_multisection_properties_file('DerivedCoreProperties')
        #
        self.load_multisection_properties_file('PropList')
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
            obj = self.property_object_map['gc']
            for v in obj.name_list_order:
                s = obj.value_map[v]
                lbl = 'p'
                if randint(1,10) <= negated_per_10:
                    s = uset_complement(s)
                    lbl = 'P'
                terms.append(template % (lbl, v, uset_popcount(uset_intersection(self.all_good_set, s))))
        if 'sc' in propgroups:
            obj = self.property_object_map['sc']
            for v in obj.name_list_order:
                s = obj.value_map[v]
                vname = obj.property_value_full_name_map[v]
                lbl = 'p'
                if randint(1,10) <= negated_per_10:
                    s = uset_complement(s)
                    lbl = 'P'
                terms.append(template % (lbl, vname, uset_popcount(uset_intersection(self.all_good_set, s))))
        if 'scx' in propgroups:
            for v in self.property_object_map['sc'].name_list_order:
                s = self.property_object_map['scx'][v]
                vname = self.property_object_map['sc'].property_value_full_name_map[v]
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
        s1 = self.property_object_map[p1].value_map[t1]
        if p2 == 'others':
            s2 = self.binary_value_map[t2]
        else: s2 = self.property_object_map[p2].value_map[t2]
        if op == 0: s3 = uset_intersection(s1, s2)
        elif op == 1: s3 = uset_difference(s1, s2)
        elif op == 2: s3 = uset_union(s1, s2)
        s3 = uset_intersection(s3, self.all_good_set)
        if p1 == 'sc' or p1 == 'scx': t1 = self.property_object_map['sc'].property_value_full_name_map[t1]
        if p2 == 'sc' or p2 == 'scx': t2 = self.property_object_map['sc'].property_value_full_name_map[t2]
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
        gc = self.property_object_map['gc'].name_list_order
        sc = self.property_object_map['sc'].name_list_order
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
    print("<greptest>")
    for t in ucd.generate_level_1_property_terms(1, ['sc', 'gc']):
        print(t)
    for p in ucd.generate_random_property_expressions(True):
        print(p)
    print("</greptest>")

if __name__ == "__main__":
    UCD_main()
