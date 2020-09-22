#
# UCD_equivalence.py (python3)
#
# Robert D. Cameron
# October 2018
#
# Licensed under Open Software License 3.0.
#
#
import re, string, os.path, cformat, codecs
import UCD_config
from unicode_set import *
from UCD_parser import *


class UCD_database():
    def __init__(self):
        self.supported_props = []
        self.property_data_headers = []
        self.missing_specs = {}
        self.binary_properties = {}
        self.load_property_name_info()
        self.load_property_value_info()
        parse_UnicodeData_txt(self.property_object_map)
        parse_property_data(self.property_object_map['dt'], 'extracted/DerivedDecompositionType.txt')
        parse_property_data(self.property_object_map['ccc'], 'extracted/DerivedCombiningClass.txt')
        fold_data = parse_CaseFolding_txt(self.property_object_map)
        self.decomp_map = self.property_object_map['dm'].cp_value_map
        self.dt_map = self.property_object_map['dt'].value_map
        self.ccc_map = self.property_object_map['ccc'].value_map
        self.cf_map = self.property_object_map['cf'].cp_value_map
        self.scf_map = self.property_object_map['scf'].cp_value_map


    def load_property_name_info(self):
        (self.property_enum_name_list, self.property_object_map) = parse_PropertyAlias_txt()
        self.property_lookup_map = getPropertyLookupMap(self.property_object_map)
        self.full_name_map = {}
        for p in self.property_enum_name_list:
            self.full_name_map[p] = self.property_object_map[p].getPropertyFullName()

    def load_property_value_info(self):
        initializePropertyValues(self.property_object_map, self.property_lookup_map)

    def NFD(self, strg):
        rslt = ""
        for c in strg:
            cp = ord(c)
            if cp in self.decomp_map.keys() and uset_member(self.dt_map['Can'], cp):
                rslt += self.NFD(self.decomp_map[cp])
            else: rslt += c
        return rslt

    def NFKD(self, strg):
        rslt = ""
        for c in strg:
            cp = ord(c)
            if cp in self.decomp_map.keys():
                rslt += self.NFKD(self.decomp_map[cp])
            else: rslt += c
        return rslt

    def CaseFold(self, strg):
        rslt = ""
        for c in strg:
            cp = ord(c)
            if cp in self.cf_map.keys():
                rslt += self.NFD(self.cf_map[cp])
            elif cp in self.scf_map.keys():
                rslt += self.NFD(self.scf_map[cp])
            else: rslt += c
        return rslt
    def SimpleCaseFold(self, strg):
        rslt = ""
        for c in strg:
            cp = ord(c)
            if cp in self.scf_map.keys():
                rslt += self.scf_map[cp]
            else: rslt += c
        return rslt

def mapping_hex(mapping):
    return " ".join("%X" % ord(c) for c in mapping)


class Trie:
    def __init__(self, prefix):
        self.value = -1
        self.branchMap = {}
        self.prefix = prefix

    def addEntry(self, mapped_value, suffix):
        if suffix == "":
            if self.value != -1 and self.value != mapped_value:
                print("Duplicate entry for %X, %X" % (self.value, mapped_value))
            self.value = mapped_value
        else:
            key = suffix[0]
            suffix1 = suffix[1:]
            if not key in self.branchMap.keys():
                self.branchMap[key] = Trie(self.prefix + key)
            self.branchMap[key].addEntry(mapped_value, suffix1)

    def nodecount(self):
        c = 1
        for k in sorted(self.branchMap.keys()):
            c += self.branchMap[k].nodecount()
        return c

    def show(self):
        rslt = " " * len(self.prefix) + self.prefix + ": %X\n" % self.value 
        for k in sorted(self.branchMap.keys()):
            rslt += self.branchMap[k].show()
        return rslt

    def showTries(self, trie_name=""):
        rslt = ""
        for k in sorted(self.branchMap.keys()):
            rslt += self.branchMap[k].showTries(trie_name)
        cp_string = "_".join("%X"% ord(c) for c in self.prefix)
        if self.prefix != "":
            mapName = trie_name + "_" + cp_string  + "_map"
        else: mapName = trie_name + "_map"
        if len(self.branchMap.keys()) > 0:
            rslt += "const static BranchMap " + mapName + " {"
            delim = "\n    "
            for k in sorted(self.branchMap.keys()):
                rslt += delim + "{0x%X, " % ord(k) + trie_name + "_"
                if cp_string == "":
                    rslt += "%X}" % ord(k)
                else: rslt += cp_string + "_%X}" % ord(k)
                delim = ",\n    "
            rslt += "};\n"
        rslt += "const static Trie %s_" % trie_name 
        if self.prefix == "":
            rslt += "Trie"
        else: rslt += cp_string
        if len(self.branchMap.keys()) == 0:
            rslt += "(0x%X);\n" % self.value
        elif self.value == -1:
            rslt += "(%s);\n" % mapName
        else:
            rslt += "(0x%X, %s);\n" % (self.value, mapName)
        return rslt


PrecomposedMappings_template = r"""
namespace UCD {
class Trie;
typedef std::map<codepoint_t, Trie> BranchMap;

class Trie {
public:
    // Interior Node constructors
    Trie(const codepoint_t p, BranchMap b) : precomposed(p), branches(b) {}
    Trie(BranchMap b) : precomposed(0x110000), branches(b) {}
    // Leaf constructor with a UnicodeSet
    Trie(const codepoint_t p) : precomposed(p) {}
    codepoint_t getPrecomposed() const { return precomposed; }
    void getMatches(std::u32string s, unsigned pos, std::vector<codepoint_t> & matches, std::vector<unsigned> & lgths) const;
    void getClusterMatches(std::u32string s, unsigned pos, std::vector<codepoint_t> & matches) const;
    codepoint_t precomposed;
    BranchMap branches;
};

%s
}
"""


Equivalence_cpp_template = r"""
namespace UCD {

bool hasOption(enum EquivalenceOptions optionSet, enum EquivalenceOptions testOption) {
    return (testOption & optionSet) != 0;
}
    
// Equivalence maps: circular chains of codepoint to codepoint mappings.
// For any codepoint, the full set of equivalent codepoints is found by
// working through the chain until we find the entry itself.
// 
typedef std::unordered_map<codepoint_t, codepoint_t> EquivalenceMap;


UnicodeSet equivalentCodepointsFromMap(codepoint_t cp, EquivalenceMap eqmap) {
    UnicodeSet equiv(cp);
    auto f = eqmap.find(cp);
    while ((f != eqmap.end()) && (f->second != cp)) {
        if (!equiv.contains(f->second)) equiv.insert(f->second);
        f = eqmap.find(f->second);
    }
    return equiv;
}

//
// Canonically equivalent codepoints 
//
%s

//
//  Equivalence for codepoints with a complex casefold-canonical decomposition.
//  (A decomposition that differs from the simple casefold of its NFD.)
//
%s

//
// Compatibly equivalent codepoints 
//
%s

UnicodeSet equivalentCodepoints(codepoint_t cp, EquivalenceOptions opt) {
    UnicodeSet equiv = equivalentCodepointsFromMap(cp, NFD_equivalents);
    if (hasOption(opt, Compatible)) {
        equiv = equiv + equivalentCodepointsFromMap(cp, NFKD_equivalents);
    }
    if (hasOption(opt, Caseless)) {
        equiv = equiv + equivalentCodepointsFromMap(cp, NFDi_equivalents);
        return caseInsensitize(equiv);
    } 
    return equiv;
}

}
"""



# Given a cp-> string map M produce an unique string -> cp R map such that
# for every key k1 in M and k2 = R[M[k]], either k1 == k2 or M[K1] = M[k2].
def unique_reverse_map(cp_string_map):
    reverse_map = {}
    for cp in cp_string_map.keys():
        mapping = cp_string_map[cp]
        eqset = []
        while mapping in reverse_map.keys():
            equiv = reverse_map[mapping]
            if equiv in eqset: raise Exception("Circular mappings!")
            eqset.append(equiv)
            mapping = chr(equiv)
        reverse_map[mapping] = cp
    return reverse_map


def genSingletonEquivalenceMap(reverse_map, mapping_name):
    rslt = "    EquivalenceMap %s_equivalents = {\n" % mapping_name
    processed = {}
    entries = []
    for s in sorted(reverse_map.keys()):
        # only singleton entries are relevent
        if len(s) == 1:
            c = ord(s[0])
            if c in processed.keys(): continue
            equiv = reverse_map[s]
            processed[c] = True
            eq_class = [c, equiv]
            equiv_s = chr(equiv)
            while equiv_s in reverse_map.keys() and not equiv in processed:
                processed[equiv] = True
                equiv = reverse_map[equiv_s]
                eq_class.append(equiv)
                equiv_s = chr(equiv)
            equiv_class_size = len(eq_class)
            if equiv_class_size > 1:
                class_entries = []
                for i in range(equiv_class_size):
                    class_entries.append("{0x%X, 0x%X}" % (eq_class[i], eq_class[(i+1) % equiv_class_size]))
                entries.append(cformat.multiline_fill(class_entries, ',', 8))

    rslt += ",\n".join(entries) + "};\n"
    return rslt

def genReverseMappingTrie(reverse_map, mapping_name):
    t = Trie("")
    for s in reverse_map.keys():
        # Omit singleton entries are relevent
        if len(s) > 1:
            t.addEntry(reverse_map[s], s)
    return t.showTries(mapping_name)


def genCompositeData():
    ucd = UCD_database()

    # Create a map from each codepoint with an NFD expansion to its 
    # full NFD composition.
    NFD_map = {}
    for cp in ucd.decomp_map.keys():
        if uset_member(ucd.dt_map['Can'], cp):
            NFD_map[cp] = ucd.NFD(ucd.decomp_map[cp])

    # For each codepoint that has an NFD or casefold expansion such that
    # the full casefold NFD expansion differs from the simple casefold
    # of the NFD expansion, create an entry mapping the codepoint to its
    # full casefold NFD expansion.
    NFDcasefold_map = {}
    for cp in ucd.decomp_map.keys():
        if uset_member(ucd.dt_map['Can'], cp):
            decomp = ucd.NFD(ucd.decomp_map[cp])
            NFDc = ucd.NFD(ucd.CaseFold(decomp))
            sc = ucd.SimpleCaseFold(decomp)
            if (sc != NFDc):
                print("Codepoint %x has simple NFD case fold %s != full case fold %s" % (cp, mapping_hex(sc), mapping_hex(NFDc)))
                NFDcasefold_map[cp] = NFDc
    for cp in ucd.cf_map.keys():
        if not uset_member(ucd.dt_map['Can'], cp):
            NFDc = ucd.NFD(ucd.CaseFold(chr(cp)))
            NFDcasefold_map[cp] = NFDc

    # Create a map from each codepoint with an NFKD expansion (that differs
    # from its NFD expansinon) to that full NFKD expansion.
    NFKD_map = {}
    for cp in ucd.decomp_map.keys():
        NFKDc = ucd.NFKD(ucd.decomp_map[cp])
        if NFKDc != ucd.NFD(chr(cp)):
            NFKD_map[cp] = NFKDc

    # Create a map from each codepoint with an NFKD or casefold expansion 
    # to its full casefolded NFKD composition.
    NFKDcasefold_map = {}
    for cp in ucd.decomp_map.keys():
        NFD = ucd.NFD(chr(cp))
        NFDc = ucd.NFD(ucd.CaseFold(NFD))
        NFKDc = ucd.NFKD(ucd.CaseFold(ucd.NFKD(ucd.CaseFold(NFD))))
        if NFKDc != NFDc:
            sNFKD = ucd.SimpleCaseFold(ucd.NFKD(NFD))
            if NFKDc != sNFKD:
                NFKDcasefold_map[cp] = NFKDc
    for cp in ucd.cf_map.keys():
        if not cp in ucd.decomp_map.keys():
            casefold = ucd.CaseFold(chr(cp))
            NFDc = ucd.NFD(casefold)
            NFKDc =  ucd.NFKD(ucd.CaseFold(ucd.NFKD(casefold)))
            if NFKDc != NFDc:
                NFKDcasefold_map[cp] =  NFKDc

    #
    # Compute the unique reverse map for NFD, and use it to generate the
    # NFD_equivalent_codepoint map as well as the NFD trie.

    NFD_reverse_map = unique_reverse_map(NFD_map)
    NFDi_reverse_map = unique_reverse_map(NFDcasefold_map)
    NFKD_reverse_map = unique_reverse_map(NFKD_map)


    NFD_equiv = genSingletonEquivalenceMap(NFD_reverse_map, "NFD")
    NFDi_equiv = genSingletonEquivalenceMap(NFDi_reverse_map, "NFDi")
    NFKD_equiv = genSingletonEquivalenceMap(NFKD_reverse_map, "NFKD")
    if len(NFKDcasefold_map.keys()) != 0:
        print("len(NFKDcasefold_map.keys()) != 0  - need to update UCD_composites.py for NFKDi")

    f = cformat.open_cpp_file_for_write('Equivalence')
    cformat.write_imports(f, ["<string>", "<map>", "<vector>", "<UCD/unicode_set.h>", "<UCD/CaseFolding.h>"])
    f.write(Equivalence_cpp_template % (NFD_equiv, NFDi_equiv, NFKD_equiv))
    cformat.close_cpp_file(f)
   

    canon_mappings = genReverseMappingTrie(NFD_reverse_map, "NFD")
    canon_mappings += r"""
//
//  Mappings for codepoints with a complex casefold-canonical decomposition.
//  (A decomposition that differs from the simple casefold of its NFD.)
//
"""
    canon_mappings += genReverseMappingTrie(NFDi_reverse_map, "NFDi")

    compat_mappings = r"""
//
//  Mappings for codepoints with an NFKD decomposition that differs from
//  its NFD decomposition.
//
"""
    compat_mappings += genReverseMappingTrie(NFKD_reverse_map, "NFKD")

#    f = cformat.open_header_file_for_write('CanonicalMappings')
#    cformat.write_imports(f, ["<string>", "<map>", "<vector>", "<UCD/unicode_set.h>", "<UCD/mappings.h>"])
#    f.write(PrecomposedMappings_template % canon_mappings)
#    cformat.close_header_file(f)

#    f = cformat.open_header_file_for_write('CompatibleMappings')
#    cformat.write_imports(f, ["<string>", "<map>", "<vector>", "<UCD/unicode_set.h>", "<UCD/mappings.h>"])
#    f.write(PrecomposedMappings_template % compat_mappings)
#    cformat.close_header_file(f)

    f = cformat.open_header_file_for_write('PrecomposedMappings')
    cformat.write_imports(f, ["<string>", "<map>", "<vector>", "<UCD/unicode_set.h>"])
    f.write(PrecomposedMappings_template % (canon_mappings + compat_mappings))
    cformat.close_header_file(f)


if __name__ == "__main__":
    genCompositeData()
