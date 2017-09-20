#
# casefold.py - parsing Unicode Character Database (UCD) files
# and generating C headers for property data using a compact bitset
# representation.
#
# Robert D. Cameron
# December 2, 2014
#
# Licensed under Open Software License 3.0.
#
#
import re, string, cformat
import UCD_config
from unicode_set import *



#
#  Processing files of the UCD
#
#  General format for skippable comments, blank lines
UCD_skip = re.compile("^#.*$|^\s*$")

#
#  UCD Property File Format 4: property aliases
#  PropertyAliases.txt
#
UCD_case_fold_regexp = re.compile("^([0-9A-F]{4,6})\s*;\s*([CSFT]);\s*((?:[-A-Za-z0-9_]+\s+)*[-A-Za-z0-9_]+)\s*(?:[;#]|$)")

def parse_CaseFolding_txt():
   fold_type = {}
   fold_value = {}
   f = open(UCD_config.UCD_src_dir + "/" + 'CaseFolding.txt')
   lines = f.readlines()
   for t in lines:
      if UCD_skip.match(t): continue  # skip comment and blank lines
      m = UCD_case_fold_regexp.match(t)
      if not m: raise Exception("Unknown case fold syntax: %s" % t)
      codepoint = int(m.group(1), 16)
      fold_t = m.group(2)
      fold_type[codepoint] = fold_t
      fold_val = m.group(3)
      if fold_t == 'T': 
         print "Skipping Turkic entry"
         continue  # skip Turkic
      if fold_t == 'F':
          fold_val = [int(x, 16) for x in fold_val.split(" ")]
      else:
          fold_val = int(fold_val, 16)
      if fold_value.has_key(codepoint): fold_value[codepoint].append(fold_val)
      else: fold_value[codepoint] = [fold_val]
   return (fold_type, fold_value)


def simple_CaseFolding_BitSets(fold_map):
   BitDiffSet = {}
   all_diff_bits = 0
   for k in fold_map.keys():
      v = fold_map[k]
      if not isinstance(v, int): continue # skip nonsimple case folds
      all_diff_bits |= v ^ k
   diff_bit_count = 0
   while all_diff_bits != 0:
      diff_bit_count += 1
      all_diff_bits >>= 1
   for bit in range(diff_bit_count):
      BitDiffSet[bit] = empty_uset()
   for k in fold_map.keys():
      s = singleton_uset(k)
      v = fold_map[k]
      if not isinstance(v, int): continue # skip nonsimple case folds
      diff_bits = v ^ k
      for b in range(diff_bit_count):
         if diff_bits & 1 == 1:  BitDiffSet[b] = uset_union(BitDiffSet[b], s)
         diff_bits >>= 1
   return BitDiffSet

def simple_CaseClosure_map(fold_map):
   cl_map = {}
   for k in fold_map.keys():
      folds = fold_map[k]
      for v in folds:
        if not isinstance(v, int): continue # skip nonsimple case folds
        if not cl_map.has_key(v): cl_map[v] = [k]
        else: cl_map[v].append(k)
        if not cl_map.has_key(k): cl_map[k] = [v]
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
            even_odd_offset_group = (abs(cp - cp0)/ abs(offset)) & 1
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
typedef unsigned codepoint_t;

struct FoldEntry {
    re::codepoint_t range_lo;
    int fold_offset;
    std::vector<re::interval_t> fold_pairs;
};


void caseInsensitiveInsertRange(re::CC * cc, const re::codepoint_t lo, const re::codepoint_t hi);

inline void caseInsensitiveInsert(re::CC * cc, const re::codepoint_t cp) {
    caseInsensitiveInsertRange(cc, cp, cp); 
}
"""

def genCaseFolding_txt_h():
   (ft, fv) = parse_CaseFolding_txt()
   cm = simple_CaseClosure_map(fv)
   f = cformat.open_header_file_for_write('CaseFolding_txt', 'casefold.py')
   cformat.write_imports(f, ["<vector>", '"re/re_cc.h"'])
   f.write(foldDeclarations)
   f.write(genFoldEntryData(cm))
   cformat.close_header_file(f)

if __name__ == "__main__":
   genCaseFolding_txt_h()



