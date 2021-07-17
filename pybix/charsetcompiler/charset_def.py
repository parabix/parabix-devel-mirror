# -*- coding: utf-8 -*-
#
#  Character sets are defined as lists of items that
#  are either individual characters or ranges of contiguous
#  characters.
#
#  Aug. 1, 2014 - convert to canonical list of ranges 
#
class CharSetDef:
    """Definitions of character sets.   Examples:
    CharSetDef('alpha_', ['a-z', 'A-Z', '_'])
    CharDef('semicolon', ';') (equiv. to CharSetDef('semicolon', [';']))
    """
    def __init__(self, name, items, invert = False):
        self.name = name
        self.items = canonical_range_list(items)
        self.complemented = invert
    def show(self): 
	if self.complemented:
	    return "CharSetDef(%s, %s, True)" % (self.name, self.items)
	else: return "CharSetDef(%s, %s)" % (self.name, self.items)

        
class CharDef(CharSetDef):
    def __init__(self, name, char, invert = False):
        self.name = name
        self.items = [(ord(char), ord(char))]
        self.complemented = invert
    def show(self): 
	if self.complemented:
	    return "CharDef(%s, '\\%X', True)" % (self.name, ord(self.items[0]))
	else: return "CharDef(%s, '\\%X')" % (self.name, ord(self.items[0]))

class CanonicalCharSetDef(CharSetDef):
    def __init__(self, name, items, invert = False):
        self.name = name
        self.items = items
        self.complemented = invert


def canonical_range_list(items):
    if items == []: return []
    items.sort()
    ranges = [(ord(item[0]), ord(item[-1])) for item in items]
    merged = []
    (lo1, hi1) = ranges[0]
    for r in ranges:
      (lo, hi) = r
      if lo > hi: raise Exception("Bad range (%s, %s)" % (lo, hi))
      if lo <= hi1 + 1: 
        if hi1 < hi: hi1 = hi
        else: pass
      else: 
        merged.append((lo1, hi1))
        (lo1, hi1) = (lo, hi)
    merged.append((lo1, hi1))
    return merged
  

