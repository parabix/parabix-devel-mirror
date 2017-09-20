#
# unicode_set.py - representing and manipulating sets of Unicode
# characters, based on data from UCD - the Unicode Character Database
#
# Robert D. Cameron
# September 8, 2014
#
# Licensed under Open Software License 3.0.
import re, cformat
#
# Unicode Sparse Bitset Representation 
#
# The Unicode Sparse Bitset representation is based on 
# (a) Dividing the Unicode codepoint space into groups of 2^k codepoints called quads.
# (b) Specifying the quads using a run-length encoding, in which each run
#     is Empty (quads contain no members), Mixed (quads contain some members and
#     some nonmembers) or Full (all codepoints in each quad are members of the set). 
# (c) Explicitly listing all the quads of Mixed type.
#

Empty = 0
Full = -1
Mixed = 1

default_log2_quad_bits = 5

log2_quad_bits = default_log2_quad_bits
quad_bits = 1 << log2_quad_bits
mod_quad_bit_mask = quad_bits - 1
UnicodeQuadCount = 0x110000 / quad_bits #  2**log2_quad_bits codepoints per quad
FullQuadMask = (1<<(quad_bits)) - 1
run_bytes = 4


class UCset:
   def __init__(self):
      self.runs = []
      self.quads = []
      self.quad_count = 0
      
   # internal methods
   def append_run(self, runtype, runlength):
      if runlength == 0: return
      if self.runs == []:  self.runs = [(runtype, runlength)]
      else:
         (lastruntype, lastrunlength) = self.runs[-1]
         if lastruntype == runtype:  self.runs[-1] = (runtype, lastrunlength + runlength)
         else: self.runs.append((runtype, runlength))
      self.quad_count += runlength
   def append_mixed_run(self, n, quadlist):
      self.append_run(Mixed, n)
      self.quads += quadlist
   def append_quad(self, q):
      if q == 0:
        self.append_run(Empty, 1)
      elif q & FullQuadMask == FullQuadMask:
        self.append_run(Full, 1)
      else:
        self.append_run(Mixed, 1)
        self.quads.append(q)

   # printing
   def showC(self, indent = 4):
      hex_specifier =  "%%#0%ix" % (quad_bits/4 + 2)
      runtype = {-1:"Full", 0:"Empty", 1: "Mixed"}
      formatted_runs = ['{%s, %i}' % (runtype[r[0]], r[1]) for r in self.runs]
      formatted_quads = [hex_specifier % q for q in self.quads]
      setrep = (" " * indent) + "{{"
      setrep += cformat.multiline_fill(formatted_runs, ',', indent+2)
      setrep += '},\n'
      setrep += (" " * indent) + " {"
      setrep += cformat.multiline_fill(formatted_quads, ',', indent+2)
      setrep += '}}'
      return setrep

   def bytes(self):
       return (len(self.runs) * run_bytes) + (len(self.quads) * quad_bits/8)


#
# Set Operations
#
def empty_uset():
   e = UCset()
   e.runs = [(Empty, UnicodeQuadCount)]
   e.quads = []
   e.quad_count = UnicodeQuadCount
   return e

def singleton_uset(codepoint):
   e = UCset()
   quad_no = codepoint >> log2_quad_bits
   quad_val = 1 << (codepoint & mod_quad_bit_mask)
   if quad_no > 0: e.append_run(Empty, quad_no)
   e.append_run(Mixed, 1)
   e.quads = [quad_val]
   if quad_no < UnicodeQuadCount - 1: e.append_run(Empty, UnicodeQuadCount - (quad_no + 1))
   e.quad_count = UnicodeQuadCount
   return e

def range_uset(lo_codepoint, hi_codepoint):
   e = UCset()
   lo_quad_no = lo_codepoint >> log2_quad_bits   
   hi_quad_no = hi_codepoint >> log2_quad_bits
   lo_offset = lo_codepoint & mod_quad_bit_mask
   hi_offset = hi_codepoint & mod_quad_bit_mask
   if lo_quad_no > 0:  e.append_run(Empty, lo_quad_no)
   if lo_quad_no == hi_quad_no:
      quad = (FullQuadMask << lo_offset) & (FullQuadMask >> (quad_bits - 1 - hi_offset))
      e.append_quad(quad)
   else:
      e.append_quad((FullQuadMask << lo_offset) & FullQuadMask)
      e.append_run(Full, hi_quad_no - (lo_quad_no + 1))
      e.append_quad((FullQuadMask >> (quad_bits - 1 - hi_offset)) & FullQuadMask)
   if hi_quad_no < UnicodeQuadCount - 1: e.append_run(Empty, UnicodeQuadCount - (hi_quad_no + 1))
   return e


class Uset_Iterator:
    def __init__(self, uSet):
        self.uSet = uSet
        self.run_no = 0
        self.offset = 0
        self.quad_no = 0
    def at_end(self):
        return self.run_no == len(self.uSet.runs)
    def current_run(self):
        (this_run_type, this_run_length) = self.uSet.runs[self.run_no]
        return (this_run_type, this_run_length - self.offset)
    def get_quad(self):
        (this_run_type, this_run_length) = self.uSet.runs[self.run_no]
        if this_run_type == Empty: return 0
        elif this_run_type == Full: return FullQuadMask
        else: return self.uSet.quads[self.quad_no]
    def advance(self, n):
        while n > 0:
           (this_run_type, this_run_length) = self.uSet.runs[self.run_no]
           remain = this_run_length - self.offset
           if remain > n:
               self.offset += n
               if this_run_type == Mixed: self.quad_no += n
               n = 0
           elif remain == n:
               self.run_no += 1
               self.offset = 0
               if this_run_type == Mixed: self.quad_no += n
               n = 0
           else:
               self.run_no += 1
               self.offset = 0
               if this_run_type == Mixed: self.quad_no += remain
               n -= remain


def uset_member(s, codepoint):
   quad_no = codepoint / quad_bits
   quad_val = 1 << (codepoint & mod_quad_bit_mask)
   it = Uset_Iterator(s)    
   it.advance(quad_no)
   return (it.get_quad() & quad_val) != 0

def uset_popcount(s):
    popcount = 0
    it = Uset_Iterator(s)
    while not it.at_end():
        (runtype, n) = it.current_run()
        if runtype == Empty:
            it.advance(n)
        elif runtype == Full:
            popcount += n * quad_bits
            it.advance(n)
        else:
            popcount += popcount_quad(it.get_quad())
            it.advance(1)
    return popcount

def popcount_quad(q):
    c = 0
    while q != 0:
        q = q & (q - 1) # clear low bit
        c += 1
    return c

def uset_complement (s):
   assert s.quad_count == UnicodeQuadCount
   iset = UCset()
   it = Uset_Iterator(s)
   while not it.at_end():
      (runtype, n) = it.current_run()
      if runtype == Empty:
         iset.append_run(Full, n)
         it.advance(n)
      elif runtype == Full:
         iset.append_run(Empty, n)
         it.advance(n)
      else: 
         for i in range(n):
            iset.append_quad(FullQuadMask ^ it.get_quad())
            it.advance(1)
   return iset

def uset_intersection (s1, s2):
   assert s1.quad_count == UnicodeQuadCount
   assert s2.quad_count == UnicodeQuadCount
   iset = UCset()
   i1 = Uset_Iterator(s1)
   i2 = Uset_Iterator(s2)
   while not i1.at_end():
      (s1_type, s1_length) = i1.current_run()
      (s2_type, s2_length) = i2.current_run()
      n = min(s1_length, s2_length)
      if s1_type == Empty or s2_type == Empty:
         iset.append_run(Empty, n)
         i1.advance(n)
         i2.advance(n)
      elif s1_type == Full and s2_type == Full:
         iset.append_run(Full, n)
         i1.advance(n)
         i2.advance(n)
      elif s1_type == Full:
         for i in range(n):
            iset.append_quad(i2.get_quad())
            i2.advance(1)
         i1.advance(n)
      elif s2_type == Full:
         for i in range(n):
            iset.append_quad(i1.get_quad())
            i1.advance(1)
         i2.advance(n)
      else: # both s1 and s2 have mixed blocks; form block-by-block intersection 
         for i in range(n):
            iset.append_quad(i1.get_quad() & i2.get_quad())
            i1.advance(1)
            i2.advance(1)
   return iset

def uset_union (s1, s2):
   assert s1.quad_count == UnicodeQuadCount
   assert s2.quad_count == UnicodeQuadCount
   iset = UCset()
   i1 = Uset_Iterator(s1)
   i2 = Uset_Iterator(s2)
   while not i1.at_end():
      (s1_type, s1_length) = i1.current_run()
      (s2_type, s2_length) = i2.current_run()
      n = min(s1_length, s2_length)
      if s1_type == Empty and s2_type == Empty:
         iset.append_run(Empty, n)
         i1.advance(n)
         i2.advance(n)
      elif s1_type == Full or s2_type == Full:
         iset.append_run(Full, n)
         i1.advance(n)
         i2.advance(n)
      elif s1_type == Empty:
         for i in range(n):
            iset.append_quad(i2.get_quad())
            i2.advance(1)
         i1.advance(n)
      elif s2_type == Empty:
         for i in range(n):
            iset.append_quad(i1.get_quad())
            i1.advance(1)
         i2.advance(n)
      else: # both s1 and s2 have mixed blocks; form block-by-block union 
         for i in range(n):
            iset.append_quad(i1.get_quad() | i2.get_quad())
            i1.advance(1)
            i2.advance(1)
   return iset

def uset_difference (s1, s2):
   assert s1.quad_count == UnicodeQuadCount
   assert s2.quad_count == UnicodeQuadCount
   iset = UCset()
   i1 = Uset_Iterator(s1)
   i2 = Uset_Iterator(s2)
   while not i1.at_end():
      (s1_type, s1_length) = i1.current_run()
      (s2_type, s2_length) = i2.current_run()
      n = min(s1_length, s2_length)
      if s1_type == Empty or s2_type == Full:
         iset.append_run(Empty, n)
         i1.advance(n)
         i2.advance(n)
      elif s1_type == Full and s2_type == Empty:
         iset.append_run(Full, n)
         i1.advance(n)
         i2.advance(n)
      elif s1_type == Full:
         for i in range(n):
            iset.append_quad(FullQuadMask ^ i2.get_quad())
            i2.advance(1)
         i1.advance(n)
      elif s2_type == Empty:
         for i in range(n):
            iset.append_quad(i1.get_quad())
            i1.advance(1)
         i2.advance(n)
      else: # both s1 and s2 have mixed blocks; form block-by-block union 
         for i in range(n):
            iset.append_quad(i1.get_quad() &~ i2.get_quad())
            i1.advance(1)
            i2.advance(1)
   return iset

def uset_symmetric_difference (s1, s2):
   assert s1.quad_count == UnicodeQuadCount
   assert s2.quad_count == UnicodeQuadCount
   iset = UCset()
   i1 = Uset_Iterator(s1)
   i2 = Uset_Iterator(s2)
   while not i1.at_end():
      (s1_type, s1_length) = i1.current_run()
      (s2_type, s2_length) = i2.current_run()
      n = min(s1_length, s2_length)
      if s1_type == Empty and s2_type == Full or s1_type == Full and s2_type == Empty:
         iset.append_run(Full, n)
         i1.advance(n)
         i2.advance(n)
      elif s1_type == Full and s2_type == Full or s1_type == Empty and s2_type == Empty:
         iset.append_run(Empty, n)
         i1.advance(n)
         i2.advance(n)
      elif s1_type == Empty:
         for i in range(n):
            iset.append_quad(i2.get_quad())
            i2.advance(1)
         i1.advance(n)
      elif s2_type == Empty:
         for i in range(n):
            iset.append_quad(i1.get_quad())
            i1.advance(1)
         i2.advance(n)
      elif s1_type == Full:
         for i in range(n):
            iset.append_quad(FullQuadMask ^ i2.get_quad())
            i2.advance(1)
         i1.advance(n)
      elif s2_type == Full:
         for i in range(n):
            iset.append_quad(FullQuadMask ^ i1.get_quad())
            i1.advance(1)
         i2.advance(n)
      else: # both s1 and s2 have mixed blocks; form block-by-block union 
         for i in range(n):
            iset.append_quad(i1.get_quad() ^ i2.get_quad())
            i1.advance(1)
            i2.advance(1)
   return iset

def uset_to_range_list(s):
    i = Uset_Iterator(s)
    rl = []
    open_range = False
    range_first = 0
    pos = 0
    while not i.at_end():
        (q_type, q_length) = i.current_run()
        if q_type == Empty:
            if open_range: 
                rl.append((range_first, pos - 1))
                open_range = False
            pos += q_length * quad_bits
            i.advance(q_length)
        elif q_type == Full:
            if not open_range:
                range_first = pos
                open_range = True
            pos += q_length * quad_bits
            i.advance(q_length)
        else:  # mixed quad
            q = i.get_quad()
            qpos = pos
            for qpos in range(pos, pos+quad_bits):
                if q & 1 == 0:
                    if open_range:
                        rl.append((range_first, qpos - 1))
                        open_range = False
                else:
                    if not open_range:
                        range_first = qpos
                        open_range = True
                q >>= 1
                qpos += 1
            pos += quad_bits
            i.advance(1)
    if open_range:
        rl.append((range_first, 0x10FFFF))
    return rl

UCD_point_regexp = re.compile("^([0-9A-F]{4,6})\s+;")
UCD_range_regexp = re.compile("^([0-9A-F]{4,6})[.][.]([0-9A-F]{4,6})\s+;")

def parse_UCD_set(lines):
    pset = empty_set()
    for t in lines:
        m = UCD_point_regexp.match(t)
        if m:
            point = m.group(1)
            pval = int(point, 16)
            pset = union(pset, singleton_set(pval))
        m = UCD_range_regexp.match(t)
        if m:
            point1 = m.group(1)
            point2 = m.group(2)
            pval1 = int(point1, 16)
            pval2 = int(point2, 16)
            pset = union(pset, make_range_set(pval1, pval2))
    return pset

def parse_UCD_file(fname, vname):
    f = open(fname)
    lines = f.readlines()
    f.close()
    s = parse_UCD_set(lines)
    print s.showC(vname)


