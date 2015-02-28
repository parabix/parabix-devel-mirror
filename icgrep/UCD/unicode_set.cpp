//
// unicode_set.cpp - representing and manipulating sets of Unicode
// characters, based on data from UCD - the Unicode Character Database
//
// Robert D. Cameron
// September 18, 2014
//
// Licensed under Open Software License 3.0.
//
// Unicode Sparse Bitset Representation 
//
// The Unicode Sparse Bitset representation is based on 
// (a) Dividing the Unicode codepoint space into groups of 2^k codepoints called quads.
// (b) Specifying the quads using a run-length encoding, in which each run
//     is Empty (quads contain no members), Mixed (quads contain some members and
//     some nonmembers) or Full (all codepoints in each quad are members of the set). 
// (c) Explicitly listing all the quads of Mixed type.
//

#include "unicode_set.h"
#include "assert.h"
#include <string>
#include <iostream>

class Uset_Iterator {
public:
    Uset_Iterator(UnicodeSet s) : uSet(s), run_no(0), offset(0), quad_no(0) {};
    bool at_end();
    RunStructure current_run();
    bitquad_t get_quad();
    void advance(int n);
private:
    UnicodeSet uSet;
    int run_no;
    int offset;
    int quad_no;
};

bool Uset_Iterator::at_end() {
  return run_no == uSet.runs.size();
}

RunStructure Uset_Iterator::current_run() {
  RunStructure this_run = uSet.runs[run_no];
  return RunStructure(this_run.run_type, this_run.run_length - offset);
}

bitquad_t Uset_Iterator::get_quad() {
  RunStructure this_run = uSet.runs[run_no];
  if (this_run.run_type == Empty) return 0;
  else if (this_run.run_type == Full) return FullQuadMask;
  else return uSet.quads[quad_no];
}

void Uset_Iterator::advance(int n) {
  while (n > 0) {
    RunStructure this_run = uSet.runs[run_no];
    int remain = this_run.run_length - offset;
    if (remain > n) {
      offset += n;
      if (this_run.run_type == Mixed) quad_no += n;
      n = 0;
    }
    else if (remain == n) {
      run_no += 1;
      offset = 0;
      if (this_run.run_type == Mixed) quad_no += n;
      n = 0;
    }
    else {
      run_no += 1;
      offset = 0;
      if (this_run.run_type == Mixed) quad_no += remain;
      n -= remain;
    }
  }
}

void UnicodeSet::append_run(run_type_t run_type, int run_length) {
    if (run_length == 0) return;
    if (runs.size() == 0) runs.push_back(RunStructure(run_type, run_length));
    else {
      RunStructure last_run = runs[runs.size()-1];
      if (last_run.run_type == run_type) runs[runs.size()-1].run_length += run_length;
      else runs.push_back(RunStructure(run_type, run_length));
    }
    quad_count += run_length;
}

void UnicodeSet::append_quad(bitquad_t q) {
    if (q == 0) {
        append_run(Empty, 1);
    }
    else if (q == FullQuadMask) {
        append_run(Full, 1);
    }
    else {
        quads.push_back(q);
        append_run(Mixed, 1);
    }
}

void Dump_uset(UnicodeSet s) {
    UnicodeSet iset;
    Uset_Iterator it(s);
    while (!it.at_end()) {
      RunStructure this_run = it.current_run();
      if (this_run.run_type == Empty) {
	std::cout << "Empty(" << this_run.run_length << ")\n";
	it.advance(this_run.run_length);
      }
      else if (this_run.run_type == Full) {
	std::cout << "Full(" << this_run.run_length << ")\n";
	it.advance(this_run.run_length);
      }
      else {
	for (int i = 0; i < this_run.run_length; i++) {
	  std::cout << "Mixed(" << std::hex << it.get_quad() << std::dec << ")\n";
          it.advance(1);
	}
      }
    }
}



UnicodeSet empty_uset() {
    UnicodeSet iset;
    iset.runs.push_back(RunStructure(Empty, UnicodeQuadCount));
    iset.quad_count = UnicodeQuadCount;
    return iset;
}

// singleton set constructor
UnicodeSet singleton_uset(int codepoint) {
    UnicodeSet iset;
    int quad_no = codepoint / quad_bits;
    bitquad_t quad_val = 1 << (codepoint & mod_quad_bit_mask);
    if (quad_no > 0) iset.append_run(Empty, quad_no);
    iset.append_run(Mixed, 1);
    iset.quads.push_back(quad_val);
    if (quad_no < UnicodeQuadCount - 1) iset.append_run(Empty, UnicodeQuadCount - (quad_no + 1));
    iset.quad_count =  UnicodeQuadCount;
    return iset;
}

// range set constructor
UnicodeSet range_uset(int lo_codepoint, int hi_codepoint) {
    UnicodeSet iset;
    int lo_quad_no = lo_codepoint / quad_bits;
    int hi_quad_no = hi_codepoint / quad_bits;
    int lo_offset = lo_codepoint & mod_quad_bit_mask;
    int hi_offset = hi_codepoint & mod_quad_bit_mask;
    if (lo_quad_no > 0) iset.append_run(Empty, lo_quad_no);
    if (lo_quad_no == hi_quad_no) {
        bitquad_t quad = (FullQuadMask << lo_offset) & (FullQuadMask >> (quad_bits - 1 - hi_offset));
        iset.append_quad(quad);
    }
    else {
        iset.append_quad((FullQuadMask << lo_offset) & FullQuadMask);
        iset.append_run(Full, hi_quad_no - (lo_quad_no + 1));
        iset.append_quad((FullQuadMask >> (quad_bits - 1 - hi_offset)) & FullQuadMask);
    }
    if (hi_quad_no < UnicodeQuadCount - 1) iset.append_run(Empty, UnicodeQuadCount - (hi_quad_no + 1));
    return iset;
}

UnicodeSet uset_complement (UnicodeSet s) {
    assert(s.quad_count == UnicodeQuadCount);
    UnicodeSet iset;
    Uset_Iterator it(s);
    while (!it.at_end()) {
      RunStructure this_run = it.current_run();
      if (this_run.run_type == Empty) {
        iset.append_run(Full, this_run.run_length);
	it.advance(this_run.run_length);
      }
      else if (this_run.run_type == Full) {
        iset.append_run(Empty, this_run.run_length);
	it.advance(this_run.run_length);
      }
      else {
	for (int i = 0; i < this_run.run_length; i++) {
	  iset.append_quad(FullQuadMask ^ it.get_quad());
          it.advance(1);
	}
      }
    }
    return iset;
}

UnicodeSet uset_intersection (UnicodeSet s1, UnicodeSet s2) {
    assert(s1.quad_count == UnicodeQuadCount);
    assert(s2.quad_count == UnicodeQuadCount);
    UnicodeSet iset;
    Uset_Iterator i1(s1);
    Uset_Iterator i2(s2);
    while (!i1.at_end()) {
      RunStructure run1 = i1.current_run();
      RunStructure run2 = i2.current_run();
      int n = std::min(run1.run_length, run2.run_length);
      if ((run1.run_type == Empty) || (run2.run_type == Empty)) {
        iset.append_run(Empty, n);
	i1.advance(n);
	i2.advance(n);
      }
      else if ((run1.run_type == Full) && (run2.run_type == Full)) {
        iset.append_run(Full, n);
	i1.advance(n);
	i2.advance(n);
      }
      else if (run1.run_type == Full) {
	for (int i = 0; i < n; i++) {
	  iset.append_quad(i2.get_quad());
          i2.advance(1);
	}
	i1.advance(n);	
      }
      else if (run2.run_type == Full) {
	for (int i = 0; i < n; i++) {
	  iset.append_quad(i1.get_quad());
          i1.advance(1);
	}
	i2.advance(n);	
      }
      else {
	for (int i = 0; i < n; i++) {
	  iset.append_quad(i1.get_quad() & i2.get_quad());
          i1.advance(1);
          i2.advance(1);
	}
      }
    }
    return iset;
}

UnicodeSet uset_union (UnicodeSet s1, UnicodeSet s2) {
    assert(s1.quad_count == UnicodeQuadCount);
    assert(s2.quad_count == UnicodeQuadCount);
    UnicodeSet iset;
    Uset_Iterator i1(s1);
    Uset_Iterator i2(s2);
    while (!i1.at_end()) {
      RunStructure run1 = i1.current_run();
      RunStructure run2 = i2.current_run();
      int n = std::min(run1.run_length, run2.run_length);
      if ((run1.run_type == Empty) && (run2.run_type == Empty)) {
        iset.append_run(Empty, n);
	i1.advance(n);
	i2.advance(n);
      }
      else if ((run1.run_type == Full) || (run2.run_type == Full)) {
        iset.append_run(Full, n);
	i1.advance(n);
	i2.advance(n);
      }
      else if (run1.run_type == Empty) {
	for (int i = 0; i < n; i++) {
	  iset.append_quad(i2.get_quad());
          i2.advance(1);
	}
	i1.advance(n);	
      }
      else if (run2.run_type == Empty) {
	for (int i = 0; i < n; i++) {
	  iset.append_quad(i1.get_quad());
          i1.advance(1);
	}
	i2.advance(n);	
      }
      else {
	for (int i = 0; i < n; i++) {
	  iset.append_quad(i1.get_quad() | i2.get_quad());
          i1.advance(1);
          i2.advance(1);
	}
      }
    }
    return iset;
}

UnicodeSet uset_difference (UnicodeSet s1, UnicodeSet s2) {
    assert(s1.quad_count == UnicodeQuadCount);
    assert(s2.quad_count == UnicodeQuadCount);
    UnicodeSet iset;
    Uset_Iterator i1(s1);
    Uset_Iterator i2(s2);
    while (!i1.at_end()) {
      RunStructure run1 = i1.current_run();
      RunStructure run2 = i2.current_run();
      int n = std::min(run1.run_length, run2.run_length);
      if ((run1.run_type == Empty) || (run2.run_type == Full)) {
        iset.append_run(Empty, n);
	i1.advance(n);
	i2.advance(n);
      }
      else if ((run1.run_type == Full) && (run2.run_type == Empty)) {
        iset.append_run(Full, n);
	i1.advance(n);
	i2.advance(n);
      }
      else if (run1.run_type == Full) {
	for (int i = 0; i < n; i++) {
	  iset.append_quad(FullQuadMask ^ i2.get_quad());
          i2.advance(1);
	}
	i1.advance(n);	
      }
      else if (run2.run_type == Empty) {
	for (int i = 0; i < n; i++) {
	  iset.append_quad(i1.get_quad());
          i1.advance(1);
	}
	i2.advance(n);	
      }
      else {
	for (int i = 0; i < n; i++) {
	  iset.append_quad(i1.get_quad() & ~i2.get_quad());
          i1.advance(1);
          i2.advance(1);
	}
      }
    }
    return iset;
}

UnicodeSet uset_symmetric_difference (UnicodeSet s1, UnicodeSet s2) {
    assert(s1.quad_count == UnicodeQuadCount);
    assert(s2.quad_count == UnicodeQuadCount);
    UnicodeSet iset;
    Uset_Iterator i1(s1);
    Uset_Iterator i2(s2);
    while (!i1.at_end()) {
      RunStructure run1 = i1.current_run();
      RunStructure run2 = i2.current_run();
      int n = std::min(run1.run_length, run2.run_length);
      if (((run1.run_type == Empty) && (run2.run_type == Full)) || ((run1.run_type == Full) && (run2.run_type == Empty))) {
        iset.append_run(Full, n);
	i1.advance(n);
	i2.advance(n);
      }
      else if (((run1.run_type == Full) && (run2.run_type == Full)) || ((run1.run_type == Empty) && (run2.run_type == Empty))) {
        iset.append_run(Empty, n);
	i1.advance(n);
	i2.advance(n);
      }
      else if (run1.run_type == Empty) {
	for (int i = 0; i < n; i++) {
	  iset.append_quad(i2.get_quad());
          i2.advance(1);
	}
	i1.advance(n);	
      }
      else if (run2.run_type == Empty) {
	for (int i = 0; i < n; i++) {
	  iset.append_quad(i1.get_quad());
          i1.advance(1);
	}
	i2.advance(n);	
      }
      else if (run1.run_type == Full) {
	for (int i = 0; i < n; i++) {
	  iset.append_quad(FullQuadMask ^ i2.get_quad());
          i2.advance(1);
	}
	i1.advance(n);	
      }
      else if (run2.run_type == Empty) {
	for (int i = 0; i < n; i++) {
	  iset.append_quad(FullQuadMask ^ i1.get_quad());
          i1.advance(1);
	}
	i2.advance(n);	
      }
      else {
	for (int i = 0; i < n; i++) {
	  iset.append_quad(i1.get_quad() ^ i2.get_quad());
          i1.advance(1);
          i2.advance(1);
	}
      }
    }
    return iset;
}

bool uset_member(UnicodeSet s, int codepoint){
    int quad_no = codepoint / quad_bits;
    bitquad_t quad_val = 1 << (codepoint & mod_quad_bit_mask);
    Uset_Iterator it(s);
    it.advance(quad_no);
    return (it.get_quad() & quad_val) != 0;
}