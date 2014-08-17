#ifndef BITBLOCK_ITERATOR_H_
#define BITBLOCK_ITERATOR_H_

#include <iterator>
#include <iostream>
#include "bitblock.hpp"

/*=============================================================================
  bitblock_iterator.hpp

  Created on: Sept 2011, revised May 2013
  Authors: Ken Herdy and Rob Cameron

  Description:

	Scanner classes implement the low level methods to scan through 
	the 'scanwords' of a 'bitblock'. Scanner classes are templated 
	on bitblock type and scanword type. 

    The 'scanfield_t' template parameter is equivalent to a 'scanword'
    in the conceptual model of scanning.

*/

//
// The following implementation of BitBlockScanner is optimized
// to eliminate branch mispredictions during scanning.  Only the
// essential methods are included.   RDC May 2013.
// 
// Usage:
//   (1) declare:  BitBlockScanner myscanner;
//   (2) initialize:  myscanner.init(&mybitblock);
//   (3) iterate:  while (myscanner.has_next()) { pos = myscanner.scan_to_next();  ...}
//

//
// Could also use FW 32.
#define FW 8
#define _FW _8
template <class bitblock_t, class scanblock_t>
class BitBlockScanner {
public:
	BitBlockScanner() {}

	IDISA_ALWAYS_INLINE void init(const BitBlock *s) {
        remaining._bitblock = *s;
        mask = hsimd<FW>::signmask(simd_not(simd<FW>::eq(simd<1>::constant<0>(), remaining._bitblock)));
    }

	IDISA_ALWAYS_INLINE int has_next() {
        return mask != 0;
    }

	IDISA_ALWAYS_INLINE int scan_to_next() {
        int item_pos = scan_forward_zeroes(mask);
        uint32_t scan_item = remaining._FW[item_pos];
        int bitpos = scan_forward_zeroes(scan_item);
        scan_item = scan_item & (scan_item - 1);
        remaining._FW[item_pos] = scan_item;
        // We could recalculate the mask, but updating it is faster.
        // Note that this update code compiles to a SETcc instruction.
        mask = mask & (mask - ((scan_item == 0) ? 1 : 0));
        int pos = item_pos * FW + bitpos;
        return pos;
    }

    // It slows things down to store the position.
	//IDISA_ALWAYS_INLINE int32_t get_pos() { return pos;}
private:
	union {bitblock_t _bitblock;
           uint8_t _8[sizeof(bitblock_t)];
           uint32_t _32[sizeof(bitblock_t)/sizeof(uint32_t)];} remaining;
    scanblock_t mask;
};
#undef _FW
#undef FW


/*
    A scanner to successively generate the positions marked by 1 bits
    in a bit stream segment of bitblock_count bit blocks. 

    Usage:

    (1) declaration, e.g.
        BitStreamScanner<BitBlock, uint32_t, u8int_t, 8> s;
        (There is considerable flexibility for using different 
         scanblock sizes for the main mask, as well as the
         scan field sizes within each block.)

    (2) initialization/reinitialization
        s.init();

    (3) load the blocks;
    for (i = 0; i < n;  i++) {mybitblock = ...; s.load_block(mybitblock, i);}

    (4) Iterative scan loop (generates each position exactly once.)
        while (s.has_next()) { pos = s.scan_to_next(); ...   };
*/

template <class bitblock_t, class scanblock_t, class scanfield_t,
          int bitblock_count = sizeof(scanblock_t) * 8 * sizeof(scanfield_t) / sizeof(bitblock_t)>
class BitStreamScanner {
public:

    // static constants
    static const int max_bitblock_count = bitblock_count;

    /* Make sure that the number of bits in the mask at least equals the number of scanblocks. */
    /* Requires flag -std=gnu++0x  */
    static_assert(sizeof(scanblock_t) * 8 >= bitblock_count * sizeof(bitblock_t)/sizeof(scanfield_t),
    "Too many bitblocks for a single scanword mask");
        
	BitStreamScanner() {}

	IDISA_ALWAYS_INLINE void init() { mask = 0;}

	IDISA_ALWAYS_INLINE void load_block(BitBlock b, int i) {
        remaining._bitblock[i] = b;
        BitBlock mask_i = simd_not(simd<sizeof(scanfield_t)*8>::eq(simd<1>::constant<0>(), b));
        mask += ((scanblock_t) hsimd<sizeof(scanfield_t)*8>::signmask(mask_i)) << ((scanblock_t) i * (sizeof(bitblock_t)/sizeof(scanfield_t)));
    }

	IDISA_ALWAYS_INLINE bool has_next() {
        return mask != 0;
    }

	IDISA_ALWAYS_INLINE int scan_to_next() {
        int item_pos = scan_forward_zeroes(mask);
        scanfield_t scan_item = remaining._scanfield[item_pos];
        int bitpos = scan_forward_zeroes(scan_item);
        scan_item = scan_item & (scan_item - 1);
        remaining._scanfield[item_pos] = scan_item;
        mask = mask & (mask - ((scan_item == 0) ? 1 : 0));
        int pos = item_pos * sizeof(scanfield_t) * 8 + bitpos;
        return pos;
    }

	IDISA_ALWAYS_INLINE int get_final_pos() {
        int item_pos = sizeof(scanblock_t) * 8 - scan_backward_zeroes((scanblock_t) mask) - 1;
        scanfield_t scan_item = remaining._scanfield[item_pos];
        int bitpos = sizeof(scanblock_t)  * 8 - scan_backward_zeroes((scanblock_t) scan_item) - 1;
        int pos = item_pos * sizeof(scanfield_t) * 8 + bitpos;
        return pos;
    }

	IDISA_ALWAYS_INLINE void clear_from(int pos) {
        int item_pos = pos / (sizeof(scanfield_t) * 8);
        int bitpos = pos % (sizeof(scanfield_t) * 8);
        remaining._scanfield[item_pos] &= ((((scanfield_t) 1) << bitpos) - 1);
        item_pos += remaining._scanfield[item_pos] == 0 ? 0 : 1;
        mask = mask & (((scanblock_t) 1) << item_pos) - 1;
        for (int i = item_pos; i < bitblock_count * 2; i++) remaining._scanfield[i] = 0;
    }
    
	IDISA_ALWAYS_INLINE int count() {
		if (mask == 0) return 0;
		int ct = 0;
#define PARALLEL_COUNT
#ifdef PARALLEL_COUNT
    BitBlock sum8 = simd<1>::constant<0>();
    for (int i = 0; i < bitblock_count/2; i++) {
        BitBlock ct4 = simd<4>::add(simd<4>::popcount(remaining._bitblock[2*i]), simd<4>::popcount(remaining._bitblock[2*i+1]));
        sum8 = simd<8>::add(sum8, simd<8>::add_hl(ct4));
    }
    if ((bitblock_count & 1) != 0) {  // Should be compiled out if bitblock_count is even.
        sum8 = simd<8>::add(sum8, simd<8>::popcount(remaining._bitblock[bitblock_count-1]));
    }
    ct = mvmd<32>::extract<0>(simd<128>::add_hl(simd<64>::add_hl(simd<32>::add_hl(simd<16>::add_hl(sum8)))));
#endif
#ifndef PARALLEL_COUNT
    for (int i = 0; i < bitblock_count; i++) {
        ct += bitblock::popcount(remaining._bitblock[i]);
    }
#endif
        return ct;
	}
    
private:
	union {bitblock_t _bitblock[bitblock_count];
           scanfield_t _scanfield[bitblock_count * sizeof(bitblock_t)/sizeof(scanfield_t)];} remaining;
    scanblock_t mask;
};

/*=============================================================================

Classes Descriptions:

    * BitBlock Scanners

    Scanner - Scanner base class.
    ForwardScanner - Scans a BitBlock of ScanWords in the forward direction.
    ReverseScanner - Scans a bitblock of ScanWords in the reverse direction.

    Deprecated (BitBlock iterators only)

    * BitBlock Iterators

    BitBlock iterator classes provide Standard Template Library (STL)
    Input iterator 'like' interface implementations as a programmer
    convenience and allow iterator classes to be used
    with STL Algorithms.

    Forward iterators can only step forward (++) marker-bit by marker-bit.
    Reverse iterators can only step backwards (--) marker-bit by marker-bit.


    ForwardIterator template class wraps ForwardScanner.

    - Implements STL Input iterator interface.
    - Reads elements only once.
    - '->' - Not implemented.

    ReverseIterator template class wraps ReverseScanner.

	- An STL 'like' iterator that implements prefix and postfix decrement operators.
    Under STL, a reverse iterator implementation necessitates the implementation of
    the STL bidirectional interface and hence scanning in both directions.
    This functionality is not yet required and hence not implemented.
	- Reads elements only once. 
    - '->' - Not implemented.

	BitBlockForwardIterator - ForwardIterator derived specialization of ForwardIterator<BitBlock, Scanword>
	BitBlockReverseIterator - ForwardIterator derived specialization of ReverseIterator<BitBlock, Scanword>

	* BitStreamIterators

    BitStreamIterator class wraps ForwardScanner.
	- Scans an array (stream) of scanwords in contiguous memory. 
	- Implements STL Input iterator interface.
	- Reads elements only once.  	
    - '->' - Not implemented.
	- May be more appropriately named ForwardBitStreamIterator.

=============================================================================*/

//
// Scanner Classes
//
#define has_bit(x) (x != 0)
#define EOS -1

template <class bitblock_t, class scanblock_t>
class Scanner {

protected:
	Scanner(): strm(NULL), pos(EOS), blk(-1), scan_blk(-1) {}
	Scanner(const bitblock_t * s, uint32_t start_pos, uint32_t start_blk, scanblock_t start_scan_blk): strm(s), pos(start_pos), blk(start_blk), scan_blk(start_scan_blk) {}

	const bitblock_t * strm;
	int32_t pos;
	int32_t blk;
	scanblock_t scan_blk;
};

template <class bitblock_t, class scanblock_t>
class ForwardScanner: public Scanner<bitblock_t, scanblock_t> {

public:

	ForwardScanner(){}
	ForwardScanner(const bitblock_t * s) {
		init(s);
	}

	IDISA_ALWAYS_INLINE void init(const bitblock_t * s) {
		this->strm = s;
		this->pos = 0;
		this->blk = 0;
		this->scan_blk = *(scanblock_t *)s;
	}

	IDISA_ALWAYS_INLINE int32_t scan_to_next() {
		while (this->blk < BLOCK_COUNT){
			if(has_bit(this->scan_blk)){
				this->pos = scan_forward_zeroes(this->scan_blk) + (this->blk * (sizeof(scanblock_t)*8));
				this->scan_blk = this->scan_blk & (this->scan_blk-1);  // clear rightmost bit

				return (this->pos);
			}

			this->blk++;
			this->scan_blk = *((scanblock_t *)this->strm + this->blk);
		};

		this->pos = EOS;
		return (this->pos);
	}

	/* Set or reset the iterator to position new_pos. */
	IDISA_ALWAYS_INLINE void move_to(uint32_t new_pos) {		
		const scanblock_t one_bit = 1;
		this->blk = new_pos / (sizeof(scanblock_t)*8);
		this->pos = new_pos % (sizeof(scanblock_t)*8);
		this->scan_blk = ((scanblock_t *)this->strm)[this->blk];
		// clear bit at pos and all positions to the right.
		scanblock_t marker = one_bit << this->pos;
		this->scan_blk = this->scan_blk &~((marker-1)|marker); 
	}

	IDISA_ALWAYS_INLINE bool is_done() const {return (EOS==this->pos);}
	IDISA_ALWAYS_INLINE void set_strm(const bitblock_t * strm) {this->strm = strm;}
	IDISA_ALWAYS_INLINE const bitblock_t * get_strm() const {return this->strm;}
	IDISA_ALWAYS_INLINE int32_t get_pos() const {return this->pos;}
	IDISA_ALWAYS_INLINE void set_pos(int32_t pos) {(this->pos = pos);}
	static const int32_t BLOCK_COUNT = sizeof(bitblock_t)/sizeof(scanblock_t);

};

class BitBlockForwardScanner: public ForwardScanner<BitBlock, ScanWord> {
public:
	BitBlockForwardScanner(){}
	BitBlockForwardScanner(BitBlock * s): ForwardScanner<BitBlock, ScanWord>(s){}
};

template <class bitblock_t, class scanblock_t>
class ReverseScanner: public Scanner<bitblock_t, scanblock_t> {

public:
	ReverseScanner(){}
	ReverseScanner(const bitblock_t * s) {
		init(s);
	}
	IDISA_ALWAYS_INLINE void init(const bitblock_t * s) {
		this->strm = s;
		this->pos = 0;
		this->blk = BLOCK_COUNT-1;
		this->scan_blk = *((scanblock_t *)s + (BLOCK_COUNT-1));
	}

	IDISA_ALWAYS_INLINE int32_t scan_to_next() {
		const scanblock_t one_bit = 1;  /* ensure enough bits for shift: one_bit << this->pos */
		while (this->blk > -1){
			if(has_bit(this->scan_blk)){
				this->pos = (sizeof(scanblock_t)*8 - scan_backward_zeroes(this->scan_blk) -1) + ( (this->blk) * sizeof(scanblock_t)*8 );
				this->scan_blk = this->scan_blk ^ (one_bit << this->pos); // clear leftmost bit
				return (this->pos);
			}

			this->blk--;
			this->scan_blk = *((scanblock_t *)this->strm + this->blk);
		};

		this->pos = EOS;
		return (this->pos);
	}

	/* Set or reset the iterator to position new_pos. */
	IDISA_ALWAYS_INLINE void move_to(uint32_t new_pos) {
		const scanblock_t one_bit = 1;
		this->blk = (new_pos / (sizeof(scanblock_t)*8));
		this->pos = new_pos % (sizeof(scanblock_t)*8);
		this->scan_blk = ((scanblock_t *)this->strm)[this->blk];
		// clear bit at pos and all positions to the left.
		scanblock_t marker = one_bit << this->pos;
		this->scan_blk = this->scan_blk & (marker-1);
	}

	IDISA_ALWAYS_INLINE bool is_done() const {return (EOS==this->pos);}
	IDISA_ALWAYS_INLINE void set_strm(const bitblock_t * strm) {this->strm = strm;}
	IDISA_ALWAYS_INLINE const bitblock_t * get_strm() const {return this->strm;}
	IDISA_ALWAYS_INLINE int32_t get_pos() const {return this->pos;}
	IDISA_ALWAYS_INLINE void set_pos(int32_t pos) {(this->pos = pos);}
	static const uint32_t BLOCK_COUNT = sizeof(bitblock_t)/sizeof(scanblock_t);

};

//
// BitBlock Iterator Classses
//
template<class bitblock_t, class scanblock_t>
class ForwardIterator : public std::iterator<std::input_iterator_tag, int>
{
public:

	ForwardIterator(bitblock_t * s): scanner(s)
	{
		scanner.scan_to_next();
	}

	// set scanner to first pos of
	void init(bitblock_t * s)
	{
		scanner.init(s);
		scanner.scan_to_next();
	}

	// equal position and stream
	bool operator==(const ForwardIterator& iter)
	{
		return ((scanner.get_strm() == iter.scanner.get_strm()) && 
                        (scanner.get_pos() == iter.scanner.get_pos()));
	}

	// not equal position and stream
	bool operator!=(const ForwardIterator& iter)
	{
		return ((scanner.get_strm() != iter.scanner.get_strm()) && 
                        (scanner.get_pos() != iter.scanner.get_pos()));
	}

	// Returns absolute position.
	IDISA_ALWAYS_INLINE int32_t operator*()
	{
		return scanner.get_pos();
	}

	// prefix increment
	IDISA_ALWAYS_INLINE ForwardIterator& operator++()
	{
		scanner.scan_to_next();
		return(*this);
	}

	// postfix increment
	ForwardIterator operator++(int)
	{
		ForwardIterator temp(*this);
		++(*this);
		return(temp);
	}

	IDISA_ALWAYS_INLINE bool isDone() const
	{
		return scanner.is_done();
	}

protected:
	ForwardIterator() {}

private:
	ForwardScanner<bitblock_t, scanblock_t> scanner;
};

class BitBlockForwardIterator: public ForwardIterator<BitBlock, ScanWord> {
public:
	BitBlockForwardIterator(){}
	BitBlockForwardIterator(BitBlock * s): ForwardIterator<BitBlock, ScanWord>(s){}
};

template<class bitblock_t, class scanblock_t>
class ReverseIterator 
{
public:
	ReverseIterator(BitBlock * s): scanner(s)
	{
		scanner.scan_to_next();
	}

	void init(bitblock_t * s)
	{
		scanner.init(s);
		scanner.scan_to_next();
	}

	// equal position and stream
	bool operator==(const ReverseIterator& iter)
	{
		return ((scanner.get_strm() == iter.scanner.get_strm()) && (scanner.get_pos() == iter.scanner.get_pos));
	}

	// not equal position and stream
	bool operator!=(const ReverseIterator& iter)
	{
		return ((scanner.get_strm() != iter.scanner.get_strm()) && (scanner.get_pos() != iter.scanner.get_pos()));
	}

	// Returns absolute position.
	IDISA_ALWAYS_INLINE int32_t operator*()
	{
		return scanner.get_pos();
	}

	// prefix decrement
	IDISA_ALWAYS_INLINE ReverseIterator& operator--()
	{
		scanner.scan_to_next();
		return(*this);
	}

	// postfix decrement
	ReverseIterator operator--(int)
	{
		ReverseIterator temp(*this);
		--(*this);
		return(temp);
	}

	IDISA_ALWAYS_INLINE bool isDone() const
	{
		return scanner.is_done();
	}

protected:
	ReverseIterator() {}
	ReverseScanner<bitblock_t, scanblock_t> scanner;
};

class BitBlockReverseIterator: public ReverseIterator<BitBlock, ScanWord> 
{
public:
	BitBlockReverseIterator(BitBlock * s): ReverseIterator<BitBlock, ScanWord>(s){}
private:
	BitBlockReverseIterator(){}
};

//
// BitStream Iterator classes
//
class BitStreamIterator: public std::iterator<std::input_iterator_tag, int>
{
public:
	BitStreamIterator():pos(EOS), blk(-1), blk_pos(-1), strm(NULL), scan_blk(-1), scan_blk_cnt(0)
	{
		// default constructor defines past-the-end of bit stream semantics, pos == EOS
	}

	BitStreamIterator(const BitBlock * s, int cnt):pos(0),	
											 blk(0),
											 blk_pos(0),
											 strm((ScanWord *)s),
											 scan_blk(*((ScanWord *)s)),
											 scan_blk_cnt(cnt)
	{
		scan_to_next();
	}

	virtual ~BitStreamIterator() {};

	// shallow copy, bit stream iterators refer to shared data
	BitStreamIterator& operator=(const BitStreamIterator& iter)
	{
		pos = iter.pos;
		blk = iter.blk;
		blk_pos = iter.blk_pos;
		strm = iter.strm;			// No copy, both
		scan_blk =  iter.scan_blk;
		scan_blk_cnt = iter.scan_blk_cnt;
		return(*this);
	}

	// equal position and stream
	bool operator==(const BitStreamIterator& iter)
	{
		return((strm == iter.strm) && (pos == iter.pos));
	}

	// not equal position and stream
	bool operator!=(const BitStreamIterator& iter)
	{
		return((strm != iter.strm) || (pos != iter.pos));
	}

	// prefix
	inline BitStreamIterator& operator++()
	{
		scan_to_next();
		return(*this);
	}

	// postfix
	BitStreamIterator operator++(int)
	{
		BitStreamIterator temp(*this);
		++(*this);
		return(temp);
	}

	// Returns absolute position.
	inline int32_t operator*()
	{
		return pos;
	}

	/*
	int operator->()
	{
		return(&*(BitStreamIterator)*this);
	}
	*/

	IDISA_ALWAYS_INLINE bool isDone() const
	{
		return (EOS == pos);
	}

	void debug() {
        std::cout << "pos: " << pos << std::endl;
        std::cout << "blk: " << blk << std::endl;
        std::cout << "blk_pos: " << blk_pos << std::endl;
	}

private:
	int32_t pos;
	uint32_t blk;
	int32_t blk_pos;
	const ScanWord * strm;
	ScanWord scan_blk;
	uint32_t scan_blk_cnt;

	// Helpers
	inline void scan_to_next() {
		while (blk<scan_blk_cnt) {
			if(scan_blk > 0){
				pos = scan_forward_zeroes(scan_blk) + blk_pos;
				scan_blk = scan_blk & (scan_blk-1);  // clear rightmost bit
				return;
			}

			blk_pos += (sizeof(ScanWord)*8);
			blk++;
			scan_blk = strm[blk];
		};

		pos = EOS;
		return;
	}
};

#undef has_bit

#endif // BITBLOCK_ITERATOR_H_

