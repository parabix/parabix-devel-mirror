/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#ifndef ZTF_LOGIC_H
#define ZTF_LOGIC_H

#include <pablo/pablo_kernel.h>
#include <kernel/core/kernel_builder.h>

namespace kernel {

struct LengthGroup {unsigned lo; unsigned hi;};
    
struct LengthGroupInfo {
    unsigned lo;                    //  the low bound of the length group
    unsigned hi;                    //  the high bound of the length group
    unsigned encoding_bytes;        //  the number of bytes for encoding this group
    unsigned prefix_base;           //  the base value of encoded prefix bytes
    unsigned hash_bits;             //  number of bits used for hash codes
    unsigned length_extension_bits; //  number of bits used for length extension
};

class EncodingInfo {
public:
    EncodingInfo(unsigned maxHashBits, std::vector<LengthGroupInfo> lengthGroups) :
        MAX_HASH_BITS(maxHashBits), byLength(lengthGroups) {}
    const unsigned MAX_HASH_BITS;
    const std::vector<LengthGroupInfo> byLength;
public:
    unsigned getLengthGroupNo(unsigned lgth) const;
    unsigned maxSymbolLength() const;
    unsigned maxEncodingBytes() const;
    unsigned prefixLengthOffset(unsigned lgth) const;
    std::string uniqueSuffix() const;
};

class WordMarkKernel : public pablo::PabloKernel {
public:
    WordMarkKernel(BuilderRef kb, StreamSet * BasisBits, StreamSet * WordMarks);
protected:
    void generatePabloMethod() override;
};

class ByteRun final: public pablo::PabloKernel {
public:
    ByteRun(BuilderRef b, StreamSet * const basis, StreamSet * excluded, StreamSet * runMask)
    : pablo::PabloKernel(b, "byteRun", {Binding{"basis", basis}, Binding{"excluded", excluded}}, {Binding{"runMask", runMask}}) {}
protected:
    void generatePabloMethod() override;
};

/*
 This kernel decodes the insertion length for two-byte ZTF code symbols.
 The insertion length is the number of zero bytes to insert so that,
 after insertion the zeroes together with the encoded symbol can be
 replaced by the dictionary symbol of the appropriate length.
*/

class ZTF_ExpansionDecoder final: public pablo::PabloKernel {
public:
    ZTF_ExpansionDecoder(BuilderRef b,
                         EncodingInfo & encodingScheme,
                         StreamSet * const basis,
                         StreamSet * insertBixNum);
protected:
    void generatePabloMethod() override;
    EncodingInfo & mEncodingScheme;
};

class ZTF_DecodeLengths : public pablo::PabloKernel {
public:
    ZTF_DecodeLengths(BuilderRef b,
                      EncodingInfo & encodingScheme,
                      StreamSet * basisBits,
                      StreamSet * groupStreams);
protected:
    void generatePabloMethod() override;
    EncodingInfo & mEncodingScheme;
};

// Parse encodable ZTF words or symbols from plaintext or ciphertext.
// The result is a stream symbolRuns marking symbol continuation bits
// with 1 bits.   Each 0 bit represents a start of a new symbol.
class ZTF_Symbols : public pablo::PabloKernel {
public:
    ZTF_Symbols(BuilderRef kb,
                StreamSet * basisBits, StreamSet * wordChar, StreamSet * symbolRuns)
    : pablo::PabloKernel(kb, "ZTF_Symbols",
                         {Binding{"basisBits", basisBits, FixedRate(1), LookAhead(1)},
                             Binding{"wordChar", wordChar, FixedRate(1), LookAhead(3)}},
                         {Binding{"symbolRuns", symbolRuns}}) { }
protected:
    void generatePabloMethod() override;
};

// Modifed ZTF_Symbols: handles multi-byte ciphertext.
// Modified suffix ranges for codeword with prefix range 0xF8-0xFF
class ZTF_Phrases : public pablo::PabloKernel {
public:
    ZTF_Phrases(BuilderRef kb,
                StreamSet * basisBits,
                StreamSet * wordChar,
                StreamSet * phraseRuns);
protected:
    void generatePabloMethod() override;
};

// Generate i-th phraseRunSeq stream with n-syms for the given phraseRuns.
// Start position of first n-sym sequence is indicated by the seqNum.
// symbol sequence continuation bits are marked with 1 bits.
// Each 0 bit represents a start of a new symbol.
class PhraseRunSeq : public pablo::PabloKernel {
public:
    PhraseRunSeq(BuilderRef kb,
                 StreamSet * phraseRuns,
                 StreamSet * phraseRunSeq,
                 unsigned numSyms,
                 unsigned seqNum);
protected:
    void generatePabloMethod() override;
    unsigned mNumSyms;
    unsigned mSeqNum;
};

class PhraseRunSeqTemp : public pablo::PabloKernel {
public:
    PhraseRunSeqTemp(BuilderRef kb,
                 StreamSet * phraseRuns,
                 StreamSet * PhraseRunSeq,
                 StreamSet * compSeqRuns,
                 unsigned numSyms,
                 unsigned seqNum);
protected:
    void generatePabloMethod() override;
    unsigned mNumSyms;
    unsigned mSeqNum;
};

// Given parsed symbol runs, produce a stream marking end positions only.
class ZTF_SymbolEnds : public pablo::PabloKernel {
public:
    ZTF_SymbolEnds(BuilderRef kb,
                   StreamSet * symbolRuns, StreamSet * overflow, StreamSet * symbolEnds)
    : pablo::PabloKernel(kb, "ZTF_SymbolEnds",
                         {Binding{"symbolRuns", symbolRuns, FixedRate(1), LookAhead(1)},
                          Binding{"overflow", overflow}},
                         {Binding{"symbolEnds", symbolEnds}}) { }
protected:
    void generatePabloMethod() override;
};

class ZTF_SymbolEncoder final: public pablo::PabloKernel {
public:
    ZTF_SymbolEncoder(BuilderRef b,
                      EncodingInfo & encodingScheme,
                      StreamSet * const basis,
                      StreamSet * bixHash,
                      StreamSet * extractionMask,
                      StreamSet * runIdx,
                      StreamSet * encoded);
protected:
    void generatePabloMethod() override;
    EncodingInfo & mEncodingScheme;
};

class LengthGroupSelector final: public pablo::PabloKernel {
public:
    LengthGroupSelector(BuilderRef b,
                 EncodingInfo & encodingScheme,
                    unsigned groupNo,
                 StreamSet * symbolRun, StreamSet * const lengthBixNum,
                 StreamSet * overflow,
                 StreamSet * selected);
protected:
    void generatePabloMethod() override;
    EncodingInfo & mEncodingScheme;
    unsigned mGroupNo;
};

class LengthSorter final: public pablo::PabloKernel {
public:
    LengthSorter(BuilderRef b,
                 EncodingInfo & encodingScheme,
                 StreamSet * symbolRun, StreamSet * const lengthBixNum,
                 StreamSet * overflow,
                 StreamSet * groupStreams);
protected:
    void generatePabloMethod() override;
    EncodingInfo & mEncodingScheme;
};
}
#endif

