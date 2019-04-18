/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef PARABIX_ARITHMETIC_COMPILER_H
#define PARABIX_ARITHMETIC_COMPILER_H

#include <pablo/pabloAST.h>
#include <pablo/builder.hpp>

namespace pablo {

using BixNum = std::vector<PabloAST *>;
    
    
class BixNumCompiler {
public:
    BixNumCompiler(PabloBuilder & pb) : mPB(pb) {}
    PabloAST * EQ(BixNum value, unsigned test);
    PabloAST * EQ(BixNum value, BixNum test);
    PabloAST * NEQ(BixNum value, unsigned test);
    PabloAST * NEQ(BixNum value, BixNum test);
    PabloAST * UGE(BixNum value, unsigned floor);
    PabloAST * UGE(BixNum value, BixNum floor);
    PabloAST * UGT(BixNum value, unsigned floor);
    PabloAST * UGT(BixNum value, BixNum floor);
    PabloAST * ULE(BixNum value, unsigned floor);
    PabloAST * ULE(BixNum value, BixNum floor);
    PabloAST * ULT(BixNum value, unsigned floor);
    PabloAST * ULT(BixNum value, BixNum floor);
    BixNum ZeroExtend(BixNum value, unsigned extended_size);
    BixNum SignExtend(BixNum value, unsigned extended_size);
    BixNum Truncate(BixNum value, unsigned truncated_size);
    BixNum HighBits(BixNum value, unsigned highBitCount);
    //
    // Modular arithmetic operations
    BixNum AddModular(BixNum augend, unsigned addend);
    BixNum AddModular(BixNum augend, BixNum addend);
    BixNum SubModular(BixNum minuend, unsigned subtrahend);
    BixNum SubModular(BixNum minuend, BixNum subtrahend);
    BixNum MulModular(BixNum multiplicand, unsigned multiplier);
    //
    // Full arithmetic operations
    BixNum AddFull(BixNum augend, BixNum addend);
    BixNum MulFull(BixNum multiplicand, unsigned multiplier);

private:
    PabloBuilder & mPB;
};


typedef std::vector<Var *> BixVar;

class BixNumTableCompilerInterface  {
public:
    void setRecursivePartitionLevels(std::vector<unsigned> & partitionBits) {mPartitionBits = partitionBits;}
protected:
    BixNumTableCompilerInterface(BixNum & input, BixVar & output, unsigned inputMax) :
        mInput(input), mOutput(output), mInputMax(inputMax) {}
    BixNum & mInput;
    BixVar & mOutput;
    unsigned mInputMax;
    std::vector<unsigned> mPartitionBits;
    virtual unsigned getTableVal(unsigned inputVal) = 0;
    virtual unsigned consecutiveFrom(unsigned inputVal) = 0;
    virtual unsigned computeOutputBitsForRange(unsigned lo, unsigned rangeSize) = 0;
    virtual void innerLogic(PabloBuilder & pb,
                    unsigned partitionBase,
                    PabloAST * partitionSelect,
                    unsigned outputBitsToSet) = 0;
    void tablePartitionLogic(PabloBuilder & pb,
                             unsigned nestingDepth,
                             unsigned partitionBase,
                             PabloAST * partitionSelect,
                             unsigned outputBitsToSet);
};


    
    
//
// A compiler that implements parallel bitwise table lookup for fixed tables.
//
class BixNumTableCompiler : public BixNumTableCompilerInterface {
public:
    BixNumTableCompiler(std::vector<unsigned> & table, BixNum & input, BixVar & output) :
        BixNumTableCompilerInterface(input, output, table.size()-1),
        mTable(table), mBitsPerOutputUnit(output.size()) {}
    void compileSubTable(PabloBuilder & pb, unsigned lo, unsigned hi, PabloAST * partitionSelect);

private:
    std::vector<unsigned> & mTable;
    unsigned mBitsPerOutputUnit;
    unsigned getTableVal(unsigned inputVal) override;
    unsigned consecutiveFrom(unsigned inputVal) override;
    unsigned computeOutputBitsForRange(unsigned lo, unsigned rangeSize) override;
    void innerLogic(PabloBuilder & pb,
                    unsigned partitionBase,
                    PabloAST * partitionSelect,
                    unsigned outputBitsToSet) override;
};
    

typedef std::vector<std::pair<unsigned, unsigned>> RangeTable;

class BixNumRangeTableCompiler : public BixNumTableCompilerInterface {
public:
    BixNumRangeTableCompiler(RangeTable & table, unsigned inputMax, BixNum & input, BixVar & output) :
        BixNumTableCompilerInterface(input, output, inputMax), mRangeTable(table) {}
    void compileTable(PabloBuilder & pb, PabloAST * tableSelect);
private:
    RangeTable mRangeTable;
    unsigned getTableIndex(unsigned inputVal);
    unsigned getTableVal(unsigned inputVal) override;
    unsigned consecutiveFrom(unsigned inputVal) override;
    unsigned computeOutputBitsForRange(unsigned lo, unsigned rangeSize) override;
    void innerLogic(PabloBuilder & pb,
                             unsigned partitionBase,
                             PabloAST * partitionSelect,
                             unsigned outputBitsToSet) override;

};

}


#endif

