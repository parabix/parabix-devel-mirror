#ifndef STRING_INSERT_H
#define STRING_INSERT_H

#include <pablo/pablo_kernel.h>  // for PabloKernel

namespace kernel {
    
//
// Given a set of strings to be inserted at various points within a stream,
// a stream set (possibly multiplexed) identifying the points immediately
// at which strings are to be inserted, a bixNum stream set is calculated
// such that the bixNum at position p is n if the string to be inserted
// at that position is of length n, or 0 if no string is to be inserted.
// If the number of streams in the insertMarks stream set is less than
// the number of strings to be inserted, then it is interpreted as a
// multiplexed set, i.e., a bixnum whose index denotes the string to
// be inserted at he given position.
//
// The result may then be used for calculation of a SpreadMask by InsertionSpreadMask.
//

class StringInsertBixNum : public pablo::PabloKernel {
public:
    StringInsertBixNum(BuilderRef b, std::vector<std::string> & insertStrs,
                       StreamSet * insertMarks, StreamSet * insertBixNum);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
    void generatePabloMethod() override;
private:
    std::vector<std::string> mInsertStrings;
    bool mMultiplexing;
    unsigned mBixNumBits;
};

class StringReplaceKernel : public pablo::PabloKernel {
public:
    StringReplaceKernel(BuilderRef b, std::vector<std::string> & insertStrs,
                        StreamSet * basis, StreamSet * spreadMask,
                        StreamSet * insertMarks, StreamSet * runIndex,
                        StreamSet * output);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
    void generatePabloMethod() override;
private:
    std::vector<std::string> mInsertStrings;
    bool mMultiplexing;
};
}

#endif // STRING_INSERT_H
