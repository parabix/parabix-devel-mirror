/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef TRACEGEN_H
#define TRACEGEN_H

#include "idisa_builder.h"
#include <string>

class TraceTool {
public:
    TraceTool(IDISA::IDISA_Builder * b, unsigned log2TraceBufSize = 16);
    
    unsigned declareTraceVar(std::string traceVarName);
    unsigned newTraceVar(std::string traceName);
    void addTraceEntry(unsigned traceVar, llvm::Value * traceVal);
    void createDumpTrace();
    
private:
    IDISA::IDISA_Builder * iBuilder;
    unsigned mLog2TraceBufSize;
    unsigned mTraceVarCount;
    std::vector<llvm::Value *> mTraceFormatString;
    llvm::Value * mTraceBufferPtr;
    llvm::Value * mTraceIndexPtr;
    llvm::Constant * mTraceIndexMask;
};

#endif
