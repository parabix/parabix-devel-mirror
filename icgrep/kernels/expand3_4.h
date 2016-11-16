/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef EXPAND3_4_H
#define EXPAND3_4_H

#include "streamset.h"
#include "interface.h"
#include "kernel.h"

namespace llvm { class Module; }

namespace IDISA { class IDISA_Builder; }

namespace kernel {

/*  expand3_4 transforms a byte sequence by duplicating every third byte. 
    Each 3 bytes of the input abc produces a 4 byte output abcc.   
    This is a useful preparatory transformation in various radix-64 encodings. */
 
class expand3_4Kernel : public KernelBuilder {
public:
    
    expand3_4Kernel(IDISA::IDISA_Builder * iBuilder) :
    KernelBuilder(iBuilder, "expand3_4",
                  {Binding{parabix::StreamSetType(iBuilder, 1, parabix::i8), "sourceStream"}},
                  {Binding{parabix::StreamSetType(iBuilder, 1, parabix::i8), "expandedStream"}},
                  {}, {}, 
                  {Binding{iBuilder->fwVectorType(parabix::i8), "pendingPack"}}) {}
    
    
private:
    void generateDoBlockMethod() override;
    void generateDoSegmentMethod() override;
    
};

    

}
#endif
