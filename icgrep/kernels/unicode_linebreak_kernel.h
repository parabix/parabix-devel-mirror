/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef UNICODE_LINEBREAK_H
#define UNICODE_LINEBREAK_H

#include <pablo/pablo_kernel.h>  // for PabloKernel
#include "kernel.h"              // for KernelBuilder
#include <string>                // for string
namespace IDISA { class IDISA_Builder; }

namespace kernel {

class UnicodeLineBreakKernelBuilder: public pablo::PabloKernel {
public:
    UnicodeLineBreakKernelBuilder(IDISA::IDISA_Builder * iBuilder, std::string unicodelinebreak, unsigned basisBitsCount);
};

}
#endif
