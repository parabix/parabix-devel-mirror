/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef SCANMATCHGEN_H
#define SCANMATCHGEN_H

namespace llvm { class Module; }

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class KernelBuilder;

void generateScanMatch(llvm::Module * m, IDISA::IDISA_Builder * iBuilder, unsigned scanWordBitWidth, KernelBuilder * kBuilder, bool isNameExpression);

}

#endif // SCANMATCHGEN_H
