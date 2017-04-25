/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef IDISA_TARGET_H
#define IDISA_TARGET_H

#include <string>
namespace llvm { class Module; }
namespace IDISA { class IDISA_Builder; }

namespace IDISA {
    
IDISA::IDISA_Builder * GetIDISA_Builder(llvm::Module * const module);

IDISA::IDISA_Builder * GetIDISA_GPU_Builder(llvm::Module * const module);

}

#endif
