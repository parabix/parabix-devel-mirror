
#ifndef CBUILDER_CONFIH_H
#define CBUILDER_CONFIH_H

#include <llvm/IR/IRBuilder.h>

namespace llvm { class ConstantFolder; }
namespace llvm { class IRBuilderDefaultInserter; }
namespace llvm { class LLVMContext; }

class CBuilderBase :
    public llvm::IRBuilder<> {
    typedef llvm::IRBuilder<llvm::ConstantFolder, llvm::IRBuilderDefaultInserter> IRBuilderB;
    public:
        CBuilderBase(llvm::LLVMContext &context) :
        IRBuilderB(context) {

      }
};
#endif
