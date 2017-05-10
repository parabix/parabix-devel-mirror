/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef SYMBOL_GENERATOR_H
#define SYMBOL_GENERATOR_H

#include <pablo/pabloAST.h>
#include <llvm/ADT/StringMap.h>
#include <boost/container/flat_map.hpp>
#include <memory>

namespace IDISA { class IDISA_Builder; }
namespace pablo { class String; }
namespace pablo { class Integer; }
namespace llvm { class LLVMContext; }

namespace pablo {

class SymbolGenerator {
    friend class PabloKernel;
    using Allocator = PabloAST::Allocator;
public:
    using IntTy = int64_t;
    String * makeString(const llvm::StringRef prefix) noexcept;
    Integer * getInteger(const IntTy value) noexcept;
    ~SymbolGenerator() { }
protected:
    SymbolGenerator(llvm::LLVMContext & C, Allocator & allocator)
    : mContext(C)
    , mAllocator(allocator) {

    }
private:
    llvm::LLVMContext &                          mContext;
    Allocator &                                  mAllocator;
    llvm::StringMap<IntTy>                       mPrefixMap;
    llvm::StringMap<String *>                    mStringMap;
    boost::container::flat_map<IntTy, Integer *> mIntegerMap;
};


}

#endif // SYMBOL_GENERATOR_H
