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

namespace IDISA { class IDISA_Builder; }
namespace pablo { class String; }
namespace pablo { class Integer; }

namespace pablo {

class SymbolGenerator {
    friend class PabloKernel;
    using Allocator = PabloAST::Allocator;
public:
    using IntTy = int64_t;
    String * makeString(const llvm::StringRef prefix, IDISA::IDISA_Builder * builder) noexcept;
    Integer * getInteger(const IntTy value, IDISA::IDISA_Builder * builder) noexcept;
    ~SymbolGenerator() { }
protected:
    SymbolGenerator(Allocator & allocator) : mAllocator(allocator) { }
private:
    Allocator &                                  mAllocator;
    llvm::StringMap<IntTy>                       mPrefixMap;
    llvm::StringMap<String *>                    mStringMap;
    boost::container::flat_map<IntTy, Integer *> mIntegerMap;
};


}

#endif // SYMBOL_GENERATOR_H
