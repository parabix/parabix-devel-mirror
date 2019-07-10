/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "symbol_generator.h"
#include <pablo/pe_string.h>
#include <pablo/pe_integer.h>
#include <IR_Gen/idisa_builder.h>

namespace pablo {

String * SymbolGenerator::makeString(const llvm::StringRef prefix) noexcept {
    auto f = mPrefixMap.find(prefix);
    if (f == mPrefixMap.end()) {
        char * const data = mAllocator.allocate<char>(prefix.size() + 1);
        std::memcpy(data, prefix.data(), prefix.size());
        data[prefix.size()] = '\0';
        llvm::StringRef name(data, prefix.size());
        mPrefixMap.insert(std::make_pair(name, 1));
        return new (mAllocator) String(llvm::IntegerType::getInt8PtrTy(mContext), name, mAllocator);
    } else { // this string already exists; make a new string using the given prefix

        // TODO: check FormatInt from "https://github.com/fmtlib/fmt/blob/master/fmt/format.h" for faster integer conversion

        size_t count = f->second++;
        size_t length = prefix.size() + 2;
        size_t digits = 10;
        while (LLVM_UNLIKELY(digits <= count)) {
            digits *= 10;
            length += 1;
        }

        llvm::SmallVector<char, 256> name(length);
        std::memcpy(name.data(), prefix.data(), prefix.size());
        char * p = name.data() + length - 1;
        while (count) {
            *p-- = (count % 10) + '0';
            count /= 10;
        }
        *p = '_';
        return makeString(llvm::StringRef(name.data(), length));
    }
}

Integer * SymbolGenerator::getInteger(const IntTy value, unsigned intWidth) noexcept {
    auto key = std::make_pair(value, intWidth);
    auto f = mIntegerMap.find(key);
    Integer * result;
    if (f == mIntegerMap.end()) {
        result = new (mAllocator) Integer(value, llvm::IntegerType::getIntNTy(mContext, intWidth), mAllocator);
        assert (result->value() == value);
        mIntegerMap.emplace(key, result);
    } else {
        result = f->second;
    }
    return result;
}

}
