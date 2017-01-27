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

String * SymbolGenerator::makeString(const llvm::StringRef prefix, IDISA::IDISA_Builder * builder) noexcept {
    auto f = mPrefixMap.find(prefix);
    if (f == mPrefixMap.end()) {   
        return getString(prefix, builder);
    } else { // this string already exists; make a new string using the given prefix

        // TODO: check FormatInt from "https://github.com/fmtlib/fmt/blob/master/fmt/format.h" for faster integer conversion

        size_t count = f->second++;
        size_t length = prefix.size() + 2;
        size_t digits = 10;
        while (LLVM_UNLIKELY(digits <= count)) {
            digits *= 10;
            length += 1;
        }
        char name[length];
        std::memcpy(name, prefix.data(), prefix.size());
        char * p = name + length - 1;
        while (count) {
            *p-- = (count % 10) + '0';
            count /= 10;
        }
        *p = '_';
        return makeString(llvm::StringRef(name, length), builder);
    }
}

String * SymbolGenerator::getString(const llvm::StringRef name, IDISA::IDISA_Builder * builder) noexcept {
    if (LLVM_UNLIKELY(name.size() == 0)) {
        throw std::runtime_error("symbol name cannot be 0-length");
    }
    const auto f = mStringMap.find(name);
    if (LLVM_LIKELY(f == mStringMap.end())) {
        assert ("prefix cannot exist for a non-existant key!" && (mPrefixMap.count(name) == 0));
        // create an internal copy of this name to prevent a temporary string from being added to the maps
        char * const data = mAllocator.allocate<char>(name.size() + 1);
        std::memcpy(data, name.data(), name.size());
        data[name.size()] = '\0';
        llvm::StringRef duplicate(data, name.size());
        mPrefixMap.insert(std::make_pair(duplicate, 1));
        String * result = new (mAllocator) String(builder->getInt8PtrTy(), duplicate, mAllocator); assert (result);
        mStringMap.insert(std::make_pair(duplicate, result));
        return result;
    }
    assert ("prefix must exist for a known key!" && (mPrefixMap.count(name) != 0));
    return f->second;
}

Integer * SymbolGenerator::getInteger(const IntTy value, IDISA::IDISA_Builder * builder) noexcept {
    auto f = mIntegerMap.find(value);
    Integer * result;
    if (f == mIntegerMap.end()) {
        result = new (mAllocator) Integer(value, builder->getSizeTy(), mAllocator);
        assert (result->value() == value);
        mIntegerMap.emplace(std::make_pair(value, result));
    } else {
        result = f->second;
    }
    return result;
}

}
