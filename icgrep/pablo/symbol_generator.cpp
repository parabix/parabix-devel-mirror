/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/symbol_generator.h>
#include <pablo/pe_string.h>
#include <pablo/pe_integer.h>
#include <IDISA/idisa_builder.h>

namespace pablo {

String * SymbolGenerator::get(const std::string name, IDISA::IDISA_Builder * builder) {
    if (LLVM_UNLIKELY(name.length() == 0)) {
        throw std::runtime_error("symbol name cannot be 0-length");
    }
    auto f = mStringMap.find(name);
    String * result = nullptr;
    if (f == mStringMap.end()) {
        result = new (mAllocator) String(builder->getInt8PtrTy(), name, mAllocator);
        assert (result);
        mStringMap.insert(std::make_pair(std::move(name), result));
    }
    else {
        result = f->second;
    }
    return result;
}

Integer * SymbolGenerator::getInteger(const integer_t value, IDISA::IDISA_Builder * builder) {
    auto f = mIntegerMap.find(value);
    Integer * result;
    if (f == mIntegerMap.end()) {
        result = new (mAllocator) Integer(value, builder->getSizeTy(), mAllocator);
        assert (result->value() == value);
        mIntegerMap.insert(std::make_pair(value, result));
    } else {
        result = f->second;
    }
    return result;
}

String * SymbolGenerator::make(const std::string prefix, IDISA::IDISA_Builder * builder) {
    auto f = mPrefixMap.find(prefix);
    if (f == mPrefixMap.end()) {
        mPrefixMap.insert(std::make_pair(prefix, 1));
        return get(prefix, builder);
    } else {
        const unsigned count = f->second++;
        return get(prefix + '_' + std::to_string(count), builder);
    }
}

}
