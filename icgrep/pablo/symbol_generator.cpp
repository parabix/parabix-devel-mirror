/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/symbol_generator.h>
#include <pablo/pe_string.h>
#include <pablo/pe_integer.h>

namespace pablo {

String * SymbolGenerator::get(const std::string name, const bool generated) {
    if (name.length() == 0) {
        throw std::runtime_error("symbol name cannot be 0-length");
    }
    auto f = mStringMap.find(name);
    String * result = nullptr;
    if (f == mStringMap.end()) {
        result = new String(name, generated);
        assert (result);
        mStringMap.insert(std::make_pair(std::move(name), result));
    }
    else {
        result = f->second;
    }
    return result;
}

Integer * SymbolGenerator::getInteger(const integer_t value) {
    auto f = mIntegerMap.find(value);
    Integer * result;
    if (f == mIntegerMap.end()) {
        result = new Integer(value);
        assert (result->value() == value);
        mIntegerMap.insert(std::make_pair(value, result));
    } else {
        result = f->second;
    }
    return result;
}

String * SymbolGenerator::make(const std::string prefix, const bool generated) {
    auto f = mPrefixMap.find(prefix);
    unsigned count = 0;
    if (f == mPrefixMap.end()) {
        mPrefixMap.insert(std::make_pair(prefix, 1));
        return get(prefix, generated);
    } else {
        count = f->second++;
        return get(prefix + std::to_string(count), generated);
    }
}

}
