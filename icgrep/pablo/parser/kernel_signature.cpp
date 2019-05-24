/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "kernel_signature.h"

namespace pablo {
namespace parse {

PabloKernelSignature::Type::Allocator PabloKernelSignature::Type::mAllocator;

std::string PabloKernelSignature::IntType::asString() const {
    return "i" + std::to_string(mBitWidth);
}

std::string PabloKernelSignature::StreamType::asString() const {
    return "<" + mElementType->asString() + ">";
}

std::string PabloKernelSignature::StreamSetType::asString() const {
    return "<" + mElementType->asString() + ">" + "[" + std::to_string(mStreamCount) + "]";
}

}
}

std::ostream & operator << (std::ostream & out, pablo::parse::PabloKernelSignature const & sig) {
    out << "kernel " << sig.getName() << " :: [";
    for (size_t i = 0; i < sig.getInputBindings().size(); ++i) {
        std::string name;
        pablo::parse::PabloKernelSignature::Type * type;
        std::tie(name, type) = sig.getInputBindings()[i];
        out << type->asString() << " " << name;
        if (i != sig.getInputBindings().size() - 1) {
            out << ", ";
        }
    }
    out << "] -> [";
    for (size_t i = 0; i < sig.getOutputBindings().size(); ++i) {
        std::string name;
        pablo::parse::PabloKernelSignature::Type * type;
        std::tie(name, type) = sig.getOutputBindings()[i];
        out << type->asString() << " " << name;
        if (i != sig.getOutputBindings().size() - 1) {
            out << ", ";
        }
    }
    out << "]";
    return out;
}
