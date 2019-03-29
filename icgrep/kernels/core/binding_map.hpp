#ifndef BINDING_MAP_HPP
#define BINDING_MAP_HPP

#include "binding.h"
#include <llvm/ADT/StringMap.h>

namespace kernel {

enum class BindingType : unsigned { StreamInput, StreamOutput, ScalarInput, ScalarOutput };

struct BindingMapEntry {
    BindingType Type;
    unsigned    Index;

    BindingMapEntry() = default;
    explicit BindingMapEntry(BindingType Type, unsigned index)
    : Type(Type), Index(index) { }

    bool operator < (const BindingMapEntry other) const {
        if (Type == other.Type) {
            return Index < other.Index;
        } else {
            using int_t = std::underlying_type<BindingType>::type;
            return static_cast<int_t>(Type) < static_cast<int_t>(other.Type);
        }
    }
};

using BindingMap = llvm::StringMap<BindingMapEntry>;

}

#endif // BINDING_MAP_HPP
