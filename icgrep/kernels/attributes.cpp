#include "attributes.h"

namespace kernel {

void AttributeSet::addAttribute(Attribute attribute) {
    for (Attribute & attr : *this) {
        if (attr.getKind() == attribute.getKind()) {
            attr.mK = attribute.mK;
            return;
        }
    }
    emplace_back(attribute);
}

bool AttributeSet::hasAttribute(const AttributeId id) const {
    for (const Attribute & attr : *this) {
        if (attr.getKind() == id) {
            return true;
        }
    }
    return false;
}

}
