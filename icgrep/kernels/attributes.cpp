#include "attributes.h"

#include <llvm/Support/raw_ostream.h>

namespace kernel {

Attribute & AttributeSet::addAttribute(Attribute attribute) {
    for (auto i = begin(), i_end = end(); i != i_end; ++i) {
        Attribute & attr = const_cast<Attribute &>(*i);
        if (attr.getKind() == attribute.getKind()) {
            attr.mAmount = attribute.mAmount;
            return attr;
        }
    }
    emplace_back(attribute);
    return back();
}

Attribute * AttributeSet::__findAttribute(const AttributeId id) const {
    for (auto i = begin(), i_end = end(); i != i_end; ++i) {
        if (i->getKind() == id) {
            return const_cast<Attribute *>(&*i);
        }
    }
    return nullptr;
}

}
