#include <pablo/pablo_type.h>
#include <llvm/Support/raw_ostream.h>

namespace pablo {

PabloType::Allocator PabloType::mAllocator;

static std::vector<PabloType *> DEFINED_TYPES;

const PabloType * getPabloType(const PabloType::TypeId typeId, const unsigned fieldWidth) {
    for (const PabloType * t : DEFINED_TYPES) {
        if (t->mTypeId == typeId && t->mFieldWidth == fieldWidth) {
            return t;
        }
    }
    PabloType * t = new PabloType(typeId, fieldWidth);
    DEFINED_TYPES.emplace_back(t);
    return t;
}

bool PabloType::operator==(const PabloType & t) const {
    return (t.mTypeId == mTypeId && t.mFieldWidth == mFieldWidth);
}

}
