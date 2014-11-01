#ifndef PABLO_METADATA_H
#define PABLO_METADATA_H

#include <pablo/pabloAST.h>
#include <vector>

namespace pablo {

class PMDNode {
    enum class ClassTypeId : unsigned {
        PMDASTVector
    };
    inline ClassTypeId getClassTypeId() const {
        return mClassTypeId;
    }
protected:
    void* operator new (std::size_t size) noexcept {
        return PabloAST::mAllocator.allocate(size);
    }
    inline PMDNode(const ClassTypeId id)
    : mClassTypeId(id)
    {

    }
private:
    const ClassTypeId   mClassTypeId;
};

class PMDVector : public PMDNode, public std::vector<PabloAST*> {
protected:
    PMDVector(std::vector<PabloAST*> && vec)
    : PMDNode(PMDNode::ClassTypeId::PMDASTVector)
    , std::vector<PabloAST*>(std::move(vec))
    {
    }
};

PMDVector * makeMetadataVector(std::vector<PabloAST*> && vec) {
    return new PMDVector(vec);
}


}

#endif // PE_METADATA_H
