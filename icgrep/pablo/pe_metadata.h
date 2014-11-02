#ifndef PABLO_METADATA_H
#define PABLO_METADATA_H

#include <pablo/pabloAST.h>
#include <vector>

namespace pablo {

class PMDNode {
public:
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
public:
    inline static PMDVector * get(std::vector<PabloAST*> && vec) {
        return new PMDVector(std::move(vec));
    }
protected:
    PMDVector(std::vector<PabloAST*> && vec)
    : PMDNode(PMDNode::ClassTypeId::PMDASTVector)
    , std::vector<PabloAST*>(std::move(vec))
    {
    }
};

}

#endif // PE_METADATA_H
