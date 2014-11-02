#ifndef PABLO_METADATA_H
#define PABLO_METADATA_H

#include <pablo/pabloAST.h>
#include <llvm/ADT/DenseSet.h>

namespace pablo {

class PMDNode {
public:
    enum class ClassTypeId : unsigned {
        Set
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

class PMDSet : public PMDNode, public llvm::DenseSet<PabloAST*> {
public:
    template<typename iterator>
    inline static PMDSet * get(iterator begin, iterator end) {
        return new PMDSet(begin, end);
    }
protected:
    template<typename iterator>
    PMDSet(iterator begin, iterator end)
    : PMDNode(PMDNode::ClassTypeId::Set)
    , llvm::DenseSet<PabloAST*>()
    {
        insert(begin, end);
    }
};



}

#endif // PE_METADATA_H
