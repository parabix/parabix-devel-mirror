#ifndef TYPE_H
#define TYPE_H

#include <util/slab_allocator.h>
#include <vector>

namespace pablo {

class PabloType {
public:

    enum TypeId {
        Scalar = 0
        , Stream = 1
        , String = 2
    };

    using Allocator = SlabAllocator<PabloType *>;

    using Allocated = std::vector<PabloType *, Allocator>;

    friend const PabloType * getPabloType(const PabloType::TypeId typeId, const unsigned fieldWidth);

    bool operator==(const PabloType & t) const;

    PabloType & operator=(PabloType & t) = delete;

    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }

    TypeId getTypeId() const {
        return mTypeId;
    }

    unsigned getFieldWidth() const {
        return mFieldWidth;
    }

    void operator delete (void * ptr) {
        mAllocator.deallocate(static_cast<Allocator::value_type *>(ptr));
    }


protected:

    inline PabloType(const TypeId typeId, const unsigned fieldWidth)
    : mTypeId(typeId)
    , mFieldWidth(fieldWidth)
    {

    }

    inline ~PabloType() {

    }

private:

    const TypeId                        mTypeId;
    const unsigned                      mFieldWidth;

    static Allocator                    mAllocator;

};

const PabloType * getPabloType(const PabloType::TypeId typeId, const unsigned fieldWidth = 1);

}

#endif // TYPE_H
