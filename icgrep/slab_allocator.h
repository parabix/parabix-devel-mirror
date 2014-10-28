#ifndef SLAB_ALLOCATOR_H
#define SLAB_ALLOCATOR_H

#include <cstddef>
#include <cstdint>
#include <new>
#include <algorithm>
#include <iostream>

template <unsigned BasePoolSize>
class SlabAllocator {
public:

    typedef std::size_t size_type;
    typedef void* pointer;
    typedef const void* const_pointer;

    inline pointer allocate(size_type n) noexcept {
        if (n > mSpaceAvail && !extend(n)) {
            exit(-1);
        }
        pointer ptr = static_cast<pointer>(mAllocationPtr);
        mAllocationPtr += n;
        mSpaceAvail -= n;
        return ptr;
    }

    inline void deallocate(pointer p) noexcept {
        /* do nothing */
    }

    void release_memory();

    SlabAllocator();
    ~SlabAllocator();

protected:
    bool extend(const size_type n) noexcept;
private:
    struct Chunk {
        Chunk *     prev;
        uint8_t     space[BasePoolSize];
    };
    size_type       mSpaceAvail;
    uint8_t *       mAllocationPtr;
    Chunk *         mCurrentChunk;
    size_type       mTotalSize;
    Chunk           mInitialChunk;
};

template <unsigned BasePoolSize>
SlabAllocator<BasePoolSize>::SlabAllocator()
: mSpaceAvail(BasePoolSize)
, mAllocationPtr(mInitialChunk.space)
, mCurrentChunk(&mInitialChunk)
, mTotalSize(BasePoolSize)
{
    mInitialChunk.prev = nullptr;
}

template <unsigned BasePoolSize>
SlabAllocator<BasePoolSize>::~SlabAllocator() {
    release_memory();
}

template <unsigned BasePoolSize>
void SlabAllocator<BasePoolSize>::release_memory() {
    while (mCurrentChunk != &mInitialChunk) {
        Chunk * n = mCurrentChunk;
        mCurrentChunk = n->prev;
        free(n);
    }
    mSpaceAvail = BasePoolSize;
    mAllocationPtr = mInitialChunk.space;
}

template <unsigned BasePoolSize>
bool SlabAllocator<BasePoolSize>::extend(const size_type n) noexcept {
    const size_type size = std::max<size_type>(n, mTotalSize) * 2;
    Chunk * newChunk = (Chunk *)malloc(sizeof(Chunk) + size - BasePoolSize);
    if (newChunk == nullptr) {
        return false;
    }
    newChunk->prev = mCurrentChunk;
    mTotalSize += size;
    mCurrentChunk = newChunk;
    mAllocationPtr = newChunk->space;
    mSpaceAvail = size;
    return true;
}

#endif // SLAB_ALLOCATOR_H
