#ifndef SLAB_ALLOCATOR_H
#define SLAB_ALLOCATOR_H

#include <cstddef>
#include <cstdint>
#include <new>
#include <algorithm>

template <typename T>
class SlabAllocator {
public:

    enum { BasePoolSize = 4096 };

    using value_type = T;
    using pointer = T*;
    using const_pointer = const T*;
    using reference = T&;
    using const_reference = const T&;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;

    template<class U>
    struct rebind {
        typedef SlabAllocator<U> other;
    };

    inline pointer allocate(size_type n) noexcept {
        if (n > mSpaceAvail && !extend(n)) {
            exit(-1);
        }
        pointer ptr = reinterpret_cast<pointer>(mAllocationPtr);
        mAllocationPtr += n;
        mSpaceAvail -= n;
        return ptr;
    }

    inline void deallocate(pointer p, size_type n) noexcept {
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

template <typename T>
SlabAllocator<T>::SlabAllocator()
: mSpaceAvail(BasePoolSize)
, mAllocationPtr(mInitialChunk.space)
, mCurrentChunk(&mInitialChunk)
, mTotalSize(BasePoolSize)
{
    mInitialChunk.prev = nullptr;
}

template <typename T>
SlabAllocator<T>::~SlabAllocator() {
    release_memory();
}

template <typename T>
void SlabAllocator<T>::release_memory() {
    while (mCurrentChunk != &mInitialChunk) {
        Chunk * n = mCurrentChunk;
        mCurrentChunk = n->prev;
        free(n);
    }
    mSpaceAvail = BasePoolSize;
    mAllocationPtr = mInitialChunk.space;
}

template <typename T>
bool SlabAllocator<T>::extend(const size_type n) noexcept {
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
