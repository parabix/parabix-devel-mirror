#ifndef SLAB_ALLOCATOR_H
#define SLAB_ALLOCATOR_H

#include <llvm/Support/Allocator.h>

namespace {

using LLVMAllocator = llvm::BumpPtrAllocator;

class __BumpPtrAllocatorProxy {
public:
    template <typename T>
    static inline T * Allocate(const size_t n) {
        return mAllocator.Allocate<T>(n);
    }
    template <typename T>
    static inline void Deallocate(const T * pointer) {
        mAllocator.Deallocate(pointer);
    }
    static inline void Reset() {
        #ifndef NDEBUG
        mAllocator.PrintStats();
        #endif
        mAllocator.Reset();
    }
    static LLVMAllocator & get_allocator() {
        return mAllocator;
    }
private:
    static llvm::BumpPtrAllocator mAllocator;
};

LLVMAllocator __BumpPtrAllocatorProxy::mAllocator;

}

template <typename T>
class SlabAllocator {
public:

    using value_type = T;
    using pointer = value_type*;
    using const_pointer = const value_type*;
    using reference = value_type&;
    using const_reference = const value_type&;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;

    template<class U>
    struct rebind {
        typedef SlabAllocator<U> other;
    };

    inline pointer allocate(size_type n, const_pointer = nullptr) noexcept {
        return mAllocator.Allocate<T>(n);
    }

    inline void deallocate(pointer p, size_type n) noexcept {
        mAllocator.Deallocate<T>(p);
    }

    inline size_type max_size() const {
        return std::numeric_limits<size_type>::max();
    }

    inline LLVMAllocator & get_allocator() {
        return mAllocator.get_allocator();
    }

    inline bool operator==(SlabAllocator<T> const&) { return true; }
    inline bool operator!=(SlabAllocator<T> const&) { return false; }

    inline SlabAllocator() noexcept {}
    inline SlabAllocator(const SlabAllocator &) noexcept {}
    template <class U> inline SlabAllocator (const std::allocator<U>&) noexcept {}
    inline ~SlabAllocator() { }
private:
    __BumpPtrAllocatorProxy mAllocator;
};

inline void releaseSlabAllocatorMemory() {
    __BumpPtrAllocatorProxy::Reset();
}

#endif // SLAB_ALLOCATOR_H
