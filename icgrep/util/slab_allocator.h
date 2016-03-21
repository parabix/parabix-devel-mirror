#ifndef SLAB_ALLOCATOR_H
#define SLAB_ALLOCATOR_H

#include <llvm/Support/Allocator.h>

using LLVMAllocator = llvm::BumpPtrAllocator;

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
        return static_cast<T*>(mAllocator.Allocate(n * sizeof(T), sizeof(void*)));
    }

    inline void deallocate(pointer p, size_type = 0) noexcept {
        mAllocator.Deallocate(p);
    }

    inline size_type max_size() const {
        return std::numeric_limits<size_type>::max();
    }

    inline LLVMAllocator & get_allocator() {
        return mAllocator;
    }

    void Reset() {
        mAllocator.Reset();
    }

    inline bool operator==(SlabAllocator<T> const&) { return true; }
    inline bool operator!=(SlabAllocator<T> const&) { return false; }

    inline SlabAllocator() noexcept {}
    inline SlabAllocator(const SlabAllocator &) noexcept {}
    template <class U> inline SlabAllocator (const std::allocator<U>&) noexcept {}
    inline ~SlabAllocator() { }
private:
    static LLVMAllocator mAllocator;
};

template <typename T> LLVMAllocator SlabAllocator<T>::mAllocator;

inline void releaseSlabAllocatorMemory() {
    SlabAllocator<void *> T;
    T.Reset();
}

template <typename T>
class LLVMAllocatorProxy {
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

    inline void deallocate(pointer p, size_type = 0) noexcept {
        mAllocator.Deallocate(p);
    }

    inline size_type max_size() const {
        return std::numeric_limits<size_type>::max();
    }

    inline bool operator==(SlabAllocator<T> const&) { return true; }
    inline bool operator!=(SlabAllocator<T> const&) { return false; }

    inline LLVMAllocatorProxy(LLVMAllocator & allocator) noexcept : mAllocator(allocator) {}
    inline LLVMAllocatorProxy(const LLVMAllocatorProxy & proxy) noexcept : mAllocator(proxy.mAllocator) {}
    inline ~LLVMAllocatorProxy() { }
private:
    LLVMAllocator & mAllocator;
};

#endif // SLAB_ALLOCATOR_H
