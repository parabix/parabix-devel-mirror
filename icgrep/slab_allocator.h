#ifndef SLAB_ALLOCATOR_H
#define SLAB_ALLOCATOR_H

#include <llvm/Support/Allocator.h>

template <typename T>
class SlabAllocator : public std::allocator<T> {
public:

    using LLVMAllocator = llvm::BumpPtrAllocator;
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
        return reinterpret_cast<pointer>(mAllocator.Allocate(n * sizeof(T), sizeof(T)));
    }

    inline void deallocate(pointer p, size_type n) noexcept {
        mAllocator.Deallocate(p);
    }

    inline size_type max_size() const {
        return std::numeric_limits<size_type>::max();
    }

    inline void release_memory() {
        mAllocator.Reset();
    }

    static inline LLVMAllocator & get_allocator() {
        return mAllocator;
    }

    inline bool operator==(SlabAllocator<T> const&) { return true; }
    inline bool operator!=(SlabAllocator<T> const&) { return false; }

    inline SlabAllocator() {}
    inline SlabAllocator(const SlabAllocator &) {}
    inline ~SlabAllocator() { release_memory(); }

private:
    static LLVMAllocator mAllocator;
};

template <typename T>
llvm::BumpPtrAllocator SlabAllocator<T>::mAllocator;

#endif // SLAB_ALLOCATOR_H
