#ifndef SLAB_ALLOCATOR_H
#define SLAB_ALLOCATOR_H

#include <llvm/Support/Allocator.h>

template <typename T>
class SlabAllocator {
    using LLVMAllocator = llvm::BumpPtrAllocator;
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

    template<typename Type = T>
    inline Type * allocate(size_type n, const_pointer = nullptr) noexcept {
        static_assert(sizeof(Type) > 0, "Cannot allocate a zero-length type.");
        assert ("Cannot allocate 0 items." && n > 0);
        auto ptr = static_cast<Type *>(mAllocator.Allocate(n * sizeof(Type), sizeof(void*)));
        assert ("Allocating returned a null pointer. Function was likely called before Allocator creation!" && ptr);
        return ptr;
    }

    template<typename Type = T>
    inline void deallocate(Type * p, size_type = 0) noexcept {
        mAllocator.Deallocate(p);
    }

    inline size_type max_size() const {
        return std::numeric_limits<size_type>::max();
    }

    inline LLVMAllocator & get_allocator() {
        return mAllocator;
    }

    void reset() {
        mAllocator.Reset();
    }

    template<typename Type = T>
    inline bool operator==(SlabAllocator<Type> const & other) {
        return &mAllocator == &other.mAllocator;
    }

    template<typename Type = T>
    inline bool operator!=(SlabAllocator<Type> const & other) {
        return &mAllocator != &other.mAllocator;
    }

    inline SlabAllocator() noexcept {}
    inline SlabAllocator(const SlabAllocator &) noexcept {}
    template <class U> inline SlabAllocator (const std::allocator<U>&) noexcept {}
    inline ~SlabAllocator() { reset(); }
private:
    LLVMAllocator mAllocator;
};

#endif // SLAB_ALLOCATOR_H
