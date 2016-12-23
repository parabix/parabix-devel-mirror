#ifndef ALIGNED_ALLOCATOR_H
#define ALIGNED_ALLOCATOR_H

#include <boost/version.hpp>
#if (BOOST_VERSION >= 105600)
#include <boost/align/aligned_allocator.hpp>
template<typename T, unsigned alignment>
using AlignedAllocator = boost::alignment::aligned_allocator<T, alignment>;
#else
#include <stdlib.h>
template <typename T, unsigned alignment>
class AlignedAllocator {
    static_assert ((alignment & (alignment - 1)) == 0, "alignment must be power of two");
public:
    typedef T         value_type;
    typedef T*        pointer;
    typedef const T*  const_pointer;
    typedef T&        reference;
    typedef const T&  const_reference;
    typedef size_t    size_type;
    typedef ptrdiff_t difference_type;

    template <class U>
    struct rebind { typedef AlignedAllocator<U, alignment> other; };

public:
    AlignedAllocator() noexcept {}

    pointer allocate(const size_type n, const_pointer = 0) {
        const auto size = n * sizeof(T);
        if (size == 0) {
            return nullptr;
        }
        void * ptr = nullptr;
        if (LLVM_UNLIKELY(::posix_memalign(&ptr, alignment, size) != 0)) {
            throw std::bad_alloc();
        }
        assert (ptr && (reinterpret_cast<size_t>(ptr) & (alignment - 1)) == 0);
        return reinterpret_cast<pointer>(ptr);
    }

    template<typename U, unsigned align>
    inline bool operator==(const AlignedAllocator<U, align> &) const {
        return false;
    }

    template<typename U, unsigned align>
    inline bool operator!=(const AlignedAllocator<U, align> &) const {
        return true;
    }

    void deallocate(pointer p, size_type) noexcept {
        return ::free(p);
    }
};
#endif
#endif // ALIGNED_ALLOCATOR_H
