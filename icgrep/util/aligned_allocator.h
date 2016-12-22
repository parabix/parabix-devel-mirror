#ifndef ALIGNED_ALLOCATOR_H
#define ALIGNED_ALLOCATOR_H

#include <boost/version.hpp>
#if BOOST_VERSION >= 105600
#include <boost/align/aligned_allocator.hpp>
template<typename T, unsigned Align>
using AlignedAllocator = boost::alignment::aligned_allocator<T, Align>;
#else

template <typename T, unsigned Align>
class AlignedAllocator
{
public:
    typedef T         value_type;
    typedef T*        pointer;
    typedef const T*  const_pointer;
    typedef T&        reference;
    typedef const T&  const_reference;
    typedef size_t    size_type;
    typedef ptrdiff_t difference_type;

    template <class U>
    struct rebind { typedef AlignedAllocator<U, Align> other; };

public:
    AlignedAllocator() noexcept {}

    pointer allocate(size_type n, const_pointer = 0) {
        const auto size = n * sizeof(T);
        if (size == 0) {
            return nullptr;
        }
        void * ptr = nullptr;
        int rc = posix_memalign(&ptr, Align, size);
        if (rc != 0) {
            throw std::bad_alloc();
        }
        return reinterpret_cast<pointer>(ptr);
    }

    template<typename Type, unsigned align>
    inline bool operator==(AlignedAllocator<Type, align> const &) {
        return false;
    }

    template<typename Type, unsigned align>
    inline bool operator!=(AlignedAllocator<Type, align> const &) {
        return true;
    }


    void deallocate(pointer p, size_type) noexcept {
        return free(p);
    }
};

#endif



#endif // ALIGNED_ALLOCATOR_H
