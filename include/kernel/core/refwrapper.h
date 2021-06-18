#ifndef REFWRAPPER_H
#define REFWRAPPER_H

#include <assert.h>

namespace kernel {

// NOTE: std::reference_wrapper is not default constructible, which is required by boost graph
template <typename T>
struct RefWrapper {
    RefWrapper() noexcept : mReference(nullptr) {}
    RefWrapper(const T & ref) noexcept : mReference(const_cast<T*>(&ref)) {}
    RefWrapper(const T * const ref) noexcept : mReference(const_cast<T*>(ref)) {}
    operator const T & () const noexcept {
        return get();
    }
    operator T & () noexcept {
        return get();
    }
    const T & get() const noexcept {
        assert (mReference && "was not set!");
        return *mReference;
    }
    T & get() noexcept {
        assert (mReference && "was not set!");
        return *mReference;
    }
private:
    T * mReference;
};

template <typename T>
constexpr inline bool operator< (const RefWrapper<T> & a, const RefWrapper<T> & b) {
    return a.get() < b.get();
}

}

#endif // REFWRAPPER_H
