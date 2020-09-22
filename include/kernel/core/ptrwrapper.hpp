#ifndef PTRWRAPPER_HPP
#define PTRWRAPPER_HPP

#include <memory>
#include <assert.h>

template <typename T>
struct PtrWrapper {

    PtrWrapper(const std::unique_ptr<T> & p) noexcept : mReference(p.get()) {  }
    PtrWrapper(const PtrWrapper<T> & p) noexcept : mReference(p.get()) { }
    PtrWrapper(T * const ref) noexcept : mReference(ref) { }

    T * operator -> () const noexcept {
        return get();
    }
    T * get() const noexcept {
        assert (mReference && "was not set!");
        return mReference;
    }
private:
    T * const mReference;
};

template <typename T>
constexpr inline bool operator< (const PtrWrapper<T> & a, const PtrWrapper<T> & b) {
    return a.get() < b.get();
}


#endif // PTRWRAPPER_HPP
