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



#endif // PTRWRAPPER_HPP
