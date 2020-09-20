#ifndef NOT_NULL_H
#define NOT_NULL_H

template<typename T>
struct not_null {
    not_null(T const value) : _value(value) { assert(_value); }
    not_null(std::nullptr_t) = delete;
    not_null(unsigned) = delete;
    operator T() const { return _value; }
    T operator-> () const { return _value; }
    T get() const { return _value; }
private:
    T const  _value;
};

template<typename T>
struct no_conversion {
    no_conversion(T const value) : _value(value) { }
    template <typename U> no_conversion(U) = delete;
    operator T() const { return _value; }
    T get() const { return _value; }
private:
    T const  _value;
};


#endif // NOT_NULL_H
