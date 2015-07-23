#ifndef PE_STRING_H
#define PE_STRING_H

#include <pablo/pabloAST.h>
#include <llvm/ADT/StringRef.h>
#include <llvm/ADT/Twine.h>
#include <llvm/ADT/SmallVector.h>
#include <string>
#include <ostream>
#include <llvm/Support/raw_os_ostream.h>

namespace pablo {

class String : public PabloAST {
    friend class SymbolGenerator;
    friend class Prototype;
    friend std::ostream & operator<< (std::ostream& stream, const String & string);
public:
    using StringAllocator = SlabAllocator<char>;
    using Value = StringRef;

    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::String;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~String(){

    }
    inline const StringRef & value() const {
        return mValue;
    }
    inline std::string to_string() const {
        return mValue.str();
    }
    inline StringRef value() {
        return mValue;
    }
    inline bool isGenerated() const {
        return mGenerated;
    }
    inline bool isUserDefined() const {
        return !mGenerated;
    }
protected:
    String(const std::string & value, const bool generated) noexcept
    : PabloAST(ClassTypeId::String)
    , mValue(duplicate(value))
    , mGenerated(generated)
    {

    }
    inline const char * duplicate(const std::string & value) {
        char * string = reinterpret_cast<char*>(mAllocator.allocate(value.length() + 1));
        std::memcpy(string, value.c_str(), value.length());
        string[value.length()] = '\0';
        return string;
    }
private:
    const StringRef         mValue;
    const bool              mGenerated;
};

inline std::ostream & operator <<(std::ostream & stream, const String & string) {
    stream << string.value().data();
    return stream;
}

inline std::ostream & operator <<(std::ostream & stream, const String * string) {
    stream << string->value().data();
    return stream;
}

inline llvm::raw_ostream & operator <<(llvm::raw_ostream & stream, const String & string) {
    stream << string.value().data();
    return stream;
}

inline llvm::raw_ostream & operator <<(llvm::raw_ostream & stream, const String * string) {
    stream << string->value().data();
    return stream;
}


}

#endif // PE_STRING_H
