#ifndef PE_STRING_H
#define PE_STRING_H

#include <pablo/pabloAST.h>
#include <llvm/ADT/StringRef.h>

namespace pablo {

class String : public PabloAST {
    friend class SymbolGenerator;
public:
    using StringAllocator = SlabAllocator<char>;

    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::String;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~String(){

    }
    inline const llvm::StringRef & value() const {
        return mValue;
    }
    inline std::string to_string() const {
        return mValue.str();
    }
    inline llvm::StringRef value() {
        return mValue;
    }
protected:
    String(llvm::Type * type, const std::string & value, Allocator & allocator) noexcept
    : PabloAST(ClassTypeId::String, type, this, allocator)
    , mValue(duplicate(value, allocator)) {

    }

    inline const char * duplicate(const std::string & value, Allocator & allocator) {
        char * string = reinterpret_cast<char*>(allocator.allocate<char*>(value.length() + 1));
        std::memcpy(string, value.c_str(), value.length());
        string[value.length()] = '\0';
        return string;
    }
private:
    const llvm::StringRef mValue;
};

}

#endif // PE_STRING_H
