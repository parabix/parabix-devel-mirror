#ifndef PE_STRING_H
#define PE_STRING_H

#include <pablo/pabloAST.h>
#include <llvm/ADT/StringRef.h>

namespace pablo {

class String : public PabloAST, public llvm::StringRef {
    friend class SymbolGenerator;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::String;
    }
    static inline bool classof(const void *) {
        return false;
    } 
    virtual ~String() { }
protected:
    String(llvm::Type * type, const llvm::StringRef str, Allocator & allocator) noexcept
    : PabloAST(ClassTypeId::String, type,  allocator)
    , llvm::StringRef(str.data(), str.size()) {

    }
};

}

#endif // PE_STRING_H
