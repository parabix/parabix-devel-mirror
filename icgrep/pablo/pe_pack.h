#ifndef PE_PACK_H
#define PE_PACK_H

#include <pablo/pabloAST.h>

namespace pablo {

class Integer;

class PackH final : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::PackH;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~PackH() {
    }
    inline Integer * getFieldWidth() const {
        return llvm::cast<Integer>(getOperand(0));
    }
    inline PabloAST * getValue() const {
        return getOperand(1);
    }
protected:
    PackH(Integer * width, PabloAST * const value, const String * name, llvm::Type * type, Allocator & allocator)
    : Statement(ClassTypeId::PackH, type, { width, value }, name, allocator) {

    }
};

class PackL final : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::PackL;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~PackL() {
    }
    inline Integer * getFieldWidth() const {
        return llvm::cast<Integer>(getOperand(0));
    }
    inline PabloAST * getValue() const {
        return getOperand(1);
    }
protected:
    PackL(Integer * width, PabloAST * const value, const String * name, llvm::Type * type, Allocator & allocator)
    : Statement(ClassTypeId::PackL, type, { width, value }, name, allocator) {

    }
};


}

#endif // PE_PACK_H
