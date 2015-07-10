#ifndef FUNCTION_H
#define FUNCTION_H

#include <pablo/pabloAST.h>

namespace pablo {

class Var;
class Assign;

enum class ArgumentType {
    In = 0
    , Out = 1
    , InOut = 2
};

using Argument = std::pair<ArgumentType, PabloAST *>;

class Function : public PabloAST {
    using ArgumentListAllocator = VectorAllocator::rebind<Argument>::other;
    using ArgumentListType = std::vector<Argument, ArgumentListAllocator>;
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Function;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Function() { }
    virtual bool operator==(const PabloAST & other) const {
        return &other == this;
    }
protected:
    Function();
    Function(std::initializer_list<Argument *> args);
    Function(const std::vector<Argument *> & args);
private:
    ArgumentListType        mArgumentList;
};

}

#endif // FUNCTION_H
