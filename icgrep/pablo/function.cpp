#include "function.h"

namespace pablo {

Function::Function(std::initializer_list<Argument> args)
: PabloAST(ClassTypeId::Function)
, mArgumentList(args.begin(), args.end(), reinterpret_cast<ArgumentListAllocator &>(mVectorAllocator)) {
    for (Argument & arg : args) {
        std::get<1>(arg)->addUser(this);
    }
}

Function::Function(const std::vector<Argument> & args)
: PabloAST(ClassTypeId::Function)
, mArgumentList(args.begin(), args.end(), reinterpret_cast<ArgumentListAllocator &>(mVectorAllocator)) {
    for (Argument & arg : args) {
        std::get<1>(arg)->addUser(this);
    }
}


}
