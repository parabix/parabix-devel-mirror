#ifndef PABLO_CODESINKING_HPP
#define PABLO_CODESINKING_HPP

namespace pablo {

class PabloKernel;
class PabloBlock;
class Statement;
class While;
class Variadic;

class CodeMotionPass {
public:
    static bool optimize(PabloKernel * kernel);
protected:
    static void movement(PabloBlock * const block);
    static void sink(PabloBlock * const block);
    static void hoistLoopInvariants(While * loop);
};

}

#endif // PABLO_CODESINKING_HPP
