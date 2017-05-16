#ifndef PABLO_CODESINKING_HPP
#define PABLO_CODESINKING_HPP

namespace pablo {
class PabloKernel;
class CodeMotionPass {
public:
    static bool optimize(PabloKernel * const kernel);
};
}

#endif // PABLO_CODESINKING_HPP
