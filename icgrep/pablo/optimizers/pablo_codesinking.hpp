#ifndef PABLO_CODESINKING_HPP
#define PABLO_CODESINKING_HPP

#include <pablo/codegenstate.h>

namespace pablo {

class CodeSinking
{
public:
    static bool optimize(PabloBlock & block);
protected:
    static void sink(PabloBlock & block);
    CodeSinking();
};

}

#endif // PABLO_CODESINKING_HPP
