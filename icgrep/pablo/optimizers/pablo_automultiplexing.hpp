#ifndef PABLO_AUTOMULTIPLEXING_HPP
#define PABLO_AUTOMULTIPLEXING_HPP

#include <pablo/codegenstate.h>

namespace pablo {

class AutoMultiplexing
{
public:
    static void optimize(PabloBlock & block);



private:
    AutoMultiplexing();
};

}

#endif // PABLO_AUTOMULTIPLEXING_HPP
