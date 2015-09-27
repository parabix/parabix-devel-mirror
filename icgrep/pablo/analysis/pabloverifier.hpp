#ifndef PABLOVERIFIER_HPP
#define PABLOVERIFIER_HPP

#include <string>

namespace pablo {

class PabloFunction;

class PabloVerifier {
public:
    static void verify(const PabloFunction & function, const std::string location = "");
};

}

#endif // PABLOVERIFIER_HPP
