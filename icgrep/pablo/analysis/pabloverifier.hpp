#ifndef PABLOVERIFIER_HPP
#define PABLOVERIFIER_HPP

namespace pablo {

class PabloFunction;

class PabloVerifier {
public:
    static void verify(const PabloFunction & function, const bool ignoreUnusedStatements = true);
private:

};

}

#endif // PABLOVERIFIER_HPP
