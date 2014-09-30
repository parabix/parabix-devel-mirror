#ifndef PE_CALL_H
#define PE_CALL_H

#include "pe_pabloe.h"
#include <string>

class Call : public PabloE
{
public:
    Call(std::string callee);
    ~Call();
    std::string getCallee();
private:
    std::string mCallee;
};

#endif // PE_CALL_H


