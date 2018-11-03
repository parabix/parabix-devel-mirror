/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_VALIDATION_H
#define RE_VALIDATION_H

#include <string>
namespace re { class RE; class CC;}
namespace cc { class Alphabet;}
namespace re { class Name; class Start; class End; class CC; class Seq; class Alt;
    class Rep; class Intersect; class Diff; class Range; class Group;
    class Assertion;
}

namespace re {
/* Check that all names within an RE are defined. */
bool validateNamesDefined(RE * r);

/* Check that all CCs within an RE have the given Alphabet */
bool validateAlphabet(const cc::Alphabet * a, RE * r);

/* Check that the RE is free of zero-width assertions */
bool validateAssertionFree(RE * r);

/* A generic visitor for validation tasks.   The generic routines
   traverse the AST attempting validation at each RE node, returning
   false immediately if the validation fails anywhere.   By default,
   validation requires that Names are defined and that the definitions
   satisfy the validation requirement, but this may be overridden. */

class RE_Validator {
public:
    bool validateRE(RE * r);
    RE_Validator(std::string name = "") : mValidatorName(name) {}
    virtual ~RE_Validator() {}
protected:
    bool validate(RE * r);
    virtual bool validateName(Name * n);
    virtual bool validateStart(Start * s);
    virtual bool validateEnd(End * e);
    virtual bool validateCC(CC * cc);
    virtual bool validateSeq(Seq * s);
    virtual bool validateAlt(Alt * a);
    virtual bool validateRep(Rep * rep);
    virtual bool validateIntersect(Intersect * e);
    virtual bool validateDiff(Diff * d);
    virtual bool validateRange(Range * rg);
    virtual bool validateGroup(Group * g);
    virtual bool validateAssertion(Assertion * a);
private:
    std::string mValidatorName;
};
    
bool validateAlphabet(const cc::Alphabet * a, RE * r);

}
#endif
