/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_VALIDATION_H
#define RE_VALIDATION_H

#include <string>
#include <re/adt/adt_forward_decl.h>

namespace cc { class Alphabet;}

namespace re {
/* Check that all names within an RE are defined. */
bool validateNamesDefined(const RE * r);

/* Check that all CCs within an RE have the given Alphabet */
bool validateAlphabet(const cc::Alphabet * a, const RE * r);

/* Check that the RE is free of zero-width assertions */
bool validateAssertionFree(const RE * r);

/* A generic visitor for validation tasks.   The generic routines
   traverse the AST attempting validation at each RE node, returning
   false immediately if the validation fails anywhere.   By default,
   validation requires that Names are defined and that the definitions
   satisfy the validation requirement, but this may be overridden. */

class RE_Validator {
public:
    bool validateRE(const RE * r);
    RE_Validator(std::string name = "") : mValidatorName(name) {}
    virtual ~RE_Validator() {}
protected:
    bool validate(const RE * r);
    virtual bool validateName(const Name * n);
    virtual bool validateStart(const Start * s);
    virtual bool validateEnd(const End * e);
    virtual bool validateCC(const CC * cc);
    virtual bool validateSeq(const Seq * s);
    virtual bool validateAlt(const Alt * a);
    virtual bool validateRep(const Rep * rep);
    virtual bool validateIntersect(const Intersect * e);
    virtual bool validateDiff(const Diff * d);
    virtual bool validateRange(const Range * rg);
    virtual bool validateGroup(const Group * g);
    virtual bool validateAssertion(const Assertion * a);
    virtual bool validateCapture(const Capture * c);
    virtual bool validateReference(const Reference * r);
private:
    std::string mValidatorName;
};

}
#endif
