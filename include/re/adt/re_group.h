#ifndef RE_GROUP_H
#define RE_GROUP_H

#include <llvm/Support/Casting.h>
#include <re/adt/re_re.h>

namespace re {
    
/*  Grouped regular expression to represent mode flags.
    (?g:...)  or (?i)....
*/

class Group : public RE {
public:
    enum class Mode {CaseInsensitiveMode, GraphemeMode, CompatibilityMode};
    enum class Sense {On, Off};
    Mode getMode() const {return mMode;}
    Sense getSense() const {return mSense;}
    RE * getRE() const {return mExpr;}
    static Group * Create(Mode m, RE * r, Sense s) {return new Group(m, r, s);}
    RE_SUBTYPE(Group)
protected:
    Group(Mode m, RE * r, Sense s) : RE(ClassTypeId::Group), mMode(m), mExpr(r), mSense(s) {}

private:
    Mode                mMode;
    RE *                mExpr;
    Sense               mSense;
};

inline Group * makeGroup(Group::Mode m, RE * r, Group::Sense s = Group::Sense::On) {
    return Group::Create(m, r, s);
}
}

#endif // RE_GROUP_H
