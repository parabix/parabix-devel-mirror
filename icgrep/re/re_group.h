#ifndef RE_GROUP_H
#define RE_GROUP_H

#include <re/re_re.h>
#include <llvm/Support/Casting.h>

namespace re {
    
/*  Grouped regular expression to represent mode flags.
    (?g:...)  or (?i)....
*/

class Group : public RE {
public:
    static inline bool classof(const RE * re) {return re->getClassTypeId() == ClassTypeId::Group;}
    static inline bool classof(const void *) {return false;}
    
    enum class Mode {CaseInsensitiveMode, GraphemeMode, CompatibilityMode};
    enum class Sense {On, Off};
    Mode getMode() const {return mMode;}
    Sense getSense() const {return mSense;}
    RE * getRE() const {return mExpr;}
    friend Group * makeGroup(Mode m, RE * r, Sense s);
    void setRE(RE * r) {mExpr = r;}
protected:
    Group(Mode m, RE * r, Sense s) : RE(ClassTypeId::Group), mMode(m), mExpr(r), mSense(s) {}

private:
    Mode                mMode;
    RE *                mExpr;
    Sense               mSense;
};

inline Group * makeGroup(Group::Mode m, RE * r, Group::Sense s = Group::Sense::On) {
    return new Group(m, r, s);
}
}

#endif // RE_GROUP_H
