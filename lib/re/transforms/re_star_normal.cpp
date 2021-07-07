#include <re/transforms/re_star_normal.h>
#include <re/transforms/re_transformer.h>
#include <re/adt/adt.h>
#include <re/analysis/nullable.h>
#include <llvm/ADT/SmallVector.h>

using namespace llvm;

namespace re {

class RE_Star_Normal final : public RE_Transformer {
public:
    RE_Star_Normal() : RE_Transformer("StarNormal") {}
    RE * transformRep(Rep * rep) override;
private:
    RE * star_rule(RE * re);
private:
    SmallVector<RE *, 16> mList;
};

inline RE * RE_Star_Normal::star_rule(RE * re) {
    if (Seq * seq = dyn_cast<Seq>(re)) {
        if (isNullable(re)) {
            mList.clear();
            mList.reserve(seq->size());
            for (RE * r : *seq) {
                if (Rep * rep = dyn_cast<Rep>(r)) {
                    if (rep->getLB() == 0) {
                        mList.push_back(rep->getRE());
                    }
                } else if (!isEmptySeq(r)) {
                    mList.push_back(r);
                }
            }
            return makeAlt(mList.begin(), mList.end());
        }
    }
    return re;
}

RE * RE_Star_Normal::transformRep(Rep * rep) {
    RE * e0 = rep->getRE();
    RE * e = transform(e0);
    if (rep->getLB() == 0 && rep->getUB() == Rep::UNBOUNDED_REP) {
        e = star_rule(e);
    }
    if (e == e0) return rep;
    return makeRep(e, rep->getLB(), rep->getUB());
}

RE * convertToStarNormalForm(RE * re) {
    return RE_Star_Normal().transformRE(re);
}

}
