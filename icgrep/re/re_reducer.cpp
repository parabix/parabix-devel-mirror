#include "re_reducer.h"
#include "re_cc.h"
#include "re_name.h"
#include "re_start.h"
#include "re_end.h"
#include "re_seq.h"
#include "re_alt.h"
#include "re_rep.h"
#include <assert.h>

namespace re {

RE * RE_Reducer::reduce(RE * re, std::map<std::string, RE*>& re_map) {
    RE * retVal = re;
    assert (re);
    if (Alt * alt = dyn_cast<Alt>(re)) {
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            *i = reduce(*i, re_map);
        }
    }
    else if (Seq * seq = dyn_cast<Seq>(re)) {
        for (auto i = seq->begin(); i != seq->end(); ++i) {
            *i = reduce(*i, re_map);
        }
        if (seq->getType() == Seq::Type::Byte) {
            //If this is a sequence of byte classes then this is a multibyte sequence for a Unicode character class.
            std::string seqname = seq->getName();
            re_map.insert(make_pair(seqname, seq));
            retVal = makeName(seqname, false, Name::Type::Unicode);
        }
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        rep->setRE(reduce(rep->getRE(), re_map));
    }
    else if (CC * cc = dyn_cast<CC>(re)) {
        std::string ccname = cc->getName();
        //If the character class isn't in the map then add it.
        re_map.insert(make_pair(ccname, cc));
        //return a new name class with the name of the character class.
        retVal = makeName(ccname);
    }
    return retVal;
}

}
