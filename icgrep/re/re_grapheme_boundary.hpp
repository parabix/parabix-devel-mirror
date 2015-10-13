#ifndef RE_GRAPHEME_BOUNDARY_HPP
#define RE_GRAPHEME_BOUNDARY_HPP

#include <re/re_re.h>

namespace re {

class GraphemeBoundary : public RE {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::GraphemeBoundary;
    }
    static inline bool classof(const void *) {
        return false;
    }

    enum class Type {
        ClusterBoundary     // g
        , WordBoundary      // w
        , LineBreakBoundary // l
        , SentenceBoundary  // s
    };

    inline RE * getExpression() const {return mExpression;}
    inline void setExpression(RE * const r) {mExpression = r; }
    inline Name * getGraphemeExtenderRule() const {return mBoundaryRule;}
    inline void setBoundaryRule(Name * const r) {mBoundaryRule = r; }
    inline Type getType() const {return mType;}

protected:
    friend GraphemeBoundary * makeGraphemeBoundary(RE * const expression, const Type type);
    GraphemeBoundary(RE * const expression, const Type type) : RE(ClassTypeId::GraphemeBoundary), mExpression(expression), mBoundaryRule(nullptr), mType(type) {}
    virtual ~GraphemeBoundary() {}

private:
    RE *        mExpression;
    Name *      mBoundaryRule;
    const Type  mType;
};

inline GraphemeBoundary * makeGraphemeBoundary(RE * const expression, const GraphemeBoundary::Type type) {
    return new GraphemeBoundary(expression, type);
}

}

#endif // RE_GRAPHEME_BOUNDARY_HPP
