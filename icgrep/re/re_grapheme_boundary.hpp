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
        ClusterBoundary = 0     // g
        , WordBoundary = 1      // w
        , LineBreakBoundary = 2 // l
        , SentenceBoundary = 3  // s
    };

    enum class Sense {Positive, Negative};

    inline Type getType() const { return mType; }
    GraphemeBoundary::Sense getSense() const {return mSense;}
    inline RE * getExpression() const {return mExpression;}
    inline void setExpression(RE * const r) { mExpression = r; }
    inline Name * getBoundaryRule() const {return mBoundaryRule;}
    inline void setBoundaryRule(Name * const r) { mBoundaryRule = r; }

protected:
    friend GraphemeBoundary * makeGraphemeBoundary(const Type type, const Sense sense, RE * expression);
    GraphemeBoundary(const Type type, const Sense sense, RE * expression) : RE(ClassTypeId::GraphemeBoundary), mType(type), mSense(sense), mExpression(expression), mBoundaryRule(nullptr) {}
    virtual ~GraphemeBoundary() {}

private:
    const Type      mType;
    const Sense     mSense;
    RE *            mExpression;
    Name *          mBoundaryRule;
};

inline GraphemeBoundary * makeGraphemeBoundary(const GraphemeBoundary::Type type, const GraphemeBoundary::Sense sense, RE * expression) {
    return new GraphemeBoundary(type, sense, expression);
}

inline GraphemeBoundary * makeGraphemeClusterBoundary(const GraphemeBoundary::Sense sense = GraphemeBoundary::Sense::Positive, RE * expression = nullptr) {
    return makeGraphemeBoundary(GraphemeBoundary::Type::ClusterBoundary, sense, expression);
}

}

#endif // RE_GRAPHEME_BOUNDARY_HPP
