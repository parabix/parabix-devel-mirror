#ifndef RE_STAR_NORMAL_H
#define RE_STAR_NORMAL_H

namespace re {

class RE;

//A regular expression E is in star normal form if, for each starred
//subexpression H * of E, the following SNF-conditions hold:
//1> The follow(H, last(H)) and first(H) are disjoint.
//2> H is not Nullable.
//
//For example: (a + b)* is the star normal form of (a*b*)* . 
//Both of them have the same Glushkov NFA.
//


class RE_Star_Normal {
public:
	static RE * star_normal(RE * re);
	static RE * helper(RE * re);
private:
	static bool isNullable(const RE * re);
};

}

#endif // RE_STAR_NORMAL_H
