/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CARRY_DATA_H
#define CARRY_DATA_H

namespace pablo {

class CarryData {
    friend class CarryManager;
public:

    enum SummaryType : int {
        NoSummary
        , ImplicitSummary
        , BorrowedSummary
        , ExplicitSummary
    };

    CarryData()
    : mSummaryType(NoSummary)
    , mInNonCollapsingCarryMode(false) {

    }
             
    bool hasSummary() const {
        return (mSummaryType != NoSummary);
    }
    
    bool hasImplicitSummary() const {
        return (mSummaryType == ImplicitSummary);
    }

    bool hasBorrowedSummary() const {
        return (mSummaryType == BorrowedSummary);
    }

    bool hasExplicitSummary() const {
        return (mSummaryType == ExplicitSummary);
    }

    bool nonCarryCollapsingMode() const {
        return mInNonCollapsingCarryMode;
    }

    void setSummaryType(const SummaryType value) {
        mSummaryType = value;
    }

    void setNonCollapsingCarryMode(const bool value = true) {
        mInNonCollapsingCarryMode = value;
    }
    
private:

    SummaryType     mSummaryType;
    bool            mInNonCollapsingCarryMode;

};


}


#endif // CARRY_DATA_H
