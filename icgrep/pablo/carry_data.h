/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CARRY_DATA_H
#define CARRY_DATA_H

namespace pablo {

class CarryData {
public:

    enum SummaryType : unsigned {
        NoSummary = 0
        , ImplicitSummary = 1
        , BorrowedSummary = 2
        , ExplicitSummary = 3
        , NonCarryCollapsingMode = 4
    };

    CarryData()
    : mSummaryType(NoSummary) {

    }
             
    bool hasSummary() const {
        return (mSummaryType & (ImplicitSummary | BorrowedSummary | ExplicitSummary)) != NoSummary;
    }
    
    bool hasImplicitSummary() const {
        return (mSummaryType & (ImplicitSummary | BorrowedSummary | ExplicitSummary)) == ImplicitSummary;
    }

    bool hasBorrowedSummary() const {
        return (mSummaryType & (ImplicitSummary | BorrowedSummary | ExplicitSummary)) == BorrowedSummary;
    }

    bool hasExplicitSummary() const {
        return (mSummaryType & (ImplicitSummary | BorrowedSummary | ExplicitSummary)) == ExplicitSummary;
    }

    bool nonCarryCollapsingMode() const {
        return (mSummaryType & (NonCarryCollapsingMode)) != 0;
    }

    void setSummaryType(const SummaryType value) {
        mSummaryType = value;
    }

    void setNonCollapsingCarryMode(const bool value = true) {
        if (value) {
            mSummaryType = (SummaryType)(mSummaryType | NonCarryCollapsingMode);
        } else {
            mSummaryType = (SummaryType)(mSummaryType & ~NonCarryCollapsingMode);
        }
    }
    
private:

    SummaryType     mSummaryType;

};


}


#endif // CARRY_DATA_H
