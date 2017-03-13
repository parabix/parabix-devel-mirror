/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CARRY_DATA_H
#define CARRY_DATA_H

/* 
 * Carry Data system.
 * 
 * Each PabloBlock (Main, If, While) has a contiguous data area for carry information.
 * The data area may be at a fixed or variable base offset from the base of the 
 * main function carry data area.
 * The data area for each block consists of contiguous space for the local carries and 
 * advances of the block plus the areas of any ifs/whiles nested within the block.

*/

namespace pablo {

class CarryData {
    friend class CarryManager;
public:

    enum SummaryType : int {
        NoSummary
        , ImplicitSummary
        , BorrowedSummary
        , ExplicitSummary
        , CountingSummary
    };

    CarryData()
    : mSummaryType(NoSummary) {

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

    bool hasCountingSummary() const {
        return (mSummaryType == CountingSummary);
    }

    void setSummaryType(const SummaryType value) {
        mSummaryType = value;
    }
    
private:

    SummaryType             mSummaryType;

};


}


#endif // CARRY_DATA_H
