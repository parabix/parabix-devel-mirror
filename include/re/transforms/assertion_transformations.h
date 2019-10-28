/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

namespace re {

class RE;

RE * expandBoundaryAssertion(RE * r);
    
RE * lookaheadPromotion(RE * r);

}
