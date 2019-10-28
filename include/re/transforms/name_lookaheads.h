/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef NAME_LOOKAHEADS_H
#define NAME_LOOKAHEADS_H

namespace re {
class RE;

/* Transform a regular expression r so that all names are
   created for all lookahead assertions. */
RE * name_lookaheads(RE * r);
}

#endif // NAME_LOOKAHEADS_H

