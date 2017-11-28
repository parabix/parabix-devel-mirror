/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef MULTIPLEX_CCS_H
#define MULTIPLEX_CCS_H

#include <vector>

namespace re { class CC; }


void doMultiplexCCs(const std::vector<const re::CC *> & CCs,
                    std::vector<std::vector<unsigned>> & exclusiveSetIDs,
                    std::vector<re::CC *> & multiplexedCCs);

#endif
