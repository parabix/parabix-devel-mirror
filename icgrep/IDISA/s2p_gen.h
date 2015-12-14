/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#ifndef S2P_GEN_H
#define S2P_GEN_H

#include <stdio.h>
#include "idisa_builder.h"

void * s2p_step(DISA_Builder * iBuilder,
                Value * s0, Value * s1, Value * hi_mask, unsigned shift, Value * p0, Value * p1);

void * s2p(DISA_Builder * iBuilder,
           Value * s0, Value * s1, Value * s2, Value * s3, Value * s4, Value * s5, Value * s6, Value * s7,
           Value * p0, Value * p1, Value * p2, Value * p3, Value * p4, Value * p5, Value * p6, Value * p7);

#endif

