/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef PIPELINE_H
#define PIPELINE_H

#include <IDISA/idisa_builder.h>
#include <kernels/interface.h>
#include <kernels/kernel.h>

void generateSegmentParallelPipeline(IDISA::IDISA_Builder * iBuilder, std::vector<kernel::KernelBuilder *> kernels);

void generatePipelineLoop(IDISA::IDISA_Builder * iBuilder, std::vector<kernel::KernelBuilder *> kernels);

void generatePipelineParallel(IDISA::IDISA_Builder * iBuilder, std::vector<kernel::KernelBuilder *> kernels);

#endif // PIPELINE_H
