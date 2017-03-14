/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef PIPELINE_H
#define PIPELINE_H

#include <vector>
namespace IDISA { class IDISA_Builder; }
namespace kernel { class KernelBuilder; }

void generateSegmentParallelPipeline(IDISA::IDISA_Builder * iBuilder, const std::vector<kernel::KernelBuilder *> & kernels);

void generatePipelineLoop(IDISA::IDISA_Builder * iBuilder, const std::vector<kernel::KernelBuilder *> & kernels);

void generateParallelPipeline(IDISA::IDISA_Builder * iBuilder, const std::vector<kernel::KernelBuilder *> & kernels);

#endif // PIPELINE_H
