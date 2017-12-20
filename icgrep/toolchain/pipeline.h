/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef PIPELINE_H
#define PIPELINE_H

#include <vector>
#include <memory>

namespace kernel { class Kernel; }
namespace kernel { class KernelBuilder; }

void generateSegmentParallelPipeline(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const std::vector<kernel::Kernel *> & kernels);
void generatePipelineLoop(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const std::vector<kernel::Kernel *> & kernels);

#endif // PIPELINE_H
