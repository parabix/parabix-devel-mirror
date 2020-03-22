#ifndef PIPELINE_KERNEL_ANALYSIS_HPP
#define PIPELINE_KERNEL_ANALYSIS_HPP

#include "../common/common.hpp"
#include <algorithm>
#include <queue>
#include <z3.h>
#include <util/maxsat.hpp>
#include <assert.h>
#include <kernel/core/streamset.h>
#include <kernel/core/kernel_builder.h>

#define PRINT_BUFFER_GRAPH

namespace kernel {

struct PipelineAnalysis : public PipelineCommonGraphFunctions {


public:

    static PipelineAnalysis analyze(BuilderRef b, PipelineKernel * const pipelineKernel) {

        PipelineAnalysis P(pipelineKernel);

        P.generateInitialPipelineGraph(b);

        // Add ordering constraints to ensure we can keep sequences of kernels with a fixed rates in
        // the same sequence. This will help us to partition the graph later and is useful to determine
        // whether we can bypass a region without testing every kernel.

        P.partitionRelationshipGraphIntoSynchronousRegions();

        for (;;) {

            // Construct the Stream and Scalar graphs
            P.transcribeRelationshipGraph();
            P.generateInitialBufferGraph();

            P.computeDataFlowRates();
            P.generatePartitioningGraph();

            if (LLVM_LIKELY(P.determinePartitionJumpIndices())) {
                break;
            }

            // If we get here, the partition jump tree was degenerate.
            // We need to modify the relation ship graph
            P.addPartitionJumpConstraintsToRelationshipGraph();

        }

        P.annotateBufferGraphWithAddAttributes();

        // Finish annotating the buffer graph       
        P.identifyLocalPortIds();
        P.identifyLinearBuffers();
        P.identifyZeroExtendedStreamSets();

        // Make the remaining graphs
        P.makeConsumerGraph();
        P.makeTerminationGraph();
        P.makePartitionJumpTree();
        P.makeKernelIOGraph();

        // Finish the buffer graph
        P.addStreamSetsToBufferGraph(b);

        P.makeInputTruncationGraph();

        #ifdef PRINT_BUFFER_GRAPH
        P.printBufferGraph(errs());
        #endif

        return P;
    }

private:

    // constructor

    PipelineAnalysis(PipelineKernel * const pipelineKernel)
    : PipelineCommonGraphFunctions(mStreamGraph, mBufferGraph)
    , mPipelineKernel(pipelineKernel)
    , mNumOfThreads(pipelineKernel->getNumOfThreads())
    , mLengthAssertions(pipelineKernel->getLengthAssertions()) {

    }

    // pipeline analysis functions

    using KernelPartitionIds = flat_map<Relationships::vertex_descriptor, unsigned>;

    void generateInitialPipelineGraph(BuilderRef b);

    using KernelVertexVec = SmallVector<Relationships::Vertex, 64>;

    void addRegionSelectorKernels(BuilderRef b, Kernels & partition, KernelVertexVec & vertex, Relationships & G);

    void addPopCountKernels(BuilderRef b, Kernels & partition, KernelVertexVec & vertex, Relationships & G);

    void combineDuplicateKernels(BuilderRef b, const Kernels & partition, Relationships & G);

    void removeUnusedKernels(const unsigned p_in, const unsigned p_out, const Kernels & partition, Relationships & G);

    void identifyPipelineInputs();

    void transcribeRelationshipGraph();

    // partitioning analysis

    void partitionRelationshipGraphIntoSynchronousRegions();

    void identifyKernelPartitions(const std::vector<unsigned> & orderingOfG);

    void addOrderingConstraintsToPartitionSubgraphs(const std::vector<unsigned> & orderingOfG);

    void generatePartitioningGraph();

    bool determinePartitionJumpIndices();

    void addPartitionJumpConstraintsToRelationshipGraph();

    void makePartitionJumpTree();

    // buffer management analysis functions

    void addStreamSetsToBufferGraph(BuilderRef b);
    void generateInitialBufferGraph();
    void identifyLinearBuffers();
    void identifyLocalPortIds();

    // consumer analysis functions

    void makeConsumerGraph();

    // dataflow analysis functions

    void computeDataFlowRates();
    PartitionConstraintGraph identifyHardPartitionConstraints(BufferGraph & G) const;
    LengthConstraintGraph identifyLengthEqualityAssertions(BufferGraph & G) const;

    // zero extension analysis function

    void identifyZeroExtendedStreamSets();

    // termination analysis functions

    void makeTerminationGraph();

    // add(k) analysis functions

    void annotateBufferGraphWithAddAttributes();

    // IO analysis functions

    void makeKernelIOGraph();

    // Input truncation analysis functions

    void makeInputTruncationGraph();

    // Debug functions

    void printBufferGraph(raw_ostream & out) const;

private:

    PipelineKernel * const          mPipelineKernel;
    const unsigned					mNumOfThreads;
    const LengthAssertions &        mLengthAssertions;
    Relationships                   mRelationships;
    KernelPartitionIds              mPartitionIds;

public:

    static constexpr unsigned       PipelineInput = 0U;
    static constexpr unsigned       FirstKernel = 1U;
    unsigned                        LastKernel = 0;
    unsigned                        PipelineOutput = 0;
    unsigned                        FirstStreamSet = 0;
    unsigned                        LastStreamSet = 0;
    unsigned                        FirstBinding = 0;
    unsigned                        LastBinding = 0;
    unsigned                        FirstCall = 0;
    unsigned                        LastCall = 0;
    unsigned                        FirstScalar = 0;
    unsigned                        LastScalar = 0;
    unsigned                        PartitionCount = 0;
    bool                            HasZeroExtendedStream = false;

    RelationshipGraph               mStreamGraph;
    RelationshipGraph               mScalarGraph;

    Partition                       KernelPartitionId;

    std::vector<Rational>           MinimumNumOfStrides;
    std::vector<Rational>           MaximumNumOfStrides;

    BufferGraph                     mBufferGraph;
    PartitioningGraph               mPartitioningGraph;
    std::vector<unsigned>           mPartitionJumpIndex;
    PartitionJumpTree               mPartitionJumpTree;


    bool                            mHasZeroExtendedStream;
    bool                            mHasThreadLocalPipelineState;

    ConsumerGraph                   mConsumerGraph;

    TerminationGraph                mTerminationGraph;    
    InputTruncationGraph            mInputTruncationGraph;
    IOCheckGraph                    mIOCheckGraph;


    OwningVector<Kernel>            mInternalKernels;
    OwningVector<Binding>           mInternalBindings;
    OwningVec<StreamSetBuffer>      mInternalBuffers;
};

}

#include "pipeline_graph_printers.hpp"
#include "relationship_analysis.hpp"
#include "buffer_analysis.hpp"
#include "consumer_analysis.hpp"
#include "dataflow_analysis.hpp"
#include "partitioning_analysis.hpp"
#include "termination_analysis.hpp"
#include "zero_extend_analysis.hpp"
#include "input_truncation_analysis.hpp"
#include "add_analysis.hpp"
#include "io_analysis.hpp"

#endif
