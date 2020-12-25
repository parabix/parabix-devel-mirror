#ifndef PIPELINE_KERNEL_ANALYSIS_HPP
#define PIPELINE_KERNEL_ANALYSIS_HPP

#include "../config.h"
#include "../common/common.hpp"
#include "../common/graphs.h"

#include <algorithm>
#include <queue>
#include <z3.h>
#include <util/maxsat.hpp>
#include <assert.h>

#include <kernel/core/kernel.h>
#include <kernel/core/relationship.h>
#include <kernel/core/streamset.h>
#include <kernel/core/kernel_builder.h>

#define EXPERIMENTAL_SCHEDULING

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

        // Construct the Stream and Scalar graphs
        P.transcribeRelationshipGraph();
        P.generateInitialBufferGraph();

        P.computeDataFlowRates();
        P.identifyTerminationChecks();
        P.determinePartitionJumpIndices();

        P.annotateBufferGraphWithAddAttributes();

        // Finish annotating the buffer graph       
        P.identifyLinearBuffers();
        P.identifyZeroExtendedStreamSets();
        P.identifyLocalPortIds();
        P.identifyLinearBuffers();

        // Make the remaining graphs
        P.makeConsumerGraph();        
        P.makePartitionJumpTree();
        P.makeTerminationPropagationGraph();

        // Finish the buffer graph
        // P.identifyDirectUpdatesToStateObjects();
        P.addStreamSetsToBufferGraph(b);

        P.gatherInfo();

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

    using KernelPartitionIds = flat_map<ProgramGraph::vertex_descriptor, unsigned>;

    void generateInitialPipelineGraph(BuilderRef b);

    using KernelVertexVec = SmallVector<ProgramGraph::Vertex, 64>;

    void addRegionSelectorKernels(BuilderRef b, Kernels & partition, KernelVertexVec & vertex, ProgramGraph & G);

    void addPopCountKernels(BuilderRef b, Kernels & partition, KernelVertexVec & vertex, ProgramGraph & G);

    void combineDuplicateKernels(BuilderRef b, const Kernels & partition, ProgramGraph & G);

    void removeUnusedKernels(const unsigned p_in, const unsigned p_out, const Kernels & partition, ProgramGraph & G);

    void identifyPipelineInputs();

    void transcribeRelationshipGraph();

    void gatherInfo() {        
        MaxNumOfInputPorts = in_degree(PipelineOutput, mBufferGraph);
        MaxNumOfOutputPorts = out_degree(PipelineInput, mBufferGraph);
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            MaxNumOfInputPorts = std::max<unsigned>(MaxNumOfInputPorts, in_degree(i, mBufferGraph));
            MaxNumOfOutputPorts = std::max<unsigned>(MaxNumOfOutputPorts, out_degree(i, mBufferGraph));
        }
    }

    // partitioning analysis

    void partitionRelationshipGraphIntoSynchronousRegions();

    void identifyKernelPartitions(const std::vector<unsigned> & orderingOfG);

    #ifdef EXPERIMENTAL_SCHEDULING
    std::vector<PartitionData> gatherPartitionData(const std::vector<unsigned> & orderingOfG) const;

    void analyzeDataflowWithinPartitions(std::vector<PartitionData> & P) const;

    PartitionDataflowGraph analyzeDataflowBetweenPartitions(std::vector<PartitionData> & P) const;

    PartitionOrdering makePartitionSchedulingGraph(std::vector<PartitionData> & P, const PartitionDataflowGraph & D) const;

    void scheduleProgramGraph(const std::vector<PartitionData> & P, PartitionOrdering & O, const PartitionDataflowGraph & D) const;

    void assembleProgramSchedule(const std::vector<PartitionData> & P, const PartitionOrdering & O);

    #endif

    void addOrderingConstraintsToPartitionSubgraphs(const std::vector<unsigned> & orderingOfG);

    void generatePartitioningGraph();

    void determinePartitionJumpIndices();

    void makePartitionJumpTree();

    // buffer management analysis functions

    void addStreamSetsToBufferGraph(BuilderRef b);
    void generateInitialBufferGraph();
    void identifyLinearBuffers();
    void identifyNonLocalBuffers();
    void identifyLocalPortIds();
    // void identifyDirectUpdatesToStateObjects();

    // consumer analysis functions

    void makeConsumerGraph();

    // dataflow analysis functions

    void computeDataFlowRates();
    PartitionConstraintGraph identifyHardPartitionConstraints(BufferGraph & G) const;
    LengthConstraintGraph identifyLengthEqualityAssertions(BufferGraph & G) const;

    // zero extension analysis function

    void identifyZeroExtendedStreamSets();

    // termination analysis functions

    void identifyTerminationChecks();
    void makeTerminationPropagationGraph();

    // add(k) analysis functions

    void annotateBufferGraphWithAddAttributes();

    // IO analysis functions

    void makeKernelIOGraph();

    // Input truncation analysis functions

    void makeInputTruncationGraph();

public:

    // Debug functions
    void printBufferGraph(raw_ostream & out) const;
    static void printRelationshipGraph(const RelationshipGraph & G, raw_ostream & out, const StringRef name = "G");

private:

    PipelineKernel * const          mPipelineKernel;
    const unsigned					mNumOfThreads;
    const LengthAssertions &        mLengthAssertions;
    ProgramGraph                    Relationships;
    KernelPartitionIds              PartitionIds;

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

    unsigned                        MaxNumOfInputPorts = 0;
    unsigned                        MaxNumOfOutputPorts = 0;


    unsigned                        MaxNumOfLocalInputPortIds = 0;
    unsigned                        MaxNumOfLocalOutputPortIds = 0;

    RelationshipGraph               mStreamGraph;
    RelationshipGraph               mScalarGraph;

    Partition                       KernelPartitionId;

    std::vector<Rational>           MinimumNumOfStrides;
    std::vector<Rational>           MaximumNumOfStrides;

    BufferGraph                     mBufferGraph;
    std::vector<unsigned>           mPartitionJumpIndex;
    PartitionJumpTree               mPartitionJumpTree;

    ConsumerGraph                   mConsumerGraph;

    TerminationChecks               mTerminationCheck;

    TerminationPropagationGraph     mTerminationPropagationGraph;

    OwningVector<Kernel>            mInternalKernels;
    OwningVector<Binding>           mInternalBindings;
    OwningVector<StreamSetBuffer>   mInternalBuffers;
};

}

#include "pipeline_graph_printers.hpp"
#include "relationship_analysis.hpp"
#include "buffer_analysis.hpp"
#include "consumer_analysis.hpp"
#include "dataflow_analysis.hpp"
#include "partitioning_analysis.hpp"
#include "scheduling_analysis.hpp"
#include "termination_analysis.hpp"
#include "zero_extend_analysis.hpp"
#include "add_analysis.hpp"

#endif
