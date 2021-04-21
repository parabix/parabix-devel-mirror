#ifndef PIPELINE_KERNEL_ANALYSIS_HPP
#define PIPELINE_KERNEL_ANALYSIS_HPP

#include "../config.h"
#include "../common/common.hpp"
#include "../common/graphs.h"

#include <algorithm>
#include <queue>
#include <z3.h>
// #include <util/maxsat.hpp>
#include <assert.h>

#include <kernel/core/kernel.h>
#include <kernel/core/relationship.h>
#include <kernel/core/streamset.h>
#include <kernel/core/kernel_builder.h>

#include <llvm/Support/Format.h>

#define EXPERIMENTAL_SCHEDULING

#include <boost/graph/connected_components.hpp>

namespace kernel {

struct PipelineAnalysis : public PipelineCommonGraphFunctions {

public:

    static PipelineAnalysis analyze(BuilderRef b, PipelineKernel * const pipelineKernel) {

        PipelineAnalysis P(pipelineKernel);

        const auto seed = std::random_device{}();

        random_engine rng(seed);

//        const auto graphSeed = 2081280305; //rng(); // 2081280305, 2081280305

//        P.generateRandomPipelineGraph(b, graphSeed, 50, 70, 10);

        errs() << "generateInitialPipelineGraph\n";

        P.generateInitialPipelineGraph(b);

      //  P.printRelationshipGraph(P.Relationships, errs(), "R");

        // Initially, we gather information about our partition to determine what kernels
        // are within each partition in a topological order

        errs() << "identifyKernelPartitions\n";

        auto partitionGraph = P.identifyKernelPartitions();

        const auto partitionCount = num_vertices(partitionGraph);

        // Add ordering constraints to ensure we can keep sequences of kernels with a fixed rates in
        // the same sequence. This will help us to partition the graph later and is useful to determine
        // whether we can bypass a region without testing every kernel.

        errs() << "computeExpectedDataFlowRates\n";

        P.computeMinimumExpectedDataflow(partitionGraph);

        errs() << "schedulePartitionedProgram\n";

        P.schedulePartitionedProgram(partitionGraph, rng, 1.0, 1);

        // Construct the Stream and Scalar graphs
        P.transcribeRelationshipGraph(partitionGraph);

        // P.printRelationshipGraph(P.mStreamGraph, errs(), "Streams");
        // P.printRelationshipGraph(P.mScalarGraph, errs(), "Scalars");

        errs() << "generateInitialBufferGraph\n";

        P.generateInitialBufferGraph();

        P.identifyOutputNodeIds();

        errs() << "computeNumOfStridesInterval\n";

//        #ifdef PRINT_BUFFER_GRAPH
//        P.printBufferGraph(errs());
//        #endif

        P.computeMaximumExpectedDataflow();

        errs() << "computeInterPartitionSymbolicRates\n";

        P.identifyInterPartitionSymbolicRates();

        errs() << "determineBufferSize\n";

        P.determineBufferSize(b);

        errs() << "determineBufferLayout\n";

        P.determineBufferLayout(b, rng);

        errs() << "identifyBufferLocality\n";

        P.markInterPartitionStreamSetsAsGloballyShared(); // linkedPartitions


        P.makePartitionIOGraph();

        errs() << "identifyTerminationChecks\n";

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

     //   exit(-1);

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

    #ifdef ENABLE_GRAPH_TESTING_FUNCTIONS
    void generateRandomPipelineGraph(BuilderRef b, const uint64_t seed,
                                     const unsigned desiredKernels, const unsigned desiredStreamSets,
                                     const unsigned desiredPartitions);
    #endif

    using KernelVertexVec = SmallVector<ProgramGraph::Vertex, 64>;

    void addRegionSelectorKernels(BuilderRef b, Kernels & partition, KernelVertexVec & vertex, ProgramGraph & G);

    void addPopCountKernels(BuilderRef b, Kernels & partition, KernelVertexVec & vertex, ProgramGraph & G);

    void combineDuplicateKernels(BuilderRef b, const Kernels & partition, ProgramGraph & G);

    void removeUnusedKernels(const unsigned p_in, const unsigned p_out, const Kernels & partition, ProgramGraph & G);

    void identifyPipelineInputs();

    void transcribeRelationshipGraph(const PartitionGraph & partitionGraph);

    void gatherInfo() {        
        MaxNumOfInputPorts = in_degree(PipelineOutput, mBufferGraph);
        MaxNumOfOutputPorts = out_degree(PipelineInput, mBufferGraph);
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            MaxNumOfInputPorts = std::max<unsigned>(MaxNumOfInputPorts, in_degree(i, mBufferGraph));
            MaxNumOfOutputPorts = std::max<unsigned>(MaxNumOfOutputPorts, out_degree(i, mBufferGraph));
        }
    }

    static void addKernelRelationshipsInReferenceOrdering(const unsigned kernel, const RelationshipGraph & G,
                                                          std::function<void(PortType, unsigned, unsigned)> insertionFunction);

    // partitioning analysis

    PartitionGraph identifyKernelPartitions();

    void makePartitionIOGraph();

    void determinePartitionJumpIndices();

    void makePartitionJumpTree();

    // scheduling analysis

    void schedulePartitionedProgram(PartitionGraph & P, random_engine & rng, const double maxCutRoundsFactor, const unsigned maxCutPasses);

    void analyzeDataflowWithinPartitions(PartitionGraph & P, random_engine & rng) const;

    SchedulingGraph makeIntraPartitionSchedulingGraph(const PartitionGraph & P, const unsigned currentPartitionId) const;

    PartitionDependencyGraph makePartitionDependencyGraph(const unsigned numOfKernels, const SchedulingGraph & S) const;

    PartitionDataflowGraph analyzeDataflowBetweenPartitions(PartitionGraph & P) const;

    PartitionOrdering makeInterPartitionSchedulingGraph(PartitionGraph & P, const PartitionDataflowGraph & D) const;

    std::vector<unsigned> scheduleProgramGraph(const PartitionGraph & P, const PartitionOrdering & O, const PartitionDataflowGraph & D, random_engine & rng,
                                               const double maxCutRoundsFactor, const unsigned maxCutPasses) const;

    void addSchedulingConstraints(const PartitionGraph & P, const std::vector<unsigned> & program);

    // buffer management analysis functions

    void addStreamSetsToBufferGraph(BuilderRef b);
    void generateInitialBufferGraph();

    void determineBufferSize(BuilderRef b);

    void determineBufferLayout(BuilderRef b, random_engine & rng);

    void identifyLinearBuffers();
    void markInterPartitionStreamSetsAsGloballyShared();
    void identifyLocalPortIds();

    void identifyOutputNodeIds();

    // void identifyDirectUpdatesToStateObjects();

    // consumer analysis functions

    void makeConsumerGraph();

    // dataflow analysis functions

    void computeMinimumExpectedDataflow(PartitionGraph & P);

    void computeMaximumExpectedDataflow();

    void identifyInterPartitionSymbolicRates();

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

    size_t                          RequiredThreadLocalStreamSetMemory = 0;

    unsigned                        MaxNumOfInputPorts = 0;
    unsigned                        MaxNumOfOutputPorts = 0;


    unsigned                        MaxNumOfLocalInputPortIds = 0;
    unsigned                        MaxNumOfLocalOutputPortIds = 0;

    RelationshipGraph               mStreamGraph;
    RelationshipGraph               mScalarGraph;

    Partition                       KernelPartitionId;

    std::vector<unsigned>           MinimumNumOfStrides;
    std::vector<unsigned>           MaximumNumOfStrides;

    BufferGraph                     mBufferGraph;

    PartitionIOGraph                mPartitionIOGraph;
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
#include "buffer_size_analysis.hpp"
#include "consumer_analysis.hpp"
#include "dataflow_analysis.hpp"
#include "partitioning_analysis.hpp"
#include "scheduling_analysis.hpp"
#include "termination_analysis.hpp"
#include "zero_extend_analysis.hpp"
#include "add_analysis.hpp"

#endif
