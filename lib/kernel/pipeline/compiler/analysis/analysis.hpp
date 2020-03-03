#ifndef PIPELINE_KERNEL_ANALYSIS_HPP
#define PIPELINE_KERNEL_ANALYSIS_HPP

#include "graphs.h"

namespace kernel {

#if 0

struct PipelineAnalysis {

public:

    static PipelineAnalysis analyze(BuilderRef b, PipelineKernel * pipelineKernel);

private:

    // constructor

    PipelineAnalysis(PipelineKernel * pipelineKernel)
    : mPipelineKernel(pipelineKernel) {

    }

    // pipeline analysis functions

    using KernelPartitionIds = flat_map<Relationships::vertex_descriptor, unsigned>;

    void makePipelineGraph(BuilderRef b, PipelineKernel * const pipelineKernel);

    Relationships generateInitialPipelineGraph(BuilderRef b,
                                               PipelineKernel * const pipelineKernel,
                                               OwningVector<Kernel> & internalKernels,
                                               OwningVector<Binding> & internalBindings);

    using KernelVertexVec = SmallVector<Relationships::Vertex, 64>;

    void addRegionSelectorKernels(BuilderRef b, Kernels & partition, KernelVertexVec & vertex, Relationships & G,
                                  OwningVector<Kernel> & internalKernels, OwningVector<Binding> & internalBindings);

    void addPopCountKernels(BuilderRef b, Kernels & partition, KernelVertexVec & vertex, Relationships & G,
                            OwningVector<Kernel> &internalKernels, OwningVector<Binding> &internalBindings);

    static void combineDuplicateKernels(BuilderRef b, const Kernels & partition, Relationships & G);
    static void removeUnusedKernels(const PipelineKernel * pipelineKernel, const unsigned p_in, const unsigned p_out, const Kernels & partition, Relationships & G);

    void identifyPipelineInputs();

    // partitioning analysis

    unsigned partitionIntoFixedRateRegionsWithOrderingConstraints(Relationships & G,
                                                                  KernelPartitionIds & partitionIds,
                                                                  const PipelineKernel * const pipelineKernel) const;

    unsigned identifyKernelPartitions(const Relationships &G,
                                                    const std::vector<unsigned> & orderingOfG,
                                                    const PipelineKernel * const pipelineKernel,
                                                    KernelPartitionIds & partitionIds) const;

    void addOrderingConstraintsToPartitionSubgraphs(Relationships & G,
                                                    const std::vector<unsigned> & orderingOfG,
                                                    const KernelPartitionIds & partitionIds,
                                                    const unsigned numOfPartitions) const;

    PartitioningGraph generatePartitioningGraph() const;

    Vec<unsigned> determinePartitionJumpIndices() const;

    PartitionJumpTree makePartitionJumpTree() const;

    // buffer management analysis functions

    BufferGraph makeBufferGraph(BuilderRef b);
    void initializeBufferGraph(BufferGraph & G) const;
    void verifyIOStructure(const BufferGraph & G) const;
    void identifyLinearBuffers(BufferGraph & G) const;
    void identifyNonLocalBuffers(BufferGraph & G) const;
    BufferPortMap constructInputPortMappings() const;
    BufferPortMap constructOutputPortMappings() const;
    bool mayHaveNonLinearIO(const size_t kernel) const;
    bool supportsInternalSynchronization() const;
    void identifyLocalPortIds(BufferGraph & G) const;
    bool isBounded() const;

    void printBufferGraph(raw_ostream & out) const;

    // dataflow analysis functions

    void computeDataFlowRates(BufferGraph & G);
    PartitionConstraintGraph identifyHardPartitionConstraints(BufferGraph & G) const;
    LengthConstraintGraph identifyLengthEqualityAssertions(BufferGraph & G) const;

    // zero extension analysis function

    bool hasZeroExtendedStreams(BufferGraph & G) const;

    // termination analysis functions

    TerminationGraph makeTerminationGraph() const;

    // add(k) analysis functions

    AddGraph makeAddGraph() const;

    // IO analysis functions

    IOCheckGraph makeKernelIOGraph() const;

public:

    PipelineKernel * const          mPipelineKernel;

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

    RelationshipGraph               Streams;
    RelationshipGraph               Scalars;
    Partition                       KernelPartitionId;


    std::vector<Rational>           MinimumNumOfStrides;
    std::vector<Rational>           MaximumNumOfStrides;

    BufferGraph                     mBufferGraph;
    PartitioningGraph               mPartitioningGraph;
    Vec<unsigned>                   mPartitionJumpIndex;
    PartitionJumpTree               mPartitionJumpTree;


    bool                            mHasZeroExtendedStream;
    bool                            mHasThreadLocalPipelineState;

    ConsumerGraph                   mConsumerGraph;

    TerminationGraph                mTerminationGraph;
    AddGraph                        mAddGraph;
    IOCheckGraph                    mIOCheckGraph;

    OwningVector<Kernel>            InternalKernels;
    OwningVector<Binding>           InternalBindings;

};

PipelineAnalysis PipelineAnalysis::analyze(BuilderRef b, PipelineKernel * pipelineKernel) {
    PipelineAnalysis PA(pipelineKernel);


}



#endif

}



#include "pipeline_graph_printers.hpp"
#include "pipeline_analysis.hpp"
#include "buffer_analysis.hpp"
#include "consumer_analysis.hpp"
#include "dataflow_analysis.hpp"
#include "partitioning_analysis.hpp"
#include "termination_analysis.hpp"
#include "zero_extend_analysis.hpp"
#include "add_analysis.hpp"
#include "io_analysis.hpp"

#endif
