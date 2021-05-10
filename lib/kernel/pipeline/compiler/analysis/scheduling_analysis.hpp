#ifndef SCHEDULING_ANALYSIS_HPP
#define SCHEDULING_ANALYSIS_HPP

#include "pipeline_analysis.hpp"
#include "evolutionary_algorithm.hpp"

#include <chrono>
#include <llvm/Support/Format.h>
#include <fstream>
#include <iostream>

namespace kernel {

#ifdef EXPERIMENTAL_SCHEDULING

#define INITIAL_TOPOLOGICAL_POPULATION_SIZE (10)

constexpr unsigned MAX_PARTITION_POPULATION_SIZE = 100;

#define MAX_EVOLUTIONARY_ROUNDS (0)

#define MUTATION_RATE (0.20)

constexpr unsigned MAX_PROGRAM_POPULATION_SIZE = 100;

constexpr static unsigned PARITION_SCHEDULING_GA_ROUNDS = 100;

constexpr static unsigned PARITION_SCHEDULING_GA_STALLS = 20;

constexpr static unsigned PROGRAM_SCHEDULING_GA_ROUNDS = 100;

constexpr static unsigned PROGRAM_SCHEDULING_GA_STALLS = 20;

#define BIPARTITE_GRAPH_UNPLACED (0U)

#define BIPARTITE_GRAPH_LEFT_HAND (1U)

#define BIPARTITE_GRAPH_RIGHT_HAND (2U)

#define INITIAL_SCHEDULING_POPULATION_ATTEMPTS (100)

#define INITIAL_SCHEDULING_POPULATION_SIZE (100)

#define SCHEDULING_FITNESS_COST_ACO_ROUNDS (200)

//constexpr static unsigned MAX_CUT_HS_ROUNDS = 100;
//constexpr static unsigned MAX_CUT_HS_INITIAL_CANDIDATES = 100;
//constexpr static double MAX_CUT_HS_CONSIDERATION_RATE = 0.997;
//constexpr static unsigned MAX_CUT_HS_POPULATION_SIZE = 100;
constexpr static unsigned MAX_CUT_MAX_NUM_OF_CONNECTED_COMPONENTS = 7;

constexpr static double HAMILTONIAN_PATH_INVERSE_K = 1.0 / 50.0;

constexpr static unsigned HAMILTONIAN_PATH_NUM_OF_ANTS = 5;

static_assert((SCHEDULING_FITNESS_COST_ACO_ROUNDS % HAMILTONIAN_PATH_NUM_OF_ANTS) == 0, "rounds must be divisible by # of ants");

constexpr static double HAMILTONIAN_PATH_EPSILON_WEIGHT = 0.1;

constexpr static double HAMILTONIAN_PATH_DELTA_WEIGHT = 100.0;

constexpr static double HAMILTONIAN_PATH_ACO_TAU_INITIAL_VALUE = 5.0;

constexpr static double HAMILTONIAN_PATH_ACO_TAU_MIN = HAMILTONIAN_PATH_EPSILON_WEIGHT;

constexpr static double HAMILTONIAN_PATH_ACO_TAU_MAX = HAMILTONIAN_PATH_DELTA_WEIGHT;

static size_t fitness_hs_time = 0;
static size_t fitness_time = 0;
static size_t repair_time = 0;
static size_t total_ga_time = 0;

static size_t total_intra_dataflow_analysis_time = 0;
static size_t total_inter_dataflow_analysis_time = 0;
static size_t total_inter_dataflow_scheduling_time = 0;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief analyzeDataflowWithinPartitions
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::schedulePartitionedProgram(PartitionGraph & P, random_engine & rng, const double maxCutRoundsFactor, const unsigned maxCutPasses) {

    // Once we analyze the dataflow within the partitions, P contains DAWG that is either
    // edgeless if any permutation of its kernels is valid or contains all of its optimal
    // orderings for the kernels within each partition.

    // TODO: look into performance problem with
    // bin/icgrep -EnableTernaryOpt -DisableMatchStar '(?g)fodder|simple' ../QA/testfiles/simple1 -colors=always

    assert (PartitionCount > 2);

    const auto t0 = std::chrono::high_resolution_clock::now();
    analyzeDataflowWithinPartitions(P, rng);
    const auto t1 = std::chrono::high_resolution_clock::now();
    total_intra_dataflow_analysis_time = (t1 - t0).count();

    // The graph itself has edges indicating a dependency between the partitions, annotated by the kernels
    // that are a producer of one of the streamsets that traverses the partitions. Ideally we'll use the
    // trie to score each of the possible orderings based on how close a kernel is to its cross-partition
    // consumers but first we need to determine the order of our partitions.

    const auto D = analyzeDataflowBetweenPartitions(P);
    const auto t2 = std::chrono::high_resolution_clock::now();
    auto I = makeInterPartitionSchedulingGraph(P, D);
    const auto t3 = std::chrono::high_resolution_clock::now();
    total_inter_dataflow_analysis_time = (t2 - t1).count();
    total_inter_dataflow_scheduling_time = (t3 - t2).count();
    const auto C = scheduleProgramGraph(P, I, D, rng);
    addSchedulingConstraints(P, selectScheduleFromDAWG(I.Kernels, C));

}

namespace { // start of anonymous namespace

#if 0

void printDAWG(const OrderingDAWG & G, raw_ostream & out, const StringRef name = "G") {

    out << "digraph \"" << name << "\" {\n";
    for (auto v : make_iterator_range(vertices(G))) {
        out << "v" << v << " [label=\"\"];\n";
    }
    for (auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);
        out << "v" << s << " -> v" << t <<
               " [label=\"" << G[e] << "\"];\n";
    }

    out << "}\n\n";
    out.flush();
}

#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief postorder_minimize
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned postorder_minimize(OrderingDAWG & O) {

    // Adapted from "Comparison of construction algorithms for minimal acyclic
    // deterministic finite-state automata from a set of strings." 2003

    // NOTE: this is not a generic post order minimization algorithm. It was
    // adapted for the tries generated by the scheduling algorithm. Specifically,
    // since final/non-final states are indicated by whether we're at the last
    // level or not, we ignore such comparisons; thus any state whose (outgoing)
    // transitions match are considered equal. Additionally, all strings are of
    // equal length but were not lexographically inserted. However, since the
    // level of each state in the DAWG cannot change w.r.t. the trie, we simplify
    // the original algorithm to avoid using a hash table.

    assert (num_edges(O) > 0);

    using Vertex = OrderingDAWG::vertex_descriptor;

    const auto n = num_vertices(O);

    BitVector P(n);

    Vertex t = 0; // common sink

    BEGIN_SCOPED_REGION

    for (unsigned i = 1; i < n; ++i) {
        if (out_degree(i, O) == 0) {
            assert (in_degree(i, O) > 0);
            const auto e = in_edge(i, O);
            const auto p = source(e, O);
            P.set(p);
            if (t == 0) {
                t = i;
            } else {
                const auto ch = O[e];
                clear_in_edges(i, O);
                add_edge(p, t, ch, O);
            }
        }
    }

    END_SCOPED_REGION

    using SV = SmallVector<std::pair<unsigned, unsigned>, 8>;

    std::vector<SV> T;

    std::vector<Vertex> L;

    for (;;) {

        L.clear();
        for (const auto u : P.set_bits()) {
            if (LLVM_UNLIKELY(u == 0)) {
                assert (P.count() == 1);
                return t;
            }
            assert (in_degree(u, O) == 1);
            L.push_back(u);
        }
        P.reset();

        const auto m = L.size();

        if (T.size() < m) {
            T.resize(m);
        }

        for (unsigned i = 0; i < m; ++i) {

            // lexographically sort our outgoing transitions for every state in L
            const auto u = L[i];
            auto & A = T[i];
            A.clear();
            for (const auto e : make_iterator_range(out_edges(u, O))) {
                A.emplace_back(O[e], target(e, O));
            }
            std::sort(A.begin(), A.end());

            // check whether the i-th node is a duplicate of another within the
            // same level L
            const auto e = in_edge(u, O);
            const auto p = source(e, O);

            P.set(p);

            for (unsigned j = 0; j < i; ++j) {
                const auto v = L[j];
                const auto & B = T[j];
                if (A == B) {
                    const auto ch = O[e];
                    clear_vertex(u, O);
                    add_edge(p, v, ch, O);
                    break;
                }

            }
        }
    }

}

using MemIntervalGraph = adjacency_list<vecS, vecS, undirectedS>;

using WeightMap = flat_map<MemIntervalGraph::edge_descriptor, double>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief MaxCutHarmonySearch
 ** ------------------------------------------------------------------------------------------------------------- */
struct MaxCutHarmonySearch : public BitStringBasedHarmonySearch {

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief initGA
     ** ------------------------------------------------------------------------------------------------------------- */
    bool initialize(Population & initialPopulation) final {
        std::uniform_int_distribution<unsigned> zeroOrOneInt(0, 1);
        Candidate C(candidateLength);
        #ifndef NDEBUG
        std::vector<bool> sanity(candidateLength);
        #endif
        unsigned inserted = 0;
        for (unsigned i = 0; i < 100; ++i) {

            #ifndef NDEBUG
            for (unsigned j = 0; j < candidateLength; ++j) {
                C.set(j, (j & 1) == 0);
            }
            #endif
            for (unsigned j = 0; j < candidateLength; ++j) {
                const auto b = zeroOrOneInt(rng);
                C.set(j, b);
                assert (C.test(j) == b);
                #ifndef NDEBUG
                sanity[j] = b;
                C.set(j, !b);
                assert (C.test(j) != b);
                C.set(j, b);
                assert (C.test(j) == b);
                #endif
            }
            #ifndef NDEBUG
            for (unsigned j = 0; j < candidateLength; ++j) {
                assert (C.test(j) == sanity[j]);
            }
            #endif
            if (insertCandidate(C, initialPopulation)) {
                if (++inserted == maxCandidates) {
                    return false;
                }
            }
        }
        return true;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief fitness
     ** ------------------------------------------------------------------------------------------------------------- */
    double fitness(const Candidate & candidate) final {
        double weight = 0;
        unsigned numOfComponents = 0;
        unvisited.set(0, candidateLength - 1);
        auto i = unvisited.find_first();
        for (;;) {
            assert ((unsigned)i < candidateLength);
            assert (unvisited.test(i));
            assert (stack.empty());
            unvisited.reset(i);
            auto u = toVertexIndex[i];
            assert (out_degree(u, I) > 0);
            ++numOfComponents;
            for (;;) {
                const auto uval = candidate.test(u);
                for (const auto e : make_iterator_range(out_edges(u, I))) {
                    const auto v = target(e, I);
                    if (unvisited.test(v) && uval != candidate.test(v)) {
                        unvisited.reset(v);
                        const auto f = maxCutWeights.find(e);
                        assert (f != maxCutWeights.end());
                        weight += f->second;
                        stack.push_back(v);
                    }
                }
                if (stack.empty()) {
                    break;
                }
                u = stack.back();
                stack.pop_back();
            }
            i = unvisited.find_next(i);
            if (i == -1) {
                break;
            }
        }

        if (numOfComponents > maxNumOfComponents) {
            weight = std::pow(weight, 1.0 / ((double)(numOfComponents - maxNumOfComponents)));
        }

        return weight;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief constructor
     ** ------------------------------------------------------------------------------------------------------------- */
    MaxCutHarmonySearch(const MemIntervalGraph & I,
                        const WeightMap & maxCutWeights,
                        const unsigned numOfRounds,
                        const unsigned populationSize,
                        const double fixedConsiderationRate,
                        const size_t seed)
    : BitStringBasedHarmonySearch(numOfNonIsolatedVertices(I), numOfRounds, populationSize, fixedConsiderationRate, seed)
    , I(I)
    , maxCutWeights(maxCutWeights)
    , toVertexIndex(candidateLength)
    , toBitIndex(num_vertices(I))
    , unvisited(num_vertices(I))
    , maxNumOfComponents(MAX_CUT_MAX_NUM_OF_CONNECTED_COMPONENTS) {
        const auto n = num_vertices(I);
        for (unsigned i = 0, j = 0; i < n; ++i) {
            if (out_degree(i, I) > 0) {
                toVertexIndex[j] = i;
                toBitIndex[i] = j;
                ++j;
            }
        }
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief constructor
     ** ------------------------------------------------------------------------------------------------------------- */
    MaxCutHarmonySearch(const MemIntervalGraph & I,
                        const WeightMap & maxCutWeights,
                        const unsigned numOfRounds,
                        const unsigned populationSize,
                        const HMCRType type,
                        const double minHMCR,
                        const double maxHMCR,
                        const double angularFrequency,
                        const size_t seed)
    : BitStringBasedHarmonySearch(numOfNonIsolatedVertices(I), numOfRounds, populationSize, type, minHMCR, maxHMCR, angularFrequency, seed)
    , I(I)
    , maxCutWeights(maxCutWeights)
    , toVertexIndex(candidateLength)
    , toBitIndex(num_vertices(I))
    , unvisited(num_vertices(I))
    , maxNumOfComponents(MAX_CUT_MAX_NUM_OF_CONNECTED_COMPONENTS) {
        const auto n = num_vertices(I);
        for (unsigned i = 0, j = 0; i < n; ++i) {
            if (out_degree(i, I) > 0) {
                toVertexIndex[j] = i;
                toBitIndex[i] = j;
                ++j;
            }
        }
    }

private:

    static size_t numOfNonIsolatedVertices(const MemIntervalGraph & I) {
        const auto n = num_vertices(I);
        size_t m = 0;
        for (unsigned i = 0; i < n; ++i) {
            if (out_degree(i, I) > 0) {
                ++m;
            }
        }
        return m;
    }

protected:

    const MemIntervalGraph & I;
    const WeightMap & maxCutWeights;

    std::vector<unsigned> toVertexIndex;
    std::vector<unsigned> toBitIndex;

    BitVector unvisited;
    SmallVector<Vertex, 16> stack;
    const unsigned maxNumOfComponents;
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief MemoryAnalysis
 ** ------------------------------------------------------------------------------------------------------------- */
class MemoryAnalysis {

    using Candidate = PermutationBasedEvolutionaryAlgorithm::Candidate;

    using IntervalEdge = typename MemIntervalGraph::edge_descriptor;

    enum class Orientation {
        Forwards = 0
        , Backwards = 1
        , Unknown = 2
    };

    struct EdgeOrientation {
        unsigned Component;
        Orientation Direction;

        EdgeOrientation(Orientation dir = Orientation::Forwards, unsigned component = 0)
        : Component(component)
        , Direction(dir) {

        }

    };

    using random_engine = std::default_random_engine;

    using TransitiveGraph = adjacency_list<vecS, vecS, undirectedS, no_property, EdgeOrientation>;

public:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief analyze
     *
     * The analyze function is an offline DSA algorithm that returns an upperbound on the required memory.
     *
     * It is based on the algorithm presented in "Comparability graph coloring for optimizing utilization of
     * software-managed stream register files for stream processors" (2012) but instead of generating spanning
     * forests to mark "long-lived" streamsets, it generates a bipartite graph, potentially taking a max-cut
     * of a non-bipartite graph to preserve as many important relationships as possible. Because the algorithm
     * has a running time of O(2^(N + 1)), where N is the number of connected components in the bipartite graph,
     * max-cut based on Prim's spanning tree algorithm. Rather than taking a random spanning tree, however, it
     * combines an ant colony heuristic with it to locate the heaviest cut that does not increase the number of
     * connected components.
     *
     * In cases where a max-cut is necessary, this analyze function will return the chromatic number of the
     * optimal colouring based on the original algorithm PLUS a greedy colouring of the "removed" edges as a
     * worst-case over-approximation for the true chromatic number.
     ** ------------------------------------------------------------------------------------------------------------- */
    size_t analyze(const Candidate & candidate) {

        assert (candidate.size() == numOfKernels);

        if (LLVM_LIKELY(numOfStreamSets == 0)) {
            return 0;
        }

        assert (numOfKernels > 1);

        // Each node value in the interval graph marks the position of the candidate schedule
        // that produced the streamset.

        MemIntervalGraph I(numOfStreamSets);

        std::fill_n(live.begin(), numOfStreamSets, 0);

        const auto firstStreamSet = (2 * numOfKernels);

        std::fill_n(weight.begin(), firstStreamSet, 0);

        BEGIN_SCOPED_REGION

        unsigned streamSetId = 0;
        unsigned position = 0;
        for (const auto kernel : candidate) {
            assert (kernel < numOfKernels);
            assert (S[kernel].Type == SchedulingNode::IsKernel);
            for (const auto output : make_iterator_range(out_edges(kernel, S))) {
                const auto streamSet = target(output, S);
                const SchedulingNode & node = S[streamSet];
                assert (node.Type == SchedulingNode::IsStreamSet);
                // assert (node.Size > 0);
                assert (streamSetId < numOfStreamSets);
                const auto i = streamSetId++;
                ordinal[i] = position;
                for (unsigned j = 0; j != i; ++j) {
                    if (live[j] != 0) {
                        add_edge(j, i, I);
                        live[j]--;
                    }
                }
                live[i] = out_degree(streamSet, S);
                // initialize the streamset weight in the graph
                const auto W = ceiling(S[streamSet].Size);
                assert (W > 0);
                weight[firstStreamSet + i] = W;
            }
            ++position;
        }

        assert (position == numOfKernels);
        assert (streamSetId == numOfStreamSets);

        END_SCOPED_REGION

        Depth = 0;

        return calculateChomaticNumber(candidate, weight, I);
    }

private:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief calculateChomaticNumber
     ** ------------------------------------------------------------------------------------------------------------- */
    size_t calculateChomaticNumber(const Candidate & candidate, std::vector<size_t> & weight, MemIntervalGraph & I) {

        const auto l = (2 * numOfKernels) + numOfStreamSets;

        TransitiveGraph G(l);

        unsigned streamSetId = 0;
        unsigned priorProducerRank = 0;

        ++Depth;

        for (const auto kernel : candidate) {
            for (const auto output : make_iterator_range(out_edges(kernel, S))) {
                const auto streamSet = target(output, S);
                const SchedulingNode & node = S[streamSet];
                assert (node.Type != SchedulingNode::IsKernel);
                assert (node.Type == SchedulingNode::IsStreamSet);
                assert (streamSetId < numOfStreamSets);
                const auto i = streamSetId++;

                // Each node value in I marks the schedule position.
                const auto producerRank = ordinal[i];
                assert (priorProducerRank <= producerRank);
                priorProducerRank = producerRank;

                auto consumerRank = producerRank;
                for (const auto e : make_iterator_range(out_edges(i, I))) {
                    const auto j = target(e, I);
                    const auto rank = ordinal[j];
                    consumerRank = std::max(consumerRank, rank);
                }

                const auto lifespan = consumerRank - producerRank;

                const auto j = (2 * numOfKernels) + i;

                const auto W = weight[j];

                if (LLVM_LIKELY(lifespan <= 1 || W == 0)) {
                    const auto k = (2 * producerRank) | lifespan;
                    assert (k < (2 * numOfKernels));
                    weight[k] += W;
                    weight[j] = 0;

                    // If the lifespan of this streamset is at most one, we can place it into the
                    // comparability graph and do not need to reason about it within the forest.
                    clear_vertex(i, I);

                } else {

                    // NOTE: we mark the direction of the edges between the "forest" and comparability
                    // graph nodes as Unknown since we do not know their orientation until we've built
                    // the spanning forest for the remaining interval graph.

                    const auto m = ((2 * consumerRank) | 1);

                    for (unsigned i = (2 * producerRank); i <= m; ++i) {
                        assert (i < j);
                        add_edge(i, j, EdgeOrientation{Orientation::Unknown}, G);
                    }

                }
            }
        }

        assert (streamSetId == numOfStreamSets);

        // fill in the comparability graph edges
        auto dir = Orientation::Forwards;
        for (unsigned i = 1; i < numOfKernels; ++i) {
            const auto s = (i - 1) * 2;
            add_edge(s, s + 1U, EdgeOrientation{dir}, G);
            add_edge(s + 1U, s + 2U, EdgeOrientation{dir}, G);
            add_edge(s, s + 2U, EdgeOrientation{dir}, G);
            assert ((s + 2U) < (2U * numOfKernels));
            dir = (dir == Orientation::Forwards) ? Orientation::Backwards : Orientation::Forwards;
        }

        // Wang et al.'s paper suggests that graph G_I (once we remove any edges
        // accounted for in G) will likely be a forest and suggest taking a
        // spanning forest otherwise. What they want is a prime comparability
        // graph. Any tree is a bipartite graph and bipartite graphs are trivally
        // comparability graphs but are not necessarily trees.

        // Check if G_I is a bipartite graph and if not, do a max-cut.

        size_t worstCaseUnderapproximation = 0;

        assert (placement.size() >= numOfStreamSets);

redo_placement_after_max_cut:

        for (unsigned i = 0; i < numOfStreamSets; ++i) {
            placement[i] = (out_degree(i, I) == 0) ? BIPARTITE_GRAPH_LEFT_HAND : BIPARTITE_GRAPH_UNPLACED;
        }

        unsigned N = 1;

        for (unsigned r = 0;;) {

            // select the first vertex to 0/1 colour.
            for (;;) {
                // if we've placed every vertex, we can ignore this phase.
                if (r == numOfStreamSets) {
                    goto is_bipartite_graph;
                }
                if (placement[r] == BIPARTITE_GRAPH_UNPLACED) {
                    break;
                }
                assert (r < numOfStreamSets);
                ++r;
            }
            assert (r < numOfStreamSets);
            placement[r] = BIPARTITE_GRAPH_LEFT_HAND;
            assert (stack.empty());

            for (auto u = r;;) {

                assert (placement[u] != BIPARTITE_GRAPH_UNPLACED);
                const auto OTHER_HAND = (placement[u] ^ (BIPARTITE_GRAPH_LEFT_HAND | BIPARTITE_GRAPH_RIGHT_HAND));
                component[u] = N;
                for (const auto e : make_iterator_range(out_edges(u, I))) {
                    const auto v = target(e, I);
                    if (placement[v] == BIPARTITE_GRAPH_UNPLACED) {
                        placement[v] = OTHER_HAND;
                        stack.push_back(v);
                    } else if (placement[v] != OTHER_HAND) {
                        stack.clear();
                        assert ("second bipartite check failed?" && (worstCaseUnderapproximation == 0));
                        // compute_max_cut will transform I into a bipartite graph
                        worstCaseUnderapproximation = compute_max_cut(candidate, weight, I);
                        goto redo_placement_after_max_cut;
                    }
                }
                if (stack.empty()) {
                    break;
                }
                u = stack.back();
                stack.pop_back();
            }

            ++N;
        }

is_bipartite_graph:

        const auto firstStreamSet = (2 * numOfKernels);

        // orient the bridging edges according to the left/right hand sidedness.
        for (unsigned i = 0; i != numOfStreamSets; ++i) {
            const auto u = firstStreamSet + i;
            const auto inA = placement[i] == BIPARTITE_GRAPH_LEFT_HAND;
            const auto componentId = component[i];

            for (const auto e : make_iterator_range(out_edges(u, G))) {
                assert (target(e, G) < firstStreamSet);
                EdgeOrientation & O = G[e];
                O.Component = componentId;
                assert (O.Direction == Orientation::Unknown);
                // these are flipped w.r.t. the src < target ordering
                O.Direction = inA ? Orientation::Backwards : Orientation::Forwards;
            }
        }

        // then add our (bipartite) interval graph edges
        for (unsigned i = 0; i != numOfStreamSets; ++i) {
            const auto u = firstStreamSet + i;
            const auto inA = placement[i] == BIPARTITE_GRAPH_LEFT_HAND;
            const auto componentId = component[i];
            for (const auto e : make_iterator_range(out_edges(i, I))) { // ST
                const auto j = target(e, I); // ST
                assert (i != j);
                const bool flipped = (j < i);
                const auto dir = (inA ^ flipped) ? Orientation::Forwards : Orientation::Backwards;
                const auto v = firstStreamSet + j;
                add_edge(u, v, EdgeOrientation{dir, componentId}, G);
            }
        }

        // Our goal now is to find a minimal maximum-weight path through every
        // acyclic orientation of G; to do so we consider the permutations of the
        // component direction flags.

        auto chromaticNumber = std::numeric_limits<size_t>::max();

        // Based on the assumption N is relatively small, we can use a single counter
        // from 0 to pow(2,N) - 1 to represent our current premutation. If N > 10,
        // we'll need another method to converge on a solution.

        assert (N <= 10);

        for (unsigned i = 0; i < (1U << N); ++i) {
            const auto X = calculate_orientation_clique_weight(i, weight, G);
            chromaticNumber = std::min<size_t>(chromaticNumber, X);
        }

        assert (chromaticNumber < std::numeric_limits<size_t>::max());

        return chromaticNumber + worstCaseUnderapproximation;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief compute_max_cut
     ** ------------------------------------------------------------------------------------------------------------- */
    size_t compute_max_cut(const Candidate & candidate, const std::vector<size_t> & weight, MemIntervalGraph & I) {

        // If G_I is not a bipartite graph, intuitively, we want to keep as many
        // interval relationships adjacent to heavy nodes as possible. So we're going
        // to apply a weighted max-cut to it to transform G_I into one by discarding
        // any "uncut" edges.



        const auto start = std::chrono::high_resolution_clock::now();

        const auto numOfEdges = num_edges(I);
        assert (numOfEdges > 0);

        const auto predictionOf95PercentCut = 0.07930 * ((double)(numOfStreamSets * numOfStreamSets))
            + 7.63712 * ((double)numOfStreamSets) - 0.19735 * ((double)numOfEdges) - 80.59364;

        const unsigned numOfRounds = std::ceil(std::max(predictionOf95PercentCut, 10.0) * maxCutRoundsFactor);

        const auto firstStreamSet = (2 * numOfKernels);

        WeightMap maxCutWeights;

        maxCutWeights.reserve(numOfEdges);

        for (const auto e : make_iterator_range(edges(I))) {
            const auto u = source(e, I);
            const auto v = target(e, I);
            const size_t Wu = weight[firstStreamSet + u]; assert (Wu > 0);
            const size_t Wv = weight[firstStreamSet + v]; assert (Wv > 0);
            maxCutWeights.emplace(e, std::sqrt((double)(Wu * Wu + Wv * Wv)));
        }

        constexpr double ANG_FREQ_PERIOD_20 = 0.3141592653589793238462643383279502884197169399375105820974944592;

        auto bestScore = std::numeric_limits<MaxCutHarmonySearch::FitnessValueType>::lowest();

        MaxCutHarmonySearch::Candidate assignment(0);
        for (unsigned r = 0; r < maxCutPasses; ++r) {
            std::uniform_int_distribution<uintmax_t> distribution(0, std::numeric_limits<uintmax_t>::max());
            const auto seed = distribution(rng);
            MaxCutHarmonySearch HS(I, maxCutWeights, numOfRounds, 10, HMCRType::Cos, 0.8, 1.0, ANG_FREQ_PERIOD_20, seed);
            HS.runHarmonySearch();
            const auto score = HS.getBestFitnessValue();
            if (bestScore < score) {
                assignment = HS.getResult();
                bestScore = score;
            }
        }

        assert (bestScore > std::numeric_limits<MaxCutHarmonySearch::FitnessValueType>::lowest());

        MemIntervalGraph residualGraph(numOfStreamSets);

        std::vector<size_t> residualWeights(2 * numOfKernels + numOfStreamSets, 0);

        remove_edge_if([&](const MemIntervalGraph::edge_descriptor e){
            const auto u = source(e, I);
            const auto v = target(e, I);
            if (assignment.test(u) != assignment.test(v)) {
                return false;
            } else {
                add_edge(u, v, residualGraph);
                residualWeights[firstStreamSet + u] = weight[firstStreamSet + u];
                residualWeights[firstStreamSet + v] = weight[firstStreamSet + v];
                return true;
            }
        }, I);

        const auto end = std::chrono::high_resolution_clock::now();
        fitness_hs_time += (end - start).count();

        return calculateChomaticNumber(candidate, residualWeights, residualGraph);

    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief calculate_orientation_clique_weight
     ** ------------------------------------------------------------------------------------------------------------- */
    unsigned calculate_orientation_clique_weight(const std::bitset<64> permutation,
                                                 const std::vector<size_t> & weight, const TransitiveGraph & G) {

        // Ideally, we want to construct a topological ordering of our vertices then
        // recursively add the sum of the heaviest path into each vertex with the
        // current node weight starting from every source.

        // Unfortunetly, the way we faked the orientation of the edges makes it
        // impossible to use a boost method for this so the following implements
        // Kahn's algorithm. Since we're not actually interested in the ordering
        // itself and only the max weight of any path, we track that instead.

        // TODO we can more intelligently determine our source/sinks for this
        // permutation by using what we know about the graph structure.

        const auto l = num_vertices(G);

        std::fill_n(live.begin(), l, 0);

        auto is_oriented_forwards = [&](const unsigned u, const unsigned v, const TransitiveGraph::edge_descriptor e) {
            // We assume that given an edge (u, v), the direction is correct
            // if and only if index(u) < index(v). Flip the direction otherwise.
            assert (u == source(e, G) && v == target(e, G));
            const EdgeOrientation & O = G[e];
            const auto flipped = (v < u) ^ permutation.test(O.Component);
            return (O.Direction == Orientation::Forwards) ^ flipped;
        };

        for (const auto e : make_iterator_range(edges(G))) {
            const auto u = source(e, G);
            const auto v = target(e, G);
            const auto w = is_oriented_forwards(u, v, e) ? v : u;
            live[w]++;
        }

        assert (stack.empty());

        for (unsigned u = 0; u < l; ++u) {
            accum[u] = weight[u];
            // find all of our sinks
            if (live[u] == 0) {
                stack.push_back(u);
            }
        }

        assert (stack.size() > 0);

        unsigned visited = 0;

        size_t maxWeight = 0;
        for (;;) {

            const auto u = stack.back();
            stack.pop_back();
            ++visited;

            assert (in_degree(u, G) == out_degree(u, G));

            size_t weight = 0;
            for (const auto e : make_iterator_range(out_edges(u, G))) {
                const auto v = target(e, G);
                if (is_oriented_forwards(u, v, e)) {
                    assert ("G cannot be an cyclic graph" && live[v] > 0);
                    if (--live[v] == 0) {
                        stack.push_back(v);
                    }

                } else {
                    assert (live[v] == 0);
                    weight = std::max(weight, accum[v]);
                }
            }
            accum[u] += weight;
            maxWeight = std::max(maxWeight, accum[u]);
            if (stack.empty()) break;
        }

        assert (visited == l);
        return maxWeight;
    };

public:

    MemoryAnalysis(const SchedulingGraph & S, const unsigned numOfKernels, random_engine & rng, const double maxCutRoundsFactor, const unsigned maxCutPasses)
    : S(S)
    , numOfKernels(numOfKernels)
    , numOfStreamSets(num_vertices(S) - numOfKernels)
    , maxCutRoundsFactor(maxCutRoundsFactor)
    , maxCutPasses(maxCutPasses)
    , rng(rng)
    , weight(2 * numOfKernels + numOfStreamSets)
    , ordinal(numOfStreamSets)
    , live(2 * numOfKernels + numOfStreamSets)
    , component(numOfStreamSets)
    , placement(numOfStreamSets)
    , accum(2 * numOfKernels + numOfStreamSets) {

    }

public:

    unsigned Depth;

protected:

    const SchedulingGraph & S;

    const unsigned numOfKernels;
    const unsigned numOfStreamSets;

    const double maxCutRoundsFactor;
    const unsigned maxCutPasses;

    random_engine & rng;

private:

    std::vector<size_t>   weight;
    std::vector<unsigned> ordinal;


    std::vector<unsigned> live;    
    std::vector<unsigned> component;
    std::vector<unsigned> placement;

    std::vector<Vertex>   stack;
    std::vector<size_t>   accum;

};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief SchedulingAnalysisWorker
 ** ------------------------------------------------------------------------------------------------------------- */
struct SchedulingAnalysisWorker {

    using Candidate = PermutationBasedEvolutionaryAlgorithm::Candidate;

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief repair
     ** ------------------------------------------------------------------------------------------------------------- */
    virtual void repair(Candidate & candidate) = 0;

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief fitness
     ** ------------------------------------------------------------------------------------------------------------- */
    virtual size_t fitness(const Candidate & candidate) {
        return analyzer.analyze(candidate);
    }

protected:

    SchedulingAnalysisWorker(const SchedulingGraph & S,
                       const unsigned numOfKernels,
                       const unsigned candidateLength,
                       random_engine & rng, const double maxCutRoundsFactor, const unsigned maxCutPasses)
    : numOfKernels(numOfKernels)
    , candidateLength(candidateLength)
    , rng(rng)
    , analyzer(S, numOfKernels, rng, maxCutRoundsFactor, maxCutPasses) {

    }

public:

    const unsigned numOfKernels;
    const unsigned candidateLength;
    random_engine & rng;
    MemoryAnalysis analyzer;


};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief PartitionSchedulingAnalysis
 ** ------------------------------------------------------------------------------------------------------------- */
struct PartitionSchedulingAnalysisWorker final : public SchedulingAnalysisWorker {

    using Candidate = PermutationBasedEvolutionaryAlgorithm::Candidate;

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief repair
     ** ------------------------------------------------------------------------------------------------------------- */
    void repair(Candidate & L) override {

        for (unsigned i = 0; i != numOfKernels; ++i) {
            remaining[i] = in_degree(i, D) + 1;
        }
        assert (replacement.size() == numOfKernels);

        for (unsigned i = 0; i < numOfKernels; ) {
            bool progress = false;
            for (unsigned j = 0; j != numOfKernels; ++j) {
                const auto k = L[j];
                if (remaining[k] == 1) {
                    assert (i < numOfKernels);
                    replacement[i++] = k;
                    remaining[k] = 0;
                    for (auto e : make_iterator_range(out_edges(k, D))) {
                        const auto v = target(e, D);
                        assert (remaining[v] > 1);
                        --remaining[v];
                    }
                    progress = true;
                }
            }
            assert (progress);
        }
        L.swap(replacement);

    }


public:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief constructor
     ** ------------------------------------------------------------------------------------------------------------- */
    PartitionSchedulingAnalysisWorker(const SchedulingGraph & S, const PartitionDependencyGraph & D,
                                      const unsigned numOfKernels, random_engine & rng)
    : SchedulingAnalysisWorker(S, numOfKernels, numOfKernels, rng, 1, 1)
    , D(D)
    , replacement(numOfKernels)
    , remaining(numOfKernels) {

    }

private:

    const PartitionDependencyGraph & D;

    Candidate replacement;

    std::vector<unsigned> remaining;

};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief PartitionSchedulingAnalysis
 ** ------------------------------------------------------------------------------------------------------------- */
struct PartitionSchedulingAnalysis final : public PermutationBasedEvolutionaryAlgorithm {

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief initGA
     ** ------------------------------------------------------------------------------------------------------------- */
    bool initGA(Population & initialPopulation) override {

        // Any topological ordering of D can generate a valid schedule for our subgraph.
        // Begin by trying to generate N initial candidates. If we fail to enumerate all
        // of them, we'll use an evolutionary algorithm to try and explore the remaining
        // solution space.

        return enumerateUpToNTopologicalOrderings(D, INITIAL_TOPOLOGICAL_POPULATION_SIZE, [&](const Candidate & L) {
            insertCandidate(L, initialPopulation);
        });

    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief repair
     ** ------------------------------------------------------------------------------------------------------------- */
    void repairCandidate(Candidate & candidate) override {
        worker.repair(candidate);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief fitness
     ** ------------------------------------------------------------------------------------------------------------- */
    size_t fitness(const Candidate & candidate) override {
        return worker.fitness(candidate);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief constructor
     ** ------------------------------------------------------------------------------------------------------------- */
    PartitionSchedulingAnalysis(const SchedulingGraph & S, const PartitionDependencyGraph & D,
                                const unsigned numOfKernels, random_engine & rng)
    : PermutationBasedEvolutionaryAlgorithm(numOfKernels, PARITION_SCHEDULING_GA_ROUNDS, PARITION_SCHEDULING_GA_STALLS, MAX_PARTITION_POPULATION_SIZE, rng)
    , D(D)
    , worker(S, D, numOfKernels, rng) {

    }

private:

    const PartitionDependencyGraph & D;

    PartitionSchedulingAnalysisWorker worker;

};

} // end of anonymous namespace

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief analyzeDataflowWithinPartitions
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::analyzeDataflowWithinPartitions(PartitionGraph & P, random_engine & rng) const {

    /// --------------------------------------------
    /// Construct our partition schedules
    /// --------------------------------------------

    for (unsigned currentPartitionId = 1; currentPartitionId < PartitionCount; ++currentPartitionId) {

        // We begin by constructing a subgraph of this partition with just enough information to
        // form a bipartite graph of the kernel and streamset nodes.

        // The input streamset(s) to this partition are surpressed from the graph but any output
        // streamsets with an internal user will be recorded but flagged as External. Memory
        // colouring only wants to consider purely local streamsets but ought to consider
        // otherwise equivalent schedules that place kernels that produce outputs towards the
        // end of the schedule as superior.

        /// -----------------------------------------------------------------
        /// Identify the nodes / streamsets belonging to our i-th partition
        /// -----------------------------------------------------------------

        PartitionData & currentPartition = P[currentPartitionId];

        const auto S = makeIntraPartitionSchedulingGraph(P, currentPartitionId);

        constexpr auto fakeInput = 0U;
        constexpr auto firstKernel = 1U;

        const auto & kernels = currentPartition.Kernels;
        const auto numOfKernels = kernels.size();
        const auto fakeOutput = numOfKernels + 1U;

        // We want to generate a subgraph of S consisting of only the kernel nodes
        // but whose edges initially represent the transitive closure of S. Once we
        // generate this graph, we remove the edges associated with the streamsets.
        // The final graph is the kernel dependency graph of S.

        // TODO: we ought to reason about paths in D independently since they are
        // subgraphs of S with a single topological ordering.

        const auto D = makePartitionDependencyGraph(numOfKernels + 2U, S);

        // Now we begin the genetic algorithm phase; our overall goal is to find
        // a schedule that permits a minimum memory schedule.

        PartitionSchedulingAnalysis SA(S, D, numOfKernels + 2U, rng);

        SA.runGA();

        auto H = SA.getResult();

        const auto t = postorder_minimize(H);

        // We make a fake input and output vertex in each partition graph to enable
        // the program to better consider external I/O. Verify that the minimization
        // process worked correctly to identify them.

        assert (child(0, H) == 1);
        assert (in_degree(0, H) == 0);
        assert (H[first_out_edge(fakeInput, H)] == fakeInput);

        assert (in_degree(t, H) == 1);
        assert (out_degree(t, H) == 0);
        assert (H[first_in_edge(t, H)] == fakeOutput);

        clear_out_edges(0, H);
        clear_in_edges(t, H);

        for (const auto e : make_iterator_range(edges(H))) {
            auto & E = H[e];
            assert (fakeInput < E && E < fakeOutput);
            E = kernels[E - firstKernel];
        }

        currentPartition.Orderings = H;
    }

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeIntraPartitionSchedulingGraph
 ** ------------------------------------------------------------------------------------------------------------- */
SchedulingGraph PipelineAnalysis::makeIntraPartitionSchedulingGraph(const PartitionGraph & P,
                                                                    const unsigned currentPartitionId) const {

    const PartitionData & currentPartition = P[currentPartitionId];

    const auto & kernels = currentPartition.Kernels;
    const auto numOfKernels = kernels.size();

    flat_set<Vertex> streamSets;

    for (const auto u : kernels) {
        const RelationshipNode & node = Relationships[u];
        assert (node.Type == RelationshipNode::IsKernel);
        assert (PartitionIds.at(u) == currentPartitionId);
        for (const auto e : make_iterator_range(in_edges(u, Relationships))) {
            const auto binding = source(e, Relationships);
            if (Relationships[binding].Type == RelationshipNode::IsBinding) {
                const auto f = first_in_edge(binding, Relationships);
                assert (Relationships[f].Reason != ReasonType::Reference);
                const auto streamSet = source(f, Relationships);
                assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
                assert (isa<StreamSet>(Relationships[streamSet].Relationship));
                streamSets.insert(streamSet);
            }
        }
        for (const auto e : make_iterator_range(out_edges(u, Relationships))) {
            const auto binding = target(e, Relationships);
            if (Relationships[binding].Type == RelationshipNode::IsBinding) {
                const auto f = first_out_edge(binding, Relationships);
                assert (Relationships[f].Reason != ReasonType::Reference);
                const auto streamSet = target(f, Relationships);
                assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
                assert (isa<StreamSet>(Relationships[streamSet].Relationship));
                streamSets.insert(streamSet);
            }
        }
    }

    const auto numOfStreamSets = streamSets.size();

    constexpr auto fakeInput = 0U;

    const auto fakeOutput = numOfKernels + 1U;

    const auto firstStreamSet = fakeOutput + 1U;

    auto getStreamSetIndex = [&](const unsigned streamSet) {
        const auto g = streamSets.find(streamSet);
        assert (g != streamSets.end());
        return firstStreamSet + std::distance(streamSets.begin(), g);
    };

    const auto n = firstStreamSet + numOfStreamSets;

    SchedulingGraph G(n);

    for (auto i = fakeInput; i <= fakeOutput; ++i) {
        SchedulingNode & N = G[i];
        N.Type = SchedulingNode::IsKernel;
    }
    for (auto i = firstStreamSet; i < n; ++i) {
        SchedulingNode & N = G[i];
        N.Type = SchedulingNode::IsStreamSet;
    }

    for (auto i = fakeInput + 1; i < fakeOutput; ++i) {
        const auto u = kernels[i - 1U];
        const RelationshipNode & node = Relationships[u];
        assert (node.Type == RelationshipNode::IsKernel);
        const auto strideSize = currentPartition.Repetitions[i - 1U] * node.Kernel->getStride();
        assert (strideSize > 0);

        for (const auto e : make_iterator_range(in_edges(u, Relationships))) {
            const auto binding = source(e, Relationships);
            if (Relationships[binding].Type == RelationshipNode::IsBinding) {
                const auto f = first_in_edge(binding, Relationships);
                assert (Relationships[f].Reason != ReasonType::Reference);
                const auto streamSet = source(f, Relationships);
                assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
                assert (isa<StreamSet>(Relationships[streamSet].Relationship));
                const auto j = getStreamSetIndex(streamSet);
                assert (j < num_vertices(G));
                const RelationshipNode & rn = Relationships[binding];
                const Binding & b = rn.Binding;
                const ProcessingRate & rate = b.getRate();
                if (rate.isGreedy()) {
                    if (in_degree(j, G) > 0) {
                        const auto f = first_in_edge(j, G);
                        add_edge(j, i, G[f], G);
                    } else {
                        #warning handle greedy rates better here
                        add_edge(j, i, Rational{1}, G);
                    }
                } else {
                    // If we have a PopCount producer/consumer in the same partition,
                    // they're both perform an identical number of strides. So long
                    // as the producing/consuming strideRate match, the equation will
                    // work. Since the lower bound of PopCounts is 0, we always use the
                    // upper bound.
                    const auto itemsPerStride = rate.getUpperBound() * strideSize;
                    add_edge(j, i, itemsPerStride, G);
                }
            }
        }

        for (const auto e : make_iterator_range(out_edges(u, Relationships))) {
            const auto binding = target(e, Relationships);
            if (Relationships[binding].Type == RelationshipNode::IsBinding) {
                const auto f = first_out_edge(binding, Relationships);
                assert (Relationships[f].Reason != ReasonType::Reference);
                const auto streamSet = target(f, Relationships);
                assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
                assert (isa<StreamSet>(Relationships[streamSet].Relationship));
                const auto j = getStreamSetIndex(streamSet);
                assert (j < num_vertices(G));

                const RelationshipNode & rn = Relationships[binding];
                const Binding & b = rn.Binding;
                const Rational bytesPerItem{b.getFieldWidth() * b.getNumElements(), 8};
                assert (bytesPerItem > Rational{0});

                SchedulingNode & SN = G[j];
                SN.Size = bytesPerItem;

                const ProcessingRate & rate = b.getRate();
                const auto itemsPerStride = rate.getUpperBound() * strideSize;
                add_edge(i, j, itemsPerStride, G);
            }
        }
    }

    // add fake input arcs
    flat_set<unsigned> externalStreamSets;
    for (const auto e : make_iterator_range(in_edges(currentPartitionId, P))) {
        externalStreamSets.insert(P[e]);
    }

    for (const auto streamSet : externalStreamSets) {

        const auto j = getStreamSetIndex(streamSet);

        const auto f = first_in_edge(streamSet, Relationships);
        assert (Relationships[f].Reason != ReasonType::Reference);
        const auto binding = source(f, Relationships);
        assert (Relationships[binding].Type == RelationshipNode::IsBinding);

        const RelationshipNode & rn = Relationships[binding];
        const Binding & b = rn.Binding;
        // prioritize inter-partition input consumption by doubling the conceptual buffer size
        const Rational bytesPerItem{b.getFieldWidth() * b.getNumElements(), (8 / 2)};

        SchedulingNode & SN = G[j];
        SN.Size = bytesPerItem;

        SchedulingGraph::out_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = out_edges(j, G);
        auto itemsPerStride = G[*ei];
        while (++ei != ei_end) {
            const auto & r = G[*ei];
            if (r > itemsPerStride) {
                itemsPerStride = r;
            }
        }

        add_edge(fakeInput, j, itemsPerStride, G);
    }

    for (auto i = fakeInput + 1; i < fakeOutput; ++i) {
        const auto u = kernels[i - 1U];
        const RelationshipNode & node = Relationships[u];
        assert (node.Type == RelationshipNode::IsKernel);
        const auto strideSize = currentPartition.Repetitions[i - 1U] * node.Kernel->getStride();
        assert (strideSize > 0);

        for (const auto e : make_iterator_range(in_edges(u, Relationships))) {
            const auto binding = source(e, Relationships);
            if (Relationships[binding].Type == RelationshipNode::IsBinding) {
                const auto f = first_in_edge(binding, Relationships);
                assert (Relationships[f].Reason != ReasonType::Reference);
                const auto streamSet = source(f, Relationships);
                assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
                assert (isa<StreamSet>(Relationships[streamSet].Relationship));
                const auto j = getStreamSetIndex(streamSet);
                assert (j < num_vertices(G));
                const RelationshipNode & rn = Relationships[binding];
                const Binding & b = rn.Binding;
                const ProcessingRate & rate = b.getRate();
                if (rate.isGreedy()) {
                    const auto f = first_in_edge(j, G);
                    add_edge(j, i, G[f], G);
                } else {
                    // If we have a PopCount producer/consumer in the same partition,
                    // they're both perform an identical number of strides. So long
                    // as the producing/consuming strideRate match, the equation will
                    // work. Since the lower bound of PopCounts is 0, we always use the
                    // upper bound.
                    const auto itemsPerStride = rate.getUpperBound() * strideSize;
                    add_edge(j, i, itemsPerStride, G);
                }
            }
        }
    }

    // add fake output arcs
    externalStreamSets.clear();
    for (const auto e : make_iterator_range(out_edges(currentPartitionId, P))) {
        externalStreamSets.insert(P[e]);
    }

    for (const auto streamSet : externalStreamSets) {
        const auto j = getStreamSetIndex(streamSet);
        SchedulingGraph::in_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = in_edges(j, G);
        auto itemsPerStride = G[*ei];
        while (++ei != ei_end) {
            const auto & r = G[*ei];
            if (r > itemsPerStride) {
                itemsPerStride = r;
            }
        }
        // defer inter-partition output production by doubling the conceptual buffer size
        SchedulingNode & SN = G[j];
        SN.Size *= 2;

        add_edge(j, fakeOutput, itemsPerStride, G);
    }

#if 0
    auto & out = errs();

    out << "digraph \"G\" {\n";
    for (auto v : make_iterator_range(vertices(G))) {
        out << "v" << v << " [label=\"" << v << ". ";
        const SchedulingNode & N = G[v];
        if (N.Type == SchedulingNode::IsKernel) {
            if (v > fakeInput && v < fakeOutput) {
                const auto u = kernels[v - 1U];
                const RelationshipNode & node = Relationships[u];
                assert (node.Type == RelationshipNode::IsKernel);
                out << node.Kernel->getName();
            } else {
                out << "K";
            }
        } else {
            assert (v >= firstStreamSet);
            const auto k = v - firstStreamSet;
            assert (k < streamSets.size());
            const auto i = streamSets.begin() + k;
            out << "S" << *i << " : ";
            const auto & R = N.Size;
            out << R.numerator() << "/" << R.denominator();
        }
        out << "\"];\n";
    }
    for (auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);
        out << "v" << s << " -> v" << t << " [label=\"";
        const auto & R = G[e];
        out << R.numerator() << "/" << R.denominator();
        out << "\"];\n";
    }

    out << "}\n\n";
    out.flush();
#endif

    #ifndef NDEBUG
    for (auto i = firstStreamSet; i < n; ++i) {
        assert (degree(i, G) != 0);
        assert (G[i].Size > Rational{0});
    }
    for (const auto e : make_iterator_range(edges(G))) {
        assert (G[e] > Rational{0});
    }
    #endif

    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePartitionDependencyGraph
 ** ------------------------------------------------------------------------------------------------------------- */
PartitionDependencyGraph PipelineAnalysis::makePartitionDependencyGraph(const unsigned numOfKernels,
                                                                        const SchedulingGraph & S) const {

    PartitionDependencyGraph G(numOfKernels);

    const auto firstStreamSet = numOfKernels;
    const auto lastStreamSet = num_vertices(S);

    flat_set<Vertex> consumers;
    for (auto streamSet = firstStreamSet; streamSet < lastStreamSet; ++streamSet) {
        const auto producer = parent(streamSet, S);
        assert (producer < numOfKernels);
        for (const auto e : make_iterator_range(out_edges(streamSet, S))) {
            consumers.insert(target(e, S));
        }
        for (const auto consumer : consumers) {
            assert (consumer < numOfKernels);
            add_edge(producer, consumer, G);
        }
        consumers.clear();
    }

    // make sure the fake I/O kernels are always first and last in the ordering.

    constexpr auto fakeInput = 0U;
    const auto fakeOutput = numOfKernels - 1U;

    for (unsigned i = 1; i < fakeOutput; ++i) {
        if (!edge(fakeInput, i, G).second) {
            add_edge(fakeInput, i, G);
        }
        if (!edge(i, fakeOutput, G).second) {
            add_edge(i, fakeOutput, G);
        }
    }

    reverse_traversal ordering{numOfKernels};
    assert (is_valid_topological_sorting(ordering, G));
    transitive_reduction_dag(ordering, G);

    return G;
}


enum class ProgramScheduleType {
    RandomWalk
    , FirstTopologicalOrdering
    , NearestTopologicalOrdering
};


namespace { // anonymous namespace

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief ProgramSchedulingAnalysis
 ** ------------------------------------------------------------------------------------------------------------- */
struct ProgramSchedulingAnalysisWorker final : public SchedulingAnalysisWorker {

    using TargetVector = std::vector<std::pair<Vertex, double>>;

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief repair
     ** ------------------------------------------------------------------------------------------------------------- */
    void repair(Candidate & candidate) override {
        const auto start = std::chrono::high_resolution_clock::now();
        if (mode == ProgramScheduleType::FirstTopologicalOrdering) {
            first_valid_schedule(candidate);
        } else if (mode == ProgramScheduleType::NearestTopologicalOrdering) {
            nearest_valid_schedule(candidate);
        }
        const auto end = std::chrono::high_resolution_clock::now();
        repair_time += (end - start).count();
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief fitness
     ** ------------------------------------------------------------------------------------------------------------- */
    size_t fitness(const Candidate & candidate) override {
        const auto start = std::chrono::high_resolution_clock::now();
        auto result = std::numeric_limits<size_t>::max();
        if (mode != ProgramScheduleType::RandomWalk || is_valid_hamiltonian_path(candidate)) {
            result = analyzer.analyze(candidate);
        }
        const auto end = std::chrono::high_resolution_clock::now();
        fitness_time += (end - start).count();
        FitnessDepth.push_back(analyzer.Depth);
        return result;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief is_valid_hamiltonian_path
     ** ------------------------------------------------------------------------------------------------------------- */
    bool is_valid_hamiltonian_path(const Candidate & candidate) const {
        assert (candidate.size() == numOfKernels);
        std::function<bool(Vertex, unsigned)> recursive_verify = [&](Vertex u, unsigned index) {
            for (const auto l : O[u]) {
                if (candidate[index] != l) {
                    return false;
                }
                ++index;
            }
            for (const auto e : make_iterator_range(out_edges(u, O))) {
                const auto v = target(e, O);
                if (recursive_verify(v, index)) {
                    return true;
                }
            }
            return (index == numOfKernels);
        };
        return recursive_verify(0, 0);
    };

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief first_valid_schedule
     ** ------------------------------------------------------------------------------------------------------------- */
    void first_valid_schedule(Candidate & candidate) {

        assert (candidate.size() == numOfKernels);
        BitVector unselected(numOfKernels, true);

        Vertex root = 0;

        Candidate replacement;
        replacement.reserve(numOfKernels);

        SmallVector<Vertex, 4> path;

        for (;;) {
            const auto i = unselected.find_first();
            assert (i != -1);
            const auto label = candidate[i];

            std::function<bool(Vertex, unsigned)> iterative_bfs = [&](const Vertex u, unsigned depth) {
                if (depth == 1) {
                    for (const auto l : O[u]) {
                        if (l == label) {
                            goto write_path;
                        }
                    }
                } else {
                    for (const auto e : make_iterator_range(out_edges(u, O))) {
                        const auto v = target(e, O);
                        if (iterative_bfs(v, depth - 1)) {
                            goto write_path;
                        }
                    }
                }
                return false;
                // -----------------------------------
write_path:     path.push_back(u);
                return true;
            };

            assert (path.empty());

            for (unsigned depth = 1;;++depth) {
                if (iterative_bfs(root, depth)) {
                    for (const auto v : reverse(path)) {
                        for (const auto l : O[v]) {
                            if (LLVM_LIKELY(unselected.test(l))) {
                                replacement.push_back(l);
                                unselected.reset(i);
                            }
                        }
                    }
                    root = path.front();
                    break;
                }
            }

            if (unselected.empty()) {
                break;
            }

            path.clear();
        }

        assert (is_valid_hamiltonian_path(replacement));

        assert (replacement.size() == numOfKernels);
        candidate.swap(replacement);

    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief nearest_valid_schedule
     ** ------------------------------------------------------------------------------------------------------------- */
    void nearest_valid_schedule(Candidate & candidate) {

        assert (candidate.size() == numOfKernels);
        assert (index.size() == numOfKernels);

        // record the index position of each kernel in the candidate
        for (unsigned i = 0; i < numOfKernels; ++i) {
            const auto j = candidate[i];
            assert (j < numOfKernels);
            index[j] = i;
        }

        const double normalizing_factor = (numOfKernels * (numOfKernels - 1)) + 1;

        auto inversion_cost = [&](const unsigned m) {

            for (unsigned i = 0; i < m; ++i) {
                const auto k = toEval[i];
                assert (k < numOfKernels);
                tau_aux[k] = i;
            }
            for (unsigned i = 0; i < m; ++i) {
                tau_offset[i] = tau_aux[index[i]];
            }

            size_t inversions = 0;

            std::function<void(unsigned, unsigned)> inversion_count = [&](const unsigned lo, const unsigned hi) {
                if (lo < hi) {
                    const auto mid = (lo + hi) / 2;
                    inversion_count(lo, mid);
                    inversion_count(mid + 1, hi);

                    for (auto i = lo; i <= hi; ++i) {
                        tau_aux[i] = tau_offset[i];
                    }
                    auto i = lo;
                    auto j = mid + 1;
                    for (auto k = lo; k <= hi; ++k) {
                        if (i > mid) {
                            tau_offset[k] = tau_aux[j++];
                        } else if (j > hi) {
                            tau_offset[k] = tau_aux[i++];
                        } else if (tau_aux[j] < tau_aux[i]) {
                            tau_offset[k] = tau_aux[j++];
                            inversions += (mid - i + 1);
                        } else {
                            tau_offset[k] = tau_aux[i++];
                        }
                    }
                }
            };

            inversion_count(0, m - 1);

            return (((double)(2 * inversions)) / normalizing_factor) + (numOfKernels - m);
        };

restart_process:

        double bestInversionCost = numOfKernels;

        std::array<double, HAMILTONIAN_PATH_NUM_OF_ANTS> pathCost;

        for (auto & e : trail) {
            e.second = HAMILTONIAN_PATH_ACO_TAU_INITIAL_VALUE;
        }

        replacement.clear();

        size_t r = 0;

        for (;r < SCHEDULING_FITNESS_COST_ACO_ROUNDS;++r) {

            visited.reset();

            Vertex u = 0;

            const auto pathIdx = (r % HAMILTONIAN_PATH_NUM_OF_ANTS);

            auto & P = path[pathIdx];

            P.clear();

            for (;;) {

                assert (u < num_vertices(O));

                assert (!visited.test(u));

                visited.set(u);

                P.push_back(u);

                targets.clear();

                double sum = 0.0;

                for (const auto e : make_iterator_range(out_edges(u, O))) {
                    const auto v = target(e, O);
                    if (visited.test(v)) continue;
                    const auto f = trail.find(std::make_pair(u, v));
                    assert (f != trail.end());
                    const auto a = f->second;
                    const auto value = O[e] * a * a * a;
                    sum += value;
                    targets.emplace_back(v, sum);
                }

                std::uniform_real_distribution<double> distribution(0.0, sum);
                const auto c = distribution(rng);
                assert (c < sum);

                static struct _TargetComparator {
                    using T = TargetVector::value_type;
                    bool operator() (const T & a, const T & b) {
                        return a.second < b.second;
                    }
                    bool operator() (const T::second_type a, const T & b) {
                        return a < b.second;
                    }
                    bool operator() (const T & a, const T::second_type b) {
                        return a.second < b;
                    }
                } comp;

                const auto f = std::upper_bound(targets.begin(), targets.end(), c, comp);
                assert (f != targets.end());
                u = f->first; // set our next target
            }

            // extract the sequence of kernel ids from the path
            toEval.clear();
            for (const auto i : P) {
                const auto & A = O[i];
                toEval.insert(toEval.end(), A.begin(), A.end());
            }
            const auto m = toEval.size();

            assert (m <= numOfKernels);

            pathCost[pathIdx] = inversion_cost(m);

            if (pathIdx == (HAMILTONIAN_PATH_NUM_OF_ANTS - 1)) {

                auto updatePath = [&](const Candidate & path, const double inversionCost) {

                    const auto l = path.size();

                    if (inversionCost > bestInversionCost) {

                        const auto d = inversionCost - bestInversionCost;
                        const auto deposit = d / (HAMILTONIAN_PATH_INVERSE_K + d);

                        for (unsigned i = 1; i < l; ++i) {
                            const auto e = std::make_pair(path[i - 1], path[i]);
                            const auto f = trail.find(e);
                            assert (f != trail.end());
                            double & t = f->second;
                            t = std::max(t - deposit, HAMILTONIAN_PATH_ACO_TAU_MIN);
                        }

                    } else if (inversionCost < bestInversionCost) {
                        const auto d = bestInversionCost - inversionCost;
                        const auto deposit = d / (HAMILTONIAN_PATH_INVERSE_K + d);

                        for (unsigned i = 1; i < l; ++i) {
                            const auto e = std::make_pair(path[i - 1], path[i]);
                            const auto f = trail.find(e);
                            assert (f != trail.end());
                            double & t = f->second;
                            t = std::min(t + deposit, HAMILTONIAN_PATH_ACO_TAU_MAX);
                        }

                    }

                };

                auto highestInversionCost = std::numeric_limits<double>::min();
                auto highestInversion = 0U;
                auto lowestInversionCost = std::numeric_limits<double>::max();
                auto lowestInversion = 0U;

                for (unsigned i = 0; i < HAMILTONIAN_PATH_NUM_OF_ANTS; ++i) {
                    if (highestInversionCost < pathCost[i]) {
                        highestInversionCost = pathCost[i];
                        highestInversion = i;
                    }
                    if (lowestInversionCost > pathCost[i]) {
                        lowestInversionCost = pathCost[i];
                        lowestInversion = i;
                    }
                }

                updatePath(path[highestInversion], highestInversionCost);
                updatePath(path[lowestInversion], lowestInversionCost);

                if (bestInversionCost > lowestInversionCost) {
                    bestInversionCost = lowestInversionCost;

                    // reconstruct the repaired ordering
                    toEval.clear();
                    for (const auto i : path[lowestInversion]) {
                        const auto & A = O[i];
                        toEval.insert(toEval.end(), A.begin(), A.end());
                    }
                    const auto m = toEval.size();
                    assert (m <= numOfKernels);
                    // Store our path if its the best one
                    if (LLVM_LIKELY(m == numOfKernels)) {
                        replacement.swap(toEval);
                    }
                }

            }

        }

        // If we converged to a solution but failed to find a valid hamiltonian path,
        // just restart the process. We're guaranteed to find one eventually.
        if (LLVM_UNLIKELY(replacement.empty())) {
            assert (bestInversionCost >= 1.0);
            goto restart_process;
        }

        assert (bestInversionCost < 1.0);
        assert (replacement.size() == numOfKernels);
        assert (is_valid_hamiltonian_path(replacement));

        candidate.swap(replacement);

    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief makeRandomCandidate
     ** ------------------------------------------------------------------------------------------------------------- */
    Candidate makeRandomCandidate() {
        Candidate candidate;
        candidate.reserve(candidateLength);

        for (;;) {
            Vertex u = 0;
            visited.reset();

            for (;;) {

                assert (u < visited.size());

                visited.set(u);

                for (const auto l : O[u]) {
                    candidate.push_back(l);
                }

                if (out_degree(u, O) == 0) {
                    break;
                }

                double sum = 0.0;
                for (const auto e : make_iterator_range(out_edges(u, O))) {
                    if (!visited.test(target(e, O))) {
                        const auto value = O[e];
                        assert (value > 0.0);
                        sum += value;
                    }
                }
                if (LLVM_UNLIKELY(sum == 0.0)) {
                    break;
                }

                std::uniform_real_distribution<double> distribution(0.0, sum);
                const auto c = distribution(rng);

                bool found = false;
                double d = std::numeric_limits<double>::epsilon();
                for (const auto e : make_iterator_range(out_edges(u, O))) {
                    if (!visited.test(target(e, O))) {
                        d += O[e];
                        if (d >= c) {
                            u = target(e, O); // set our next target
                            found = true;
                            break;
                        }
                    }
                }
                assert (found);
            }
            if (candidate.size() == candidateLength) {
                break;
            }
            candidate.clear();
        }
        assert (is_valid_hamiltonian_path(candidate));
        return candidate;
    }

public:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief constructor
     ** ------------------------------------------------------------------------------------------------------------- */
    ProgramSchedulingAnalysisWorker(const SchedulingGraph & S,
                                    const PartitionOrderingGraph & O,
                                    const ProgramScheduleType mode,
                                    const unsigned numOfKernels,
                                    const unsigned maxPathLength,
                                    random_engine & rng, const double maxCutRoundsFactor, const unsigned maxCutPasses)
    : SchedulingAnalysisWorker(S, numOfKernels, numOfKernels, rng, maxCutRoundsFactor, maxCutPasses)
    , O(O)
    , mode(mode)
    , visited(num_vertices(O))
    , index(numOfKernels)
    , tau_aux(numOfKernels)
    , tau_offset(numOfKernels) {

        assert (num_vertices(O) > 0);

        for (unsigned i = 0; i < HAMILTONIAN_PATH_NUM_OF_ANTS; ++i) {
            path[i].reserve(maxPathLength);
        }

        replacement.reserve(numOfKernels);
        toEval.reserve(numOfKernels);
        trail.reserve(num_edges(O));

        for (const auto e : make_iterator_range(edges(O))) {
            const unsigned u = source(e, O);
            const unsigned v = target(e, O);
            trail.emplace(std::make_pair(u, v), HAMILTONIAN_PATH_ACO_TAU_INITIAL_VALUE);
        }

        FitnessDepth.reserve(10000);

    }

public:

    std::vector<unsigned> FitnessDepth;

private:

    const PartitionOrderingGraph & O;

    const ProgramScheduleType mode;

    BitVector visited;
    TargetVector targets;

    flat_map<std::pair<Vertex, Vertex>, double> trail;
    std::vector<unsigned> index;
    std::array<Candidate, HAMILTONIAN_PATH_NUM_OF_ANTS> path;
    Candidate toEval;

    Candidate bestPath;
    Candidate worstPath;

    Candidate replacement;

    std::vector<unsigned> tau_aux;
    std::vector<unsigned> tau_offset;

};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief ProgramSchedulingAnalysis
 ** ------------------------------------------------------------------------------------------------------------- */
struct ProgramSchedulingAnalysis final : public PermutationBasedEvolutionaryAlgorithm {

    static_assert(INITIAL_SCHEDULING_POPULATION_ATTEMPTS >= INITIAL_SCHEDULING_POPULATION_SIZE,
        "cannot have fewer attemps than population size");

    static_assert(INITIAL_SCHEDULING_POPULATION_SIZE <= MAX_PROGRAM_POPULATION_SIZE,
        "cannot have a larger initial population size than generational population size");

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief initGA
     ** ------------------------------------------------------------------------------------------------------------- */
    bool initGA(Population & initialPopulation) override {
        for (unsigned i = 0; i < INITIAL_SCHEDULING_POPULATION_ATTEMPTS; ++i) {
            if (insertCandidate(worker.makeRandomCandidate(), initialPopulation)) {
                if (initialPopulation.size() >= INITIAL_SCHEDULING_POPULATION_SIZE) {
                    return false;
                }
            }
        }
        return true;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief repair
     ** ------------------------------------------------------------------------------------------------------------- */
    void repairCandidate(Candidate & candidate) override {
        worker.repair(candidate);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief fitness
     ** ------------------------------------------------------------------------------------------------------------- */
    size_t fitness(const Candidate & candidate) override {
        return worker.fitness(candidate);
    }

    void report() {
        size_t sum = 0;
        for (auto depth : worker.FitnessDepth) {
            sum += depth;
        }
        const double avg = ((double)sum) / ((double)worker.FitnessDepth.size());

        double var = 0.0;
        for (auto depth : worker.FitnessDepth) {
            const double d = (double)depth - avg;
            var += (d * d);
        }

        // mean, stddev, count
        errs() << format("%.5f", avg) << "," << format("%.5f", std::sqrt(var)) << "," << worker.FitnessDepth.size();
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief constructor
     ** ------------------------------------------------------------------------------------------------------------- */
    ProgramSchedulingAnalysis(const SchedulingGraph & S,
                              const PartitionOrderingGraph & O,
                              const ProgramScheduleType mode,
                              const unsigned numOfKernels,
                              const unsigned maxPathLength,
                              random_engine & rng, const double maxCutRoundsFactor, const unsigned maxCutPasses)
    : PermutationBasedEvolutionaryAlgorithm(numOfKernels, PROGRAM_SCHEDULING_GA_ROUNDS, PROGRAM_SCHEDULING_GA_STALLS, MAX_PROGRAM_POPULATION_SIZE, rng)
    , worker(S, O, mode, numOfKernels, maxPathLength, rng, maxCutRoundsFactor, maxCutPasses) {

    }

private:

    ProgramSchedulingAnalysisWorker worker;

};

} // end of anonymous namespace

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief analyzeDataflowBetweenPartitions
 ** ------------------------------------------------------------------------------------------------------------- */
PartitionDataflowGraph PipelineAnalysis::analyzeDataflowBetweenPartitions(PartitionGraph & P) const {

    const auto activePartitions = (PartitionCount - 1);

    // create a bipartite graph consisting of partitions and cross-partition
    // streamset nodes and relationships

    flat_set<Vertex> streamSets;

    for (unsigned partitionId = 1; partitionId < PartitionCount; ++partitionId) {
        for (const auto e : make_iterator_range(out_edges(partitionId, P))) {
            const auto streamSet = P[e];
            assert (streamSet < num_vertices(Relationships));
            assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
            assert (isa<StreamSet>(Relationships[streamSet].Relationship));
            streamSets.insert(streamSet);
        }
    }

    const auto numOfStreamSets = streamSets.size();

    const auto n = activePartitions + numOfStreamSets;


    PartitionDataflowGraph G(n);

    for (unsigned i = 0; i < numOfStreamSets; ++i) {

        const auto streamSet = *streamSets.nth(i);

        const auto streamSetNode = activePartitions + i;
        assert (in_degree(streamSetNode, G) == 0);

        const auto f = first_in_edge(streamSet, Relationships);
        assert (Relationships[f].Reason != ReasonType::Reference);

        const auto binding = source(f, Relationships);
        const RelationshipNode & output = Relationships[binding];
        assert (output.Type == RelationshipNode::IsBinding);

        const auto g = first_in_edge(binding, Relationships);
        assert (Relationships[g].Reason != ReasonType::Reference);
        const unsigned producer = source(g, Relationships);

        const auto producerPartitionId = PartitionIds.find(producer)->second;
        assert (producerPartitionId > 0);

        const Binding & outputBinding = output.Binding;
        const Rational bytesPerItem{outputBinding.getFieldWidth() * outputBinding.getNumElements(), 8};

        G[streamSetNode] = bytesPerItem;

        const ProcessingRate & rate = outputBinding.getRate();


        Rational expectedOutput{0};
        if (LLVM_LIKELY(!rate.isUnknown())) {

            const RelationshipNode & node = Relationships[producer];
            assert (node.Type == RelationshipNode::IsKernel);
            const PartitionData & N = P[producerPartitionId];
            const auto & K = N.Kernels;
            const auto h = std::find(K.begin(), K.end(), producer);
            assert (h != K.end());
            const auto index = std::distance(K.begin(), h);

            const auto strideSize = node.Kernel->getStride() * N.Repetitions[index];
            const auto sum = rate.getLowerBound() + rate.getUpperBound();

            expectedOutput = sum * strideSize * Rational{1, 2};
        }

        add_edge(producerPartitionId - 1, streamSetNode, PartitionDataflowEdge{producer, expectedOutput}, G);

        assert (in_degree(streamSetNode, G) == 1);

        for (const auto e : make_iterator_range(out_edges(streamSet, Relationships))) {
            const auto binding = target(e, Relationships);
            const RelationshipNode & input = Relationships[binding];
            if (LLVM_LIKELY(input.Type == RelationshipNode::IsBinding)) {
                const auto f = first_out_edge(binding, Relationships);
                assert (Relationships[f].Reason != ReasonType::Reference);
                const unsigned consumer = target(f, Relationships);

                const auto consumerPartitionId = PartitionIds.find(consumer)->second;
                assert (producerPartitionId <= consumerPartitionId);

                if (producerPartitionId != consumerPartitionId) {

                    const Binding & inputBinding = input.Binding;
                    const ProcessingRate & rate = inputBinding.getRate();

                    Rational expectedInput{0};

                    if (LLVM_LIKELY(!rate.isGreedy())) {

                        const RelationshipNode & node = Relationships[consumer];
                        assert (node.Type == RelationshipNode::IsKernel);

                        const PartitionData & N = P[consumerPartitionId];
                        const auto & K = N.Kernels;
                        const auto h = std::find(K.begin(), K.end(), consumer);
                        assert (h != K.end());
                        const auto index = std::distance(K.begin(), h);

                        const auto strideSize = node.Kernel->getStride() * N.Repetitions[index];

                        const auto sum = rate.getLowerBound() + rate.getUpperBound();
                        expectedInput = sum * strideSize * Rational{1, 2};
                    }

                    add_edge(streamSetNode, consumerPartitionId - 1, PartitionDataflowEdge{consumer, expectedInput}, G);
                }
            }
        }
    }

#if 0

    auto & out = errs();

    out << "digraph \"PD\" {\n";
    for (auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);

        const PartitionDataflowEdge & E = G[e];

        out << "v" << s << " -> v" << t << " [label=\""
            << E.KernelId << " : "
            << E.Expected.numerator() << "/" << E.Expected.denominator()
            << "\"];\n";
    }

    out << "}\n\n";
    out.flush();

#endif

    for (unsigned partitionId = 1; partitionId < PartitionCount; ++partitionId) {
        PartitionData & N = P[partitionId];
        N.ExpectedRepetitions = Rational{1};

//        errs() << "PARTITION " << partitionId << " := " <<
//                  N.ExpectedRepetitions.numerator() << "/" << N.ExpectedRepetitions.denominator() << "\n";
    }

    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeInterPartitionSchedulingGraph
 ** ------------------------------------------------------------------------------------------------------------- */
PartitionOrdering PipelineAnalysis::makeInterPartitionSchedulingGraph(PartitionGraph & P,
                                                                      const PartitionDataflowGraph & D) const {

    // Our goal is to find a topological ordering of the partitions such that
    // (1) the distance each partition can "jump" (i.e. the number of subsequent
    // partitions it can safely skip given the observation that if this partition
    // produces no data, any partition that is strictly dominated by the output
    // of this partition cannot either) is maximal and (2) the expected memory
    // usage is minimal.

    // To satisfy (1), we know that every topological ordering that we could want
    // is a depth-first ordering of the transitive reduction of D.

    // We begin this algorithm by constructing an auxillary graph H in which
    // any *hamiltonian path* through H would be a valid topological ordering
    // of D. We then use H to construct a more complicated graph that contains
    // the kernel nodes that have cross-partition I/O and return it to the user.

    using BV = dynamic_bitset<>;
    using PathGraph = adjacency_list<hash_setS, vecS, bidirectionalS, no_property, double>;

    const auto activePartitions = (PartitionCount - 1);

    assert (activePartitions > 1);

    PathGraph H(activePartitions + 2);

    flat_set<unsigned> kernels;
    kernels.reserve(PartitionIds.size());

    // since we could have multiple source/sink nodes in P, we always
    // add two fake nodes to H for a common source/sink.

    const auto l = activePartitions + 2U;

    BV M(l);

    for (unsigned i = 0; i < activePartitions; ++i) {

        if (in_degree(i, D) == 0) {
            add_edge(0, i + 1U, HAMILTONIAN_PATH_EPSILON_WEIGHT, H);
        } else {
            for (const auto e : make_iterator_range(in_edges(i, D))) {
                const PartitionDataflowEdge & E = D[e];
                kernels.insert(E.KernelId);
            }
        }

        if (out_degree(i, D) == 0) {
            add_edge(i + 1U, activePartitions + 1U, HAMILTONIAN_PATH_EPSILON_WEIGHT, H);
        } else {
            assert (M.none());
            for (const auto e : make_iterator_range(out_edges(i, D))) {
                const PartitionDataflowEdge & E = D[e];
                assert (Relationships[E.KernelId].Type == RelationshipNode::IsKernel);
                kernels.insert(E.KernelId);
                const auto streamSet = target(e, D);
                assert (streamSet >= activePartitions);
                for (const auto f : make_iterator_range(out_edges(streamSet, D))) {
                    assert (Relationships[D[f].KernelId].Type == RelationshipNode::IsKernel);
                    const auto k = target(f, D);
                    assert (i < k && k < activePartitions);
                    M.set(k);
                }
            }
            assert (M.any());
            for (auto j = M.find_first(); j != BV::npos; j = M.find_next(j)) {
                add_edge(i + 1U, j + 1U, HAMILTONIAN_PATH_EPSILON_WEIGHT, H);
            }
            M.reset();
        }
        assert (in_degree(i + 1, H) > 0);
        assert (out_degree(i + 1, H) > 0);
    }


    BEGIN_SCOPED_REGION
    const reverse_traversal ordering{l};
    assert (is_valid_topological_sorting(ordering, H));
    transitive_closure_dag(ordering, H, HAMILTONIAN_PATH_EPSILON_WEIGHT);
    transitive_reduction_dag(ordering, H);
    END_SCOPED_REGION

    // To find our hamiltonian path later, we need a path from each join
    // in the graph to the other forked paths (including the implicit
    // "terminal" node.) Determine the anscestors of each node then
    // link the parent of each join to the child of its associated fork
    // so long as the child is not an ancestor of the parent.

    BEGIN_SCOPED_REGION

    std::vector<BV> postdom(l);

    // We use Lengauer-Tarjan algorithm but since we know H is acyclic,
    // we know that we reach the fix-point after a single round.

    for (unsigned i = l; i--; ) { // reverse topological ordering
        BV & P = postdom[i];
        P.resize(activePartitions + 2);
        if (LLVM_LIKELY(out_degree(i, H) > 0)) {
            P.set(); // set all to 1
            for (const auto e : make_iterator_range(out_edges(i, H))) {
                const auto v = target(e, H);
                // assert (v > i);
                const BV & D = postdom[v];
                assert (D.size() == (activePartitions + 2));
                assert (D.test(v));
                P &= D;
            }
        }
        P.set(i);
    }

    std::vector<unsigned> rank(l);
    for (unsigned i = 0; i < l; ++i) { // forward topological ordering
        unsigned newRank = 0;
        for (const auto e : make_iterator_range(in_edges(i, H))) {
            newRank = std::max(newRank, rank[source(e, H)]);
        }
        rank[i] = newRank + 1;
    }

    std::vector<unsigned> occurences(l);
    std::vector<unsigned> singleton(l);

    std::vector<std::bitset<2>> ancestors(l);

    flat_set<std::pair<Vertex, Vertex>> toAdd;

    for (unsigned i = 0; i < l; ++i) {  // forward topological ordering
        const auto d = in_degree(i, H);
        if (d > 1) {

            PathGraph::in_edge_iterator begin, end;
            std::tie(begin, end) = in_edges(i, H);

            for (auto ei = begin; (++ei) != end; ) {
                const auto x = source(*ei, H);
                for (auto ej = begin; ej != ei; ++ej) {
                    const auto y = source(*ej, H);

                    // Determine the common ancestors of each input to node_i
                    for (unsigned j = 0; j < l; ++j) {
                        ancestors[j].reset();
                    }
                    ancestors[x].set(0);
                    ancestors[y].set(1);

                    std::fill_n(occurences.begin(), rank[i] - 1, 0);
                    for (auto j = l; j--; ) { // reverse topological ordering
                        for (const auto e : make_iterator_range(out_edges(j, H))) {
                            const auto v = target(e, H);
                            ancestors[j] |= ancestors[v];
                        }
                        if (ancestors[j].all()) {
                            const auto k = rank[j];
                            occurences[k]++;
                            singleton[k] = j;
                        }
                    }
                    // Now scan again through them to determine the single ancestor
                    // to the pair of inputs that is of highest rank.
                    auto lca = i;
                    for (auto j = rank[i] - 1; j--; ) {
                        if (occurences[j] == 1) {
                            lca = singleton[j];
                            break;
                        }
                    }

                    assert (lca < i);

                    const BV & Px = postdom[x];
                    const BV & Py = postdom[y];

                    for (const auto e : make_iterator_range(out_edges(lca, H))) {
                        const auto z = target(e, H);
                        const BV & Pz = postdom[z];

                        // Do not arc back to the start of a dominating path.

                        // NOTE: we delay adding the edges to H to prevent any changes
                        // to the in degree of a vertex we have not yet visited on a
                        // parallel path from being given an unintended edge.

                        if (!Px.is_subset_of(Pz)) {
                            toAdd.emplace(x, z);
                        }

                        if (!Py.is_subset_of(Pz)) {
                            toAdd.emplace(y, z);
                        }
                    }
                }
            }
        }
    }

    for (const auto & e : toAdd) {
        add_edge(e.first, e.second, HAMILTONIAN_PATH_DELTA_WEIGHT, H);
    }

    END_SCOPED_REGION

    const auto numOfPartitionNodes = (2U * activePartitions) + 2U;

    PartitionOrderingGraph G(numOfPartitionNodes);

    // Split every node except for the common sink/source in H into two nodes in G;
    // this will form the backbone of the partition scheduling constraint graph

    for (unsigned i = 0; i <= activePartitions; ++i) {
        const auto partitionOut = (i * 2);
        for (const auto e : make_iterator_range(out_edges(i, H))) {
            const auto j = target(e, H);
            const auto outgoing = (j * 2) - 1;
            assert (outgoing < numOfPartitionNodes);
            add_edge(partitionOut, outgoing, H[e], G);
        }
    }

    BEGIN_SCOPED_REGION

    flat_map<OrderingDAWG::edge_descriptor, Vertex> mapping;

    auto get = [&mapping](const OrderingDAWG::edge_descriptor & e) {
        const auto f = mapping.find(e);
        assert (f != mapping.end());
        return f->second;
    };

    for (unsigned i = 0; i < activePartitions; ++i) {

        // Each partition has one or more optimal orderings of kernel invocations.
        // We filter each ordering trie to only contain the kernels with cross-
        // partition I/O and then compute the minimal acyclic dfa. The edges
        // (i.e., line graph) of those subgraphs will map to nodes in the subsequent
        // ordering graph.

        // TODO: we could combine both phases (and avoid the post order minimization
        // phase) but the intention of the algorithm will be less clear.

        const PartitionData & D = P[i + 1U];

        const OrderingDAWG & O = D.Orderings;

        OrderingDAWG H(1);

        std::function<void(unsigned, unsigned)> filter_trie = [&](const unsigned u, const unsigned i) {
            for (const auto e : make_iterator_range(out_edges(i, O))) {
                const auto t = O[e];
                assert (std::find(D.Kernels.begin(), D.Kernels.end(), t) != D.Kernels.end());
                auto v = u;
                // Is kernel "t" a kernel with cross partition I/O?
                const auto f = kernels.find(t);
                if (f != kernels.end()) {
                    const auto k = std::distance(kernels.begin(), f);
                    v = add_vertex(H);
                    add_edge(u, v, k, H);
                }
                filter_trie(v, target(e, O));
            }
        };

        filter_trie(0, 1);

        // H will never be 0 in a connected graph with more than 1 partition but
        // this will cause an infinite loop if it occurs.
        if (LLVM_LIKELY(num_edges(H) == 0)) {
            report_fatal_error("Internal error: inter-partition scheduling graph has no cross-partition relationships?");
        }

        postorder_minimize(H);

        // Insert the line graph of each partition DAWG between the partition nodes.

        mapping.reserve(num_edges(H));

        for (const auto e : make_iterator_range(edges(H))) {
            std::vector<unsigned> A(1);
            A[0] = H[e];
            const auto u = add_vertex(std::move(A), G);
            mapping.emplace(e, u);
        }

        const Vertex partitionIn = (i * 2) + 1;
        const Vertex partitionOut = (i * 2) + 2;

        for (const auto u : make_iterator_range(vertices(H))) {
            if (in_degree(u, H) == 0) {
                for (const auto e : make_iterator_range(out_edges(u, H))) {
                    add_edge(partitionIn, get(e), HAMILTONIAN_PATH_DELTA_WEIGHT, G);
                }
            } else if (out_degree(u, H) == 0) {
                for (const auto e : make_iterator_range(in_edges(u, H))) {
                    add_edge(get(e), partitionOut, HAMILTONIAN_PATH_EPSILON_WEIGHT, G);
                }
            } else {
                for (const auto e : make_iterator_range(in_edges(u, H))) {
                    const auto v = get(e);
                    for (const auto f : make_iterator_range(out_edges(u, H))) {
                        const auto w = get(f);
                        add_edge(v, w, HAMILTONIAN_PATH_DELTA_WEIGHT, G);
                    }
                }
            }
        }

        mapping.clear();
    }

    END_SCOPED_REGION

    assert (out_degree(numOfPartitionNodes - 1, G) == 0);

#if 0

    auto printOrderingGraph = [&](const PartitionOrderingGraph & GP, raw_ostream & out, const StringRef name) {
        out << "digraph \"" << name << "\" {\n";
        for (auto v : make_iterator_range(vertices(GP))) {
            const auto & V = GP[v];
            out << "v" << v << " [shape=record,label=\"";
            bool addComma = false;
            for (const auto k : V) {
                if (addComma) out << ',';
                out << *kernels.nth(k);
                addComma = true;
            }
            out << "\"];\n";
        }
        for (auto e : make_iterator_range(edges(GP))) {
            const auto s = source(e, GP);
            const auto t = target(e, GP);
            out << "v" << s << " -> v" << t << ";\n";
        }
        out << "}\n\n";
        out.flush();

    };

#endif

    const auto n = num_vertices(G);

    for (;;) {
        bool unchanged = true;
        // Even though G is likely cyclic, it was constructed from an acyclic
        // graph whose vertices were indexed in topological order. Traversing
        // from the last to first tends to reach the fixpoint faster.
        for (unsigned i = n; i--; ) {
            if (out_degree(i, G) == 1) {
                const auto j = child(i, G);
                if (in_degree(j, G) == 1) {
                    // merge both vertex i and j and cons their lists
                    auto & A = G[i];
                    const auto & B = G[j];
                    A.insert(A.end(), B.begin(), B.end());
                    for (const auto e : make_iterator_range(out_edges(j, G))) {
                        add_edge(i, target(e, G), G[e], G);
                    }
                    clear_vertex(j, G);
                    G[j].clear();
                    unchanged = false;
                }
            }
        }
        if (unchanged) break;
    }

    // Since deleting vertices from a vector based graph is difficult with
    // boost::graph, regenerate G sans any isolated nodes.

    unsigned numOfKernelSets = 0;
    unsigned numOfEmptyNodes = 0;
    #ifndef NDEBUG
    std::vector<unsigned> index(n, -1U);
    #else
    std::vector<unsigned> index(n);
    #endif

    for (unsigned i = 0; i < n; ++i) {

        const auto & V = G[i];
        if (!V.empty()) {
            ++numOfKernelSets;
        } else if (in_degree(i, G) > 0 || out_degree(i, G) > 0) {
            ++numOfEmptyNodes;
        }

    }

    const auto m = (numOfKernelSets + numOfEmptyNodes);

    assert (m > 0 || activePartitions == 1);

    PartitionOrderingGraph compressedGraph(m);

    for (unsigned i = 0, j = numOfKernelSets, k = 0; i < n; ++i) {
        auto & V = G[i];

        if (V.empty() && (in_degree(i, G) == 0 && out_degree(i, G) == 0)) {
            continue;
        }

        unsigned t;
        if (V.empty()) {
            t = j++;
        } else {
            t = k++;
        }
        index[i] = t;
        compressedGraph[t] = std::move(V);
    }

    for (const auto e : make_iterator_range(edges(G))) {
        const auto u = index[source(e, G)];
        assert (u < m);
        const auto v = index[target(e, G)];
        assert (v < m);
        add_edge(u, v, G[e], compressedGraph);
    }

//    printOrderingGraph(compressedGraph, errs(), "O");

    return PartitionOrdering{std::move(compressedGraph), numOfKernelSets, std::move(kernels)};
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief scheduleProgramGraph
 ** ------------------------------------------------------------------------------------------------------------- */
OrderingDAWG PipelineAnalysis::scheduleProgramGraph(const PartitionGraph & P,
        const PartitionOrdering & partitionOrdering,
        const PartitionDataflowGraph & D,
        random_engine & rng) const {

    auto & O = partitionOrdering.Graph;
    const auto & kernels = partitionOrdering.Kernels;

    assert (PartitionCount > 0);

    const auto activePartitions = (PartitionCount - 1);

    const auto numOfFrontierKernels = kernels.size();

    const auto maxPathLength = (2U * activePartitions) + 2U + numOfFrontierKernels;

    assert (num_vertices(D) >= activePartitions);

    const auto lastStreamSet = num_vertices(D);
    const auto firstStreamSet = activePartitions;

    const auto numOfStreamSets = lastStreamSet - firstStreamSet;

    const auto m = numOfFrontierKernels + numOfStreamSets;
    SchedulingGraph S(m);

    for (unsigned i = 0; i < numOfFrontierKernels; ++i) {
        SchedulingNode & N = S[i];
        N.Type = SchedulingNode::IsKernel;
    }

    auto kernelSetIdOf = [&](const unsigned kernelId) -> unsigned {
        const auto f = std::lower_bound(kernels.begin(), kernels.end(), kernelId);
        assert (f != kernels.end() && *f == kernelId);
        return std::distance(kernels.begin(), f);
    };

    for (unsigned currentPartition = 0; currentPartition < activePartitions; ++currentPartition) {

        const PartitionData & Pi = P[currentPartition + 1U];

        for (const auto e : make_iterator_range(in_edges(currentPartition, D))) {
            const PartitionDataflowEdge & C = D[e];
            assert (Relationships[C.KernelId].Type == RelationshipNode::IsKernel);
            const auto consumer = kernelSetIdOf(C.KernelId);
            const auto v = source(e, D);
            assert (v >= activePartitions);
            const auto streamSet = v - activePartitions;
            assert (streamSet < numOfStreamSets);
            const auto k = numOfFrontierKernels + streamSet;
            add_edge(k, consumer, S);
        }

        for (const auto e : make_iterator_range(out_edges(currentPartition, D))) {
            const PartitionDataflowEdge & De = D[e];
            assert (Relationships[De.KernelId].Type == RelationshipNode::IsKernel);
            const auto producer = kernelSetIdOf(De.KernelId);
            const auto v = target(e, D);
            assert (v >= activePartitions);
            const auto streamSet = v - activePartitions;
            assert (streamSet < numOfStreamSets);
            const auto k = numOfFrontierKernels + streamSet;
            add_edge(producer, k, S);
            SchedulingNode & node = S[k];
            node.Type = SchedulingNode::IsStreamSet;
            node.Size = Pi.ExpectedRepetitions * De.Expected * D[v]; // bytes per segment
        }
    }

    // ProgramScheduleType::NearestTopologicalOrdering

    fitness_hs_time = 0;
    fitness_time = 0;
    repair_time = 0;
    total_ga_time = 0;

    const auto start = std::chrono::high_resolution_clock::now();

    ProgramSchedulingAnalysis SA(S, O, ProgramScheduleType::RandomWalk, numOfFrontierKernels, maxPathLength, rng, 1, 1);

    SA.runGA();

    const auto end = std::chrono::high_resolution_clock::now();
    total_ga_time = (end - start).count();

//    errs() << "," << numOfFrontierKernels << "," << numOfStreamSets
//           << ',' << total_intra_dataflow_analysis_time << ',' << total_inter_dataflow_analysis_time << ',' << total_inter_dataflow_scheduling_time
//           << "," << total_ga_time << "," << fitness_time << "," << fitness_hs_time << ",";

////    errs() << "REPAIR TIME:     " << repair_time << "\n"
////              "FITNESS TIME:    " << fitness_time << "\n"
////              "FITNESS HS TIME: " << fitness_hs_time << "\n"
////              "TOTAL GA TIME:   " << total_ga_time << "\n";

//    SA.report();

//    errs() << "," << SA.getBestFitnessValue() << "\n";

//    errs() << "HS UNIQUE INSERTIONS: " << HS_UniqueInsertions << " of " << HS_InsertionAttempts << "\n";


    return SA.getResult();

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addSchedulingConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<unsigned> PipelineAnalysis::selectScheduleFromDAWG(const KernelIdVector & kernels,
                                                               const OrderingDAWG & schedule) {

    // TODO: if we have multiple memory optimal schedules, look for the one that
    // keeps calls to the same kernel closer or permits a better memory layout
    // w.r.t. sequential memory prefetchers?

    std::vector<unsigned> program;

    program.reserve(kernels.size());

    Vertex position = 0;


    while (out_degree(position, schedule) > 0) {
        const auto e = first_out_edge(position, schedule);

        const auto s = schedule[e];
        assert (s < kernels.size());
        const auto k = kernels[s];
        program.push_back(k);

//        const auto local = schedule[e];
//        assert (local < num_vertices(O));
//        for (const auto k : O[local]) {
//            program.push_back(kernels[k]);
//        }
        const auto next = target(e, schedule);
        assert (position != next);
        position = next;
    }

    return program;

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addSchedulingConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::addSchedulingConstraints(const PartitionGraph & P,
                                                const std::vector<unsigned> & program) {

    // Since we compressed the graph, nodes within O represent 0 to many kernels that
    // have cross partition I/O. These kernels could be from multiple partitions so
    // to simplify the logic, we initially create a partial program list then fill it
    // in by selecting a partition schedule that matches the selected program.

    std::vector<unsigned> subgraph;

    std::vector<unsigned> path;
    // underflow sentinal node
    path.push_back(-1U);

    auto u = PipelineInput;

//    auto printProgram =[](const std::vector<unsigned> & L, unsigned start, unsigned end) {
//        bool addComma = false;
//        for (auto i = start; i < end; ++i) {
//            if (addComma) errs() << ',';
//            errs() << L[i];
//            addComma = true;
//        }
//        errs() << "\n";
//    };

//    errs() << "PROGRAM: ";
//    printProgram(program, 0, program.size());


    const auto end = program.end();

    for (auto i = program.begin(); i != end; ) {

        path.resize(1);

        unsigned currentPartitionId = 0;

        while (i != end) {

            const auto node = *i;

            const auto f = PartitionIds.find(node);
            assert (f != PartitionIds.end());
            const auto pid = f->second - 1;

            if (path.size() == 1) {
                currentPartitionId = pid;
            } else if (currentPartitionId != pid) {
                break;
            }
            path.push_back(node);

            ++i;
        }

        // overflow sentinal node
        path.push_back(-1U);

        assert (path.size() > 2);

        const PartitionData & partition = P[currentPartitionId + 1U];

        const auto & G = partition.Orderings;
        const auto & K = partition.Kernels;


//        printDAWG(G, errs(), "P" + std::to_string(currentPartitionId));

//        errs() << "PATH: ";
//        printProgram(path, 1, path.size() - 1);


        const auto numOfKernels = K.size();
        assert (path.size() <= numOfKernels + 2);

        assert (subgraph.empty());

        subgraph.reserve(numOfKernels);

        unsigned offset = 1;

        std::function<bool(unsigned)> select_path = [&](const unsigned u) {

            if (LLVM_UNLIKELY(out_degree(u, G) == 0)) {
                // when we find an ordering of the kernels within this
                // partition that matches the desired global ordering,
                // exit the function.
                return (offset == (path.size() - 1));
            } else {
                for (const auto e : make_iterator_range(out_edges(u, G))) {
                    const auto k = G[e];
                    assert (offset < path.size());
                    if (path[offset] == k) {
                        ++offset;
                    }
                    subgraph.push_back(k);
                    const auto v = target(e, G);
                    if (select_path(v)) {
                        return true;
                    }
                    assert (subgraph.back() == k);

                    subgraph.pop_back();
                    if (path[offset - 1] == k) {
                        --offset;
                    }
                }
                return false;
            }

        };

        const auto found = select_path(1);
        assert (found);

        for (const auto v : subgraph) {
            if (PipelineInput != v && u != PipelineOutput) {
                add_edge(u, v, RelationshipType{ReasonType::OrderingConstraint}, Relationships);
            }
            u = v;
        }

        subgraph.clear();

    }

    if (u != PipelineOutput) {
        add_edge(u, PipelineOutput, RelationshipType{ReasonType::OrderingConstraint}, Relationships);
    }
}

#endif

}

#endif // SCHEDULING_ANALYSIS_HPP
