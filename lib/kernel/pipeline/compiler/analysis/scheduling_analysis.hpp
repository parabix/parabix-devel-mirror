#ifndef SCHEDULING_ANALYSIS_HPP
#define SCHEDULING_ANALYSIS_HPP

#include "pipeline_analysis.hpp"
#include "evolutionary_algorithm.hpp"

#include <chrono>
#include <llvm/Support/Format.h>
#include <fstream>
#include <iostream>

namespace kernel {

constexpr static unsigned INITIAL_TOPOLOGICAL_POPULATION_SIZE = 10;

constexpr static unsigned MAX_PARTITION_POPULATION_SIZE = 20;

constexpr static unsigned MAX_PROGRAM_POPULATION_SIZE = 20;

constexpr static unsigned PARITION_SCHEDULING_GA_ROUNDS = 50;

constexpr static unsigned PARITION_SCHEDULING_GA_STALLS = 10;

constexpr static unsigned PROGRAM_SCHEDULING_GA_ROUNDS = 50;

constexpr static unsigned PROGRAM_SCHEDULING_GA_STALLS = 10;

constexpr static unsigned BIPARTITE_GRAPH_UNPLACED = 0;

constexpr static unsigned BIPARTITE_GRAPH_LEFT_HAND = 1;

constexpr static unsigned BIPARTITE_GRAPH_RIGHT_HAND = 2;

constexpr static unsigned MAX_CUT_HS_POPULATION_SIZE = 10;

constexpr static double MAX_CUT_HS_AVERAGE_STALL_THRESHOLD = 3.0;

constexpr static unsigned MAX_CUT_HS_MAX_AVERAGE_STALLS = 20;

constexpr static unsigned INITIAL_SCHEDULING_POPULATION_ATTEMPTS = 100;

constexpr static unsigned INITIAL_SCHEDULING_POPULATION_SIZE = 20;

static_assert(INITIAL_SCHEDULING_POPULATION_ATTEMPTS >= INITIAL_SCHEDULING_POPULATION_SIZE,
    "cannot have fewer attemps than population size");

static_assert(INITIAL_SCHEDULING_POPULATION_SIZE <= MAX_PROGRAM_POPULATION_SIZE,
    "cannot have a larger initial population size than generational population size");


constexpr static unsigned SCHEDULING_FITNESS_COST_ACO_ROUNDS = 200;

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

#if 1

void printDAWG(const OrderingDAWG & G, raw_ostream & out, const StringRef name = "G") {

    out << "digraph \"" << name << "\" {\n";
    for (auto v : make_iterator_range(vertices(G))) {
        out << "v" << v << " [label=\"" << v << "\"];\n";
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
 * @brief analyzeDataflowWithinPartitions
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::schedulePartitionedProgram(PartitionGraph & P, random_engine & rng) {

    // Once we analyze the dataflow within the partitions, P contains DAWG that is either
    // edgeless if any permutation of its kernels is valid or contains all of its optimal
    // orderings for the kernels within each partition.

    // TODO: look into performance problem with
    // bin/icgrep -EnableTernaryOpt -DisableMatchStar '(?g)fodder|simple' ../QA/testfiles/simple1 -colors=always

//    printRelationshipGraph(Relationships, errs(), "Initial");

//    errs() << "analyzeDataflowWithinPartitions\n";

    analyzeDataflowWithinPartitions(P, rng);

//    errs() << "scheduleProgramGraph\n";

    const auto partial = scheduleProgramGraph(P, rng);

//    printDAWG(partial, errs(), "Partial");

//    errs() << "assembleFullSchedule\n";

    const auto full = assembleFullSchedule(P, partial);

//    printDAWG(full, errs(), "Full");

//    errs() << "selectScheduleFromDAWG\n";

    const auto schedule = selectScheduleFromDAWG(full);

//    errs() << "addSchedulingConstraints\n";

    addSchedulingConstraints(schedule);

//    printRelationshipGraph(Relationships, errs(), "Final");


}

namespace { // start of anonymous namespace


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
        assert (unvisited.find_first() == 0);
        int i = 0;
        for (;;) {
            assert ((unsigned)i < candidateLength);
            assert (unvisited.test(i));
            assert (stack.empty());
            unvisited.reset(i);
            auto u = toVertexIndex[i];
            assert (out_degree(u, I) > 0);
            ++numOfComponents;
            for (;;) {
                const auto uval = candidate.test(i);
                for (const auto e : make_iterator_range(out_edges(u, I))) {
                    const auto v = target(e, I);
                    const auto j = toBitIndex[v];

                    if (unvisited.test(j) && uval != candidate.test(j)) {
                        unvisited.reset(j);
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
                        const size_t seed)
    : BitStringBasedHarmonySearch(numOfNonIsolatedVertices(I), numOfRounds,
                                  MAX_CUT_HS_POPULATION_SIZE, MAX_CUT_HS_AVERAGE_STALL_THRESHOLD, MAX_CUT_HS_MAX_AVERAGE_STALLS, seed)
    , I(I)
    , maxCutWeights(maxCutWeights)
    , toVertexIndex(candidateLength)
    , toBitIndex(num_vertices(I))
    , unvisited(num_vertices(I))
    , maxNumOfComponents(MAX_CUT_MAX_NUM_OF_CONNECTED_COMPONENTS) {
        assert (candidateLength > 0);
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
                // assert (W > 0);
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

        const auto numOfEdges = num_edges(I);
        assert (numOfEdges > 0);

        const auto predictionOf95PercentCut = 0.07930 * ((double)(numOfStreamSets * numOfStreamSets))
            + 7.63712 * ((double)numOfStreamSets) - 0.19735 * ((double)numOfEdges) - 80.59364;

        const unsigned numOfRounds = std::ceil(std::max(predictionOf95PercentCut, 10.0));

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

        if (LLVM_UNLIKELY(maxCutWeights.empty())) {
            return 0;
        }

        std::uniform_int_distribution<uintmax_t> distribution(0, std::numeric_limits<uintmax_t>::max());
        const auto seed = distribution(rng);
        MaxCutHarmonySearch HS(I, maxCutWeights, numOfRounds, seed);
        HS.runHarmonySearch();
        const auto assignment = HS.getResult();

        assert (HS.getBestFitnessValue() > std::numeric_limits<MaxCutHarmonySearch::FitnessValueType>::lowest());

        MemIntervalGraph residualGraph(numOfStreamSets);

        bool anyResiduals = false;

        std::vector<size_t> residualWeights(2 * numOfKernels + numOfStreamSets, 0);

        remove_edge_if([&](const MemIntervalGraph::edge_descriptor e){
            const auto u = source(e, I);
            const auto v = target(e, I);
            if (assignment.test(u) != assignment.test(v)) {
                return false;
            } else {
                add_edge(u, v, residualGraph);
                const auto Wu = weight[firstStreamSet + u];
                const auto Wv = weight[firstStreamSet + v];

                if (Wu > 0 && Wv > 0) {
                    anyResiduals = true;
                    residualWeights[firstStreamSet + u] = weight[firstStreamSet + u];
                    residualWeights[firstStreamSet + v] = weight[firstStreamSet + v];
                }
                return true;
            }
        }, I);

        if (anyResiduals) {
            return calculateChomaticNumber(candidate, residualWeights, residualGraph);
        } else {
            return 0;
        }
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

    MemoryAnalysis(const SchedulingGraph & S, const unsigned numOfKernels, random_engine & rng)
    : S(S)
    , numOfKernels(numOfKernels)
    , numOfStreamSets(num_vertices(S) - numOfKernels)
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
                       random_engine & rng)
    : numOfKernels(numOfKernels)
    , candidateLength(candidateLength)
    , rng(rng)
    , analyzer(S, numOfKernels, rng) {

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
    : SchedulingAnalysisWorker(S, numOfKernels, numOfKernels, rng)
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
            insertCandidate(Candidate{L}, initialPopulation);
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

    assert (PartitionCount > 2);

    for (unsigned currentPartitionId = 0; currentPartitionId < PartitionCount; ++currentPartitionId) {

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
        assert (numOfKernels > 0);
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

        #ifndef NDEBUG
        std::vector<unsigned> L;
        flat_set<unsigned> C;
        std::function<void(unsigned)> verify_all_paths_contain_all_kernels = [&](const unsigned u) {
            if (out_degree(u, H) == 0) {
                assert ("partition path does not contain enough kernels?" && L.size() == numOfKernels);
                assert (C.empty());
                for (const auto k : L) {
                    C.insert(k);
                }
                assert ("partition path contains duplicate kernels?" && C.size() == numOfKernels);
                C.clear();
            } else {
                for (const auto e : make_iterator_range(out_edges(u, H))) {
                    L.push_back(H[e]);
                    verify_all_paths_contain_all_kernels(target(e, H));
                    assert (H[e] == L.back());
                    L.pop_back();
                }

            }
        };
        assert ("no partition paths found?" && num_edges(H) >= numOfKernels);
        verify_all_paths_contain_all_kernels(1);
        #endif

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
        assert (strideSize > Rational{0});

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
                        const auto & itemsPerStride = G[f];
                        assert (itemsPerStride > Rational{0});
                        add_edge(j, i, itemsPerStride, G);
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
                    assert (itemsPerStride > Rational{0});
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
                assert (itemsPerStride > Rational{0});
                add_edge(i, j, itemsPerStride, G);
            }
        }
    }

    // add fake input arcs
    flat_set<unsigned> externalStreamSets;

    for (const auto e : make_iterator_range(in_edges(currentPartitionId, P))) {
        const auto streamSet = P[e];
        // a streamSet with a value 0 denotes a non-I/O ordering constraint
        if (streamSet) {
            externalStreamSets.insert(streamSet);
        }
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

        assert (bytesPerItem > Rational{0});
        SchedulingNode & SN = G[j];
        assert (SN.Size == Rational{0});

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
        assert (itemsPerStride > Rational{0});
        add_edge(fakeInput, j, itemsPerStride, G);
    }

//    for (auto i = firstStreamSet; i < n; ++i) {
//        if (LLVM_UNLIKELY(in_degree(i, G) == 0)) {
//            assert (out_degree(i, G) > 0);

//            SchedulingGraph::out_edge_iterator ei, ei_end;
//            std::tie(ei, ei_end) = out_edges(i, G);
//            auto itemsPerStride = G[*ei];
//            while (++ei != ei_end) {
//                const auto & r = G[*ei];
//                if (r > itemsPerStride) {
//                    itemsPerStride = r;
//                }
//            }

//            add_edge(fakeInput, i, itemsPerStride, G);
//        }
//    }

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
                    const auto & itemsPerStride = G[f];
                    assert (itemsPerStride > Rational{0});
                    add_edge(j, i, itemsPerStride, G);
                } else {
                    // If we have a PopCount producer/consumer in the same partition,
                    // they're both perform an identical number of strides. So long
                    // as the producing/consuming strideRate match, the equation will
                    // work. Since the lower bound of PopCounts is 0, we always use the
                    // upper bound.
                    const auto itemsPerStride = rate.getUpperBound() * strideSize;
                    assert (itemsPerStride > Rational{0});
                    add_edge(j, i, itemsPerStride, G);
                }
            }
        }
    }

    // add fake output arcs
    externalStreamSets.clear();
    for (const auto e : make_iterator_range(out_edges(currentPartitionId, P))) {
        const auto streamSet = P[e];
        // a streamSet with a value 0 denotes a non-I/O ordering constraint
        if (streamSet) {
            externalStreamSets.insert(streamSet);
        }
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
        assert (itemsPerStride > Rational{0});
        add_edge(j, fakeOutput, itemsPerStride, G);
    }

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


namespace { // anonymous namespace

using GlobalDependencyGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, no_property>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief ProgramSchedulingAnalysis
 ** ------------------------------------------------------------------------------------------------------------- */
struct ProgramSchedulingAnalysisWorker final : public SchedulingAnalysisWorker {

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief repair
     ** ------------------------------------------------------------------------------------------------------------- */
    void repair(Candidate & candidate) override {

        assert (candidate.size() == numOfKernels);

        assert (replacement.empty());

        assert (stack.empty());

        for (unsigned i = 0; i < numOfKernels; ++i) {
            const auto k = candidate[i];
            assert (k < numOfKernels);
            position[k] = i;
        }


        SmallVector<Vertex, 8> tmp;

        stack.push(0);
        remaining.assign(initialDegree.begin(), initialDegree.end());

        for (;;) {

            const auto u = stack.top();
            stack.pop();

            assert (remaining[u] == 0);

            assert (u < hierarchicalSubgraphs.size());

            const OrderingDAWG & H = hierarchicalSubgraphs[u];
            assert (num_edges(H) > 0);
            assert (in_degree(0, H) == 0);
            assert (out_degree(0, H) > 0);

            // explore the subgraph and greedily choose a path through the DAWG
            // and build up the replacement from it.

            assert (replacement.size() < numOfKernels);

            for (unsigned t = 0;;) {

                OrderingDAWG::out_edge_iterator begin, end;
                std::tie(begin, end) = out_edges(t, H);
                auto selected = begin;
                auto best = position[H[*begin]];
                for (auto ei = begin; ++ei != end; ) {
                    const auto pos = position[H[*ei]];
                    if (pos < best) {
                        selected = ei;
                        best = pos;
                    }
                }
                replacement.push_back(H[*selected]);
                t = target(*selected, H);
                if (out_degree(t, H) == 0) {
                    break;
                }
            }

            assert (tmp.empty());

            for (const auto e : make_iterator_range(out_edges(u, G))) {
                const auto v = target(e, G);
                assert (remaining[v] > 0);
                if (remaining[v] == 1) {
                    tmp.push_back(v);
                }
                remaining[v]--;
            }

            if (tmp.empty()) {
                if (LLVM_UNLIKELY(stack.empty())) {
                    break;
                }
            } else {
                std::sort(tmp.begin(), tmp.end(), [&](const Vertex u, const Vertex v) {
                    return position[u] > position[v];
                });
                for (const auto v : tmp) {
                    stack.push(v);
                }
                tmp.clear();
            }
        }

        assert (replacement.size() == numOfKernels);



        candidate.swap(replacement);
        replacement.clear();
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief makeRandomCandidate
     ** ------------------------------------------------------------------------------------------------------------- */
    Candidate makeRandomCandidate() {
        Candidate candidate;
        candidate.reserve(candidateLength);

        // depth-first topological ordering

        assert (stack.empty());

        SmallVector<Vertex, 8> tmp;
        stack.push(0);
        remaining.assign(initialDegree.begin(), initialDegree.end());

        while (!stack.empty()) {

            const auto u = stack.top();
            stack.pop();

            assert (u < hierarchicalSubgraphs.size());

            const OrderingDAWG & H = hierarchicalSubgraphs[u];
            if (LLVM_UNLIKELY(num_edges(H) == 0)) {
                continue;
            }

            assert (num_edges(H) > 0);
            assert (in_degree(0, H) == 0);
            assert (out_degree(0, H) > 0);

            // fill in the kernels from the partition subgraph
            for (auto r = 0;;) {
                const auto d = out_degree(r, H);
                if (d == 0) {
                    break;
                }
                std::uniform_int_distribution<unsigned> dist(0, d - 1);
                const auto k = dist(rng);
                const auto begin = out_edges(r, H).first;
                const auto e = *(begin + k);
                const auto t = H[e];
                assert (t < candidateLength);
                candidate.push_back(t);
                r = target(e, H);
            }

            for (const auto e : make_iterator_range(out_edges(u, G))) {
                const auto v = target(e, G);
                assert (remaining[v] > 0);
                if (remaining[v] == 1) {
                    tmp.push_back(v);
                }
                remaining[v]--;
            }

            const auto m = tmp.size();
            if (m == 0) {
                continue;
            } else if (m == 1) {
                stack.push(tmp.front());
            } else {
                std::shuffle(tmp.begin(), tmp.end(), rng);
                for (const auto v : tmp) {
                    stack.push(v);
                }
            }
            tmp.clear();

        }

        assert (candidate.size() == numOfKernels);
        #ifndef NDEBUG
        flat_set<unsigned> C;
        C.reserve(numOfKernels);
        for (const auto c: candidate) {
            C.insert(c);
        }
        assert ("duplicate kernels in candidate?" && C.size() == numOfKernels);
        #endif
        return candidate;
    }

public:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief constructor
     ** ------------------------------------------------------------------------------------------------------------- */
    ProgramSchedulingAnalysisWorker(const SchedulingGraph & S,
                                    const GlobalDependencyGraph & G,
                                    const std::vector<OrderingDAWG> & hierarchicalSubgraphs,
                                    const std::vector<unsigned> & initialDegree,
                                    const unsigned numOfKernels,
                                    random_engine & rng)
    : SchedulingAnalysisWorker(S, numOfKernels, numOfKernels, rng)
    , G(G)
    , hierarchicalSubgraphs(hierarchicalSubgraphs)
    , initialDegree(initialDegree)
    , remaining(num_vertices(G))
    , position(numOfKernels)
    {
        replacement.reserve(numOfKernels);
    }

private:

    const GlobalDependencyGraph & G;

    const std::vector<OrderingDAWG> & hierarchicalSubgraphs;
    const std::vector<unsigned> & initialDegree;

    std::stack<Vertex> stack;
    Candidate replacement;
    std::vector<unsigned> remaining;
    std::vector<unsigned> position;

};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief ProgramSchedulingAnalysis
 ** ------------------------------------------------------------------------------------------------------------- */
struct ProgramSchedulingAnalysis final : public PermutationBasedEvolutionaryAlgorithm {

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

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief constructor
     ** ------------------------------------------------------------------------------------------------------------- */
    ProgramSchedulingAnalysis(const SchedulingGraph & S,
                              const GlobalDependencyGraph & G,
                              const std::vector<OrderingDAWG> & hierarchicalSubgraphs,
                              const std::vector<unsigned> & initialDegree,
                              const unsigned numOfKernels,
                              random_engine & rng)
    : PermutationBasedEvolutionaryAlgorithm(numOfKernels, PROGRAM_SCHEDULING_GA_ROUNDS,
                                            PROGRAM_SCHEDULING_GA_STALLS, MAX_PROGRAM_POPULATION_SIZE, rng)
    , worker(S, G, hierarchicalSubgraphs, initialDegree, numOfKernels, rng) {

    }

private:

    ProgramSchedulingAnalysisWorker worker;

};

} // end of anonymous namespace

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief scheduleProgramGraph
 ** ------------------------------------------------------------------------------------------------------------- */
OrderingDAWG PipelineAnalysis::scheduleProgramGraph(const PartitionGraph & P, random_engine & rng) const {

    // create a bipartite graph consisting of partitions and cross-partition
    // streamset nodes and relationships

    flat_set<Vertex> streamSets;

    for (unsigned partitionId = 1; partitionId < PartitionCount; ++partitionId) {
        for (const auto e : make_iterator_range(out_edges(partitionId, P))) {
            const auto streamSet = P[e];
            // a streamSet with a value 0 denotes a non-I/O ordering constraint
            if (LLVM_UNLIKELY(streamSet == 0)) continue;
            assert (streamSet < num_vertices(Relationships));
            assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
            assert (isa<StreamSet>(Relationships[streamSet].Relationship));
            streamSets.insert(streamSet);
        }

//        // insert any streamSets with no consumers to ensure we capture them in
//        // out I/O orderings.
//        const auto & K = P[partitionId].Kernels;
//        for (const auto k : K) {
//            for (const auto e : make_iterator_range(out_edges(k, Relationships))) {
//                const auto binding = target(e, Relationships);
//                const RelationshipNode & output = Relationships[binding];
//                if (LLVM_LIKELY(output.Type == RelationshipNode::IsBinding)) {
//                    for (const auto f : make_iterator_range(out_edges(binding, Relationships))) {
//                        const auto streamSet = target(f, Relationships);
//                        assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
//                        assert (isa<StreamSet>(Relationships[streamSet].Relationship));
//                        if (out_degree(streamSet, Relationships) == 0) {
//                            streamSets.insert(streamSet);
//                        }
//                    }
//                }
//            }
//        }

    }

    const auto numOfStreamSets = streamSets.size();

    GlobalDependencyGraph G(PartitionCount);

    auto getPartitionId = [&](const unsigned kernel) {
        const auto f = PartitionIds.find(kernel);
        assert (f != PartitionIds.end());
        const auto partitionId = f->second;
        assert (partitionId > 0);
        return partitionId;
    };

    flat_set<unsigned> kernels;

    flat_set<unsigned> consumerPartitions;

    for (unsigned i = 0; i < numOfStreamSets; ++i) {

        const auto streamSet = *streamSets.nth(i);

        const auto f = first_in_edge(streamSet, Relationships);
        assert (Relationships[f].Reason != ReasonType::Reference);

        const auto binding = source(f, Relationships);
        const RelationshipNode & output = Relationships[binding];
        assert (output.Type == RelationshipNode::IsBinding);

        const auto g = first_in_edge(binding, Relationships);
        assert (Relationships[g].Reason != ReasonType::Reference);
        const unsigned producer = source(g, Relationships);

        const auto producerPartitionId = getPartitionId(producer);
        kernels.insert(producer);

        assert (consumerPartitions.empty());

        for (const auto e : make_iterator_range(out_edges(streamSet, Relationships))) {
            const auto binding = target(e, Relationships);
            const RelationshipNode & input = Relationships[binding];
            if (LLVM_LIKELY(input.Type == RelationshipNode::IsBinding)) {
                const auto f = first_out_edge(binding, Relationships);
                assert (Relationships[f].Reason != ReasonType::Reference);
                const unsigned consumer = target(f, Relationships);

                const auto consumerPartitionId = getPartitionId(consumer);
                assert (consumerPartitionId >= producerPartitionId);

                if (producerPartitionId != consumerPartitionId) {
                    kernels.insert(consumer);
                    consumerPartitions.insert(consumerPartitionId);
                }
            }
        }

        for (const auto consumerPartitionId : consumerPartitions) {
            add_edge(producerPartitionId, consumerPartitionId, G);
        }
        consumerPartitions.clear();

    }

    assert (PartitionCount > 2);

    for (unsigned partitionId = 1; partitionId < (PartitionCount - 1); ++partitionId) {
        if (in_degree(partitionId, G) == 0) {
            add_edge(0, partitionId, G);
        }
        if (out_degree(partitionId, G) == 0) {
            add_edge(partitionId, PartitionCount - 1, G);
        }
    }

    // We need to ensure that when we filter a partition ordering DAWG later that
    // we account for source kernels. The easiest way is simply to force them to
    // be included in the kernel filter.
    for (unsigned partitionId = 0; partitionId < PartitionCount; ++partitionId) {
        const PartitionData & currentPartition = P[partitionId];
        const auto & K = currentPartition.Kernels;
        #ifndef NDEBUG
        if (partitionId == 0 || partitionId == (PartitionCount - 1)) {
            assert (K.size() == 1);
            const auto k = K.front();
            const RelationshipNode & rn = Relationships[k];
            assert (rn.Type == RelationshipNode::IsKernel);
            assert (rn.Kernel == mPipelineKernel);
        }
        #endif
        for (const auto k : K) {
             for (const auto e : make_iterator_range(in_edges(k, Relationships))) {
                const auto binding = source(e, Relationships);
                const RelationshipNode & input = Relationships[binding];
                if (LLVM_LIKELY(input.Type == RelationshipNode::IsBinding)) {
                    goto not_a_source;
                }
            }
            kernels.insert(k);
 not_a_source: continue;
        }
    }

    const auto numOfFrontierKernels = kernels.size();

    auto getKernelId = [&](const unsigned kernel) {
        const auto f = kernels.find(kernel);
        assert (f != kernels.end());
        return std::distance(kernels.begin(), f);
    };

    const auto n = numOfFrontierKernels + numOfStreamSets;

    assert (n > 0);

    SchedulingGraph S(n);

    for (unsigned i = 0; i < numOfStreamSets; ++i) {

        const auto streamSet = *streamSets.nth(i);

        const auto streamSetNode = numOfFrontierKernels + i;
        assert (in_degree(streamSetNode, S) == 0);
        assert (out_degree(streamSetNode, S) == 0);

        const auto f = first_in_edge(streamSet, Relationships);
        assert (Relationships[f].Reason != ReasonType::Reference);

        const auto binding = source(f, Relationships);
        const RelationshipNode & output = Relationships[binding];
        assert (output.Type == RelationshipNode::IsBinding);

        const auto g = first_in_edge(binding, Relationships);
        assert (Relationships[g].Reason != ReasonType::Reference);
        const unsigned producer = source(g, Relationships);

        const auto producerPartitionId = getPartitionId(producer);
        const Binding & outputBinding = output.Binding;
        const ProcessingRate & rate = outputBinding.getRate();

        SchedulingNode & node = S[streamSetNode];
        node.Type = SchedulingNode::IsStreamSet;

        if (LLVM_UNLIKELY(rate.isUnknown())) {
            node.Size = Rational{0};
        } else {
            const RelationshipNode & rn = Relationships[producer];
            assert (node.Type == RelationshipNode::IsKernel);
            const PartitionData & N = P[producerPartitionId];
            const auto & K = N.Kernels;
            const auto h = std::find(K.begin(), K.end(), producer);
            assert (h != K.end());
            const auto index = std::distance(K.begin(), h);

            const auto strideSize = rn.Kernel->getStride();
            const auto sum = rate.getLowerBound() + rate.getUpperBound();

            const auto expectedItemsPerStride = sum * strideSize * Rational{1, 2};
            const PartitionData & Pi = P[producerPartitionId];
            const auto expectedItemsPerSegment = N.Repetitions[index] * expectedItemsPerStride;
            const Rational bytesPerItem{outputBinding.getFieldWidth() * outputBinding.getNumElements(), 8};
            node.Size = expectedItemsPerSegment * bytesPerItem; // bytes per segment
        }

        add_edge(getKernelId(producer), streamSetNode, S);

        for (const auto e : make_iterator_range(out_edges(streamSet, Relationships))) {
            const auto binding = target(e, Relationships);
            const RelationshipNode & input = Relationships[binding];
            if (LLVM_LIKELY(input.Type == RelationshipNode::IsBinding)) {
                const auto f = first_out_edge(binding, Relationships);
                assert (Relationships[f].Reason != ReasonType::Reference);
                const unsigned consumer = target(f, Relationships);

                const auto consumerPartitionId = getPartitionId(consumer);
                assert (consumerPartitionId >= producerPartitionId);

                if (producerPartitionId != consumerPartitionId) {
                    add_edge(streamSetNode, getKernelId(consumer), S);
                }
            }
        }
    }

    std::vector<OrderingDAWG> hierarchicalSubgraphs;
    hierarchicalSubgraphs.reserve(PartitionCount);

    for (unsigned partitionId = 0; partitionId < PartitionCount; ++partitionId) {

        assert (partitionId < num_vertices(P));

        const PartitionData & currentPartition = P[partitionId];

        const auto & O = currentPartition.Orderings;

        assert (num_edges(O) > 0);

        OrderingDAWG H(1);

        std::function<void(unsigned, unsigned)> filter_trie = [&](const unsigned u, const unsigned i) {
            for (const auto e : make_iterator_range(out_edges(i, O))) {
                const auto t = O[e];
                auto v = u;
                // Is kernel "t" a kernel with cross partition I/O?
                const auto f = kernels.find(t);
                if (f != kernels.end()) {
                    const auto k = std::distance(kernels.begin(), f);
                    v = add_vertex(H);
                    add_edge(u, v, k, H);
                } else {
                    for (const auto k : kernels) {
                        assert (k != t);
                    }
                }
                filter_trie(v, target(e, O));
            }
        };

        filter_trie(0, 1);
        assert (num_edges(H) > 0);
        postorder_minimize(H);
        assert (num_edges(H) > 0);

        hierarchicalSubgraphs.emplace_back(H);
    }

    std::vector<unsigned> initialDegree(PartitionCount);
    for (unsigned partitionId = 0; partitionId < PartitionCount; ++partitionId) {
        const auto d = in_degree(partitionId, G);
        assert ((partitionId == 0) ^ (d != 0));
        initialDegree[partitionId] = d;
    }

    ProgramSchedulingAnalysis SA(S, G, hierarchicalSubgraphs, initialDegree, numOfFrontierKernels, rng);

    SA.runGA();

    auto partial = SA.getResult();

    for (auto e : make_iterator_range(edges(partial))) {
        auto & k = partial[e];
        assert (k < numOfFrontierKernels);        
        k = *kernels.nth(k);
    }

    return partial;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief assembleFullSchedule
 ** ------------------------------------------------------------------------------------------------------------- */
OrderingDAWG PipelineAnalysis::assembleFullSchedule(const PartitionGraph & P, const OrderingDAWG & partial) const {


    // The partial DAWG is a partial schedule for the program; filter and stitch together all partition
    // subgraph DAWGs to create the final program schedule.

    OrderingDAWG schedule(1);

    auto getPartitionId = [&](const unsigned kernel) {
        const auto f = PartitionIds.find(kernel);
        assert (f != PartitionIds.end());
        const auto partitionId = f->second;
        return partitionId;
    };

    auto findOrAdd = [&](const Vertex u, const Vertex k) {
        for (const auto e : make_iterator_range(out_edges(u, schedule))) {
            if (schedule[e] == k) {
                return target(e, schedule);
            }
        }
        const auto v = add_vertex(schedule);
        add_edge(u, v, k, schedule);
        return v;
    };

    #ifndef NDEBUG
    unsigned numOfKernels = 0;
    for (const auto u : make_iterator_range(vertices(Relationships))) {
        const RelationshipNode & rn = Relationships[u];
        if (rn.Type == RelationshipNode::IsKernel) {
            numOfKernels++;
        }
    }
    #endif

    std::vector<Vertex> path;

    std::function<void(Vertex, Vertex, unsigned, unsigned)> buildFullSchedule =
        [&](const Vertex u, const Vertex x, const unsigned kernelGoal, const unsigned pid) {

        assert (pid < PartitionCount);

        const PartitionData & pd = P[pid];
        const auto & O = pd.Orderings;

//        errs().indent(path.size() * 4);
//        errs() << "u=" << u << ", x=" << x << ", goal=" << kernelGoal <<
//                  ", pid=" << pid << " :";
//        for (const auto k : path) {
//            errs() << ' ' << k;
//        }
//        errs() << "\n";

        assert (x > 0 && x < num_vertices(O));

        // are we transitioning to a new partition? if so, permit it
        // only if we've reached the end of the current partition.

        if (out_degree(x, O) == 0) {
            if (LLVM_UNLIKELY(out_degree(u, partial) == 0)) {
                assert (kernelGoal == PipelineOutput);
                assert (path.size() == numOfKernels - 1);
                Vertex r = 0;
                for (const auto k : path) {
                    r = findOrAdd(r, k);
                }
                findOrAdd(r, kernelGoal);
            } else {
                const auto nid = getPartitionId(kernelGoal);
                if (LLVM_LIKELY(nid != pid)) {
                    buildFullSchedule(u, 1, kernelGoal, nid);
                }
            }
        } else {
            for (const auto e : make_iterator_range(out_edges(x, O))) {
                const auto nextKernel = O[e];
                const auto y = target(e, O);
                // if this vertex is the vertex on our partial path,
                // advance the state of the traversal through it.
                path.push_back(nextKernel);
                if (kernelGoal == nextKernel) {
                    for (const auto f : make_iterator_range(out_edges(u, partial))) {
                        const auto newGoal = partial[f];
                        assert (newGoal != kernelGoal);
                        const auto v = target(f, partial);
                        buildFullSchedule(v, y, newGoal, pid);
                    }
                } else {
                    buildFullSchedule(u, y, kernelGoal, pid);
                }
                assert (path.back() == nextKernel);
                path.pop_back();
            }
        }
    };

    assert (in_degree(0, partial) == 0);
    assert (out_degree(0, partial) > 0);

    assert (path.empty());

    #ifndef NDEBUG
    const auto & O = P[0].Orderings;
    assert (in_degree(1, O) == 0);
    assert (out_degree(1, O) > 0);
    #endif
    for (const auto f : make_iterator_range(out_edges(0, partial))) {
        const auto firstGoal = partial[f];
        buildFullSchedule(target(f, partial), 1, firstGoal, getPartitionId(firstGoal));
    }
    assert (path.empty());

    return schedule;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addSchedulingConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<unsigned> PipelineAnalysis::selectScheduleFromDAWG(const OrderingDAWG & schedule) const {

    // TODO: if we have multiple memory optimal schedules, look for the one that
    // keeps calls to the same kernel closer or permits a better memory layout
    // w.r.t. sequential memory prefetchers?

    std::vector<unsigned> program;
    Vertex position = 0;
    #ifndef NDEBUG
    flat_set<unsigned> included;
    #endif
    while (out_degree(position, schedule) > 0) {
        const auto e = first_out_edge(position, schedule);
        const auto s = schedule[e];
        assert (Relationships[s].Type == RelationshipNode::IsKernel);
        assert (included.insert(s).second);
        program.push_back(s);
        const auto next = target(e, schedule);
        assert (position != next);
        position = next;
    }

    #ifndef NDEBUG
    unsigned numOfKernels = 0;
    for (const auto u : make_iterator_range(vertices(Relationships))) {
        const RelationshipNode & rn = Relationships[u];
        if (rn.Type == RelationshipNode::IsKernel) {
            numOfKernels++;
        }
    }
    assert (program.size() == numOfKernels);
    #endif
    assert (program.front() == 0);
    assert (program.back() == PipelineOutput);
    return program;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addSchedulingConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::addSchedulingConstraints(const std::vector<unsigned> & program) {

    // Since we compressed the graph, nodes within O represent 0 to many kernels that
    // have cross partition I/O. These kernels could be from multiple partitions so
    // to simplify the logic, we initially create a partial program list then fill it
    // in by selecting a partition schedule that matches the selected program.


    auto itr = program.begin();
    const auto end = program.end();
    auto u = *itr;
    while (++itr != end) {
        const auto v = *itr;
        add_edge(u, v, RelationshipType{ReasonType::OrderingConstraint}, Relationships);
        u = v;
    }


}

} // end of namespace kernel

#endif // SCHEDULING_ANALYSIS_HPP
