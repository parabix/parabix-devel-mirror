#ifndef SCHEDULING_ANALYSIS_HPP
#define SCHEDULING_ANALYSIS_HPP

#include "pipeline_analysis.hpp"
#include <algorithm>
#include <random>

#include <chrono>

namespace kernel {

#ifdef EXPERIMENTAL_SCHEDULING

#define INITIAL_CANDIDATE_ATTEMPTS (100)

#define INITIAL_POPULATION_SIZE (5)

#define MAX_POPULATION_SIZE (15)

#define MAX_EVOLUTIONARY_ROUNDS (10)

#define CROSSOVER_PROBABILITY (0.2)

#define MUTATION_RATE (0.01)

#define MAX_CUT_ACO_ROUNDS (50)

#define UNPLACED (0U)

#define LEFT_HAND (1U)

#define RIGHT_HAND (2U)

#define SCHEDULING_FITNESS_COST_ACO_ALPHA (20)

#define SCHEDULING_FITNESS_COST_ACO_BETA (4)

#define SCHEDULING_FITNESS_COST_ACO_RHO (0.1)

#define SCHEDULING_FITNESS_COST_ACO_ROUNDS (10)

#define USE_COMPRESSED_PATH_SCHEDULING_GRAPH

static size_t init_time = 0;
static size_t fitness_time = 0;
static size_t repair_time = 0;
static size_t evolutionary_time = 0;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief gatherPartitionData
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<PartitionData> PipelineAnalysis::gatherPartitionData(const std::vector<unsigned> & orderingOfG) const {

    // Each program is constructed from some ordering of partitions.
    std::vector<PartitionData> P(PartitionCount - 1);

    for (const auto u : orderingOfG) {
        const RelationshipNode & node = Relationships[u];
        switch (node.Type) {
            case RelationshipNode::IsKernel:
            BEGIN_SCOPED_REGION
            const auto f = PartitionIds.find(u);
            assert (f != PartitionIds.end());
            const auto id = f->second;
            // partition 0 is reserved for fake pipeline I/O kernels
            if (id == 0) {
                continue;
            }
            assert (id < PartitionCount);
            P[id - 1].Kernels.push_back(u);
            END_SCOPED_REGION
            default: break;
        }
    }

    return P;
}

namespace {

using Vertex = unsigned;

using Candidate = std::vector<Vertex>;

struct SchedulingNode {

    enum NodeType {
        IsKernel = 0
        , IsStreamSet = 1
        , IsExternal = 2
    };

    NodeType Type = NodeType::IsKernel;
    Rational Size;

    SchedulingNode() = default;

    SchedulingNode(NodeType ty, Rational size = Rational{0})
    : Type(ty)
    , Size(size) {

    }
};

using SchedulingGraph = adjacency_list<vecS, vecS, bidirectionalS, SchedulingNode, Rational>;

class MemoryAnalysis {

    struct ACO {
        double Weight;
        double Pheromone;
    };

    using IntervalGraph = adjacency_list<vecS, vecS, undirectedS, no_property, ACO>;

    using IntervalEdge = typename IntervalGraph::edge_descriptor;

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
     * is based on Prim's spanning tree algorithm. Rather than taking a random spanning tree, however, it
     * combines an ant colony heuristic with it to locate the heaviest cut that does not increase the number of
     * connected components.
     *
     * In cases where a max-cut is necessary, this analyze function will return the chromatic number of the
     * optimal colouring based on the original algorithm PLUS a greedy colouring of the "uncut" edges as a
     * worst-case proxy for the true chromatic number.
     ** ------------------------------------------------------------------------------------------------------------- */
    size_t analyze(const Candidate & candidate) {

        assert (candidate.size() == numOfKernels);

        if (LLVM_LIKELY(numOfStreamSets == 0)) {
            return 0;
        }

        assert (numOfKernels > 1);

        const auto fitness_start = std::chrono::high_resolution_clock::now();

        // Each node value in the interval graph marks the position of the candidate schedule
        // that produced the streamset.

        IntervalGraph I(numOfStreamSets);

        std::fill_n(live.begin(), numOfStreamSets, 0);

        BEGIN_SCOPED_REGION

        unsigned streamSetId = 0;
        unsigned position = 0;
        for (const auto kernel : candidate) {
            assert (kernel < numOfKernels);
            assert (S[kernel].Type == SchedulingNode::IsKernel);
            for (const auto output : make_iterator_range(out_edges(kernel, S))) {
                const auto streamSet = target(output, S);
                const SchedulingNode & node = S[streamSet];
                if (LLVM_UNLIKELY(node.Type == SchedulingNode::IsExternal)) {
                    continue;
                }
                assert (node.Type == SchedulingNode::IsStreamSet);
                // assert (node.Size > 0);
                assert (streamSetId < numOfStreamSets);
                const auto i = streamSetId++;
                component[i] = position;
                for (unsigned j = 0; j != i; ++j) {
                    if (live[j] != 0) {
                        add_edge(j, i, I);
                        live[j]--;
                    }
                }
                live[i] = out_degree(streamSet, S);
            }
            ++position;
        }

        assert (position == numOfKernels);
        assert (streamSetId == numOfStreamSets);

        END_SCOPED_REGION

        const auto l = (2 * numOfKernels) + numOfStreamSets;

        TransitiveGraph G(l);

        std::fill_n(weight.begin(), l, 0);

        unsigned streamSetId = 0;
        unsigned priorProducerRank = 0;

        for (const auto kernel : candidate) {
            for (const auto output : make_iterator_range(out_edges(kernel, S))) {
                const auto streamSet = target(output, S);
                const SchedulingNode & node = S[streamSet];
                assert (node.Type != SchedulingNode::IsKernel);
                if (LLVM_UNLIKELY(node.Type == SchedulingNode::IsExternal)) {
                    continue;
                }
                assert (node.Type == SchedulingNode::IsStreamSet);
                assert (streamSetId < numOfStreamSets);
                const auto i = streamSetId++;

                // Each node value in I marks the schedule position.
                const auto producerRank = component[i];
                assert (priorProducerRank <= producerRank);
                priorProducerRank = producerRank;

                auto consumerRank = producerRank;
                for (const auto e : make_iterator_range(out_edges(i, I))) {
                    const auto j = target(e, I);
                    const auto rank = component[j];
                    consumerRank = std::max(consumerRank, rank);
                }

                const auto lifespan = consumerRank - producerRank;

                const auto & W = S[streamSet].Size;

                assert (W.denominator() == 1);
                assert (W.numerator() > 0);

                if (LLVM_LIKELY(lifespan <= 1)) {
                    const auto j = (2 * producerRank) | lifespan;
                    assert (j < (2 * numOfKernels));
                    weight[j] += W.numerator();

                    // If the lifespan of this streamset is at most one, we can place it into the
                    // comparability graph and do not need to reason about it within the forest.
                    clear_vertex(i, I);

                } else {

                    const auto j = (2 * numOfKernels) + i;
                    weight[j] = W.numerator();

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

redo_bipartite_check_after_max_cut:

        assert (placement.size() >= numOfStreamSets);

        for (unsigned i = 0; i < numOfStreamSets; ++i) {
            placement[i] = (out_degree(i, I) == 0) ? LEFT_HAND : UNPLACED;
        }

        unsigned N = 1;

        for (unsigned r = 0;;) {

            // select the first vertex to 0/1 colour.
            for (;;) {
                // if we've placed every vertex, we can ignore this phase.
                if (r == numOfStreamSets) {
                    goto is_bipartite_graph;
                }
                if (placement[r] == UNPLACED) {
                    break;
                }
                assert (r < numOfStreamSets);
                ++r;
            }
            assert (r < numOfStreamSets);
            placement[r] = LEFT_HAND;
            assert (stack.empty());

            for (auto u = r;;) {

                assert (placement[u] != UNPLACED);
                const auto OTHER_HAND = (placement[u] ^ (LEFT_HAND | RIGHT_HAND));
                component[u] = N;
                for (const auto e : make_iterator_range(out_edges(u, I))) {
                    const auto v = target(e, I);
                    if (placement[v] == UNPLACED) {
                        placement[v] = OTHER_HAND;
                        stack.push_back(v);
                    } else if (placement[v] != OTHER_HAND) {
                        stack.clear();
                        assert ("second bipartite check failed?" && (worstCaseUnderapproximation == 0));
                        worstCaseUnderapproximation = compute_max_cut(I);
                        goto redo_bipartite_check_after_max_cut;
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
            const auto inA = placement[i] == LEFT_HAND;
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
            const auto inA = placement[i] == LEFT_HAND;
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
        // from 0 to pow(2,N) - 1 to represent our current premutation. If N > 6,
        // we'll need another method to converge on a solution.

        assert (N <= 10);

        for (unsigned i = 0; i < (1U << N); ++i) {
            const auto weight = calculate_orientation_clique_weight(i, G);
            chromaticNumber = std::min<size_t>(chromaticNumber, weight);
        }

        const auto result = chromaticNumber + worstCaseUnderapproximation;

        const auto fitness_end = std::chrono::high_resolution_clock::now();
        fitness_time += (fitness_end - fitness_start).count();

//        errs() << "  candidate: ";
//        char joiner = '{';
//        for (const auto k : candidate) {
//            errs() << joiner << k;
//            joiner = ',';
//        }
//        errs() << "} := " << result << "\n";

        return result;
    }

private:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief calculate_orientation_clique_weight
     ** ------------------------------------------------------------------------------------------------------------- */
    unsigned calculate_orientation_clique_weight(const std::bitset<64> permutation, const TransitiveGraph & G) {

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


    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief compute_max_cut
     ** ------------------------------------------------------------------------------------------------------------- */
    size_t compute_max_cut(IntervalGraph & I) {

        // If G_I is not a bipartite graph, intuitively, we want to keep as many
        // interval relationships adjacent to heavy nodes as possible. So we're going
        // to apply a weighted max-cut to it to transform G_I into one by discarding
        // any "uncut" edges.

        constexpr double T_Init = 1.0;
        constexpr double T_Min = 0.001;

        constexpr double MAX_CUT_RHO = 0.005;
        constexpr double MAX_CUT_BETA = 1.0;

        // The following algorithm was originally based on the paper "An ant colony
        // algorithm for solving Max-cut problems" (2008). However, solutions found
        // by that approach typically resulted in many connected components (CCs).
        // Since the run-time of the interval colouring is O(2^(N + 1)), where N is
        // the number of CCs, general max-cut algorithms had a catastrophic impact
        // on the useability of the comparability graph approach.

        // To preserve the actual number of CCs, the algorithm was rewritten to use
        // Prims's spanning tree algorithm to define each cut set. The greedy local
        // search phase was also discarded.

        // Through empirical analysis, I modified the pheromone calculation function
        // to more quickly converge on a "good enough" solution but kept the same
        // +/- flavour of the paper's original deposit function.

        // TODO: I'm currently applying the Pythagorean theorem to the endpoint
        // weights to make the edge weight; hopefully this will prioritize placing
        // pairs of heavy nodes into differing sets over a heavy and light or two
        // light nodes. Investigate an alternate metric.

        for (const auto e : make_iterator_range(edges(I))) {
            const auto u = source(e, I);
            const auto v = target(e, I);
            const size_t Wu = weight[u + (2 * numOfKernels)];
            const size_t Wv = weight[v + (2 * numOfKernels)];
            const auto weight = std::log10(std::pow(std::sqrt(Wu * Wu + Wv * Wv), MAX_CUT_BETA));

            ACO & M = I[e];
            M.Weight = weight;
            assert (M.Weight > 0.0);
            M.Pheromone = T_Init;
        }

        const auto numOfComponents = collect_connected_components(I);

        compute_spanning_tree_to_determine_placement(I, numOfComponents);

        std::vector<unsigned> solution(placement);

        auto bestWeight = calculate_cut_weight(I);

        for (const auto e : make_iterator_range(edges(I))) {
            const auto u = source(e, I);
            const auto v = target(e, I);
            if (placement[u] != placement[v]) {
                ACO & M = I[e];
                M.Pheromone += 1.0;
            }
        }

        for (unsigned r = 0; r < MAX_CUT_ACO_ROUNDS; ++r) {

            compute_spanning_tree_to_determine_placement(I, numOfComponents);

            // check effect of this change
            const auto currentWeight = calculate_cut_weight(I);

            // update the pheromone matrix
            double deposit = 0.0;
            if (currentWeight > bestWeight) {
                const auto r = std::sqrt(currentWeight - bestWeight);
                deposit = r / (0.2 + r);
            } else if (currentWeight < bestWeight) {
                const auto r = std::sqrt(bestWeight - currentWeight);
                deposit = -r / (0.2 + r);
            }

            for (const auto e : make_iterator_range(edges(I))) {
                ACO & M = I[e];
                M.Pheromone *= (1.0 - MAX_CUT_RHO);
                const auto u = source(e, I);
                const auto v = target(e, I);
                if (placement[u] != placement[v]) {
                    M.Pheromone += deposit;
                }
                M.Pheromone = std::max(M.Pheromone, T_Min);
            }

            if (bestWeight < currentWeight) {
                bestWeight = currentWeight;
                solution.swap(placement);
            }

        }

        IntervalGraph B(numOfStreamSets);

        remove_edge_if([&](const IntervalGraph::edge_descriptor e){
            const auto u = source(e, I);
            const auto v = target(e, I);
            if (solution[u] == solution[v]) {
                return true;
            } else {
                add_edge(u, v, B);
                return false;
            }
        }, I);

        // For every node we place in which we have deleted an edge to transform
        // G_I into a bipartite graph, our colouring could be at most wrong by the
        // the total colours needed to colour the graph G_I' constructed from its
        // deleted edges. G_I is an interval graph but G_I' is not necessarily one
        // since we're generating it by removing edges not vertices. However,
        // LexBFS may still return a good ordering for a greedy colouring.

        // TODO: either prove the vertex index ordering of G_I is a reverse PEO
        // or implement LexBFS here. I suspect it is a reverse PEO since we can
        // orient any edges of a clique from left to right.

        I.swap(B);

        // Note: B here is originally I due to the prior swap
        return greedy_colouring(B);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief determine_component
     ** ------------------------------------------------------------------------------------------------------------- */
    unsigned collect_connected_components(const IntervalGraph & I) {

        assert (num_edges(I) > 0);

        std::fill_n(placement.begin(), numOfStreamSets, 0);

        std::fill_n(accum.begin(), numOfStreamSets, 0);

        unsigned k = 1;
        unsigned pos = 0;

        accum[0] = 0;

        for (unsigned i = 0; i < numOfStreamSets; ++i) {
            assert (stack.empty());
            if (placement[i] == 0 && out_degree(i, I) > 0) {
                auto u = i;
                for (;;) {
                    component[pos++] = u;
                    for (const auto e : make_iterator_range(out_edges(u, I))) {
                        const auto v = target(e, I);
                        if (placement[v] == 0) {
                            placement[v] = k;
                            stack.push_back(v);
                        }
                        assert (placement[v] == k);
                    }
                    if (stack.empty()) {
                        break;
                    }
                    u = stack.back();
                    stack.pop_back();
                }
                accum[k++] = pos; // store the first position of the next component
            }
        }
        assert (k > 1);
        return k - 1;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief compute_spanning_tree_to_determine_placement
     *
     * Uses a modified version of Prim's algorithm to find a maximal spanning forest; makes edge inclusion choices
     * using weights an ant colony heuristic.
     ** ------------------------------------------------------------------------------------------------------------- */
    void compute_spanning_tree_to_determine_placement(const IntervalGraph & I, const unsigned k) {

        SmallVector<unsigned, 4> roots(k);
        for (unsigned i = 0; i < k; ++i) {
            assert (accum[i + 1] > accum[i]);
            const auto m = accum[i + 1] - accum[i];
            std::uniform_int_distribution<unsigned> initial(0, m - 1);
            const auto j = initial(rng) + accum[i];
            assert (j < numOfStreamSets);
            roots[i] = component[j];
        }

        std::fill_n(placement.begin(), numOfStreamSets, UNPLACED);

        BitVector in_tree(numOfStreamSets);
        for (const auto root : roots) {
            placement[root] = LEFT_HAND;
            in_tree.set(root);
        }

        SmallVector<IntervalEdge, 8> selected;
        SmallVector<double, 8> probability;

        for (;;) {

            #ifndef NDEBUG
            auto remaining = in_tree.count();
            #endif

            assert (probability.empty() && selected.empty());

            double sum = 0.0;
            for (const auto u : in_tree.set_bits()) {
                assert (placement[u] != UNPLACED);
                bool all_adjacencies_added = true;
                for (const auto e : make_iterator_range(out_edges(u, I))) {
                    const auto v = target(e, I);
                    assert (v != u);
                    if (placement[v] == UNPLACED) {
                        all_adjacencies_added = false;
                        const ACO & M = I[e];
                        assert (M.Pheromone > 0);
                        const auto w = std::pow(M.Pheromone, 2.0) * M.Weight;
                        selected.push_back(e);
                        probability.push_back(w);
                        sum += w;
                    }
                }
                if (all_adjacencies_added) {
                    in_tree.reset(u);
                }
                #ifndef NDEBUG
                --remaining;
                #endif
            }

            assert ("failed to visit every incomplete node?" && remaining == 0);

            if (selected.empty()) {
                break;
            }

            assert (sum > 0.0);

            std::uniform_real_distribution<double> distribution(0.0, sum);
            const auto c = distribution(rng);

            double d = std::numeric_limits<double>::epsilon();
            const auto m = probability.size();
            bool found = false;
            for (unsigned i = 0; i < m; ++i) {
                d += probability[i];
                if (d >= c) {
                    const auto e = selected[i];
                    const auto u = source(e, I);
                    const auto v = target(e, I);
                    assert (placement[u] != UNPLACED && placement[v] == UNPLACED);
                    assert (!in_tree.test(v));
                    in_tree.set(v);
                    placement[v] = placement[u] ^ (LEFT_HAND | RIGHT_HAND);
                    found = true;
                    break;
                }
            }
            assert (found);
            probability.clear();
            selected.clear();
        }

        #ifndef NDEBUG
        for (unsigned i = 0; i < numOfStreamSets; ++i) {
            assert ((out_degree(i, I) == 0) || (placement[i] != UNPLACED));
        }
        #endif
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief calculate_cut_weight
     ** ------------------------------------------------------------------------------------------------------------- */
    double calculate_cut_weight(const IntervalGraph & I) const {
        double weight = 0;
        for (const auto e : make_iterator_range(edges(I))) {
            const auto u = source(e, I);
            const auto v = target(e, I);
            if (placement[u] != placement[v]) {
                weight += I[e].Weight;
            }
        }
        return weight;
    };

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief greedy_colouring
     ** ------------------------------------------------------------------------------------------------------------- */
    size_t greedy_colouring(const IntervalGraph & I) {

        using Interval = std::pair<unsigned, unsigned>;

        using ColourLine = flat_set<Interval>;

        std::vector<unsigned> remaining(numOfStreamSets, 0);

        std::vector<Interval> GC_Intervals(numOfStreamSets);

        ColourLine GC_CL;

        unsigned max_colours = 0;
        for (unsigned i = 0; i != numOfStreamSets; ++i) {
            const auto w = weight[i + (2 * numOfKernels)];

            if (w == 0) {
                remaining[i] = -1U;
            } else {
                remaining[i] = out_degree(i, I);
                unsigned first = 0;
                for (const auto & interval : GC_CL) {
                    const auto last = interval.first;
                    assert (first <= last);
                    if ((first + w) < last) {
                        break;
                    }
                    first = interval.second;
                }
                const auto last = first + w;
                assert (first <= last);
                if (last > max_colours) {
                    max_colours = last;
                }

                GC_Intervals[i] = std::make_pair(first, last);

                GC_CL.emplace(first, last);

                for (const auto e : make_iterator_range(out_edges(i, I))) {
                    const auto j = target(e, I);
                    if (j < i) {
                        assert (remaining[j] > 0 && remaining[j] < -1U);
                        remaining[j]--;
                    }
                }

                for (unsigned j = 0; j <= i; ++j) {
                    if (remaining[j] == 0) {
                        const auto f = GC_CL.find(GC_Intervals[j]);
                        assert (f != GC_CL.end());
                        GC_CL.erase(f);
                        remaining[j] = -1U;

                    }
                }
            }
        }
        return max_colours;
    };

public:

    MemoryAnalysis(const SchedulingGraph & S, const unsigned numOfKernels, const unsigned numOfStreamSets)
    : S(S)
    , numOfKernels(numOfKernels)
    , numOfStreamSets(numOfStreamSets)
    , rng(std::random_device()())
    , live(2 * numOfKernels + numOfStreamSets)
    , weight(2 * numOfKernels + numOfStreamSets)
    , component(numOfStreamSets)
    , placement(numOfStreamSets)
    , accum(2 * numOfKernels + numOfStreamSets) {

    }

protected:

    const SchedulingGraph & S;

    const unsigned numOfKernels;
    const unsigned numOfStreamSets;

    random_engine rng;

private:

    std::vector<unsigned> live;
    std::vector<unsigned> weight;
    std::vector<unsigned> component;
    std::vector<unsigned> placement;
    std::vector<Vertex> stack;
    std::vector<size_t> accum;

};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief SchedulingAnalysis
 *
 * Both the partition scheduling algorithm and whole program scheduling algorithm rely on the following class.
 * Within it is a genetic algorithm designed to find a minimum memory schedule of a given SchedulingGraph.
 * However, the phenotype of of the partition algorithm is a topological ordering and the phenotype of the
 * whole program is a hamiltonian path. This consitutes a significant enough difference that it is difficult
 * to call with only one function. Instead both the "initGA" and "repair" functions are implemented within
 * the actual scheduling functions.
 ** ------------------------------------------------------------------------------------------------------------- */
class SchedulingAnalysis {

public:

    using random_engine = std::default_random_engine;

    constexpr static size_t BITS_PER_SIZET = (CHAR_BIT * sizeof(size_t));

    struct permutation_bitset {

        permutation_bitset(const size_t N)
        : _value((N + BITS_PER_SIZET - 1) / BITS_PER_SIZET, 0) {

        }

        void randomize(random_engine & rng) {
            std::uniform_int_distribution<size_t> distribution(std::numeric_limits<size_t>::min(), std::numeric_limits<size_t>::max());
            for (auto & a : _value) {
                a = distribution(rng);
            }
        }

        bool test(unsigned i) const {
            return (_value[i / BITS_PER_SIZET] & (i & (BITS_PER_SIZET - 1))) != 0;
        }

    private:
        SmallVector<size_t, 4> _value;
    };

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief runGA
     ** ------------------------------------------------------------------------------------------------------------- */
    void runGA(OrderingDAWG & result) {

        if (initGA()) {
            goto found_all_orderings;
        }

        BEGIN_SCOPED_REGION

        const auto evolutionary_start = std::chrono::high_resolution_clock::now();

        using Item = std::pair<Candidate, unsigned>;

//        struct Item {
//            Candidate candidate;
//            unsigned  weight;
//        };


        using List = std::vector<Item>;

        permutation_bitset bitString(numOfKernels);

        BitVector V(numOfKernels);

        std::uniform_int_distribution<unsigned> randomKernel(0, numOfKernels - 1);

        std::uniform_real_distribution<double> zeroToOneReal(0.0, 1.0);

        assert (candidates.size() >= INITIAL_POPULATION_SIZE);

        List P1;
        P1.reserve(MAX_POPULATION_SIZE);

        List P2;
        P2.reserve(MAX_POPULATION_SIZE);


//        List P3;
//        P3.reserve(MAX_POPULATION_SIZE);

        // std::priority_queue<Item, std::vector<Item>


        for (unsigned round = 0; round < MAX_EVOLUTIONARY_ROUNDS; ++round) {

//            errs() << "EA: round=" << round << "\n";

            // SELECTION:

            P2.clear();

            P1.clear();

            P1.reserve(candidates.size());

            for (auto i : candidates) {
                P1.emplace_back(i.first, i.second);
            }

            std::sort(P1.begin(), P1.end(), [](const Item & A, const Item & B) {
                return A.second < B.second;
            });

            if (P1.size() > MAX_POPULATION_SIZE) {
                P1.resize(MAX_POPULATION_SIZE);
            }

            const auto populationSize = P1.size();

            std::uniform_int_distribution<unsigned> upToN(0, populationSize - 1);

            auto tournament_select = [&](const List & L) -> const Item & {
                const auto & A = L[upToN(rng)];
                const auto & B = L[upToN(rng)];
                if (A.second < B.second) {
                    return A;
                } else {
                    return B;
                }
            };

//            for (unsigned i = 0; i < populationSize; ++i) {
//                errs() << "  population: ";
//                const Item & I = P1[i];
//                char joiner = '{';
//                for (const auto k : I.first) {
//                    errs() << joiner << k;
//                    joiner = ',';
//                }
//                errs() << "} := " << I.second << "\n";
//            }

            for (unsigned i = 0; i < populationSize; ++i) {
                P2.emplace_back(tournament_select(P1));
            }

            assert (P2.size() == populationSize);


            // CROSSOVER:

//            P3.clear();
            for (unsigned i = 0; i < populationSize; ++i) {

                const Candidate & A = tournament_select(P2).first;
                const Candidate & B = tournament_select(P2).first;

                // generate a random bit string
                bitString.randomize(rng);

                auto crossover = [&](const Candidate & A, const Candidate & B, const bool selector) {

                    Candidate C(numOfKernels);

                    V.reset();

                    for (unsigned k = 0; k < numOfKernels; ++k) {
                        const auto t = bitString.test(k);
                        if (t == selector) {
                            V.set(A[k]);
                        } else {
                            C[k] = A[k];
                        }
                    }

                    for (unsigned k = 0U, p = -1U; k < numOfKernels; ++k) {
                        const auto t = bitString.test(k);
                        if (t == selector) {
                            // V contains 1-bits for every entry we did not
                            // directly copy from A into C. We now insert them
                            // into C in the same order as they are in B.
                            for (;;){
                                ++p;
                                assert (p < numOfKernels);
                                if (V.test(B[p])) break;
                            }
                            C[k] = B[p];
                        }
                    }
                    insertCandidate(C);
                };

                crossover(A, B, true);

                crossover(B, A, false);

            }

            // MUTATION:

            // Since we generated our initial candidates by taking the first N
            // orderings (with the hope that we have a simple enough graph that
            // we visit all of them), our first set of candidates may be
            // relatively uniform. Try mutating all of them on the first round.

            for (unsigned i = 0; i < populationSize; ++i) {
                const auto j = upToN(rng);
                auto & A = P2[j];
                if (round == 0 || zeroToOneReal(rng) <= MUTATION_RATE) {
                    auto a = randomKernel(rng);
                    for (;;) {
                        auto b = randomKernel(rng);
                        if (a == b) continue;
                        if (b < a) {
                            std::swap(a, b);
                        }
                        Candidate & C = A.first;
                        std::shuffle(C.begin() + a, C.begin() + b, rng);
                        insertCandidate(C);
                        break;
                    }
                }
            }

        }

        const auto evolutionary_end = std::chrono::high_resolution_clock::now();
        evolutionary_time += (evolutionary_end - evolutionary_start).count();

        END_SCOPED_REGION

found_all_orderings:

        // Construct a trie of all possible best orderings of this partition
        auto bestWeight = std::numeric_limits<size_t>::max();
        std::vector<Candidate> bestCandidates;

        for (auto pair : candidates) {
            if (pair.second < bestWeight) {
                bestCandidates.clear();
                bestWeight = pair.second;
                goto add_candidate;
            }
            if (pair.second == bestWeight) {
add_candidate: bestCandidates.emplace_back(std::move(pair.first));
            }
        }

        assert (bestCandidates.size() > 0);
        for (const Candidate & C : bestCandidates) {
            make_trie(C, result);
        }
        postorder_minimize(result);
    }

protected:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief insertCandidate
     ** ------------------------------------------------------------------------------------------------------------- */
    bool insertCandidate(Candidate & C) {
        repair(C);
        const auto f = candidates.emplace(C, 0);
        if (LLVM_LIKELY(f.second)) {
            f.first->second = fitness(f.first->first);
            return true;
        } else {
            return false;
        }
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief initGA
     ** ------------------------------------------------------------------------------------------------------------- */
    virtual bool initGA() = 0;

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief repair
     ** ------------------------------------------------------------------------------------------------------------- */
    virtual void repair(Candidate & L) = 0;

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief fitness
     ** ------------------------------------------------------------------------------------------------------------- */
    unsigned fitness(const Candidate & candidate) {

        const auto fitness_start = std::chrono::high_resolution_clock::now();

        const auto result = analyzer.analyze(candidate);

        const auto fitness_end = std::chrono::high_resolution_clock::now();
        fitness_time += (fitness_end - fitness_start).count();

        return result;
    }

private:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief make_trie
     ** ------------------------------------------------------------------------------------------------------------- */
    void make_trie(const Candidate & C, OrderingDAWG & O) {
        assert (num_vertices(O) > 0);
        assert (C.size() == numOfKernels);
        auto u = 0;

        for (unsigned i = 0; i != numOfKernels; ++i) {
            const auto j = C[i];
            assert (j < numOfKernels);
            for (const auto e : make_iterator_range(out_edges(u, O))) {
                if (O[e] == j) {
                    u = target(e, O);
                    goto in_trie;
                }
            }
            BEGIN_SCOPED_REGION
            const auto v = add_vertex(O);
            add_edge(u, v, j, O);
            u = v;
            END_SCOPED_REGION
in_trie:    continue;
        }
    };

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief postorder_minimize
     ** ------------------------------------------------------------------------------------------------------------- */
    void postorder_minimize(OrderingDAWG & O) {

        // Adapted from "Comparison of construction algorithms for minimal acyclic
        // deterministic finite-state automata from a set of strings." 2003

        // Since final/non-final states are indicated by whether we're at the last
        // level or not, we ignore such comparisons; thus any state whose (outgoing)
        // transitions match are considered equal. Additionally, all strings are of
        // equal length but were not lexographically inserted. However, since the
        // level of each state in the DAWG cannot change w.r.t. the trie, we simplify
        // the original algorithm to avoid using a hash table.

        using Vertex = OrderingDAWG::vertex_descriptor;

        std::vector<Vertex> L;

        const auto n = num_vertices(O);

        BitVector P(n);

        BEGIN_SCOPED_REGION

        unsigned sink = 0;

        for (unsigned i = 1; i < n; ++i) {
            if (out_degree(i, O) == 0) {
                assert (in_degree(i, O) > 0);
                const auto e = in_edge(i, O);
                const auto p = source(e, O);
                P.set(p);
                if (sink == 0) {
                    sink = i;
                } else {
                    const auto ch = O[e];
                    clear_in_edges(i, O);
                    add_edge(p, sink, ch, O);
                }
            }
        }

        END_SCOPED_REGION

        using SV = SmallVector<std::pair<unsigned, unsigned>, 8>;

        std::vector<SV> T;

        for (;;) {

            L.clear();
            for (const auto u : P.set_bits()) {
                if (LLVM_UNLIKELY(u == 0)) {
                    assert (P.count() == 1);
                    return;
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

    };

public:

    SchedulingAnalysis(const SchedulingGraph & S,
                       const unsigned numOfKernels, const unsigned numOfStreamSets, const unsigned populationSize)
    : S(S)
    , numOfKernels(numOfKernels)
    , numOfStreamSets(numOfStreamSets)
    , populationSize(populationSize)
    , analyzer(S, numOfKernels, numOfStreamSets)
    , rng(std::random_device()()) {

    }

protected:

    const SchedulingGraph & S;

    const unsigned numOfKernels;
    const unsigned numOfStreamSets;
    const unsigned populationSize;

    MemoryAnalysis analyzer;

    std::map<Candidate, size_t> candidates;

    random_engine rng;


};

using PartitionDependencyGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, no_property>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief PartitionSchedulingAnalysis
 ** ------------------------------------------------------------------------------------------------------------- */
struct PartitionSchedulingAnalysis : public SchedulingAnalysis {

protected:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief initGA
     ** ------------------------------------------------------------------------------------------------------------- */
    bool initGA() final {

        // Any topological ordering of D can generate a valid schedule for our subgraph.
        // Begin by trying to generate N initial candidates. If we fail to enumerate all
        // of them, we'll use an evolutionary algorithm to try and explore the remaining
        // solution space.

#warning switch this back before release

        return enumerateUpToNTopologicalOrderings(D, populationSize, [&](Candidate & L) {
            insertCandidate(L);
            // candidates.emplace(L, fitness(L));
        });

    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief repair
     ** ------------------------------------------------------------------------------------------------------------- */
    void repair(Candidate & L) final {

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

    PartitionSchedulingAnalysis(const SchedulingGraph & S, const PartitionDependencyGraph & D,
                                const unsigned numOfKernels, const unsigned numOfStreamSets)
    : SchedulingAnalysis(S, numOfKernels, numOfStreamSets, 30)
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
 * @brief ProgramSchedulingAnalysis
 ** ------------------------------------------------------------------------------------------------------------- */
struct ProgramSchedulingAnalysis : public SchedulingAnalysis {
protected:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief initGA
     ** ------------------------------------------------------------------------------------------------------------- */
    bool initGA() final {

        bool r = true;

        const auto init_start = std::chrono::high_resolution_clock::now();

        Candidate candidate(numOfKernels);
        std::iota(candidate.begin(), candidate.end(), 0);
        for (unsigned i = 0; i < 10; ++i) {
            std::shuffle(candidate.begin(), candidate.end(), rng);
            if (insertCandidate(candidate)) {
                if (candidates.size() >= populationSize) {
                    r = false;
                    break;
                }
            }
        }

        const auto init_end = std::chrono::high_resolution_clock::now();
        init_time += (init_end - init_start).count();

        return r;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief repair
     ** ------------------------------------------------------------------------------------------------------------- */
    void repair(Candidate & candidate) final {

#error here

        const auto repair_start = std::chrono::high_resolution_clock::now();

        // const double T_Max = 1000.0;
        const double T_Init = 1.0;
        const double T_Min = 0.001;

        const auto firstKernelNode = (ActivePartitionCount * 2) + 2;

        assert (candidate.size() == numOfKernels);
        assert (index.size() == numOfKernels);

        // record the index position of each kernel in the candidate
        for (unsigned i = 0; i < numOfKernels; ++i) {
            const auto j = candidate[i];
            assert (j < numOfKernels);
            index[j] = i;
        }

        auto missing_cost = [](const size_t n) -> size_t {

            // Suppose we we generate a path that is the complete reverse of our candidate L.
            // The first element would add (n - 1) inversions to the score; the second, (n - 3),
            // and so on. SUM |n - 2k - 1| <= n^2/2.

            const auto k = (n + 10);
            const auto k3 = k * k * k;
            assert ("overflow error?" && (k3 >= 1000));
            return k3 - 1000;
        };

        auto bestInversions = missing_cost(numOfKernels);

        const auto n = num_vertices(O);

        unsigned converged = 0;

restart_process:

        for (const auto e : make_iterator_range(edges(O))) {
            const unsigned u = source(e, O);
            const unsigned v = target(e, O);
            double value = T_Init;
            if (u < firstKernelNode || v < firstKernelNode) {
                value = T_Min;
            }
            pheromone[std::make_pair(u, v)] = value;
        }

        replacement.clear();

        for (unsigned r = 0; r < 30; ++r) {

            std::fill_n(visited.begin(), n, false);

            Vertex u = 0;

            path.clear();

            for (;;) {

                assert (!visited[u]);

                visited[u] = true;

                path.push_back(u);

                targets.clear();

                for (const auto e : make_iterator_range(out_edges(u, O))) {
                    const auto v = target(e, O);
                    if (visited[v]) continue;
                    const auto f = pheromone.find(std::make_pair(u, v));
                    assert (f != pheromone.end());
                    targets.emplace_back(v, f->second);
                }

                const auto m = targets.size();

                if (m <= 1) {
                    if (LLVM_UNLIKELY(m == 0)) {
                        break;
                    } else {
                        u = targets.front().first;
                        continue;
                    }
                }

                double sum = 0.0;

                for (const auto & t : targets) {
                    sum += t.second;
                }

                std::uniform_real_distribution<double> distribution(0.0, sum);
                const auto c = distribution(rng);

                double d = std::numeric_limits<double>::epsilon();
                for (const auto & t : targets) {
                    d += t.second;
                    if (d >= c) {
                        u = t.first; // set our next target
                        break;
                    }
                }

            }

            const auto m = path.size();

            assert (m <= n);

            // Count how many inversions occured in this path but since we may not
            // have acutally constructed a hamiltonian path, initialize the cost to
            // penalize such solutions.

            assert (result.empty());

            for (unsigned i = 0; i < m; ++i) {
                const auto u = path[i];
                const auto & V = O[u];
                if (V.empty()) continue;
                result.insert(result.end(), V.begin(), V.end());
            }

            const auto l = result.size();
            assert (l <= numOfKernels);

            auto inversions = missing_cost(numOfKernels - l);
            for (unsigned i = 0; i < l; ++i) {
                const auto j = result[i];
                assert (j < numOfKernels);
                const auto k = index[j];
                const auto d = (k < i) ? (i - k) : (k - i);
                inversions += d;
            }

            double deposit = 0.0;
            if (inversions > bestInversions) {
                const auto d = (inversions - bestInversions);
                const auto r = std::sqrt(d);
                deposit = -5.0 * r / (20.0 + r);
            } else if (inversions < bestInversions) {
                const auto d = std::log10(bestInversions - inversions);
                const auto r = std::sqrt(d);
                deposit = r / (0.125 + r);
            }

            for (auto & p : pheromone) {
                double & trail = p.second;
                trail *= (1.0 - SCHEDULING_FITNESS_COST_ACO_RHO);
            }

            for (unsigned i = 1; i < m; ++i) {
                const auto e = std::make_pair(path[i - 1], path[i]);
                const auto f = pheromone.find(e);
                assert (f != pheromone.end());
                double & trail = f->second;
                trail += deposit;
            }

            for (auto & p : pheromone) {
                double & trail = p.second;
                trail = std::max(trail, T_Min);
            }

            // Store our path if its the best one
            if (inversions == bestInversions) {
                ++converged;
            } else if (inversions < bestInversions) {
                converged = 0;
                if (l == numOfKernels) {
                    replacement.swap(result);
                }
                bestInversions = inversions;
            }

            result.clear();

            if (converged == 3) {
                break;
            }

        }

        // If we converged to a solution but failed to find a valid hamiltonian path,
        // just restart the process. We're guaranteed to find one eventually.
        if (replacement.empty()) {
            goto restart_process;
        }

        assert (replacement.size() == numOfKernels);

        candidate.swap(replacement);

        const auto repair_end = std::chrono::high_resolution_clock::now();
        repair_time += (repair_end - repair_start).count();

    }

public:

    ProgramSchedulingAnalysis(const SchedulingGraph & S,
                              const PartitionOrderingGraph & O,
                              const unsigned partitionCount,
                              const unsigned numOfKernels, const unsigned numOfStreamSets)
    : SchedulingAnalysis(S, numOfKernels, numOfStreamSets, 5)
    , O(O)
    , ActivePartitionCount(partitionCount)
    , visited(num_vertices(O))
    , index(numOfKernels) {
        path.reserve(num_vertices(O));
        result.reserve(numOfKernels);
        replacement.reserve(numOfKernels);
        pheromone.reserve(num_edges(O));
    }

private:

    const PartitionOrderingGraph & O;

    const unsigned ActivePartitionCount;

    std::vector<bool> visited;

    std::vector<std::pair<Vertex, double>> targets;

    flat_map<std::pair<Vertex, Vertex>, double> pheromone;

    std::vector<unsigned> index;

    std::vector<unsigned> result;

    Candidate path;

    Candidate replacement;

};

}; // end of anonymous namespace

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief analyzeDataflowWithinPartitions
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::analyzeDataflowWithinPartitions(std::vector<PartitionData> & P) const {

    using SchedulingMap = flat_map<Vertex, Vertex>;

    /// --------------------------------------------
    /// Construct our partition schedules
    /// --------------------------------------------

    const auto cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "model", "true");
    Z3_set_param_value(cfg, "proof", "false");
    const auto ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);

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

        PartitionData & currentPartition = P[currentPartitionId - 1];

        const auto & kernels = currentPartition.Kernels;
        const auto numOfKernels = kernels.size();

        SchedulingGraph S(numOfKernels);

        unsigned internalStreamSets = 0;

        BEGIN_SCOPED_REGION

        SchedulingMap M;

        unsigned currentKernelId = 0;

        for (const auto u : kernels) {
            const RelationshipNode & node = Relationships[u];
            assert (node.Type == RelationshipNode::IsKernel);

            const auto f = PartitionIds.find(u);
            assert (f != PartitionIds.end());
            const auto id = f->second;
            if (LLVM_LIKELY(id == currentPartitionId)) {

                const auto strideSize = node.Kernel->getStride();

                const auto su = currentKernelId++;

                for (const auto e : make_iterator_range(in_edges(u, Relationships))) {
                    const auto binding = source(e, Relationships);
                    if (Relationships[binding].Type == RelationshipNode::IsBinding) {
                        const auto f = first_in_edge(binding, Relationships);
                        assert (Relationships[f].Reason != ReasonType::Reference);
                        const auto streamSet = source(f, Relationships);
                        // is this streamset in the subgraph? if so, add an edge.
                        const auto m = M.find(streamSet);
                        if (LLVM_LIKELY(m != M.end())) {
                            const RelationshipNode & rn = Relationships[binding];
                            const Binding & b = rn.Binding;
                            const ProcessingRate & rate = b.getRate();

                            // If we have a PopCount producer/consumer in the same partition,
                            // they're both perform an identical number of strides. So long
                            // as the producing/consuming strideRate match, the equation will
                            // work. Since the lower bound of PopCounts is 0, we always use the
                            // upper bound.
                            Rational itemsPerStride{rate.getUpperBound() * strideSize};
                            const auto sv = m->second;
                            add_edge(sv, su, itemsPerStride, S);
                        }
                    }
                }

                for (const auto e : make_iterator_range(out_edges(u, Relationships))) {
                    const auto binding = target(e, Relationships);
                    if (Relationships[binding].Type == RelationshipNode::IsBinding) {
                        const auto f = first_out_edge(binding, Relationships);
                        assert (Relationships[f].Reason != ReasonType::Reference);
                        const auto streamSet = target(f, Relationships);
                        assert ("a streamset can only have one producer" && M.count(streamSet) == 0);
                        assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
                        assert (isa<StreamSet>(Relationships[streamSet].Relationship));

                        // Check whether this streamset node ought to be marked as External or ignored.
                        bool isExternal = false;
                        bool hasInternal = false;
                        for (const auto e : make_iterator_range(out_edges(streamSet, Relationships))) {
                            const auto binding = target(e, Relationships);
                            assert (Relationships[binding].Type == RelationshipNode::IsBinding);
                            const auto input = first_out_edge(binding, Relationships);
                            const auto kernel = target(input, Relationships);
                            assert (Relationships[kernel].Type == RelationshipNode::IsKernel);
                            const auto f = PartitionIds.find(kernel);
                            assert (f != PartitionIds.end());
                            const auto id = f->second;
                            if (LLVM_LIKELY(id == currentPartitionId)) {
                                hasInternal = true;
                            } else {
                                isExternal = true;
                            }
                            if (hasInternal && isExternal) {
                                goto checked_users;
                            }
                        }
checked_users:          if (hasInternal) {
                            const RelationshipNode & rn = Relationships[binding];
                            const Binding & b = rn.Binding;
                            const ProcessingRate & rate = b.getRate();
                            Rational itemsPerStride{rate.getUpperBound() * strideSize};
                            const auto type = isExternal ? SchedulingNode::IsExternal : SchedulingNode::IsStreamSet;
                            internalStreamSets += isExternal ? 0 : 1;
                            const Rational bytesPerItem{b.getFieldWidth() * b.getNumElements(), 8};
                            const auto sv = add_vertex(SchedulingNode{type, bytesPerItem}, S);
                            add_edge(su, sv, itemsPerStride, S);
                            M.emplace(streamSet, sv);
                        }
                    }
                }
            }
        }

        END_SCOPED_REGION


        BEGIN_SCOPED_REGION

        /// ------------------------------------------------------------------------
        /// Determine how many "invocations" are required by each kernel so that we
        /// can correctly scale the streamset sizes based on the dataflow rates
        /// ------------------------------------------------------------------------

        const auto solver = Z3_mk_solver(ctx);
        Z3_solver_inc_ref(ctx, solver);

        const auto varType = Z3_mk_real_sort(ctx);

        auto constant = [&](const Rational value) {
            return Z3_mk_real(ctx, value.numerator(), value.denominator());
        };

        const auto ONE = constant(1);

        auto free_variable = [&]() {
            auto v = Z3_mk_fresh_const(ctx, nullptr, varType);
            auto c1 = Z3_mk_ge(ctx, v, ONE);
            Z3_solver_assert(ctx, solver, c1);
            return v;
        };

        auto multiply =[&](Z3_ast X, Z3_ast Y) {
            Z3_ast args[2] = { X, Y };
            return Z3_mk_mul(ctx, 2, args);
        };

        auto assert_equals =[&](Z3_ast X, Z3_ast Y) {
            const auto Z = Z3_mk_eq(ctx, X, Y);
            Z3_solver_assert(ctx, solver, Z);
        };

        const auto n = num_vertices(S);

        std::vector<Z3_ast> VarList(n, nullptr);

        for (unsigned u = 0; u < numOfKernels; ++u) {
            assert (S[u].Type == SchedulingNode::IsKernel);
            VarList[u] = free_variable();
            for (const auto e : make_iterator_range(in_edges(u, S))) {
                const auto streamSet = source(e, S);
                const auto producedRate = VarList[streamSet];
                const auto fixedRateVal = constant(S[e]);
                const auto consumedRate = multiply(VarList[u], fixedRateVal);
                assert_equals(producedRate, consumedRate);
            }
            for (const auto e : make_iterator_range(out_edges(u, S))) {
                const auto streamSet = target(e, S);
                const auto fixedRateVal = constant(S[e]);
                const auto producedRate = multiply(VarList[u], fixedRateVal);
                VarList[streamSet] = producedRate;
            }
        }

        if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) == Z3_L_FALSE)) {
            report_fatal_error("Z3 failed to find a solution to synchronous dataflow graph");
        }

        const auto model = Z3_solver_get_model(ctx, solver);
        Z3_model_inc_ref(ctx, model);

        size_t lcm = 1;

        for (unsigned u = 0; u < numOfKernels; ++u) {
            assert (S[u].Type == SchedulingNode::IsKernel);
            assert(VarList[u]);
            Z3_ast value;
            if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, VarList[u], Z3_L_TRUE, &value) != Z3_L_TRUE)) {
                report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
            }

            __int64 num, denom;
            if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
                report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
            }
            assert (num > 0);

            // scale each streamSet node size field by the replication vector
            const Rational replicationFactor{num, denom};
            S[u].Size = replicationFactor;
            for (const auto e : make_iterator_range(out_edges(u, S))) {
                const auto v = target(e, S);
                assert (u != v);
                auto & Sv = S[v];
                assert (Sv.Type != SchedulingNode::IsKernel);
                const auto & itemsPerStride = S[e];
                Sv.Size *= (itemsPerStride * replicationFactor);
                lcm = boost::lcm(lcm, Sv.Size.denominator());
            }
        }

        Z3_model_dec_ref(ctx, model);
        Z3_solver_dec_ref(ctx, solver);

        // NOTE: the LCM should always be 1 here but Z3 converges on a solution faster when rational numbers
        // are allowed so the following handles any degenerate solution. I've never seen this branch entered
        // during testing but the remainder of this function requires an integer solution and assumes it to
        // be coprime.

        if (LLVM_UNLIKELY(lcm != 1)) {
            size_t gcd = 0;
            for (unsigned u = 0; u < n; ++u) {
                auto & Su = S[u];
                Su.Size *= lcm;
                assert (Su.Size.denominator() == 1);
                if (gcd == 0) {
                    gcd = Su.Size.numerator();
                } else {
                    gcd = boost::gcd(gcd, Su.Size.numerator());
                }
            }
            if (LLVM_UNLIKELY(gcd > 1)) {
                for (unsigned u = 0; u < n; ++u) {
                    auto & Su = S[u];
                    Su.Size /= gcd;
                }
            }
        }

        currentPartition.Repetitions.resize(numOfKernels);

        for (unsigned u = 0; u < numOfKernels; ++u) {
            auto & Su = S[u];
            assert (Su.Size.denominator() == 1);
            currentPartition.Repetitions[u] = Su.Size.numerator();
        }

        END_SCOPED_REGION

        // We want to generate a subgraph of S consisting of only the kernel nodes
        // but whose edges initially represent the transitive closure of S. Once we
        // generate this graph, we remove the edges associated with the streamsets.
        // The final graph is the kernel dependency graph of S.

        // TODO: we ought to reason about paths in D independently since they are
        // subgraphs of S with a single topological ordering.

        PartitionDependencyGraph D(numOfKernels);

        if (numOfKernels > 1) {
            const auto n = num_vertices(S);
            BitVector transitive(n);

            // Generate the transitive closure as we create the dependency graph D
            for (unsigned u = 0; u < numOfKernels; ++u) {
                assert (S[u].Type == SchedulingNode::IsKernel);

                // determine which streamsets this kernel transitively depends on.
                transitive.reset();

                for (const auto e : make_iterator_range(in_edges(u, S))) {
                    const auto v = source(e, S);
                    assert (S[v].Type != SchedulingNode::IsKernel);
                    for (const auto f : make_iterator_range(in_edges(v, S))) {
                        const auto w = source(f, S);
                        assert (S[w].Type == SchedulingNode::IsKernel);
                        assert (w < transitive.size());
                        transitive.set(w);
                    }
                }

                assert (in_degree(u, S) == 0 || transitive.any());

                for (const auto w : transitive.set_bits()) {
                    add_edge(w, u, D);
                }
            }

            // Since transitive reductions of DAGs are unique, we take the transitive reduction of D
            // to slightly simplify the problem later.
            transitive_reduction_dag(reverse_traversal{numOfKernels - 1}, D);
        }

        // Now we begin the genetic algorithm phase; our overall goal is to find a schedule that
        // permits a minimum memory schedule.

        PartitionSchedulingAnalysis SA(S, D, numOfKernels, internalStreamSets);

        SA.runGA(currentPartition.Orderings);

    }

    Z3_reset_memory();
    Z3_del_context(ctx);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief analyzeDataflowBetweenPartitions
 ** ------------------------------------------------------------------------------------------------------------- */
PartitionDataflowGraph PipelineAnalysis::analyzeDataflowBetweenPartitions(std::vector<PartitionData> & P) const {

    using DataflowMap = flat_map<Vertex, Vertex>;

    const auto activePartitions = (PartitionCount - 1);

    PartitionDataflowGraph G(activePartitions);

    DataflowMap M;

    // create a bipartite graph consisting of partitions and cross-partition
    // streamset nodes and relationships
    for (unsigned partitionId = 0; partitionId < activePartitions; ++partitionId) {
        PartitionData & N = P[partitionId];
        const auto n = N.Kernels.size();

        // consumers
        for (unsigned i = 0; i != n; ++i) {
            const auto consumer = N.Kernels[i];
            const RelationshipNode & node = Relationships[consumer];
            assert (node.Type == RelationshipNode::IsKernel);
            for (const auto e : make_iterator_range(in_edges(consumer, Relationships))) {
                const auto binding = source(e, Relationships);
                const RelationshipNode & input = Relationships[binding];
                if (LLVM_LIKELY(input.Type == RelationshipNode::IsBinding)) {

                    const auto f = first_in_edge(binding, Relationships);
                    assert (Relationships[f].Reason != ReasonType::Reference);
                    const auto streamSet = source(f, Relationships);
                    assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
                    assert (isa<StreamSet>(Relationships[streamSet].Relationship));

                    // Is this an external streamset?
                    const auto m = M.find(streamSet);
                    if (LLVM_LIKELY(m == M.end())) {
                        continue;
                    }

                    const auto streamSetNode = m->second;
                    const auto strideSize = node.Kernel->getStride() * N.Repetitions[i];

                    const Binding & b = input.Binding;
                    const ProcessingRate & rate = b.getRate();

                    const auto sum = rate.getLowerBound() + rate.getUpperBound();
                    const auto expected = sum * Rational{strideSize, 2};

                    add_edge(streamSetNode, partitionId, PartitionDataflowEdge{consumer, rate.getKind(), expected}, G);
                }
            }
        }

        // producers
        for (unsigned i = 0; i != n; ++i) {

            const auto producer = N.Kernels[i];
            const RelationshipNode & node = Relationships[producer];
            assert (node.Type == RelationshipNode::IsKernel);

            for (const auto e : make_iterator_range(out_edges(producer, Relationships))) {
                const auto binding = target(e, Relationships);
                const RelationshipNode & output = Relationships[binding];
                if (LLVM_LIKELY(output.Type == RelationshipNode::IsBinding)) {

                    const auto f = first_out_edge(binding, Relationships);
                    assert (Relationships[f].Reason != ReasonType::Reference);
                    const auto streamSet = target(f, Relationships);
                    assert (Relationships[streamSet].Type == RelationshipNode::IsRelationship);
                    assert (isa<StreamSet>(Relationships[streamSet].Relationship));

                    // Check whether this streamset is used externally

                    for (const auto g : make_iterator_range(out_edges(streamSet, Relationships))) {
                        const auto binding = target(g, Relationships);
                        const RelationshipNode & input = Relationships[binding];
                        assert (input.Type == RelationshipNode::IsBinding);
                        const auto f = first_out_edge(binding, Relationships);
                        assert (Relationships[f].Reason != ReasonType::Reference);
                        const auto consumer = target(f, Relationships);
                        assert (Relationships[consumer].Type == RelationshipNode::IsKernel);
                        const auto h = PartitionIds.find(consumer);
                        assert (h != PartitionIds.end());
                        const auto consumingPartitionId = h->second - 1;
                        if (partitionId != consumingPartitionId) {
                            goto is_external_streamset;
                        }
                    }

                    continue;

is_external_streamset:

                    const Binding & b = output.Binding;
                    const Rational bytesPerItem{b.getFieldWidth() * b.getNumElements(), 8};
                    const auto streamSetNode = add_vertex(bytesPerItem, G);

                    M.emplace(streamSet, streamSetNode);

                    const ProcessingRate & rate = b.getRate();

                    const auto strideSize = node.Kernel->getStride() * N.Repetitions[i];
                    const auto sum = rate.getLowerBound() + rate.getUpperBound();
                    const auto expected = sum * Rational{strideSize, 2};

                    add_edge(partitionId, streamSetNode, PartitionDataflowEdge{producer, rate.getKind(), expected}, G);
                }
            }
        }
    }

    #warning incorporate length equality assertions

    const auto cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "model", "true");
    Z3_set_param_value(cfg, "proof", "false");
    const auto ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);

    const auto varType = Z3_mk_real_sort(ctx);

    auto constant = [&](const Rational value) {
        return Z3_mk_real(ctx, value.numerator(), value.denominator());
    };

    const auto ONE = constant(1);

    std::vector<Rational> expectedStrides(activePartitions);

    std::vector<Z3_ast> assumptions;

    const auto n = num_vertices(G);

    std::vector<Z3_ast> VarList(activePartitions);

    const auto solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);

    auto free_variable = [&]() {
        auto v = Z3_mk_fresh_const(ctx, nullptr, varType);
        auto c1 = Z3_mk_ge(ctx, v, ONE);
        Z3_solver_assert(ctx, solver, c1);
        return v;
    };

    auto multiply =[&](Z3_ast X, Z3_ast Y) {
        Z3_ast args[2] = { X, Y };
        return Z3_mk_mul(ctx, 2, args);
    };

    for (unsigned i = 0; i < activePartitions; ++i) {
        VarList[i] = free_variable();
    }

    #warning this doesn't correctly handle greedy/unknown rates

    for (unsigned streamSet = activePartitions; streamSet < n; ++streamSet) {
        const auto output = in_edge(streamSet, G);
        const auto outputRate = constant(G[output].Expected);
        const auto producer = source(output, G);
        assert (producer < activePartitions);
        const auto outputRateVar = multiply(VarList[producer], outputRate);
        for (const auto input : make_iterator_range(out_edges(streamSet, G))) {
            const auto inputRate = constant(G[input].Expected);
            const auto consumer = target(input, G);
            assert (consumer < activePartitions);
            const auto inputRateVar = multiply(VarList[consumer], inputRate);
            assumptions.push_back(Z3_mk_eq(ctx, outputRateVar, inputRateVar));
        }
    }

    const auto m = Z3_maxsat(ctx, solver, assumptions);
    if (LLVM_UNLIKELY(m == 0)) {
        Z3_solver_pop(ctx, solver, 1);
        Z3_solver_check(ctx, solver);
    }

    const auto model = Z3_solver_get_model(ctx, solver);
    Z3_model_inc_ref(ctx, model);
    for (unsigned i = 0; i < activePartitions; ++i) {

        Z3_ast const stridesPerSegmentVar = VarList[i];
        Z3_ast value;
        if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, stridesPerSegmentVar, Z3_L_TRUE, &value) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to obtain value from model!");
        }

        __int64 num, denom;
        if (LLVM_UNLIKELY(Z3_get_numeral_rational_int64(ctx, value, &num, &denom) != Z3_L_TRUE)) {
            report_fatal_error("Unexpected Z3 error when attempting to convert model value to number!");
        }
        assert (num > 0);

        expectedStrides[i] = Rational{num, denom};
    }
    Z3_model_dec_ref(ctx, model);
    Z3_solver_dec_ref(ctx, solver);

    Z3_reset_memory();
    Z3_del_context(ctx);

//    size_t lcmFactor{1};
    for (unsigned partitionId = 0; partitionId < activePartitions; ++partitionId) {
        PartitionData & N = P[partitionId];
        N.ExpectedRepetitions = expectedStrides[partitionId];
    }

//    for (unsigned partitionId = 0; partitionId < m; ++partitionId) {
//        PartitionData & N = P[partitionId];
//        const auto rep = expectedStrides[partitionId] * lcmFactor;
//        assert (rep.denominator() == 1);
//        const auto n = N.Repetitions.size();
//        for (unsigned i = 0; i < n; ++i) {
//            const auto r = rep * N.Repetitions[i];
//            assert (r.denominator() == 1);
//            N.Repetitions[i] = r.numerator();
//        }
//    }

//    for (unsigned streamSet = m; streamSet < n; ++streamSet) {
//        const auto output = in_edge(streamSet, G);
//        const auto producer = source(output, G);
//        PartitionDataflowEdge & O = G[output];
//        assert (producer < m);
//        O.Expected *= expectedStrides[producer] * lcmFactor;
//    }

    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePartitionSchedulingGraph
 ** ------------------------------------------------------------------------------------------------------------- */
PartitionOrdering PipelineAnalysis::makePartitionSchedulingGraph(std::vector<PartitionData> & P, const PartitionDataflowGraph & D) const {

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
    using PathGraph = adjacency_list<vecS, vecS, bidirectionalS>;

    const auto activePartitions = (PartitionCount - 1);

    PathGraph H(activePartitions + 2);

    flat_set<unsigned> kernels;
    kernels.reserve(PartitionIds.size());

    // since we could have multiple source/sink nodes in P, we always
    // add two fake nodes to H for a common source/sink.

     const auto l = activePartitions + 2U;

    BV M(l);

    for (unsigned i = 0; i < activePartitions; ++i) {

        if (in_degree(i, D) == 0) {
            add_edge(0, i + 1U, H);
        } else {
            for (const auto e : make_iterator_range(in_edges(i, D))) {
                const PartitionDataflowEdge & C = D[e];
                kernels.insert(C.KernelId);
            }
        }

        if (out_degree(i, D) == 0) {
            add_edge(i + 1U, activePartitions + 1U, H);
        } else {
            assert (M.none());
            for (const auto e : make_iterator_range(out_edges(i, D))) {
                const PartitionDataflowEdge & C = D[e];
                assert (Relationships[C.KernelId].Type == RelationshipNode::IsKernel);
                kernels.insert(C.KernelId);
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
                add_edge(i + 1U, j + 1U, H);
            }
            M.reset();
        }
        assert (in_degree(i + 1, H) > 0);
        assert (out_degree(i + 1, H) > 0);
    }

    transitive_closure_dag(reverse_traversal{l - 1}, H);
    transitive_reduction_dag(reverse_traversal{l - 1}, H);

    // To find our hamiltonian path later, we need a path from each join
    // in the graph to the other forked paths (including the implicit
    // "terminal" node.) Compute the post-dominator tree of H then insert
    // the appropriate edges from the immediate predecessor of each join
    // to the child of each dominating fork.

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

    std::vector<BV> ancestors(l);
    for (unsigned i = 0; i < l; ++i) {  // forward topological ordering
        const auto d = in_degree(i, H);
        if (d > 1) {
            // Determine the common ancestors of each input to node_i
            for (unsigned j = 0; j < l; ++j) {
                ancestors[j].resize(d);
                ancestors[j].reset();
            }
            unsigned k = 0;
            for (const auto e : make_iterator_range(in_edges(i, H))) {
                const auto v = source(e, H);
                ancestors[v].set(k++);
            }
            std::fill_n(occurences.begin(), rank[i] - 1, 0);
            for (auto j = i; j--; ) { // reverse topological ordering
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
            // to all inputs that is of highest rank.
            auto lca = i;
            for (auto j = rank[i] - 1; j--; ) {
                if (occurences[j] == 1) {
                    lca = singleton[j];
                    break;
                }
            }
            assert (lca < i);

            for (const auto e : make_iterator_range(out_edges(lca, H))) {
                const auto y = target(e, H);
                const BV & Py = postdom[y];
                for (const auto f : make_iterator_range(in_edges(i, H))) {
                    const auto x = source(f, H);
                    const BV & Px = postdom[x];
                    // do not arc back to the start of the dominating path.
                    if (!Px.is_subset_of(Py)) {
                        add_edge(x, y, H);
                    }
                }
            }


        }
    }

    END_SCOPED_REGION

    // We now have our auxillary graph complete and are ready to construct our
    // scheduling graph G.

    const auto firstKernelNode = (2U * activePartitions) + 2U;

    const auto numOfFrontierKernels = kernels.size();

    PartitionOrderingGraph G(firstKernelNode + numOfFrontierKernels);

    // Split every node except for the common sink/source in H into two nodes in G;
    // this will form the backbone of the partition scheduling constraint graph

    for (unsigned i = 0; i <= activePartitions; ++i) {
        const auto partitionOut = (i * 2);
        for (const auto e : make_iterator_range(out_edges(i, H))) {
            const auto j = target(e, H);
            const auto outgoing = (j * 2) - 1;
            assert (outgoing < firstKernelNode);
            add_edge(partitionOut, outgoing, G);
        }
    }

    // Iterage through each partition and add any kernel with cross-partition I/O
    // to G. For every optimal ordering recorded in the DAWG of each partition,
    // add an edge from kernel node u to v if the index of u is less than the index
    // of v in a particular ordering. This may form cliques within the partition
    // subgraph in G if u and v are task parallel and both orderings are equally
    // optimal.

    BEGIN_SCOPED_REGION

    std::vector<Vertex> subgraph;

    for (unsigned i = 0; i < activePartitions; ++i) {

        PartitionData & D = P[i];

        const OrderingDAWG & O = D.Orderings;
        assert (num_vertices(O) > 0);

        assert (subgraph.empty());

        const auto n = D.Kernels.size();

        const Vertex partitionIn = (i * 2) + 1;
        const Vertex partitionOut = (i * 2) + 2;

        assert (num_edges(O) >= (n - 1));

        subgraph.resize(n);
        for (unsigned i = 0; i < n; ++i) {
            const auto f = kernels.find(D.Kernels[i]);
            unsigned k = 0;
            if (LLVM_UNLIKELY(f != kernels.end())) {
                k = std::distance(kernels.begin(), f) + firstKernelNode;
                assert (k < num_vertices(G));
                assert (degree(k, G) == 0);
            }
            subgraph[i] = k;
        }

        std::function<void(unsigned, unsigned)> link_subgraph = [&](const unsigned u, const unsigned i) {

            if (LLVM_UNLIKELY(out_degree(i, O) == 0)) {
                add_edge(u, partitionOut, G);
            } else {
                for (const auto e : make_iterator_range(out_edges(i, O))) {
                    const auto t = O[e];
                    assert (t < n);
                    auto w = u;
                    if (subgraph[t]) {
                        w = subgraph[t];
                        add_edge(u, w, G);
                    }
                    link_subgraph(w, target(e, O));
                }
            }
        };

        link_subgraph(partitionIn, 0);

        subgraph.clear();

    }

    END_SCOPED_REGION

    assert (out_degree(firstKernelNode - 1, G) == 0);

    // path compress G to speed up the subsequent phases
    for (unsigned i = 0; i < numOfFrontierKernels; ++i) {
        auto & A = G[firstKernelNode + i];
        assert (A.empty());
        A.push_back(i);
    }

    const auto n = firstKernelNode + numOfFrontierKernels;
    for (;;) {
        bool unchanged = true;
        // Even though G is likely cyclic, it was constructed from an acyclic
        // graph whose vertices were indexed in topological order. Traversing
        // from the last to first tends to reach the fixpoint faster.
        for (unsigned i = n; --i; ) {
try_to_compress_further:
            if (out_degree(i, G) == 1) {
                const auto j = child(i, G);
                if (in_degree(j, G) == 1) {
                    auto & A = G[i];
                    const auto & B = G[j];
                    A.insert(A.end(), B.begin(), B.end());
                    for (const auto e : make_iterator_range(out_edges(j, G))) {
                        add_edge(i, target(e, G), G);
                    }
                    clear_vertex(j, G);
                    G[j].clear();
                    unchanged = false;
                    goto try_to_compress_further;
                }
            }
        }
        if (unchanged) break;
    }

    return PartitionOrdering{std::move(G), std::move(kernels)};
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief scheduleProgramGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::scheduleProgramGraph(
        const std::vector<PartitionData> & P,
        PartitionOrdering & partitionOrdering,
        const PartitionDataflowGraph & D) const {

    auto & O = partitionOrdering.Graph;
    auto & kernels = partitionOrdering.Kernels;

    const auto activePartitions = (PartitionCount - 1);
    const auto numOfFrontierKernels = kernels.size();
    assert (num_vertices(D) >= activePartitions);

    const auto lastStreamSet = num_vertices(D);
    const auto firstStreamSet = activePartitions;

    const auto numOfStreamSets = lastStreamSet - firstStreamSet;

    // Our ordering graph will be used to generate and validate our legal kernel invocation orderings
    // but we still need to create a DAG to indicate the relationships between our kernels and
    // streamset nodes to evaluate with the GA.

#ifdef USE_COMPRESSED_PATH_SCHEDULING_GRAPH

    // Prior to starting the GA, we need to construct a scheduling graph. Since we did path compression,
    // we want to take advantage of the fact some of the kernels and streamsets may have forced
    // relationships that we can amalgamate to simplify the resulting graph. By removing kernels due
    // to path compression, we avoid many trivially impossible hamiltonian paths, reducing work in the
    // repair algorithm. By removing streamsets, we eliminate work in the fitness function.


    unsigned kernelSets = 0;

    std::vector<unsigned> indexOfKernelSet(numOfFrontierKernels);

    std::vector<unsigned> kernelSet(numOfFrontierKernels);

    // Identify which kernels belong to the same compressed path set.
    const auto n = num_vertices(O);
    for (unsigned i = 0; i < n; ++i) {
        const auto & K = O[i];
        if (K.empty()) continue;
        indexOfKernelSet[kernelSets] = i;
        for (const auto k : K) {
            kernelSet[k] = kernelSets;
        }
        ++kernelSets;
    }

    auto localIdOf = [&](const unsigned kernelId) {
        const auto f = std::lower_bound(kernels.begin(), kernels.end(), kernelId);
        assert (f != kernels.end() && *f == kernelId);
        return kernelSet[std::distance(kernels.begin(), f)];
    };

    // Identify which streamsets are not fully consumed within a compressed path node.
    std::vector<Vertex> streamSetProducer(numOfStreamSets);
    BitVector hasExternalConsumer(numOfStreamSets, false);

    for (unsigned currentPartition = 0; currentPartition < activePartitions; ++currentPartition) {
        for (const auto e : make_iterator_range(in_edges(currentPartition, D))) {
            const PartitionDataflowEdge & C = D[e];
            assert (Relationships[C.KernelId].Type == RelationshipNode::IsKernel);
            const auto consumer = localIdOf(C.KernelId);
            const auto v = source(e, D);
            assert (v >= activePartitions);
            const auto streamSet = v - activePartitions;
            if (streamSetProducer[streamSet] != consumer) {
                hasExternalConsumer.set(streamSet);
            }
        }
        for (const auto e : make_iterator_range(out_edges(currentPartition, D))) {
            const PartitionDataflowEdge & De = D[e];
            assert (Relationships[De.KernelId].Type == RelationshipNode::IsKernel);
            const auto producer = localIdOf(De.KernelId);
            const auto v = target(e, D);
            assert (v >= activePartitions);
            const auto streamSet = v - activePartitions;
            streamSetProducer[streamSet] = producer;
        }
    }

    // And which streamsets are purely used along the path.

    BitVector hasInternalStreamSets(kernelSets, false);

    for (unsigned currentPartition = 0; currentPartition < activePartitions; ++currentPartition) {
        for (const auto e : make_iterator_range(in_edges(currentPartition, D))) {
            const PartitionDataflowEdge & C = D[e];
            assert (Relationships[C.KernelId].Type == RelationshipNode::IsKernel);
            const auto consumer = localIdOf(C.KernelId);
            const auto v = source(e, D);
            assert (v >= activePartitions);
            const auto streamSet = v - activePartitions;
            if (hasExternalConsumer.test(streamSet)) {
                continue;
            }
            if (LLVM_LIKELY(streamSetProducer[streamSet] == consumer)) {
                hasInternalStreamSets.set(consumer);
            }
        }
    }

    const auto externalStreamSetNodes = hasExternalConsumer.count();
    const auto streamSetNodes = externalStreamSetNodes + hasInternalStreamSets.count();
    const auto m = kernelSets + streamSetNodes;
    SchedulingGraph S(m);

    for (unsigned i = 0; i < kernelSets; ++i) {
        SchedulingNode & N = S[i];
        N.Type = SchedulingNode::IsKernel;
    }

    BEGIN_SCOPED_REGION

    auto internalStreamSet = (kernelSets + externalStreamSetNodes);

    BitVector includeStreamSet(numOfStreamSets);

    for (unsigned currentKernelSet = 0; currentKernelSet < kernelSets; ++currentKernelSet) {
        // Each path compressed kernel node with an internal streamset will be
        // one-to-one associated with a stream set node. To determine the weight
        // of this amalgamated streamset, we perform the partition logic on it.
        if (hasInternalStreamSets[currentKernelSet]) {

            const auto & path = O[indexOfKernelSet[currentKernelSet]];
            const auto numOfKernels = path.size();

            flat_set<Vertex> kernels;
            kernels.insert_unique(path.begin(), path.end());

            includeStreamSet.reset();

            for (unsigned j = 0; j < numOfStreamSets; ++j) {
                if (hasExternalConsumer.test(j)) {
                    continue;
                }
                if (kernels.count(streamSetProducer[j])) {
                    includeStreamSet.set(j);
                }
            }

            const auto numOfStreamSets = includeStreamSet.count();

            SchedulingGraph G(numOfKernels + numOfStreamSets);

            for (unsigned j = 0; j < numOfKernels; ++j) {
                SchedulingNode & N = G[j];
                N.Type = SchedulingNode::IsKernel;
            }

            auto streamSet = includeStreamSet.find_first();

            for (unsigned j = 0; j < numOfStreamSets; ++j) {

                const auto output = in_edge(firstStreamSet + streamSet, D);
                const PartitionDataflowEdge & R = D[output];

                const auto p = kernels.find(R.KernelId);
                assert (p != kernels.end());
                const auto producer = std::distance(kernels.begin(), p);

                const auto partition = source(output, D);

                SchedulingNode & node = S[j];
                node.Type = SchedulingNode::IsStreamSet;
                node.Size = P[partition].ExpectedRepetitions * R.Expected * D[j];

                add_edge(producer, j, G);

                for (const auto input : make_iterator_range(out_edges(firstStreamSet + streamSet, D))) {
                    const PartitionDataflowEdge & R = D[input];
                    const auto c = kernels.find(R.KernelId);
                    assert (c != kernels.end());
                    const auto consumer = std::distance(kernels.begin(), c);
                    add_edge(j, consumer, G);
                }

                streamSet = includeStreamSet.find_next(streamSet);
            }

            MemoryAnalysis MA(G, numOfKernels, numOfStreamSets);

            const auto maxWeight = MA.analyze(path);

            SchedulingNode & N = S[internalStreamSet];
            N.Type = SchedulingNode::IsStreamSet;
            N.Size = Rational{maxWeight};

            add_edge(currentKernelSet, internalStreamSet, S);
            add_edge(internalStreamSet, currentKernelSet, S);

            ++internalStreamSet;
        }
    }

    assert (internalStreamSet == m);

    END_SCOPED_REGION

    for (unsigned currentPartition = 0; currentPartition < activePartitions; ++currentPartition) {

        const PartitionData & Pi = P[currentPartition];

        for (const auto e : make_iterator_range(in_edges(currentPartition, D))) {
            const PartitionDataflowEdge & C = D[e];
            assert (Relationships[C.KernelId].Type == RelationshipNode::IsKernel);
            const auto consumer = localIdOf(C.KernelId);
            const auto v = source(e, D);
            assert (v >= activePartitions);
            const auto streamSet = v + activePartitions;
            if (hasExternalConsumer.test(streamSet)) {
                add_edge(streamSet + kernelSets, consumer, S);
            }
        }

        for (const auto e : make_iterator_range(out_edges(currentPartition, D))) {
            const PartitionDataflowEdge & De = D[e];
            assert (Relationships[De.KernelId].Type == RelationshipNode::IsKernel);
            const auto producer = localIdOf(De.KernelId);
            const auto v = target(e, D);
            assert (v >= activePartitions);
            const auto streamSet = v + activePartitions;
            if (hasExternalConsumer.test(streamSet)) {
                add_edge(producer, streamSet + kernelSets, S);
                SchedulingNode & node = S[streamSet];
                node.Type = SchedulingNode::IsStreamSet;
                node.Size = Pi.ExpectedRepetitions * De.Expected * D[v]; // bytes per segment
                assert (node.Size.denominator() == 1);
            }
        }
    }

    ProgramSchedulingAnalysis SA(S, O, activePartitions, kernelSets, streamSetNodes);

#else

    SchedulingGraph S(numOfFrontierKernels + numOfStreamSets);

    const Rational ONE{1};

    for (unsigned i = 0; i < numOfFrontierKernels; ++i) {
        SchedulingNode & N = S[i];
        N.Type = SchedulingNode::IsKernel;
        N.Size = ONE;
    }

    auto localIdOf = [&kernels](const unsigned kernelId) {
        const auto f = std::lower_bound(kernels.begin(), kernels.end(), kernelId);
        assert (f != kernels.end() && *f == kernelId);
        return std::distance(kernels.begin(), f);
    };

    const auto firstStreamSetNode = numOfFrontierKernels - activePartitions;

    for (unsigned currentPartition = 0; currentPartition < activePartitions; ++currentPartition) {

        const PartitionData & Pi = P[currentPartition];

        for (const auto e : make_iterator_range(in_edges(currentPartition, D))) {
            const PartitionDataflowEdge & C = D[e];
            assert (Relationships[C.KernelId].Type == RelationshipNode::IsKernel);
            const auto consumer = localIdOf(C.KernelId);
            const auto v = source(e, D);
            assert (v >= activePartitions);
            const auto streamSet = v + firstStreamSetNode;
            assert (streamSet >= numOfFrontierKernels && streamSet < num_vertices(S));
            add_edge(streamSet, consumer, C.Expected, S);
        }

        for (const auto e : make_iterator_range(out_edges(currentPartition, D))) {
            const PartitionDataflowEdge & De = D[e];
            assert (Relationships[De.KernelId].Type == RelationshipNode::IsKernel);
            const auto producer = localIdOf(De.KernelId);
            const auto v = target(e, D);
            assert (v >= activePartitions);
            const auto streamSet = v + firstStreamSetNode;
            assert (streamSet >= numOfFrontierKernels && streamSet < num_vertices(S));
            add_edge(producer, streamSet, De.Expected, S);
            SchedulingNode & node = S[streamSet];
            node.Type = SchedulingNode::IsStreamSet;
            node.Size = Pi.ExpectedRepetitions * De.Expected * D[v]; // bytes per segment
            assert (node.Size.denominator() == 1);
        }
    }

    ProgramSchedulingAnalysis SA(S, O, activePartitions, numOfFrontierKernels, numOfStreamSets);

#endif

    SA.runGA(partitionOrdering.Ordering);

    errs () << "init_time: " << init_time << "\n"
               "fitness_time: " << fitness_time << "\n"
               "repair_time: " << repair_time << "\n"
               "evolutionary_time: " << evolutionary_time << "\n";

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief assembleProgramSchedule
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::assembleProgramSchedule(const std::vector<PartitionData> & P,
                                               const PartitionOrdering & interPartition) {

    unsigned position = 0;

    std::vector<unsigned> program;
    const auto n = PartitionIds.size();
    program.reserve(n);

    const auto & schedule = interPartition.Ordering;
    const auto & kernels = interPartition.Kernels;

    std::vector<unsigned> path;

    while (out_degree(position, schedule) > 0) {

        assert (path.empty());
        // underflow sentinal node
        path.push_back(-1U);

        unsigned currentPartitionId = 0;

        while (out_degree(position, schedule) > 0) {
            const auto e = first_out_edge(position, schedule);
            const auto local = schedule[e];
            assert (local < kernels.size());
            const auto node = kernels[local];

            const auto f = PartitionIds.find(node);
            assert (f != PartitionIds.end());
            const auto pid = f->second - 1;

            if (path.size() == 1) {
                currentPartitionId = pid;
            } else if (currentPartitionId != pid) {
                break;
            }
            path.push_back(node);

            const auto next = target(e, schedule);
            assert (position != next);
            position = next;
        }
        // overflow sentinal node
        path.push_back(-1U);

        assert (path.size() > 2);

        const PartitionData & partition = P[currentPartitionId];

        const auto & G = partition.Orderings;
        const auto & K = partition.Kernels;

        const auto numOfKernels = K.size();
        assert (path.size() <= numOfKernels + 2);

        #ifndef NDEBUG
        const auto start = program.size();
        #endif

        if (LLVM_LIKELY(num_vertices(G) > 1)) {

            unsigned offset = 1;

            std::function<bool(unsigned)> select_path = [&](const unsigned u) {

                if (LLVM_UNLIKELY(out_degree(u, G) == 0)) {
                    // when we find an ordering of the kernels within this
                    // partition that matches the desired global ordering,
                    // exit the function.
                    return (offset == (path.size() - 1));
                } else {
                    for (const auto e : make_iterator_range(out_edges(u, G))) {
                        const auto t = G[e];
                        assert (t < K.size());
                        const auto k = K[t];
                        assert (offset < path.size());
                        if (path[offset] == k) {
                            ++offset;
                        }
                        program.push_back(k);
                        const auto v = target(e, G);
                        if (select_path(v)) {
                            return true;
                        }
                        assert (program.back() == k);

                        program.pop_back();
                        if (path[offset - 1] == k) {
                            --offset;
                        }
                    }
                    return false;
                }
            };

            const auto found = select_path(0);
            assert (found);

        } else { // any ordering will suffice that matches the path given
            assert (path.front() == -1U && path.back() == -1U);
            if (LLVM_UNLIKELY(path.size() < (numOfKernels + 2))) {
                path.pop_back();
                path.reserve(numOfKernels + 1);
                const auto begin = path.begin() + 1;
                const auto end = path.begin() + path.size();
                for (unsigned i = 0; i < numOfKernels; ++i) {
                    if (std::find(begin, end, K[i]) == end) {
                        path.push_back(K[i]);
                    }
                }
            }
            assert (path.size() >= (numOfKernels + 1));
            program.insert(program.end(), path.begin() + 1, path.begin() + (numOfKernels + 1));
        }

        #ifndef NDEBUG
        const auto end = program.size();
        flat_set<unsigned> T(K.begin(), K.end());
        for (auto i = start; i < end; ++i) {
            const auto f = T.find(program[i]);
            assert (f != T.end());
            T.erase(f);
        }
        assert (T.empty());
        #endif

        path.clear();
    }

    #ifndef NDEBUG
    flat_set<unsigned> T;
    T.reserve(n);
    for (const auto & Pi : P) {
        for (const auto k : Pi.Kernels) {
            assert (k < num_vertices(Relationships));
            const RelationshipNode & K = Relationships[k];
            assert (K.Type == RelationshipNode::IsKernel);
            T.insert(k);
        }
    }
    for (const auto p : program) {
        const auto f = T.find(p);
        assert (f != T.end());
        T.erase(f);
    }
    assert (T.empty());
    #endif

    auto u = PipelineInput;
    for (const auto v : program) {
        add_edge(u, v, RelationshipType{ReasonType::OrderingConstraint}, Relationships);
        u = v;
    }
    add_edge(u, PipelineOutput, RelationshipType{ReasonType::OrderingConstraint}, Relationships);

    #ifndef NDEBUG
    std::vector<Vertex> ordering;
    ordering.reserve(num_vertices(Relationships));
    topological_sort(Relationships, std::back_inserter(ordering));
    #endif


}

#endif

}

#endif // SCHEDULING_ANALYSIS_HPP
