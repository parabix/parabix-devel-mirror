#ifndef BUFFER_SIZE_ANALYSIS_HPP
#define BUFFER_SIZE_ANALYSIS_HPP

#include "pipeline_analysis.hpp"
#include "evolutionary_algorithm.hpp"

namespace kernel {

#ifdef PERMIT_BUFFER_MEMORY_REUSE

// TODO: nested pipeline kernels could report how much internal memory they require
// and reason about that here (and in the scheduling phase)

#define BUFFER_LAYOUT_INITIAL_CANDIDATES (20)
#define BUFFER_LAYOUT_INITIAL_CANDIDATE_ATTEMPTS (100)

#define BUFFER_LAYOUT_MAX_POPULATION_SIZE (30)

#define BUFFER_LAYOUT_MAX_EVOLUTIONARY_ROUNDS (30)

#define BUFFER_LAYOUT_MUTATION_RATE (0.20)

namespace { // anonymous namespace

struct Pack {
    size_t   S = 0;
    unsigned A = 0;
    unsigned D = 0;

    Pack() = default;
    Pack(size_t s, unsigned a, unsigned d) : S(s), A(a), D(d) { }
};

using IntervalGraph = adjacency_list<hash_setS, vecS, undirectedS>;

using Interval = std::pair<unsigned, unsigned>;

struct BufferLayoutOptimizer final : public OrderingBasedEvolutionaryAlgorithm{

    using ColourLine = flat_set<Interval>;

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief initGA
     ** ------------------------------------------------------------------------------------------------------------- */
    bool initGA(Population & initialPopulation) override {

        assert (candidates.empty());
        assert (initialPopulation.empty());

        Candidate candidate;
        candidate.reserve(candidateLength);

        std::vector<Pack> H;
        H.reserve(candidateLength);

        std::vector<unsigned> J;
        J.reserve(candidateLength);

        struct PackWeightComparator {
            bool operator()(const Pack & a,const Pack & b) const{
                return a.S > b.S;
            }
        };

        auto intersects = [](const Pack & A, const Pack & B) -> bool {
            const auto r = A.A <= B.D && B.A <= A.D;
            assert (r == (std::max(A.A, B.A) <= std::min(A.D, B.D)));
            return r;
        };

        for (unsigned r = 0; r < BUFFER_LAYOUT_INITIAL_CANDIDATE_ATTEMPTS; ++r) {
            assert (H.empty());

            H.emplace_back(0, firstKernel, lastKernel);

            J.resize(candidateLength);

            // by shuffling the order of J, we introduce some randomness to
            // the otherwise deterministic algorithm.
            std::iota(J.begin(), J.end(), 0);
            std::shuffle(J.begin(), J.end(), rng);

            assert (candidate.empty());

            for (;;) {

                assert (!H.empty());
                // pick a h=(w,l,r) from H s.t. w is minimal and remove h from H
                const auto h = H.front();
                std::pop_heap(H.begin(), H.end(), PackWeightComparator{});
                H.pop_back();
                assert (H.front().S >= h.S);

                // if there exists a (s,a,d) in J s.t. (a,d) ∩ (l,r) ≠ ∅ and
                // for all g=(q,x,y) in H, (a,d) ∩ (x,y) = ∅, do ...

                for (auto j = J.begin(); j != J.end(); ) {
                    const auto & a = allocations[*j];

                    assert (a.S > 0);

                    if (intersects(a, h)) {

                        for (const auto & g : H) {
                            if (intersects(a, g)) {
                                goto more_than_one_intersection;
                            }
                        }

                        // record j as in our candidate list and remove it from J
                        candidate.push_back(*j);
                        J.erase(j);

                        // update the idealized memory layout to accommodate the
                        // chosen allocation.
                        const auto l = std::max(a.A, h.A);
                        const auto r = std::min(a.D, h.D);

                        H.emplace_back(h.S + a.S, l, r);
                        std::push_heap(H.begin(), H.end(), PackWeightComparator{});

                        if (h.A < a.A) {
                            H.emplace_back(h.S, h.A, a.A);
                            std::push_heap(H.begin(), H.end(), PackWeightComparator{});
                        }

                        if (a.D < h.D) {
                            H.emplace_back(h.S, a.D, h.D);
                            std::push_heap(H.begin(), H.end(), PackWeightComparator{});
                        }
                        break;
                    }
    more_than_one_intersection:
                    ++j;
                }

                if (J.empty()) {
                    break;
                }
            }

            // given our candidate, do a first-fit allocation to determine the actual
            // memory layout.

            FitnessValueType fitness;
            if (insertCandidate(candidate, initialPopulation, fitness)) {
                if (initialPopulation.size() >= BUFFER_LAYOUT_INITIAL_CANDIDATES) {
                    return false;
                }
            }

            H.clear();
            candidate.clear();
        }

        return true;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief repair
     ** ------------------------------------------------------------------------------------------------------------- */
    void repairCandidate(Candidate & /* candidate */) override { };

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief fitness
     ** ------------------------------------------------------------------------------------------------------------- */
    size_t fitness(const Candidate & candidate) override {

        assert (candidate.size() == candidateLength);

        std::fill_n(remaining.begin(), candidateLength, 0);

        GC_CL.clear();

        for (unsigned i = 0; i < candidateLength; ++i) {
            GC_ordering[candidate[i]] = i;
        }

        size_t max_colours = 0;
        for (unsigned i = 0; i < candidateLength; ++i) {
            const auto u = candidate[i];
            assert (u < candidateLength);
            const auto w = weight[u];

            if (w == 0) {
                remaining[u] = -1;
                assert (degree(u, I) == 0);
            } else {
                remaining[u] = out_degree(u, I);
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

                GC_Intervals[u] = std::make_pair(first, last);

                GC_CL.emplace(first, last);

                for (const auto e : make_iterator_range(out_edges(u, I))) {
                    const auto j = target(e, I);
                    if (GC_ordering[j] < i) {
                        assert (remaining[j] > 0);
                        remaining[j]--;
                    }
                }

                for (unsigned j = 0; j <= i; ++j) {
                    const auto v = candidate[i];
                    assert (v < candidateLength);
                    if (remaining[v] == 0) {
                        const auto f = GC_CL.find(GC_Intervals[v]);
                        assert (f != GC_CL.end());
                        GC_CL.erase(f);
                        remaining[v] = -1;
                    }
                }
            }
        }
        return max_colours;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief getIntervals
     ** ------------------------------------------------------------------------------------------------------------- */
    std::vector<Interval> getIntervals(const OrderingDAWG & O) {
        Candidate chosen;
        chosen.reserve(candidateLength);
        Vertex u = 0;
        while (out_degree(u, O) != 0) {
            const auto e = first_out_edge(u, O);
            const auto k = O[e];
            chosen.push_back(k);
            u = target(e, O);
        }
        fitness(chosen);
        return GC_Intervals;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief constructor
     ** ------------------------------------------------------------------------------------------------------------- */
    BufferLayoutOptimizer(const unsigned numOfLocalStreamSets
                         , const unsigned firstKernel
                         , const unsigned lastKernel
                         , IntervalGraph && I
                         , std::vector<Pack> && allocations
                         , std::vector<size_t> && weight
                         , std::vector<int> && remaining
                         , random_engine & rng)
    : OrderingBasedEvolutionaryAlgorithm (numOfLocalStreamSets, BUFFER_LAYOUT_MAX_POPULATION_SIZE, BUFFER_LAYOUT_MAX_EVOLUTIONARY_ROUNDS, BUFFER_LAYOUT_MUTATION_RATE, rng)
    , firstKernel(firstKernel)
    , lastKernel(lastKernel)
    , I(std::move(I))
    , allocations(std::move(allocations))
    , weight(std::move(weight))
    , remaining(std::move(remaining))
    , GC_Intervals(numOfLocalStreamSets)
    , GC_ordering(numOfLocalStreamSets) {

    }


private:

    const unsigned firstKernel;
    const unsigned lastKernel;

    const IntervalGraph I;

    const std::vector<Pack> allocations;

    const std::vector<size_t> weight;

    std::vector<int> remaining;
    std::vector<Interval> GC_Intervals;
    std::vector<unsigned> GC_ordering;

    ColourLine GC_CL;

};

} // end of anonymous namespace

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determineBufferLayout
 *
 * Given our buffer graph, we want to identify the best placement to maximize memory reuse byway of minimizing
 * the total memory required. Although this assumes that the memory-aware scheduling algorithm was first called,
 * it does not actually use any data from it. The reason for this disconnection is to enable us to explore
 * the impact of static memory allocation independent of the chosen scheduling algorithm.
 *
 * The following is a genetic algorithm that adapts Gergov's incremental 2-approx from "Algorithms for
 * Compile time memory optimization" (1999) to generate some initial layout candidates then attempts
 * to refine them.
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::determineBufferLayout(BuilderRef b, random_engine & rng) {

    // Construct the weighted interval graph for our local streamsets

    const auto n = LastStreamSet - FirstStreamSet + 1U;

    #warning TODO: can we insert a zero-extension region rather than having a secondary buffer?

    IntervalGraph I(n);
    std::vector<size_t> weight(n, 0);
    std::vector<int> remaining(n, 0); // NOTE: signed int type is necessary here
    std::vector<unsigned> mapping(n, -1U);
    std::vector<Pack> allocations(n);

    const auto firstKernel = out_degree(PipelineInput, mBufferGraph) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, mBufferGraph) == 0 ? LastKernel : PipelineOutput;

    unsigned numOfLocalStreamSets = 0;

    BEGIN_SCOPED_REGION

    // The buffer graph is constructed in order of how the compiler will structure the pipeline program
    // (i.e., the invocation order of its kernels.) Construct our allocation "rectangles" based on its
    // ordering.

    DataLayout DL(b->getModule());


    auto alignment = b->getCacheAlignment();


    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        const auto lastStreamSet = numOfLocalStreamSets;

        for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const auto streamSet = target(output, mBufferGraph);
            BufferNode & bn = mBufferGraph[streamSet];

            const auto i = streamSet - FirstStreamSet;
            assert (i < n);

            if (bn.Locality == BufferLocality::ThreadLocal) {
                // determine the number of bytes this streamset requires
                const BufferPort & producerRate = mBufferGraph[output];
                const Binding & outputRate = producerRate.Binding;


                Type * const type = StreamSetBuffer::resolveType(b, outputRate.getType());
                #if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(10, 0, 0)
                const auto typeSize = DL.getTypeAllocSize(type);
                #else
                const auto typeSize = DL.getTypeAllocSize(type).getFixedSize();
                #endif

                assert (bn.UnderflowCapacity == 0);

                const auto c = bn.RequiredCapacity + bn.OverflowCapacity;
                const auto w = round_up_to(c * typeSize, alignment);

                assert (w > 0);

                mapping[i] = numOfLocalStreamSets;

                Pack & P = allocations[numOfLocalStreamSets];
                assert (P.A == 0);
                P.A = kernel;
                P.D = kernel;
                P.S = w;
                weight[numOfLocalStreamSets] = w;

                // record how many consumers exist before the streamset memory can be reused
                remaining[numOfLocalStreamSets] = out_degree(streamSet, mBufferGraph);

                ++numOfLocalStreamSets;
            }
        }

        // Mark any overlapping allocations in our interval graph.

        // NOTE: although Gergov's constructs an interval graph based on the mapping
        // of the rectangle to starting offset discovered when performing the packing
        // process, we do not have that same information to guide us after the initial
        // candidates have been generated. Instead we use a general interval graph
        // within the first-fit process but guide colouring based on candidate order.

        for (unsigned i = lastStreamSet; i < numOfLocalStreamSets; ++i) {
            // NOTE: remaining may be less than 0
            if (remaining[i] > 0) {
                for (unsigned j = 0; j < i; ++j) {
                    if (remaining[j] > 0) {
                        add_edge(i, j, I);
                    }
                }
            }
        }

        // Determine which streamsets are no longer alive
        for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            const auto streamSet = source(input, mBufferGraph);
            // NOTE: remaining may go below 0 for non local streamsets
            const auto i = streamSet - FirstStreamSet;
            assert (i < n);
            const auto j = mapping[i];
            if (j != -1U) {
                auto & r = remaining[j];
                if ((--r) == 0) {
                    Pack & P = allocations[j];
                    assert (P.A == P.D);
                    assert (firstKernel <= P.A && P.A <= kernel);
                    P.D = kernel;
                    r = -1;
                }
            }
        }
    }

    END_SCOPED_REGION

//    auto & out = errs();

//    out << "graph \"" << "I" << "\" {\n";
//    for (auto v : make_iterator_range(vertices(I))) {

//        const auto & a = allocations[v];

//        out << "v" << v << " [label=\"" << v << " [" << a.A << ',' << a.D << "] {" << a.S << "}\"];\n";
//    }
//    for (auto e : make_iterator_range(edges(I))) {
//        const auto s = source(e, I);
//        const auto t = target(e, I);
//        out << "v" << s << " -- v" << t << ";\n";
//    }

//    out << "}\n\n";
//    out.flush();


    BufferLayoutOptimizer BA(numOfLocalStreamSets, firstKernel, lastKernel,
                             std::move(I), std::move(allocations), std::move(weight), std::move(remaining), rng);

    BA.runGA();

    RequiredThreadLocalStreamSetMemory = BA.getBestFitnessValue();

    auto O = BA.getResult();

    // TODO: apart from total memory, when would one layout be better than another?
    // Can we quantify it based on the buffer graph order? Currently, we just take
    // the first one.

    const auto intervals = BA.getIntervals(O);

    unsigned l = 0;

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {
        for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const auto streamSet = target(output, mBufferGraph);
            BufferNode & bn = mBufferGraph[streamSet];
            if (bn.Locality == BufferLocality::ThreadLocal) {
                assert ("inconsistent graph traversal?" && (mapping[streamSet - FirstStreamSet] == l));
                const auto & interval = intervals[l];
                bn.BufferStart = interval.first;

                ++l;
            }
        }
    }

    assert (l == numOfLocalStreamSets);

    assert (RequiredThreadLocalStreamSetMemory > 0);

}

#else // #ifndef PERMIT_BUFFER_MEMORY_REUSE

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determineBufferLayout
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::determineBufferLayout(BuilderRef /* b */) {


}

#endif

}

#endif // BUFFER_SIZE_ANALYSIS_HPP
