#ifndef BUFFER_SIZE_ANALYSIS_HPP
#define BUFFER_SIZE_ANALYSIS_HPP

#include "pipeline_analysis.hpp"
#include "evolutionary_algorithm.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determineBufferSize
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::determineBufferSize(BuilderRef b) {

    const auto blockWidth = b->getBitBlockWidth();

    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {

        BufferNode & bn = mBufferGraph[streamSet];
        const auto producerOutput = in_edge(streamSet, mBufferGraph);
        const BufferPort & producerRate = mBufferGraph[producerOutput];

        if (LLVM_LIKELY(bn.Buffer == nullptr)) { // is internal buffer

            // TODO: If we have an open system, then the input rate to this pipeline cannot
            // be bounded a priori. During initialization, we could pass a "suggestion"
            // argument to indicate what the outer pipeline believes its I/O rates will be.


            // If this buffer is externally used, we cannot analyze the dataflow rate of
            // external consumers. Default to dynamic for such buffers.

            // Similarly if any internal consumer has a deferred rate, we cannot analyze
            // any consumption rates.

            auto maxDelay = producerRate.Delay;
            auto maxLookAhead = producerRate.LookAhead;
            auto maxLookBehind = producerRate.LookBehind;

            const auto producer = source(producerOutput, mBufferGraph);

            auto bMin = floor(producerRate.Minimum * MinimumNumOfStrides[producer]);
            auto bMax = ceiling(producerRate.Maximum * MaximumNumOfStrides[producer]);


            for (const auto e : make_iterator_range(out_edges(streamSet, mBufferGraph))) {

                const BufferPort & consumerRate = mBufferGraph[e];

                const auto consumer = target(e, mBufferGraph);

                const auto cMin = floor(consumerRate.Minimum * MinimumNumOfStrides[consumer]);
                const auto cMax = ceiling(consumerRate.Maximum * MaximumNumOfStrides[consumer]);

                assert (cMax >= cMin);

                assert (consumerRate.Maximum >= consumerRate.Minimum);

                bMin = std::min(bMin, cMin);
                bMax = std::max(bMax, cMax);

//                // Get output overflow size
                auto lookAhead = consumerRate.LookAhead;
                if (consumerRate.Minimum < consumerRate.Maximum) {
                    lookAhead += ceiling(consumerRate.Maximum - consumerRate.Minimum);
                }

                maxDelay = std::max(maxDelay, consumerRate.Delay);
                maxLookAhead = std::max(maxLookAhead, lookAhead);
                maxLookBehind = std::max(maxLookBehind, consumerRate.LookBehind);
            }

            // calculate overflow (copyback) and fascimile (copyforward) space
            bn.LookAhead = maxLookAhead;
            bn.LookBehind = maxLookBehind;

            const auto overflow0 = std::max(bn.MaxAdd, bn.LookAhead);
            const auto overflow1 = std::max(overflow0, bMax);
            const auto overflowSize = round_up_to(overflow1, blockWidth) / blockWidth;

            const auto underflow0 = std::max(bn.LookBehind, maxDelay);
            const auto underflowSize = round_up_to(underflow0, blockWidth) / blockWidth;
            const auto required = (bMax * 2) - bMin;

            const auto reqSize1 = round_up_to(required, blockWidth) / blockWidth;
            const auto reqSize2 = 2 * (overflowSize + underflowSize);
            auto requiredSize = std::max(reqSize1, reqSize2);

            bn.OverflowCapacity = overflowSize;
            bn.UnderflowCapacity = underflowSize;
            bn.RequiredCapacity = requiredSize;

        }
    }



}

#ifdef PERMIT_BUFFER_MEMORY_REUSE

// TODO: nested pipeline kernels could report how much internal memory they require
// and reason about that here (and in the scheduling phase)

namespace { // anonymous namespace

constexpr static unsigned BUFFER_LAYOUT_INITIAL_CANDIDATES = 20;

constexpr static unsigned BUFFER_LAYOUT_INITIAL_CANDIDATE_ATTEMPTS = 100;

constexpr static unsigned BUFFER_SIZE_POPULATION_SIZE = 30;

constexpr static unsigned BUFFER_SIZE_GA_ROUNDS = 30;

constexpr static unsigned BUFFER_SIZE_GA_STALLS = 10;


struct Pack {
    size_t   S = 0;
    unsigned A = 0;
    unsigned D = 0;

    Pack() = default;
    Pack(size_t s, unsigned a, unsigned d) : S(s), A(a), D(d) { }
};

using IntervalGraph = adjacency_list<hash_setS, vecS, undirectedS>;

using Interval = std::pair<unsigned, unsigned>;

static size_t bs_init_time = 0;
static size_t bs_fitness_time = 0;
static size_t bs_fitness_calls = 0;

struct BufferLayoutOptimizer final : public PermutationBasedEvolutionaryAlgorithm{

    using ColourLine = flat_set<Interval>;

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief initGA
     ** ------------------------------------------------------------------------------------------------------------- */
    bool initGA(Population & initialPopulation) override {

        assert (candidates.empty());
        assert (initialPopulation.empty());

        const auto t0 = std::chrono::high_resolution_clock::now();

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

            if (insertCandidate(candidate, initialPopulation)) {
                if (initialPopulation.size() >= BUFFER_LAYOUT_INITIAL_CANDIDATES) {
                    return false;
                }
            }

            H.clear();
            candidate.clear();
        }

        const auto t1 = std::chrono::high_resolution_clock::now();

        bs_init_time += (t1 - t0).count();

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

        const auto t0 = std::chrono::high_resolution_clock::now();

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

        const auto t1 = std::chrono::high_resolution_clock::now();

        bs_fitness_time += (t1 - t0).count();
        ++bs_fitness_calls;

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
    : PermutationBasedEvolutionaryAlgorithm (numOfLocalStreamSets, BUFFER_SIZE_GA_ROUNDS, BUFFER_SIZE_GA_STALLS, BUFFER_SIZE_POPULATION_SIZE, rng)
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

    const auto firstKernel = out_degree(PipelineInput, mBufferGraph) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, mBufferGraph) == 0 ? LastKernel : PipelineOutput;

    #warning TODO: can we insert a zero-extension region rather than having a secondary buffer?

    IntervalGraph I(n);
    std::vector<size_t> weight(n, 0);
    std::vector<int> remaining(n, 0); // NOTE: signed int type is necessary here
    std::vector<Pack> allocations(n);
    std::vector<unsigned> mapping(n, -1U);


    unsigned numOfLocalStreamSets = 0;

    BEGIN_SCOPED_REGION

    // The buffer graph is constructed in order of how the compiler will structure the pipeline program
    // (i.e., the invocation order of its kernels.) Construct our allocation "rectangles" based on its
    // ordering.

    DataLayout DL(b->getModule());

    const auto alignment = b->getCacheAlignment();

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const auto streamSet = target(output, mBufferGraph);
            const BufferNode & bn = mBufferGraph[streamSet];

            if (bn.Locality == BufferLocality::ThreadLocal) {
                // determine the number of bytes this streamset requires
                const BufferPort & producerRate = mBufferGraph[output];
                const Binding & outputRate = producerRate.Binding;


                Type * const type = StreamSetBuffer::resolveType(b, outputRate.getType());
                #if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(11, 0, 0)
                const auto typeSize = DL.getTypeAllocSize(type);
                #else
                const auto typeSize = DL.getTypeAllocSize(type).getFixedSize();
                #endif
                assert (typeSize > 0);

                assert (bn.UnderflowCapacity == 0);

                const auto c = bn.RequiredCapacity + bn.OverflowCapacity;
                assert (c > 0);
                const auto w = round_up_to(c * typeSize, alignment);
                assert (w > 0);

                const auto i = streamSet - FirstStreamSet;
                assert (i < n);

                const auto j = numOfLocalStreamSets++;

                mapping[i] = j;

                Pack & P = allocations[j];
                assert (P.A == 0);
                P.A = kernel;
                P.D = kernel;
                P.S = w;
                weight[j] = w;

                // record how many consumers exist before the streamset memory can be reused
                // (NOTE: the +1 is to indicate this kernel requires each output streamset
                // to be distinct even if one or more of the outputs is not used later.)
                remaining[j] = out_degree(streamSet, mBufferGraph) + 1U;
            }
        }

        // Mark any overlapping allocations in our interval graph.

        // NOTE: although Gergov's constructs an interval graph based on the mapping
        // of the rectangle to starting offset discovered when performing the packing
        // process, we do not have that same information to guide us after the initial
        // candidates have been generated. Instead we use a general interval graph
        // within the first-fit process but guide colouring based on candidate order.

        for (unsigned i = 0; i < numOfLocalStreamSets; ++i) {
            if (remaining[i] > 0) {
                for (unsigned j = 0; j < i; ++j) {
                    if (remaining[j] > 0) {
                        add_edge(i, j, I);
                    }
                }
            }
        }

        auto markFinishedStreamSets = [&](const unsigned streamSet) {

            const auto i = streamSet - FirstStreamSet;
            assert (FirstStreamSet <= streamSet && streamSet <= LastStreamSet);
            assert (i < n);
            const auto j = mapping[i];

            if (j != -1U) {
                assert (j < numOfLocalStreamSets);
                auto & r = remaining[j];
                if ((--r) == 0) {
                    Pack & P = allocations[j];
                    assert (P.A == P.D);
                    assert (firstKernel <= P.A && P.A <= kernel);
                    P.D = kernel;
                }
            }

        };

        // Determine which streamsets are no longer alive
        for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            markFinishedStreamSets(target(output, mBufferGraph));
        }
        for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            markFinishedStreamSets(source(input, mBufferGraph));
        }
    }

    END_SCOPED_REGION

//   auto & out = errs();

//    out << "graph \"I\" {\n";

//    for (unsigned i = 0; i < n; ++i) {
//        const auto j = mapping[i];
//        if (j != -1U) {
//            out << "v" << j << " [label=\"" << FirstStreamSet + i << "\"];\n";
//        }
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

    for (unsigned i = 0; i < n; ++i) {
        const auto j = mapping[i];
        if (j != -1U) {
            BufferNode & bn = mBufferGraph[FirstStreamSet + i];
            const auto & interval = intervals[j];
            bn.BufferStart = interval.first;
        }
    }

//    out << "graph \"I2\" {\n";

//    for (unsigned i = 0; i < n; ++i) {
//        const auto j = mapping[i];
//        if (j != -1U) {

//            const BufferNode & bn = mBufferGraph[FirstStreamSet + i];


//            out << "v" << j << " [label=\"" << FirstStreamSet + i  << " : " << bn.BufferStart << "\"];\n";
//        }
//    }

//    for (auto e : make_iterator_range(edges(I))) {
//        const auto s = source(e, I);
//        const auto t = target(e, I);
//        out << "v" << s << " -- v" << t << ";\n";
//    }

//    out << "}\n\n";
//    out.flush();

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
