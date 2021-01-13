#ifndef PIPELINE_COMPILER_ANALYSIS_EVOLUTIONARY_ALGORITHM_HPP
#define PIPELINE_COMPILER_ANALYSIS_EVOLUTIONARY_ALGORITHM_HPP

#include "pipeline_analysis.hpp"

namespace kernel {

using Vertex = unsigned;

using Candidate = std::vector<Vertex>;

using Candidates = std::map<Candidate, size_t>;

using Individual = Candidates::const_iterator;

using Population = std::vector<Individual>;

using random_engine = std::default_random_engine; // TODO: look into xorshift for this

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief EvolutionaryAlgorithm
 *
 * Both the partition scheduling algorithm and whole program scheduling algorithm rely on the following class.
 * Within it is a genetic algorithm designed to find a minimum memory schedule of a given SchedulingGraph.
 * However, the phenotype of of the partition algorithm is a topological ordering and the phenotype of the
 * whole program is a hamiltonian path. This consitutes a significant enough difference that it is difficult
 * to call with only one function. Instead both the "initGA" and "repair" functions are implemented within
 * the actual scheduling functions.
 ** ------------------------------------------------------------------------------------------------------------- */
class EvolutionaryAlgorithm {
protected:

    struct FitnessComparator {
        bool operator()(const Individual & a,const Individual & b) const{
            return a->second < b->second;
        }
    };

public:

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
    size_t runGA(OrderingDAWG & result) {

        Population P1;
        P1.reserve(maxCandidates);

        if (initGA(P1)) {
            goto found_all_orderings;
        }

        assert (candidateLength > 1);

        BEGIN_SCOPED_REGION

        permutation_bitset bitString(candidateLength);

        BitVector V(candidateLength);

        std::uniform_real_distribution<double> zeroToOneReal(0.0, 1.0);

        Population P2;
        P2.reserve(3 * maxCandidates);

        for (unsigned round = 0; round < maxRounds; ++round) {

//            errs() << "EA: round=" << round << " of " << maxRounds << "\n";

            const auto populationSize = P1.size();

            assert (populationSize > 1);

            std::uniform_int_distribution<unsigned> upToN(0, populationSize - 1);

            auto tournament_select = [&]() -> const Candidate & {
                const auto & A = P1[upToN(rng)];
                const auto & B = P1[upToN(rng)];
                if (A->second < B->second) {
                    return A->first;
                } else {
                    return B->first;
                }
            };

//            for (unsigned i = 0; i < populationSize; ++i) {
//                errs() << "  population: ";
//                const auto & I = P1[i];
//                char joiner = '{';
//                for (const auto k : I->first) {
//                    errs() << joiner << k;
//                    joiner = ',';
//                }
//                errs() << "} := " << I->second << "\n";
//            }

            // CROSSOVER:

            assert (P2.empty());

            for (unsigned i = 0; i < populationSize; ++i) {

                const Candidate & A = tournament_select();
                const Candidate & B = tournament_select();

                // generate a random bit string
                bitString.randomize(rng);

                auto crossover = [&](const Candidate & A, const Candidate & B, const bool selector) {

                    Candidate C(candidateLength);

                    V.reset();

                    for (unsigned k = 0; k < candidateLength; ++k) {
                        const auto t = bitString.test(k);
                        if (t == selector) {
                            const auto v = A[k];
                            assert (v < candidateLength);
                            V.set(v);
                        } else {
                            C[k] = A[k];
                        }
                    }

                    for (unsigned k = 0U, p = -1U; k < candidateLength; ++k) {
                        const auto t = bitString.test(k);
                        if (t == selector) {
                            // V contains 1-bits for every entry we did not
                            // directly copy from A into C. We now insert them
                            // into C in the same order as they are in B.
                            for (;;){
                                ++p;
                                assert (p < candidateLength);
                                const auto v = B[p];
                                assert (v < candidateLength);
                                if (V.test(v)) break;
                            }
                            C[k] = B[p];
                        }
                    }

                    repairCandidate(C);
                    insertCandidate(C, P2);

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
                auto & A = P1[j];
                if (zeroToOneReal(rng) <= mutationRate) {

                    const auto a = std::uniform_int_distribution<unsigned>{0, candidateLength - 2}(rng);
                    const auto b = std::uniform_int_distribution<unsigned>{a + 1, candidateLength - 1}(rng);

                    Candidate C{A->first};
                    std::shuffle(C.begin() + a, C.begin() + b, rng);

                    repairCandidate(C);
                    insertCandidate(C, P2);
                }
            }

            // SELECTION:

            for (const auto & I : P2) {
                assert (P1.size() <= maxCandidates);
                if (P1.size() == maxCandidates) {
                    if (I->second <= P1.front()->second) {
                        std::pop_heap(P1.begin(), P1.end(), FitnessComparator{});
                        P1.pop_back();
                    } else {
                        // New item exceeds the weight of the heaviest candiate
                        // in the population.
                        continue;
                    }
                }
                P1.emplace_back(I);
                std::push_heap(P1.begin(), P1.end(), FitnessComparator{});
            }

            P2.clear();
        }

        END_SCOPED_REGION

found_all_orderings:

        if (LLVM_UNLIKELY(P1.empty())) {
            return 0;
        }

        std::sort_heap(P1.begin(), P1.end(), FitnessComparator{});

        assert (std::is_sorted(P1.begin(), P1.end(), FitnessComparator{}));

        // Construct a trie of all possible best (lowest) orderings of this partition

        auto i = P1.begin();
        const auto end = P1.end();

        const auto bestWeight = (*i)->second;
        do {
            make_trie((*i)->first, result);
        } while ((++i != end) && (bestWeight == (*i)->second));

        return bestWeight;
    }

protected:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief insertCandidate
     ** ------------------------------------------------------------------------------------------------------------- */
    bool insertCandidate(const Candidate & candidate, Population & population) {
        const auto f = candidates.emplace(candidate, 0);
        if (LLVM_LIKELY(f.second)) {
            f.first->second = fitness(f.first->first);
            population.emplace_back(f.first);
            std::push_heap(population.begin(), population.end(), FitnessComparator{});
            return true;
        }
        return false;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief initGA
     ** ------------------------------------------------------------------------------------------------------------- */
    virtual bool initGA(Population & initialPopulation) = 0;

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief repairCandidate
     ** ------------------------------------------------------------------------------------------------------------- */
    virtual void repairCandidate(Candidate & candidate) = 0;

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief fitness
     ** ------------------------------------------------------------------------------------------------------------- */
    virtual size_t fitness(const Candidate & candidate) = 0;

private:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief make_trie
     ** ------------------------------------------------------------------------------------------------------------- */
    void make_trie(const Candidate & C, OrderingDAWG & O) {
        assert (num_vertices(O) > 0);
        assert (C.size() == candidateLength);
        auto u = 0;

        for (unsigned i = 0; i != candidateLength; ) {
            const auto j = C[i];
            assert (j < candidateLength);
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
in_trie:    ++i;
        }
    };

protected:

    EvolutionaryAlgorithm(const unsigned candidateLength
                          , const unsigned maxRounds
                          , const unsigned maxCandidates
                          , const double mutationRate)
    : candidateLength(candidateLength)
    , maxRounds(maxRounds)
    , maxCandidates(maxCandidates)
    , mutationRate(mutationRate)
    , rng(std::random_device()()) {
        assert (0.0 <= mutationRate && mutationRate <= 1.0);
    }

protected:

    const unsigned candidateLength;
    const unsigned maxRounds;
    const unsigned maxCandidates;
    const double mutationRate;

    std::map<Candidate, size_t> candidates;

    random_engine rng;

};

}

#endif // EVOLUTIONARY_ALGORITHM_HPP
