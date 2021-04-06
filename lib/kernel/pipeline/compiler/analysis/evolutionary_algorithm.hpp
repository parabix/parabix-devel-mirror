#ifndef PIPELINE_COMPILER_ANALYSIS_EVOLUTIONARY_ALGORITHM_HPP
#define PIPELINE_COMPILER_ANALYSIS_EVOLUTIONARY_ALGORITHM_HPP

#include "pipeline_analysis.hpp"
#include <llvm/Support/Format.h>
#include <llvm/ADT/DenseMap.h>
#include "../common/common.hpp"

#warning cleanup this code if the approach works


namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief OrderingBasedEvolutionaryAlgorithm
 *
 * Both the partition scheduling algorithm and whole program scheduling algorithm rely on the following class.
 * Within it is a genetic algorithm designed to find a minimum memory schedule of a given SchedulingGraph.
 * However, the phenotype of of the partition algorithm is a topological ordering and the phenotype of the
 * whole program is a hamiltonian path. This consitutes a significant enough difference that it is difficult
 * to call with only one function. Instead both the "initGA" and "repair" functions are implemented within
 * the actual scheduling functions.
 ** ------------------------------------------------------------------------------------------------------------- */
class OrderingBasedEvolutionaryAlgorithm {
public:

    using Candidate = std::vector<Vertex>;

    using FitnessValueType = size_t;

    using Candidates = std::map<Candidate, FitnessValueType>;

    using Individual = typename Candidates::const_iterator;

    using Population = std::vector<Individual>;

    struct FitnessValueEvaluator {
        constexpr static bool eval(const FitnessValueType a,const FitnessValueType b) {
            return a < b;
        }
    };

    struct FitnessComparator {
        bool operator()(const Individual & a,const Individual & b) const{
            return FitnessValueEvaluator::eval(a->second, b->second);;
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
     * @brief initGAWithDefaultPopulation
     ** ------------------------------------------------------------------------------------------------------------- */
    bool initGAWithDefaultPopulation() {
        return initGA(population);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief initGA
     ** ------------------------------------------------------------------------------------------------------------- */
    virtual bool initGA(Population & initialPopulation) = 0;

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief runGA
     ** ------------------------------------------------------------------------------------------------------------- */
    const OrderingBasedEvolutionaryAlgorithm & runGA(const bool print = false) {

        population.reserve(maxCandidates);

        const auto enumeratedAll = initGA(population);

        if (print) {
            printDataPoint();
        }

        if (LLVM_UNLIKELY(enumeratedAll)) {
            goto enumerated_entire_search_space;
        }

        assert (candidateLength > 1);

        BEGIN_SCOPED_REGION

        permutation_bitset bitString(candidateLength);

        BitVector V(candidateLength);

        std::uniform_real_distribution<double> zeroToOneReal(0.0, 1.0);

        Population nextGeneration;
        nextGeneration.reserve(3 * maxCandidates);

        constexpr auto minFitVal = std::numeric_limits<FitnessValueType>::min();
        constexpr auto maxFitVal = std::numeric_limits<FitnessValueType>::max();
        constexpr auto worstFitnessValue = FitnessValueEvaluator::eval(minFitVal, maxFitVal) ? maxFitVal : minFitVal;

        unsigned averageStallCount = 0;
        unsigned bestStallCount = 0;

        auto priorAverageFitness = worstFitnessValue;
        auto priorBestFitness = worstFitnessValue;

        for (unsigned g = 0; g < maxGenerations; ++g) {

            const auto populationSize = population.size();

            assert (populationSize > 1);

//            std::uniform_int_distribution<unsigned> upToN(0, populationSize - 1);

//            auto tournament_select = [&]() -> const Candidate & {
//                const auto & A = population[upToN(rng)];
//                const auto & B = population[upToN(rng)];
//                if (A->second < B->second) {
//                    return A->first;
//                } else {
//                    return B->first;
//                }
//            };

            const double currentMutationRate = (double)(g + 1) / (double)(maxGenerations);
            const double currentCrossoverRate = 1.0 - currentMutationRate;

            redo_generation:

            // CROSSOVER:

            assert (nextGeneration.empty());

            FitnessValueType sumOfGenerationalFitness = 0.0;
            FitnessValueType bestGenerationalFitness = worstFitnessValue;
            unsigned generationCount = 0;

            for (unsigned i = 1; i < populationSize; ++i) {

//                const Candidate & A = tournament_select();
//                const Candidate & B = tournament_select();

                for (unsigned j = 0; j < i; ++j) {
                    if (zeroToOneReal(rng) <= currentCrossoverRate) {

                        const Candidate & A = population[i]->first;
                        const Candidate & B = population[j]->first;

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
                            FitnessValueType fitness;
                            insertCandidate(C, nextGeneration, fitness);
                            sumOfGenerationalFitness += fitness;
                            ++generationCount;
                            if (FitnessValueEvaluator::eval(fitness, bestGenerationalFitness)) {
                                bestGenerationalFitness = fitness;
                            }
                        };

                        crossover(A, B, true);

                        crossover(B, A, false);

                    }
                }

            }

            // MUTATION:

            for (unsigned i = 0; i < populationSize; ++i) {
                if (zeroToOneReal(rng) <= currentMutationRate) {

                    auto & A = population[i];

                    const auto a = std::uniform_int_distribution<unsigned>{0, candidateLength - 2}(rng);
                    const auto b = std::uniform_int_distribution<unsigned>{a + 1, candidateLength - 1}(rng);

                    Candidate C{A->first};
                    std::shuffle(C.begin() + a, C.begin() + b, rng);

                    repairCandidate(C);
                    FitnessValueType fitness;
                    insertCandidate(C, nextGeneration, fitness);
                    sumOfGenerationalFitness += fitness;
                    ++generationCount;
                    if (FitnessValueEvaluator::eval(fitness, bestGenerationalFitness)) {
                        bestGenerationalFitness = fitness;
                    }
                }
            }

            // SELECTION:

            for (const auto & I : nextGeneration) {
                assert (population.size() <= maxCandidates);
                if (population.size() == maxCandidates) {
                    if (I->second <= population.front()->second) {
                        std::pop_heap(population.begin(), population.end(), FitnessComparator{});
                        population.pop_back();
                    } else {
                        // New item exceeds the weight of the heaviest candiate
                        // in the population.
                        continue;
                    }
                }
                population.emplace_back(I);
                std::push_heap(population.begin(), population.end(), FitnessComparator{});
            }

            if (print) {
                printDataPoint();
            }

            if (LLVM_UNLIKELY(generationCount == 0)) {
                goto redo_generation;
            }

            const auto averageGenerationFitness = (sumOfGenerationalFitness + generationCount / 2) / generationCount;

            nextGeneration.clear();

            if (std::abs<FitnessValueType>(averageGenerationFitness - priorAverageFitness) <= averageStallThreshold) {
                if (++averageStallCount == maxStallGenerations) {
                    break;
                }
            } else {
                averageStallCount = 0;
            }
            assert (averageStallCount < maxStallGenerations);
            priorAverageFitness = averageGenerationFitness;

            if (std::abs<FitnessValueType>(bestGenerationalFitness - priorBestFitness) <= averageStallThreshold) {
                if (++bestStallCount == maxStallGenerations) {
                    break;
                }
            } else {
                bestStallCount = 0;
            }
            assert (bestStallCount < maxStallGenerations);
            priorBestFitness = bestGenerationalFitness;
        }

        END_SCOPED_REGION

enumerated_entire_search_space:

        std::sort_heap(population.begin(), population.end(), FitnessComparator{});

        return *this;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief printDataPoint
     ** ------------------------------------------------------------------------------------------------------------- */
    virtual void printDataPoint() {
//        FitnessValueType min = std::numeric_limits<FitnessValueType>::max();
//        FitnessValueType invalid = 0;

//        for (const auto & I : population) {
//            const auto v = I->second;
//            if (v == std::numeric_limits<FitnessValueType>::max()) {
//                invalid++;
//            } else if (v < min)  {
//                min = v;
//            }
//        }

//        outs() << ',' << min << ',' << invalid << ',' << candidates.size();
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief getResult
     ** ------------------------------------------------------------------------------------------------------------- */
    OrderingDAWG getResult() const {
        assert (std::is_sorted(population.begin(), population.end(), FitnessComparator{}));

        // Construct a trie of all possible best (lowest) orderings of this partition

        auto i = population.begin();
        const auto end = population.end();
        OrderingDAWG result(1);
        const auto bestWeight = (*i)->second;
        do {
            make_trie((*i)->first, result);
        } while ((++i != end) && (bestWeight == (*i)->second));

        return result;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief getBestFitnessValue
     ** ------------------------------------------------------------------------------------------------------------- */
    FitnessValueType getBestFitnessValue() const {
        assert (std::is_sorted(population.begin(), population.end(), FitnessComparator{}));
        return population.front()->second;
    }

protected:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief insertCandidate
     ** ------------------------------------------------------------------------------------------------------------- */
    bool insertCandidate(const Candidate & candidate, Population & population, FitnessValueType & fitnessValue) {
        const auto f = candidates.emplace(candidate, 0);
        if (LLVM_LIKELY(f.second)) {
            const auto value = fitness(f.first->first);
            fitnessValue = value;
            f.first->second = value;
            population.emplace_back(f.first);
            std::push_heap(population.begin(), population.end(), FitnessComparator{});
            return true;
        } else {
            fitnessValue = f.first->second;
        }
        return false;
    }

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
    void make_trie(const Candidate & C, OrderingDAWG & O) const {
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

    OrderingBasedEvolutionaryAlgorithm(const unsigned candidateLength
                          , const unsigned maxRounds
                          , const unsigned maxCandidates
                          , const double mutationRate
                          , random_engine & rng)
    : candidateLength(candidateLength)
    , maxGenerations(maxRounds)
    , maxCandidates(maxCandidates)
    , mutationRate(mutationRate)
    , averageStallThreshold(3)
    , maxStallThreshold(3)
    , maxStallGenerations(50)
    , rng(rng) {
        assert (0.0 <= mutationRate && mutationRate <= 1.0);
    }

protected:

    const unsigned candidateLength;
    const unsigned maxGenerations;
    const unsigned maxCandidates;
    const double mutationRate;

    const FitnessValueType averageStallThreshold;
    const FitnessValueType maxStallThreshold;
    const unsigned maxStallGenerations;

    Population population;

    std::map<Candidate, FitnessValueType> candidates;

    random_engine rng;

};

static unsigned HS_InsertionAttempts = 0;
static unsigned HS_UniqueInsertions = 0;

enum class HMCRType {
    Fixed = 0
    , Cos = 1
    , AbsCos = 2
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief BitStringBasedHarmonySearch
 ** ------------------------------------------------------------------------------------------------------------- */
class BitStringBasedHarmonySearch {
public:

    struct Candidate {

        using BitWord = uintptr_t;

        static constexpr BitWord BITWORD_SIZE{sizeof(BitWord) * CHAR_BIT};

        explicit Candidate(const size_t N)
        : _value((N + BITWORD_SIZE - 1) / BITWORD_SIZE, 0) {

        }

        Candidate(const Candidate & other)
        : _value(other._value) {

        }

        Candidate(Candidate && other)
        : _value(std::move(other._value)) {

        }

        Candidate & operator=(const Candidate & other ) {
            _value = other._value;
            return *this;
        }

        Candidate & operator=(Candidate && other ) {
            _value = std::move(other._value);
            return *this;
        }

        void set(const BitWord i, const bool value) {
            constexpr BitWord ZERO{0};
            constexpr BitWord ONE{1};
            auto & V = _value[i / BITWORD_SIZE];
            const auto mask = BitWord(1) << (i & (BITWORD_SIZE - ONE));
            V = (V | (value ? mask : ZERO)) & ~(value ? ZERO : mask);
        }

        bool test(const BitWord i) const {
            constexpr BitWord ONE{1};
            const auto & V = _value[i / BITWORD_SIZE];
            const auto mask = BitWord(1) << (i & (BITWORD_SIZE - ONE));
            return (V & mask) != 0;
        }

        size_t hash() const {
            std::size_t seed = 0;
            for (const auto & v : _value) {
                boost::hash_combine(seed, v);
            }
            return seed;
        }

        bool operator==(const Candidate & other) const {
            assert (other._value.size() == _value.size());
            auto i = _value.begin();
            auto j = other._value.begin();
            const auto end = _value.end();
            for (; i != end; ++i, ++j) {
                if (*i != *j) return false;
            }
            return true;
        }

    private:
        SmallVector<size_t, 4> _value;
    };


    struct __CandidateHash {
        size_t operator()(const Candidate & key) const {
           return key.hash();
        }
    };


    using FitnessValueType = double;

    using Candidates = std::unordered_map<Candidate, FitnessValueType, __CandidateHash>;

    using Individual = typename Candidates::const_iterator;

    using CandidateList = std::vector<Candidate>;

    using Population = std::vector<Individual>;

    struct FitnessComparator {
        bool operator()(const Individual & a,const Individual & b) const{
            return a->second > b->second;
        }
    };

public:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief runGA
     ** ------------------------------------------------------------------------------------------------------------- */
    const BitStringBasedHarmonySearch & runHarmonySearch() {

        population.reserve(maxCandidates);

        if (LLVM_UNLIKELY(initialize(population))) {
            goto enumerated_entire_search_space;
        }

        assert (candidateLength > 1);

        BEGIN_SCOPED_REGION

        std::uniform_real_distribution<double> zeroToOneReal(0.0, 1.0);

        std::uniform_int_distribution<unsigned> zeroOrOneInt(0, 1);

        Population nextGeneration;
        nextGeneration.reserve(maxCandidates);

        Candidate newCandidate(candidateLength);

//        errs() << ",0," << format("%.3f", bestResult);

//        auto lastBestResult = bestResult;

        double CosAmplitude = 0.0;
        double CosShift = 0.0;
        if (HMCRModelType == HMCRType::Cos) {
            CosAmplitude = (MaxHMCR - MinHMCR) / 2.0;
            CosShift = (MaxHMCR + MinHMCR) / 2.0;
        } else if (HMCRModelType == HMCRType::AbsCos) {
            CosAmplitude = (MaxHMCR - MinHMCR);
            CosShift = MinHMCR;
        }

        for (unsigned round = 0; round < maxRounds; ++round) {

            const auto populationSize = population.size();

            assert (populationSize <= maxCandidates);

            auto considerationRate = fixedConsiderationRate;
            if (HMCRModelType == HMCRType::Cos) {
                considerationRate = CosAmplitude * std::cos(CosAngularFrequency * (double)round) + CosShift;
            }
            if (HMCRModelType == HMCRType::AbsCos) {
                considerationRate = CosAmplitude * std::abs(std::cos(CosAngularFrequency * (double)round)) + CosShift;
            }

            std::uniform_int_distribution<unsigned> upToN(0, populationSize - 1);

            for (unsigned j = 0; j < candidateLength; ++j) {
                if (zeroToOneReal(rng) < considerationRate) {
                    const auto k = upToN(rng);
                    const bool v = population[k]->first.test(j);
                    newCandidate.set(j, v);
                } else {
                    newCandidate.set(j, zeroOrOneInt(rng));
                }
            }

            const auto f = candidates.insert(std::make_pair(newCandidate, 0));
            if (LLVM_LIKELY(f.second)) {
                const auto val = fitness(f.first->first);
                if (val >= population.front()->second) {
                    f.first->second = val;
                    bestResult = std::max(bestResult, val);
                    std::pop_heap(population.begin(), population.end(), FitnessComparator{});
                    population.pop_back();
                    population.emplace_back(f.first);
                    std::push_heap(population.begin(), population.end(), FitnessComparator{});
                }
            }

//            if (lastBestResult < bestResult) {
//                errs() << ',' << (round + 1U) << ',' << format("%.3f", bestResult);
//                lastBestResult = bestResult;
//            }

        }

//        errs() << ',' << maxRounds << ',' << format("%.3f", bestResult);

        END_SCOPED_REGION

enumerated_entire_search_space:

        std::sort_heap(population.begin(), population.end(), FitnessComparator{});

        return *this;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief removeLeastFitCandidates
     ** ------------------------------------------------------------------------------------------------------------- */
    void updatePopulation(Population & nextGeneration) {
        for (const auto & I : nextGeneration) {
            assert (population.size() <= maxCandidates);
            if (population.size() == maxCandidates) {
                if (I->second >= population.front()->second) {
                    std::pop_heap(population.begin(), population.end(), FitnessComparator{});
                    population.pop_back();
                } else {
                    // New item exceeds the weight of the heaviest candiate
                    // in the population.
                    continue;
                }
            }
            population.emplace_back(I);
            std::push_heap(population.begin(), population.end(), FitnessComparator{});
        }

    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief getResult
     ** ------------------------------------------------------------------------------------------------------------- */
    Candidate getResult() const {
        assert (std::is_sorted(population.begin(), population.end(), FitnessComparator{}));
        return population.front()->first;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief getBestFitnessValue
     ** ------------------------------------------------------------------------------------------------------------- */
    FitnessValueType getBestFitnessValue() const {
        assert (std::is_sorted(population.begin(), population.end(), FitnessComparator{}));
        return population.front()->second;
    }

protected:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief insertCandidate
     ** ------------------------------------------------------------------------------------------------------------- */
    bool insertCandidate(const Candidate & candidate, Population & population) {
        const auto f = candidates.insert(std::make_pair(candidate, 0));
        ++HS_InsertionAttempts;
        if (LLVM_LIKELY(f.second)) {
            ++HS_UniqueInsertions;
            const auto val = fitness(f.first->first);
            f.first->second = val;
            bestResult = std::max(bestResult, val);
            population.emplace_back(f.first);
            std::push_heap(population.begin(), population.end(), FitnessComparator{});
            return true;
        }
        return false;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief initGA
     ** ------------------------------------------------------------------------------------------------------------- */
    virtual bool initialize(Population & initialPopulation) = 0;

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief fitness
     ** ------------------------------------------------------------------------------------------------------------- */
    virtual FitnessValueType fitness(const Candidate & candidate) = 0;

protected:

    BitStringBasedHarmonySearch(const unsigned candidateLength
                          , const unsigned maxRounds
                          , const unsigned maxCandidates
                          , const double considerationRate
                          , const size_t seed)
    : candidateLength(candidateLength)
    , maxRounds(maxRounds)
    , maxCandidates(maxCandidates)
    , HMCRModelType(HMCRType::Fixed)
    , fixedConsiderationRate(considerationRate)
    , MinHMCR(0.0)
    , MaxHMCR(0.0)
    , CosAngularFrequency(0.0)
    , rng(seed) {
        assert (0.0 <= considerationRate && considerationRate <= 1.0);
    }

    BitStringBasedHarmonySearch(const unsigned candidateLength
                          , const unsigned maxRounds
                          , const unsigned maxCandidates
                          , const HMCRType type
                          , const double minHMCR
                          , const double maxHMCR
                          , const double frequency
                          , const size_t seed)
    : candidateLength(candidateLength)
    , maxRounds(maxRounds)
    , maxCandidates(maxCandidates)
    , HMCRModelType(type)
    , fixedConsiderationRate(0.00)
    , MinHMCR(minHMCR)
    , MaxHMCR(maxHMCR)
    , CosAngularFrequency(frequency)
    , rng(seed) {
        assert (maxHMCR >= minHMCR);
    }

public:

    const unsigned candidateLength;
    const unsigned maxRounds;
    const unsigned maxCandidates;
    const HMCRType HMCRModelType;

    const double fixedConsiderationRate;

    const double MinHMCR;
    const double MaxHMCR;
    const double CosAngularFrequency;

    FitnessValueType bestResult = 0;


    Population population;

    Candidates candidates;

    random_engine rng;

};

}

#endif // EVOLUTIONARY_ALGORITHM_HPP
