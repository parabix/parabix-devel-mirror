/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <fstream>

#include <pablo/pablo_compiler.h>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/optimizers/codemotionpass.h>
#include <pablo/passes/flattenassociativedfg.h>
#include <pablo/passes/factorizedfg.h>
#ifdef ENABLE_MULTIPLEXING
#include <pablo/optimizers/pablo_automultiplexing.hpp>
#include <pablo/optimizers/pablo_bddminimization.h>
#include <pablo/optimizers/distributivepass.h>
#include <pablo/optimizers/schedulingprepass.h>
#endif
#include <pablo/function.h>
#include <pablo/analysis/pabloverifier.hpp>
#include <pablo/printer_pablos.h>
#include <sstream>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/FileSystem.h>


using namespace pablo;


static cl::OptionCategory dPabloDumpOptions("Pablo Dump Options", "These options control printing of intermediate Pablo code.");

static cl::opt<bool> PrintOptimizedREcode("print-pablo", cl::init(false), cl::desc("print final optimized Pablo code"), cl::cat(dPabloDumpOptions));
static cl::opt<bool> PrintCompiledCCcode("print-CC-pablo", cl::init(false), cl::desc("print Pablo output from character class compiler"), cl::cat(dPabloDumpOptions));
static cl::opt<bool> PrintCompiledREcode("print-RE-pablo", cl::init(false), cl::desc("print Pablo output from the regular expression compiler"), cl::cat(dPabloDumpOptions));
static cl::opt<std::string> PabloOutputFilename("print-pablo-output", cl::init(""), cl::desc("output Pablo filename"), cl::cat(dPabloDumpOptions));


static cl::OptionCategory cPabloOptimizationsOptions("Pablo Optimizations", "These options control Pablo optimization passes.");

static cl::opt<bool> DisableSimplification("disable-simplification", cl::init(false),
                                     cl::desc("Disable Pablo Simplification pass (not recommended)"),
                                     cl::cat(cPabloOptimizationsOptions));

static cl::opt<bool> PabloSinkingPass("sinking", cl::init(false),
                                      cl::desc("Moves all instructions into the innermost legal If-scope so that they are only executed when needed."),
                                      cl::cat(cPabloOptimizationsOptions));

#ifdef ENABLE_MULTIPLEXING
static cl::opt<bool> PrintUnloweredCode("print-unlowered-pablo", cl::init(false), cl::desc("print Pablo output prior to lowering. "), cl::cat(dPabloDumpOptions));

static cl::opt<bool> EnableMultiplexing("multiplexing", cl::init(false),
                                        cl::desc("combine Advances whose inputs are mutual exclusive into the fewest number of advances possible (expensive)."),
                                        cl::cat(cPabloOptimizationsOptions));

static cl::opt<bool> EnableLowering("lowering", cl::init(false),
                                         cl::desc("coalesce associative functions prior to optimization passes."),
                                         cl::cat(cPabloOptimizationsOptions));

static cl::opt<bool> EnablePreDistribution("pre-dist", cl::init(false),
                                         cl::desc("apply distribution law optimization prior to multiplexing."),
                                         cl::cat(cPabloOptimizationsOptions));

static cl::opt<bool> EnablePostDistribution("post-dist", cl::init(false),
                                         cl::desc("apply distribution law optimization after multiplexing."),
                                         cl::cat(cPabloOptimizationsOptions));

static cl::opt<bool> EnablePrePassScheduling("pre-pass-scheduling", cl::init(false),
                                         cl::desc("apply pre-pass scheduling prior to LLVM IR generation."),
                                         cl::cat(cPabloOptimizationsOptions));
#endif


#ifdef PRINT_TIMING_INFORMATION
#define READ_CYCLE_COUNTER(name) name = read_cycle_counter();
#else
#define READ_CYCLE_COUNTER(name)
#endif

#ifdef PRINT_TIMING_INFORMATION
unsigned COUNT_STATEMENTS(const PabloFunction * const entry) {
    std::stack<const Statement *> scope;
    unsigned statements = 0;
    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    for (const Statement * stmt = entry->getEntryBlock()->front(); ; ) {
        while ( stmt ) {
            ++statements;
            if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope and push the
                // next statement of the current statement into the scope stack.
                const PabloBlock * const nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                scope.push(stmt->getNextNode());
                stmt = nested->front();
                assert (stmt);
                continue;
            }
            stmt = stmt->getNextNode();
        }
        if (scope.empty()) {
            break;
        }
        stmt = scope.top();
        scope.pop();
    }
    return statements;
}

unsigned COUNT_ADVANCES(const PabloFunction * const entry) {

    std::stack<const Statement *> scope;
    unsigned advances = 0;

    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    for (const Statement * stmt = entry->getEntryBlock()->front(); ; ) {
        while ( stmt ) {
            if (isa<Advance>(stmt)) {
                ++advances;
            }
            else if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope and push the
                // next statement of the current statement into the scope stack.
                const PabloBlock * const nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                scope.push(stmt->getNextNode());
                stmt = nested->front();
                assert (stmt);
                continue;
            }
            stmt = stmt->getNextNode();
        }
        if (scope.empty()) {
            break;
        }
        stmt = scope.top();
        scope.pop();
    }
    return advances;
}

using DistributionMap = boost::container::flat_map<unsigned, unsigned>;

DistributionMap SUMMARIZE_VARIADIC_DISTRIBUTION(const PabloFunction * const entry) {
    std::stack<const Statement *> scope;
    DistributionMap distribution;
    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    for (const Statement * stmt = entry->getEntryBlock()->front(); ; ) {
        while ( stmt ) {
            if (isa<Variadic>(stmt)) {
                auto f = distribution.find(stmt->getNumOperands());
                if (f == distribution.end()) {
                    distribution.emplace(stmt->getNumOperands(), 1);
                } else {
                    f->second += 1;
                }
            }
            else if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope and push the
                // next statement of the current statement into the scope stack.
                const PabloBlock * const nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                scope.push(stmt->getNextNode());
                stmt = nested->front();
                assert (stmt);
                continue;
            }
            stmt = stmt->getNextNode();
        }
        if (scope.empty()) {
            break;
        }
        stmt = scope.top();
        scope.pop();
    }
    return distribution;
}
#endif

void pablo_function_passes(PabloFunction * function) {
    
    if (PrintCompiledREcode) {
        //Print to the terminal the AST that was generated by the pararallel bit-stream compiler.
        llvm::raw_os_ostream cerr(std::cerr);
        cerr << "Initial Pablo AST:\n";
        PabloPrinter::print(*function, cerr);
    }
    
#ifndef NDEBUG
    PabloVerifier::verify(*function, "creation");
#endif
    
    // Scan through the pablo code and perform DCE and CSE

#ifdef PRINT_TIMING_INFORMATION
    timestamp_t simplification_start = 0, simplification_end = 0;
    timestamp_t coalescing_start = 0, coalescing_end = 0;
    timestamp_t sinking_start = 0, sinking_end = 0;
    timestamp_t pre_distribution_start = 0, pre_distribution_end = 0;
    timestamp_t multiplexing_start = 0, multiplexing_end = 0;
    timestamp_t post_distribution_start = 0, post_distribution_end = 0;
    timestamp_t lowering_start = 0, lowering_end = 0;
    timestamp_t scheduling_start = 0, scheduling_end = 0;
    DistributionMap distribution;
    const timestamp_t optimization_start = read_cycle_counter();
#endif
    if (!DisableSimplification) {
        READ_CYCLE_COUNTER(simplification_start);
        Simplifier::optimize(*function);
        READ_CYCLE_COUNTER(simplification_end);
    }
#ifdef ENABLE_MULTIPLEXING
    if (EnableLowering || EnablePreDistribution || EnablePostDistribution) {
        READ_CYCLE_COUNTER(coalescing_start);
        CanonicalizeDFG::transform(*function);
        READ_CYCLE_COUNTER(coalescing_end);
    }
    if (EnablePreDistribution) {
        READ_CYCLE_COUNTER(pre_distribution_start);
        DistributivePass::optimize(*function);
        READ_CYCLE_COUNTER(pre_distribution_end);
    }
    if (EnableMultiplexing) {
        READ_CYCLE_COUNTER(multiplexing_start);
        MultiplexingPass::optimize(*function);
        READ_CYCLE_COUNTER(multiplexing_end);
        if (EnableLowering || EnablePreDistribution || EnablePostDistribution) {
            CanonicalizeDFG::transform(*function);
        }
    }
    if (EnablePostDistribution) {
        READ_CYCLE_COUNTER(post_distribution_start);
        DistributivePass::optimize(*function);
        READ_CYCLE_COUNTER(post_distribution_end);
    }
#endif
    if (PabloSinkingPass) {
        READ_CYCLE_COUNTER(sinking_start);
        CodeMotionPass::optimize(*function);
        READ_CYCLE_COUNTER(sinking_end);
    }
#ifdef ENABLE_MULTIPLEXING
    if (PrintUnloweredCode) {
        //Print to the terminal the AST that was generated by the pararallel bit-stream compiler.
        llvm::raw_os_ostream cerr(std::cerr);
        cerr << "Unlowered Pablo AST:\n";
        PabloPrinter::print(*function, cerr);
    }
    #ifdef PRINT_TIMING_INFORMATION
    distribution = SUMMARIZE_VARIADIC_DISTRIBUTION(function);
    #endif
    if (EnableLowering || EnablePreDistribution || EnablePostDistribution) {
        READ_CYCLE_COUNTER(lowering_start);
        FactorizeDFG::transform(*function);
        READ_CYCLE_COUNTER(lowering_end);
    }
    if (EnablePrePassScheduling) {
        READ_CYCLE_COUNTER(scheduling_start);
        SchedulingPrePass::optimize(*function);
        READ_CYCLE_COUNTER(scheduling_end);
    }
#endif
#ifdef PRINT_TIMING_INFORMATION
    const timestamp_t optimization_end = read_cycle_counter();
#endif
    if (PrintOptimizedREcode) {
        if (PabloOutputFilename.empty()) {
            //Print to the terminal the AST that was generated by the pararallel bit-stream compiler.
            llvm::raw_os_ostream cerr(std::cerr);
            cerr << "Final Pablo AST:\n";
            PabloPrinter::print(*function, cerr);
        } else {
            std::error_code error;
            llvm::raw_fd_ostream out(PabloOutputFilename, error, sys::fs::OpenFlags::F_None);
            PabloPrinter::print(*function, out);
        }
    }
#ifdef PRINT_TIMING_INFORMATION
    std::cerr << "PABLO OPTIMIZATION TIME: " << (optimization_end - optimization_start) << std::endl;
    std::cerr << "  SIMPLIFICATION TIME: " << (simplification_end - simplification_start) << std::endl;
    std::cerr << "  COALESCING TIME: " << (coalescing_end - coalescing_start) << std::endl;
    std::cerr << "  SINKING TIME: " << (sinking_end - sinking_start) << std::endl;
    std::cerr << "  PRE-DISTRIBUTION TIME: " << (pre_distribution_end - pre_distribution_start) << std::endl;
    std::cerr << "  MULTIPLEXING TIME: " << (multiplexing_end - multiplexing_start) << std::endl;
    std::cerr << "  MULTIPLEXING SEED: " << MultiplexingPass::SEED << std::endl;
    std::cerr << "  MULTIPLEXING NODES USED: " << MultiplexingPass::NODES_USED << std::endl;
    std::cerr << "  MULTIPLEXING NODES ALLOCATED: " << MultiplexingPass::NODES_ALLOCATED << std::endl;
    std::cerr << "  LOWERING TIME: " << (lowering_end - lowering_start) << std::endl;
    std::cerr << "  POST-DISTRIBUTION TIME: " << (post_distribution_end - post_distribution_start) << std::endl;
    std::cerr << "  SCHEDULING TIME: " << (scheduling_end - scheduling_start) << std::endl;
    std::cerr << "PABLO STATEMENTS: " << COUNT_STATEMENTS(function) << std::endl;
    std::cerr << "PABLO ADVANCES: " << COUNT_ADVANCES(function) << std::endl;
    std::cerr << "PRE-LOWERING VARIADIC DISTRIBUTION: ";
    bool join = false;
    for (auto dist : distribution) {
        if (join) {
            std::cerr << ';';
        }
        std::cerr << dist.first << '|' << dist.second;
        join = true;
    }
    std::cerr << std::endl;
#endif
}

