/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <cc/cc_compiler.h>
#include <UCD/unicode_set.h>
#include <UCD/PropertyObjectTable.h>
#include <UCD/ucd_compiler.hpp>
#include <pablo/pablo_compiler.h>
#include <pablo/builder.hpp>
#include <pablo/function.h>
#include <llvm/Support/CommandLine.h>
#include <utf_encoding.h>
#include <pablo/analysis/pabloverifier.hpp>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/optimizers/codemotionpass.h>
#ifdef ENABLE_MULTIPLEXING
#include <pablo/optimizers/pablo_bddminimization.h>
#include <pablo/optimizers/pablo_automultiplexing.hpp>
#endif
#include <pablo/optimizers/booleanreassociationpass.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/TargetRegistry.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Target/TargetLibraryInfo.h>
#include <llvm/Target/TargetMachine.h>
#include <llvm/Support/Host.h>
#include <llvm/ADT/Triple.h>
#include <llvm/Support/ToolOutputFile.h>
#include <llvm/Pass.h>
#include <llvm/PassManager.h>
#include <llvm/ADT/STLExtras.h>
#include <llvm/Target/TargetSubtargetInfo.h>
#include <llvm/Support/FormattedStream.h>
#include "llvm/Support/FileSystem.h"
#include <llvm/Transforms/Scalar.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Analysis/DependenceAnalysis.h>
#include <boost/container/flat_map.hpp>
#include <queue>
#include <unordered_map>
#include <pablo/printer_pablos.h>
#include <llvm/Analysis/PostDominators.h>

using namespace pablo;
using namespace UCD;
using namespace cc;
using namespace llvm;
using namespace boost::container;

enum IfHierarchy {DefaultIfHierarchy, NoIfHierarchy};

static cl::opt<std::string>
ObjectFilename("o", cl::desc("Output object filename"), cl::value_desc("filename"), cl::Required);

static cl::opt<std::string>
UCDSourcePath("dir", cl::desc("UCD source code directory"), cl::value_desc("directory"), cl::Required);

static cl::opt<std::string>
PrintLongestDependenceChain("ldc", cl::desc("print longest dependency chain metrics."), cl::value_desc("filename"));

static cl::opt<IfHierarchy> IfHierarchyStrategy(cl::desc("If Hierarchy strategy:"),
                                                cl::values(clEnumVal(DefaultIfHierarchy, "Default"),
                                                           clEnumVal(NoIfHierarchy, "None"),
                                                           clEnumValEnd));

static cl::opt<bool> EnableReassociation("reassoc", cl::init(false),
                                      cl::desc("Enable reassociation and distribution optimization of Boolean functions."), cl::Optional);


static raw_fd_ostream * LongestDependenceChainFile = nullptr;

#ifdef ENABLE_MULTIPLEXING
static cl::opt<bool> EnableMultiplexing("multiplexing", cl::init(false),
    cl::desc("combine Advances whose inputs are mutual exclusive into the fewest number of advances possible (expensive)."));

static cl::opt<std::string>
MultiplexingDistribution("multiplexing-dist",
    cl::desc("Generate a CSV containing the # of Advances found in each UCD function before and after applying multiplexing."),
    cl::value_desc("filename"));

static raw_fd_ostream * MultiplexingDistributionFile = nullptr;
#else
const bool EnableMultiplexing = false;
#endif

using property_list = std::vector<std::string>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfAdvances
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned getNumOfAdvances(const PabloBlock & entry) {
    unsigned advances = 0;
    for (const Statement * stmt : entry ) {
        if (isa<Advance>(stmt)) {
            ++advances;
        }
        else if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            advances += getNumOfAdvances(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        }
    }
    return advances;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computePabloDependencyMetrics
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned computePabloDependencyChainMetrics(const PabloBlock & b, std::unordered_map<const PabloAST *, unsigned> & G) {
    unsigned lpl = 0;
    flat_map<const PabloAST *, unsigned> L;
    for (const Statement * stmt : b) {
        unsigned local_lpl = 0;
        unsigned global_lpl = 0;
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            const PabloAST * const op = stmt->getOperand(i);
            if (isa<String>(op) || isa<Integer>(op)) {
                continue;
            }
            const auto l = L.find(op);
            if (l != L.end()) {
                local_lpl = std::max<unsigned>(local_lpl, l->second);
            }
            const auto g = G.find(op);
            if (LLVM_UNLIKELY(g == G.end())) {
                throw std::runtime_error("Could not find dependency chain length for all operands!");
            }
            global_lpl = std::max<unsigned>(global_lpl, g->second);
        }
        L.emplace(stmt, local_lpl + 1);
        G.insert(std::make_pair(stmt, global_lpl + 1));
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            for (const auto & l : L) {
                lpl = std::max(lpl, l.second);
            }
            L.clear();
            lpl = std::max(lpl, computePabloDependencyChainMetrics(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), G));
        }
    }
    return lpl;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computePabloDependencyMetrics
 ** ------------------------------------------------------------------------------------------------------------- */
std::pair<unsigned, unsigned> computePabloDependencyChainMetrics(const PabloFunction * f) {
    std::unordered_map<const PabloAST *, unsigned> G;
    G.insert(std::make_pair(PabloBlock::createZeroes(), 0));
    G.insert(std::make_pair(PabloBlock::createOnes(), 0));
    for (unsigned i = 0; i != f->getNumOfParameters(); ++i) {
        G.insert(std::make_pair(f->getParameter(i), 0));
    }
    const unsigned local_lpl = computePabloDependencyChainMetrics(f->getEntryBlock(), G);
    unsigned global_lpl = 0;
    for (unsigned i = 0; i != f->getNumOfResults(); ++i) {
        const auto e = G.find(f->getResult(i));
        if (e == G.end()) {
            throw std::runtime_error("No result computed!");
        }
        global_lpl = std::max<unsigned>(global_lpl, e->second);
    }
    return std::make_pair(global_lpl, local_lpl);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeLLVMDependencyMetrics
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned computeLLVMDependencyChainMetrics(const DomTreeNode * t, std::unordered_map<const Value *, unsigned> & G) {
    unsigned lpl = 0;
    if (true) {
        flat_map<const Value *, unsigned> L;
        const BasicBlock * b = t->getBlock();
        for (auto itr = b->rbegin(); itr != b->rend(); ++itr) {
            unsigned local_lpl = 0;
            unsigned global_lpl = 0;
            const Instruction & inst = *itr;
            for (const Value * user : inst.users()) {
                if (LLVM_LIKELY(isa<Instruction>(user))) {
                    const auto l = L.find(user);
                    if (l != L.end()) {
                        local_lpl = std::max<unsigned>(local_lpl, l->second);
                    }
                    const auto g = G.find(user);
                    if (LLVM_UNLIKELY(g == G.end())) {
                        throw std::runtime_error("Could not find chain length for all users!");
                    }
                    global_lpl = std::max<unsigned>(global_lpl, g->second);
                }
            }
            L.emplace(&inst, local_lpl + 1);
            G.insert(std::make_pair(&inst, global_lpl + 1));
            lpl = std::max(lpl, local_lpl + 1);
        }
    }
    for (const DomTreeNode * pt : *t) {
        lpl = std::max(lpl, computeLLVMDependencyChainMetrics(pt, G));
    }
    return lpl;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeLLVMDependencyMetrics
 ** ------------------------------------------------------------------------------------------------------------- */
std::pair<unsigned, unsigned> computeLLVMDependencyChainMetrics(llvm::Function * f) {
    std::unordered_map<const llvm::Value *, unsigned> G;

    auto itr = f->getArgumentList().begin();
    const Argument & input = *itr++;
    const Argument & output = *itr;
    for (const User * user : output.users()) {
        G.insert(std::make_pair(user, 0));
    }

    PostDominatorTree dt;
    dt.runOnFunction(*f);
    const unsigned local_lpl = computeLLVMDependencyChainMetrics(dt.getRootNode(), G);
    dt.releaseMemory();

    unsigned global_lpl = 0;
    for (const User * user : input.users()) {
        const auto e = G.find(user);
        if (e == G.end()) {
            throw std::runtime_error("No result computed!");
        }
        global_lpl = std::max<unsigned>(global_lpl, e->second);
    }
    return std::make_pair(global_lpl, local_lpl);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief compileUnicodeSet
 ** ------------------------------------------------------------------------------------------------------------- */
void compileUnicodeSet(std::string name, UnicodeSet && set, PabloCompiler & pc, Module * module) {
    #ifdef ENABLE_MULTIPLEXING
    if (MultiplexingDistributionFile) {
        (*MultiplexingDistributionFile) << name;
    }
    #endif
    if (LongestDependenceChainFile) {
        (*LongestDependenceChainFile) << name;
    }
    #ifndef NDEBUG
    std::cerr << name << std::endl;
    #endif
    PabloFunction * pbFunction = PabloFunction::Create(std::move(name), 8, 1);
    Encoding encoding(Encoding::Type::UTF_8, 8);
    CC_Compiler ccCompiler(*pbFunction, encoding);
    UCDCompiler ucdCompiler(ccCompiler);
    PabloBuilder builder(pbFunction->getEntryBlock());
    // Build the unicode set function
    PabloAST * result = nullptr;
    if (IfHierarchyStrategy == IfHierarchy::DefaultIfHierarchy) {
        result = ucdCompiler.generateWithDefaultIfHierarchy(&set, builder);
    } else if (IfHierarchyStrategy == IfHierarchy::NoIfHierarchy) {
        result = ucdCompiler.generateWithoutIfHierarchy(&set, builder);
    } else {
        throw std::runtime_error("Unknown if hierarchy strategy!");
    }
    pbFunction->setResult(0, builder.createAssign("matches", result));
    // Optimize it at the pablo level
    PabloVerifier::verify(*pbFunction, "creation");
    Simplifier::optimize(*pbFunction);
    CodeMotionPass::optimize(*pbFunction);
    #ifdef ENABLE_MULTIPLEXING
    BDDMinimizationPass::optimize(*function);
    if (EnableMultiplexing) {
        if (LongestDependenceChainFile) {
            const auto pablo_metrix = computePabloDependencyChainMetrics(function);
            (*LongestDependenceChainFile) << ',' << pablo_metrix.first << ',' << pablo_metrix.second;
            Module module("tmp", getGlobalContext());
            llvm::Function * func = pc.compile(function, &module);
            const auto llvm_metrix = computeLLVMDependencyChainMetrics(func);
            (*LongestDependenceChainFile) << ',' << llvm_metrix.first << ',' << llvm_metrix.second;
        }

        if (MultiplexingDistributionFile) {
            (*MultiplexingDistributionFile) << ',' << getNumOfAdvances(function->getEntryBlock());
        }
        AutoMultiplexing::optimize(*function);
        if (MultiplexingDistributionFile) {
            (*MultiplexingDistributionFile) << ',' << getNumOfAdvances(function->getEntryBlock()) << '\n';
        }
    }
    #endif
    if (EnableReassociation) {
        BooleanReassociationPass::optimize(*pbFunction);
    }

    // Now compile the function ...
    llvm::Function * func = pc.compile(pbFunction, module);

    if (LongestDependenceChainFile) {
        const auto pablo_metrix = computePabloDependencyChainMetrics(pbFunction);
        (*LongestDependenceChainFile) << ',' << pablo_metrix.first << ',' << pablo_metrix.second;
        const auto llvm_metrix = computeLLVMDependencyChainMetrics(func);
        (*LongestDependenceChainFile) << ',' << llvm_metrix.first << ',' << llvm_metrix.second << '\n';
    }

    delete pbFunction;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writePropertyInstaller
 ** ------------------------------------------------------------------------------------------------------------- */

void writePrecompiledProperties(property_list && properties) {

    const std::string headerFilename = UCDSourcePath + "/precompiled_properties.h";
    #ifdef USE_LLVM_3_5
    std::string error;
    raw_fd_ostream header(headerFilename.c_str(), error, sys::fs::F_None);
    if (!error.empty()) {
        throw std::runtime_error(error);
    }
    #else
    std::error_code error;
    raw_fd_ostream header(headerFilename, error, sys::fs::F_None);
    if (error) {
        throw std::runtime_error(error.message());
    }
    #endif

    header << "#ifndef PRECOMPILED_PROPERTIES\n";
    header << "#define PRECOMPILED_PROPERTIES\n\n";
    header << "#include <string>\n\n";
    header << "#include <tuple>\n";
    header << "namespace UCD {\n\n";
    header << "using ExternalProperty = std::tuple<void *, unsigned, unsigned>;\n\n";
    header << "const ExternalProperty & resolveExternalProperty(const std::string & name);\n\n";
    header << "}\n\n";
    header << "#endif\n";
    header.close();

    const std::string cppFilename = UCDSourcePath + "/precompiled_properties.cpp";
    #ifdef USE_LLVM_3_5
    raw_fd_ostream cpp(cppFilename.c_str(), error, sys::fs::F_None);
    if (!error.empty()) {
        throw std::runtime_error(error);
    }
    #else
    raw_fd_ostream cpp(cppFilename, error, sys::fs::F_None);
    if (error) {
        throw std::runtime_error(error.message());
    }
    #endif

    cpp << "#include \"precompiled_properties.h\"\n";
    cpp << "#include <include/simd-lib/bitblock.hpp>\n";
    cpp << "#include <stdexcept>\n";
    cpp << "#include <unordered_map>\n\n";
    cpp << "namespace UCD {\nnamespace {\n\n";
    cpp << "struct Input {\n    BitBlock bit[8];\n};\n\n";
    cpp << "struct Output {\n    BitBlock bit[1];\n};\n\n";
    for (auto prop : properties) {
        cpp << "extern \"C\" void " + prop + "(const Input &, Output &);\n";
    }

    cpp << "\nconst static std::unordered_map<std::string, ExternalProperty> EXTERNAL_UCD_PROPERTY_MAP = {\n";
    for (auto itr = properties.begin(); itr != properties.end(); ) {
        cpp << "    {\"" + *itr + "\", std::make_tuple(reinterpret_cast<void *>(&" + *itr + "), 8, 1)}";
        if (++itr != properties.end()) {
            cpp << ",";
        }
        cpp << "\n";
    }
    cpp << "};\n\n} // end of anonymous namespace\n\n";

    cpp << "const ExternalProperty & resolveExternalProperty(const std::string & name) {\n";
    cpp << "    auto f = EXTERNAL_UCD_PROPERTY_MAP.find(name);\n";
    cpp << "    if (f == EXTERNAL_UCD_PROPERTY_MAP.end())\n";
    cpp << "        throw std::runtime_error(\"No external property named \\\"\" + name + \"\\\" found!\");\n";
    cpp << "    return f->second;\n";
    cpp << "}\n\n} // end of UCD namespace\n";

    cpp.close();

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateUCDModule
 ** ------------------------------------------------------------------------------------------------------------- */
Module * generateUCDModule() {

    property_list properties;

    PabloCompiler pc(VectorType::get(IntegerType::get(getGlobalContext(), 64), BLOCK_SIZE/64));
    Module * module = new Module("ucd", getGlobalContext());
    for (PropertyObject * obj : property_object_table) {
        if (EnumeratedPropertyObject * enumObj = dyn_cast<EnumeratedPropertyObject>(obj)) {
            for (const std::string value : *enumObj) {
                UnicodeSet set = enumObj->GetCodepointSet(canonicalize_value_name(value));
                std::string name = "__get_" + property_enum_name[enumObj->getPropertyCode()] + "_" + value;
                compileUnicodeSet(name, std::move(set), pc, module);
                properties.emplace_back(name);
            }
        }
        else if (ExtensionPropertyObject * extObj = dyn_cast<ExtensionPropertyObject>(obj)) {
            for (const std::string value : *extObj) {
                UnicodeSet set = extObj->GetCodepointSet(canonicalize_value_name(value));
                std::string name = "__get_" + property_enum_name[extObj->getPropertyCode()] + "_" + value;
                compileUnicodeSet(name, std::move(set), pc, module);
                properties.emplace_back(name);
            }
        }
        else if (BinaryPropertyObject * binObj = dyn_cast<BinaryPropertyObject>(obj)) {
            UnicodeSet set = binObj->GetCodepointSet(Binary_ns::Y);
            std::string name = "__get_" + property_enum_name[binObj->getPropertyCode()] + "_Y";
            compileUnicodeSet(name, std::move(set), pc, module);
            properties.emplace_back(name);
        }
    }

    // Print an error message if our module is malformed in any way.
    verifyModule(*module, &dbgs());

    writePrecompiledProperties(std::move(properties));

    return module;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief compileUCDModule
 ** ------------------------------------------------------------------------------------------------------------- */
void compileUCDModule(Module * module) {
    Triple TheTriple;

    TheTriple.setTriple(sys::getDefaultTargetTriple());

    // Get the target specific parser.
    std::string msg;
    const Target * TheTarget = TargetRegistry::lookupTarget(TheTriple.getTriple(), msg);
    if (TheTarget == nullptr) {
        throw std::runtime_error(msg);
    }

    TargetOptions Options;

    std::unique_ptr<TargetMachine> Target(
                TheTarget->createTargetMachine(TheTriple.getTriple(), sys::getHostCPUName(), "", Options,
                                               Reloc::Default, CodeModel::Small, CodeGenOpt::Aggressive));

    if (Target == nullptr) {
        throw std::runtime_error("Could not allocate target machine!");
    }

    #ifdef USE_LLVM_3_5
    std::string error;
    std::unique_ptr<tool_output_file> out = make_unique<tool_output_file>(ObjectFilename.c_str(), error, sys::fs::F_None);
    if (!error.empty()) {
        throw std::runtime_error(error);
    }
    #else
    std::error_code error;
    std::unique_ptr<tool_output_file> out = make_unique<tool_output_file>(ObjectFilename, error, sys::fs::F_None);
    if (error) {
        throw std::runtime_error(error.message());
    }
    #endif

    // Build up all of the passes that we want to do to the module.
    PassManager PM;

    // Add an appropriate TargetLibraryInfo pass for the module's triple.
    PM.add(new TargetLibraryInfo(TheTriple));

    // Add the target data from the target machine, if it exists, or the module.
    #ifdef USE_LLVM_3_5
    const DataLayout * DL = Target->getDataLayout();
    #else
    const DataLayout * DL = Target->getSubtargetImpl()->getDataLayout();
    #endif
    if (DL) {
        module->setDataLayout(DL);
    }
    #ifdef USE_LLVM_3_5
    PM.add(new DataLayoutPass(module));
    #else
    PM.add(new DataLayoutPass());
    #endif    
    PM.add(createReassociatePass());
    PM.add(createInstructionCombiningPass());
    PM.add(createSinkingPass());

    formatted_raw_ostream outStream(out->os());
    // Ask the target to add backend passes as necessary.
    if (Target->addPassesToEmitFile(PM, outStream, TargetMachine::CGFT_ObjectFile)) {
        throw std::runtime_error("Target does not support generation of object file type!\n");
    }

    PM.run(*module);


    out->keep();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief main
 ** ------------------------------------------------------------------------------------------------------------- */
int main(int argc, char *argv[]) {
    // Initialize targets first, so that --version shows registered targets.
    InitializeAllTargets();
    InitializeAllTargetMCs();
    InitializeAllAsmPrinters();
    InitializeAllAsmParsers();
    cl::ParseCommandLineOptions(argc, argv, "UCD Compiler\n");


    #ifdef ENABLE_MULTIPLEXING
    if (MultiplexingDistribution.length() > 0) {
        #ifdef USE_LLVM_3_5
        std::string error;
        MultiplexingDistributionFile = new raw_fd_ostream(MultiplexingDistribution.c_str(), error, sys::fs::F_Text);
        if (!error.empty()) {
            throw std::runtime_error(error);
        }
        #else
        std::error_code error;
        MultiplexingDistributionFile = new raw_fd_ostream(MultiplexingDistribution, error, sys::fs::F_Text);
        if (error) {
            throw std::runtime_error(error.message());
        }
        #endif
    }
    #endif

    if (PrintLongestDependenceChain.length() > 0) {
        #ifdef USE_LLVM_3_5
        std::string error;
        LongestDependenceChainFile = new raw_fd_ostream(PrintLongestDependenceChain.c_str(), error, sys::fs::F_Text);
        if (!error.empty()) {
            throw std::runtime_error(error);
        }
        #else
        std::error_code error;
        LongestDependenceChainFile = new raw_fd_ostream(PrintLongestDependenceChain, error, sys::fs::F_Text);
        if (error) {
            throw std::runtime_error(error.message());
        }
        #endif

        if (LongestDependenceChainFile) {
            if (EnableMultiplexing) {
                (*LongestDependenceChainFile) << ",Pre-Multiplexing,,,,Post-Multiplexing\n";
            }
            (*LongestDependenceChainFile) << ",Pablo,,LLVM,";
            if (EnableMultiplexing) {
                (*LongestDependenceChainFile) << ",Pablo,,LLVM,";
            }
            (*LongestDependenceChainFile) << "\nName,Global,Max Local,Global,Max Local";
            if (EnableMultiplexing) {
                (*LongestDependenceChainFile) << ",Global,Max Local,Global,Max Local";
            }
            (*LongestDependenceChainFile) << "\n";
        }
    }

    Module * module = generateUCDModule();
    #ifdef ENABLE_MULTIPLEXING
    if (MultiplexingDistributionFile) {
        MultiplexingDistributionFile->close();
        delete MultiplexingDistributionFile;
    }    
    #endif
    if (LongestDependenceChainFile) {
        LongestDependenceChainFile->close();
        delete LongestDependenceChainFile;
    }
    compileUCDModule(module);
    return 0;
}
