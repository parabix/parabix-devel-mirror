/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>


#include <toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/MCJIT.h>
#include "llvm/Linker/Linker.h"

#include <llvm/Support/CommandLine.h>
#include <llvm/Support/raw_ostream.h>

#include <re/re_cc.h>
#include <cc/cc_compiler.h>
#include <pablo/function.h>
#include <pablo/pablo_compiler.h>
#include <pablo/pablo_kernel.h>
#include <IDISA/idisa_builder.h>
#include <IDISA/idisa_target.h>
#include <kernels/streamset.h>
#include <kernels/interface.h>
#include <kernels/kernel.h>
#include <kernels/s2p_kernel.h>
#include <editd/editdscan_kernel.h>
#include <kernels/pipeline.h>

#include <re/re_alt.h>
#include <editd/pattern_compiler.h>

// mmap system
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
#include <fcntl.h>
static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<regex> <input file ...>"), cl::OneOrMore);

static cl::list<std::string> pattVector("e", cl::desc("pattern"), cl::ZeroOrMore);
static cl::opt<std::string> PatternFilename("f", cl::desc("Take patterns (one per line) from a file"), cl::value_desc("regex file"), cl::init(""));

static cl::opt<bool> CaseInsensitive("i", cl::desc("Ignore case distinctions in the pattern and the file."));

static cl::opt<int> editDistance("edit-dist", cl::desc("Edit Distance Value"), cl::init(2));
static cl::opt<int> optPosition("opt-pos", cl::desc("Optimize position"), cl::init(8));
static cl::opt<int> stepSize("step-size", cl::desc("Step Size"), cl::init(3));

using namespace kernel;
using namespace pablo;

struct matchPosition
{
    size_t pos;
    size_t dist;
};

std::vector<struct matchPosition> matchList;

void sort_match_list(){
    
}

void run_second_filter(int total_len, int pattern_segs, float errRate){
    
    if(matchList.size() == 0) return;

    //Sort match position
    bool exchanged = true;
    while(exchanged){
        exchanged = false;
        for (unsigned i=0; i<matchList.size()-1; i++){
            if(matchList[i].pos > matchList[i+1].pos){
                size_t tmp_pos = matchList[i].pos;
                size_t tmp_dist = matchList[i].dist;
                matchList[i].pos = matchList[i+1].pos;
                matchList[i].dist = matchList[i+1].dist;
                matchList[i+1].pos = tmp_pos;
                matchList[i+1].dist = tmp_dist;
                exchanged = true;
            }
        }
    }

    std::cerr << "pattern_segs = " << pattern_segs << ", total_len = " << total_len << std::endl;

    int v = pattern_segs * (editDistance+1) - total_len * errRate;

    int startPos = matchList[0].pos;
    int sum = matchList[0].dist;
    int curIdx = 0;
    unsigned i = 0;
    int count = 0;
    while (i < matchList.size()){
        if(matchList[i].pos - startPos < total_len * (errRate+1)){
            sum += matchList[i].dist;
            i++;
        }
        else{
            if(sum > v) count++;
            sum -= matchList[curIdx].dist;
            curIdx++;
            startPos = matchList[curIdx].pos;
        }
    }
    std::cout << "matching value is " << v << std::endl;
    std::cout << "total candidate from the first filter is " << matchList.size() << std::endl;
    std::cout << "total candidate from the second filter is " << count << std::endl;
}

extern "C" {
void wrapped_report_pos(size_t match_pos, int dist) {
        struct matchPosition curMatch;
        curMatch.pos = match_pos;
        curMatch.dist = dist;
        matchList.push_back(curMatch);
        std::cout << "pos: " << match_pos << ", dist:" << dist << "\n";
    }

}

void icgrep_Linking(Module * m, ExecutionEngine * e) {
    Module::FunctionListType & fns = m->getFunctionList();
    for (Module::FunctionListType::iterator it = fns.begin(), it_end = fns.end(); it != it_end; ++it) {
        std::string fnName = it->getName().str();
        if (fnName == "wrapped_report_pos") {
            e->addGlobalMapping(cast<GlobalValue>(it), (void *)&wrapped_report_pos);
        }
    }
}

void get_editd_pattern(int & pattern_segs, int & total_len) {
  
    if (PatternFilename != "") {
        std::ifstream pattFile(PatternFilename.c_str());
        std::string r;
        if (pattFile.is_open()) {
            while (std::getline(pattFile, r)) {
                pattVector.push_back(r);
                pattern_segs ++; 
                total_len += r.size(); 
            }
            pattFile.close();
        }
    }
    
    // if there are no regexes specified through -e or -f, the first positional argument
    // must be a regex, not an input file.
    
    if (pattVector.size() == 0) {
        pattVector.push_back(inputFiles[0]);
        inputFiles.erase(inputFiles.begin());
    }
}

Function * editdPipeline(Module * mMod, IDISA::IDISA_Builder * iBuilder, pablo::PabloFunction * function) {
    Type * mBitBlockType = iBuilder->getBitBlockType();
    
    ExternalFileBuffer ChStream(iBuilder, StreamSetType(4, i1));
    SingleBlockBuffer MatchResults(iBuilder, StreamSetType(editDistance+1, i1));

    pablo_function_passes(function);
    pablo::PabloKernel  editdk(iBuilder, "editd", function, {});
    kernel::editdScanKernel editdScanK(iBuilder, editDistance);
    
    std::unique_ptr<Module> editdM = editdk.createKernelModule({&ChStream}, {&MatchResults});
    std::unique_ptr<Module> scanM = editdScanK.createKernelModule({&MatchResults}, {});                
    
    editdk.addKernelDeclarations(mMod);
    editdScanK.addKernelDeclarations(mMod);

    Type * const size_ty = iBuilder->getSizeTy();
    Type * const voidTy = Type::getVoidTy(mMod->getContext());
    Type * const inputType = PointerType::get(ArrayType::get(ArrayType::get(mBitBlockType, 8), 1), 0);
    
    Function * const main = cast<Function>(mMod->getOrInsertFunction("Main", voidTy, inputType, size_ty, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");
    
    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", main,0));

    ChStream.setStreamSetBuffer(inputStream, fileSize);
    MatchResults.allocateBuffer();
    
    Value * editdInstance = editdk.createInstance({});
    Value * scanMatchInstance = editdScanK.createInstance({});
   
    generatePipelineLoop(iBuilder, {&editdk, &editdScanK}, {editdInstance, scanMatchInstance}, fileSize);
        
    iBuilder->CreateRetVoid();
    
    Linker L(*mMod);
    L.linkInModule(std::move(editdM));
    L.linkInModule(std::move(scanM));
    
    return main;
}

Function * preprocessPipeline(Module * mMod, IDISA::IDISA_Builder * iBuilder, pablo::PabloFunction * function) {
    Type * mBitBlockType = iBuilder->getBitBlockType();
    
    ExternalFileBuffer ByteStream(iBuilder, StreamSetType(1, i8));
    SingleBlockBuffer BasisBits(iBuilder, StreamSetType(8, i1));
    ExternalFileBuffer CCResults(iBuilder, StreamSetType(4, i1));

    s2pKernel  s2pk(iBuilder);
    std::unique_ptr<Module> s2pM = s2pk.createKernelModule({&ByteStream}, {&BasisBits});

    pablo_function_passes(function);
    pablo::PabloKernel  ccck(iBuilder, "ccc", function, {});
    
    std::unique_ptr<Module> cccM = ccck.createKernelModule({&BasisBits}, {&CCResults});
    
    s2pk.addKernelDeclarations(mMod);
    ccck.addKernelDeclarations(mMod);

    Type * const size_ty = iBuilder->getSizeTy();
    Type * const voidTy = Type::getVoidTy(mMod->getContext());
    Type * const inputType = PointerType::get(ArrayType::get(ArrayType::get(mBitBlockType, 8), 1), 0);
    Type * const outputType = PointerType::get(ArrayType::get(mBitBlockType, 4), 0);
    
    Function * const main = cast<Function>(mMod->getOrInsertFunction("Main", voidTy, inputType, size_ty, outputType, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");
    Value * const outputStream = &*(args++);
    outputStream->setName("output");
    
    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", main,0));

    ByteStream.setStreamSetBuffer(inputStream, fileSize);
    BasisBits.allocateBuffer();
    CCResults.setStreamSetBuffer(outputStream, fileSize);
    
    Value * s2pInstance = s2pk.createInstance({});
    Value * cccInstance = ccck.createInstance({});
    
    generatePipelineLoop(iBuilder, {&s2pk, &ccck}, {s2pInstance, cccInstance}, fileSize);
        
    iBuilder->CreateRetVoid();
    
    Linker L(*mMod);
    L.linkInModule(std::move(s2pM));
    L.linkInModule(std::move(cccM));
    
    return main;
}


typedef void (*preprocessFunctionType)(char * byte_data, size_t filesize, char * output_data);
static ExecutionEngine * preprocessEngine = nullptr;

preprocessFunctionType preprocessCodeGen() {
                            
    LLVMContext TheContext;
    Module * M = new Module("preprocess", TheContext);
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);

    PabloFunction * function = PabloFunction::Create("preprocess", 8, 4);
    cc::CC_Compiler ccc(*function);
    pablo::PabloBuilder pBuilder(ccc.getBuilder().getPabloBlock(), ccc.getBuilder());

    pablo::PabloAST * A = ccc.compileCC(re::makeCC(re::makeCC(0x41), re::makeCC(0x61)));
    pablo::PabloAST * C = ccc.compileCC(re::makeCC(re::makeCC(0x43), re::makeCC(0x63)));
    pablo::PabloAST * T = ccc.compileCC(re::makeCC(re::makeCC(0x54), re::makeCC(0x74)));
    pablo::PabloAST * G = ccc.compileCC(re::makeCC(re::makeCC(0x47), re::makeCC(0x67)));

    function->setResult(0, pBuilder.createAssign("A", A));
    function->setResult(1, pBuilder.createAssign("C", C));
    function->setResult(2, pBuilder.createAssign("T", T));
    function->setResult(3, pBuilder.createAssign("G", G));

    llvm::Function * main_IR = preprocessPipeline(M, idb, function);

    preprocessEngine = JIT_to_ExecutionEngine(M);
    
    preprocessEngine->finalizeObject();

    delete idb;
    return reinterpret_cast<preprocessFunctionType>(preprocessEngine->getPointerToFunction(main_IR));
}

typedef void (*editdFunctionType)(char * byte_data, size_t filesize);
static ExecutionEngine * editdEngine = nullptr;

editdFunctionType editdCodeGen() {
                            
    LLVMContext TheContext;
    Module * M = new Module("editd", TheContext);
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);

    PabloFunction * function = PabloFunction::Create("editd", 4, editDistance+1);
    pablo::PabloBuilder main (function->getEntryBlock());

    const PabloType * streamType = getPabloType(PabloType::Stream, 1);

    std::vector<pablo::Var *>   basisBits(4);
    function->setParameter(0, basisBits[0] = main.createVar("PatA", streamType));
    function->setParameter(1, basisBits[1] = main.createVar("PatC", streamType));
    function->setParameter(2, basisBits[2] = main.createVar("PatT", streamType));
    function->setParameter(3, basisBits[3] = main.createVar("PatG", streamType));

    re::Pattern_Compiler pattern_compiler(*function);
    pattern_compiler.compile(pattVector, main, basisBits, editDistance, optPosition, stepSize);

    llvm::Function * main_IR = editdPipeline(M, idb, function);

    editdEngine = JIT_to_ExecutionEngine(M);
    
    editdEngine->finalizeObject();

    delete idb;
    return reinterpret_cast<editdFunctionType>(editdEngine->getPointerToFunction(main_IR));
}

char * preprocess(preprocessFunctionType fn_ptr, int & size) {
    std::string fileName = inputFiles[0];
    size_t fileSize;
    char * fileBuffer;
    
    const boost::filesystem::path file(fileName);
    if (exists(file)) {
        if (is_directory(file)) {
            exit(0);
        }
    } else {
        std::cerr << "Error: cannot open " << fileName << " for processing. Skipped.\n";
        exit(0);
    }
    
    fileSize = file_size(file);
    boost::iostreams::mapped_file_source mappedFile;
    if (fileSize == 0) {
        fileBuffer = nullptr;
    }
    else {
        try {
            mappedFile.open(fileName);
        } catch (std::exception &e) {
            std::cerr << "Error: Boost mmap of " << fileName << ": " << e.what() << std::endl;
            exit(0);
        }
        fileBuffer = const_cast<char *>(mappedFile.data());
    }
    char * chStream = (char *) aligned_alloc(32, fileSize);
    fn_ptr(fileBuffer, fileSize, chStream);
    size = fileSize;

    mappedFile.close();

    return chStream;
    
}

void editd(editdFunctionType fn_ptr, char * chStream, int size) {
 
    if (size == 0) {
        chStream = nullptr;
    }

    fn_ptr(chStream, size);
    
}

int main(int argc, char *argv[]) {

    cl::ParseCommandLineOptions(argc, argv);

    int pattern_segs = 0;
    int total_len = 0;

    get_editd_pattern(pattern_segs, total_len);
 
    preprocessFunctionType preprocess_ptr = preprocessCodeGen();
    int size = 0;
    char * chStream = preprocess(preprocess_ptr, size);
    
    editdFunctionType editd_ptr = editdCodeGen();
    editd(editd_ptr, chStream, size);

    if(pattVector.size()>1)
        run_second_filter(pattern_segs, total_len, 0.15);


    return 0;
}















