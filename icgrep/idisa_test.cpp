/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <vector>
#include <string>
#include <toolchain/toolchain.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/raw_ostream.h>
#include <kernels/kernel_builder.h>
#include <IR_Gen/idisa_target.h>
#include <kernels/interface.h>
#include <kernels/streamset.h>
#include <kernels/source_kernel.h>
#include <kernels/hex_convert.h>
#include <kernels/s2p_kernel.h>
#include <kernels/stdout_kernel.h>
#include <toolchain/toolchain.h>
#include <toolchain/cpudriver.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

using namespace llvm;

static cl::OptionCategory testFlags("Command Flags", "test options");

static cl::opt<std::string> TestOperation(cl::Positional, cl::desc("Operation to test"), cl::Required, cl::cat(testFlags));

static cl::opt<int> TestFieldWidth(cl::Positional, cl::desc("Test field width (default 64)."), cl::init(64), cl::Required, cl::cat(testFlags));

static cl::opt<std::string> Operand1TestFile(cl::Positional, cl::desc("Operand 1 data file."), cl::Required, cl::cat(testFlags));
static cl::opt<std::string> Operand2TestFile(cl::Positional, cl::desc("Operand 1 data file."), cl::Required, cl::cat(testFlags));
static cl::opt<std::string> TestOutputFile(cl::Positional, cl::desc("Test output file."), cl::cat(testFlags));

class IdisaBinaryOpTestKernel : public kernel::MultiBlockKernel {
public:
    IdisaBinaryOpTestKernel(const std::unique_ptr<kernel::KernelBuilder> & b, std::string idisa_op, unsigned fw);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateMultiBlockLogic(const std::unique_ptr<kernel::KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
private:
    const std::string mIdisaOperation;
    const unsigned mTestFw;
};

IdisaBinaryOpTestKernel::IdisaBinaryOpTestKernel(const std::unique_ptr<kernel::KernelBuilder> & b, std::string idisa_op, unsigned fw)
: kernel::MultiBlockKernel(idisa_op + std::to_string(fw) + "_test",
     {kernel::Binding{b->getStreamSetTy(1, 1), "operand1"}, kernel::Binding{b->getStreamSetTy(1, 1), "operand2"}},
     {kernel::Binding{b->getStreamSetTy(1, 1), "result"}},
     {}, {}, {}),
mIdisaOperation(idisa_op), mTestFw(fw) {}

void IdisaBinaryOpTestKernel::generateMultiBlockLogic(const std::unique_ptr<kernel::KernelBuilder> & kb, llvm::Value * const numOfBlocks) {
    BasicBlock * entry = kb->GetInsertBlock();
    BasicBlock * processBlock = kb->CreateBasicBlock("processBlock");
    BasicBlock * done = kb->CreateBasicBlock("done");
    Constant * const ZeroConst = kb->getSize(0);
    kb->CreateBr(processBlock);
    kb->SetInsertPoint(processBlock);
    PHINode * blockOffsetPhi = kb->CreatePHI(kb->getSizeTy(), 2);
    blockOffsetPhi->addIncoming(ZeroConst, entry);
    Value * operand1 = kb->fwCast(mTestFw, kb->loadInputStreamBlock("operand1", ZeroConst, blockOffsetPhi));
    Value * operand2 = kb->fwCast(mTestFw, kb->loadInputStreamBlock("operand2", ZeroConst, blockOffsetPhi));
    Value * result = nullptr;
    if (mIdisaOperation == "simd_add") {
        result = kb->simd_add(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_sub") {
        result = kb->simd_sub(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_mult") {
        result = kb->simd_mult(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_eq") {
        result = kb->simd_eq(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_gt") {
        result = kb->simd_gt(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_ugt") {
        result = kb->simd_ugt(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_lt") {
        result = kb->simd_lt(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_ult") {
        result = kb->simd_ult(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_max") {
        result = kb->simd_max(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_min") {
        result = kb->simd_min(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_umax") {
        result = kb->simd_umax(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_umin") {
        result = kb->simd_umin(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_sllv") {
        result = kb->simd_sllv(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_srlv") {
        result = kb->simd_srlv(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_pext") {
        result = kb->simd_pext(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "simd_pdep") {
        result = kb->simd_pdep(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "hsimd_packh") {
        result = kb->hsimd_packh(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "hsimd_packl") {
        result = kb->hsimd_packl(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "esimd_mergeh") {
        result = kb->esimd_mergeh(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "esimd_mergel") {
        result = kb->esimd_mergel(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "mvmd_shuffle") {
        result = kb->mvmd_shuffle(mTestFw, operand1, operand2);
    } else if (mIdisaOperation == "mvmd_compress") {
        result = kb->mvmd_compress(mTestFw, operand1, operand2);
    } else {
        llvm::report_fatal_error("Binary operation " + mIdisaOperation + " is unknown to the IdisaBinaryOpTestKernel kernel.");
    }
    kb->storeOutputStreamBlock("result", ZeroConst, blockOffsetPhi, kb->bitCast(result));
    Value * nextBlk = kb->CreateAdd(blockOffsetPhi, kb->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, processBlock);
    Value * moreToDo = kb->CreateICmpNE(nextBlk, numOfBlocks);
    kb->CreateCondBr(moreToDo, processBlock, done);
    kb->SetInsertPoint(done);
}

// Open a file and return its file desciptor.
int32_t openFile(const std::string & fileName, llvm::raw_ostream & msgstrm) {
    if (fileName == "-") {
        return STDIN_FILENO;
    }
    else {
        struct stat sb;
        int32_t fileDescriptor = open(fileName.c_str(), O_RDONLY);
        if (LLVM_UNLIKELY(fileDescriptor == -1)) {
            if (errno == EACCES) {
                msgstrm << "idisa_test: " << fileName << ": Permission denied.\n";
            }
            else if (errno == ENOENT) {
                msgstrm << "idisa_test: " << fileName << ": No such file.\n";
            }
            else {
                msgstrm << "idisa_test: " << fileName << ": Failed.\n";
            }
            return fileDescriptor;
        }
        if (stat(fileName.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
            msgstrm << "idisa_test: " << fileName << ": Is a directory.\n";
            close(fileDescriptor);
            return -1;
        }
        return fileDescriptor;
    }
}

typedef void (*IDISAtestFunctionType)(int32_t fd1, int32_t fd2);

using namespace parabix;

void pipelineGen(ParabixDriver & pxDriver) {

    auto & idb = pxDriver.getBuilder();
    Module * m = idb->getModule();
    Value * useMMap = idb->CreateZExt(idb->getTrue(), idb->getInt8Ty());
    const auto bufferSize = codegen::SegmentSize * codegen::BufferSegments;
    
    Type * const int32Ty = idb->getInt32Ty();
    Type * const voidTy = idb->getVoidTy();

    FunctionType * const mainType = FunctionType::get(voidTy, {int32Ty, int32Ty}, false);
    Function * const main = cast<Function>(m->getOrInsertFunction("Main", mainType));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();    
    Value * const fileDecriptor1 = &*(args++);
    fileDecriptor1->setName("operand1FileDecriptor");
    Value * const fileDecriptor2 = &*(args++);
    fileDecriptor2->setName("operand2FileDecriptor");

    idb->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", main,0));

    StreamSetBuffer * Operand1HexStream = pxDriver.addBuffer<ExternalBuffer>(idb, idb->getStreamSetTy(1, 8));
    kernel::Kernel * sourceK1 = pxDriver.addKernelInstance<kernel::FDSourceKernel>(idb);
    sourceK1->setInitialArguments({useMMap, fileDecriptor1});
    pxDriver.makeKernelCall(sourceK1, {}, {Operand1HexStream});
    
    StreamSetBuffer * Operand1BitStream = pxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), bufferSize);
    kernel::Kernel * hexbinK = pxDriver.addKernelInstance<kernel::HexToBinary>(idb);
    pxDriver.makeKernelCall(hexbinK, {Operand1HexStream}, {Operand1BitStream});
    
    StreamSetBuffer * Operand2HexStream = pxDriver.addBuffer<ExternalBuffer>(idb, idb->getStreamSetTy(1, 8));
    kernel::Kernel * sourceK2 = pxDriver.addKernelInstance<kernel::FDSourceKernel>(idb);
    sourceK2->setInitialArguments({useMMap, fileDecriptor2});
    pxDriver.makeKernelCall(sourceK2, {}, {Operand2HexStream});
    
    StreamSetBuffer * Operand2BitStream = pxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), bufferSize);
    kernel::Kernel * hexbinK2 = pxDriver.addKernelInstance<kernel::HexToBinary>(idb);
    pxDriver.makeKernelCall(hexbinK2, {Operand2HexStream}, {Operand2BitStream});

    StreamSetBuffer * ResultBitStream = pxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 1), bufferSize);
    kernel::Kernel * testK = pxDriver.addKernelInstance<IdisaBinaryOpTestKernel>(idb, TestOperation, TestFieldWidth);
    pxDriver.makeKernelCall(testK, {Operand1BitStream, Operand2BitStream}, {ResultBitStream});
    
    StreamSetBuffer * ResultHexStream = pxDriver.addBuffer<StaticBuffer>(idb, idb->getStreamSetTy(1, 8), bufferSize);
    kernel::Kernel * binhexK = pxDriver.addKernelInstance<kernel::BinaryToHex>(idb);
    pxDriver.makeKernelCall(binhexK, {ResultBitStream}, {ResultHexStream});
    
    kernel::Kernel * outK = nullptr;
    if (TestOutputFile.empty()) {
        outK = pxDriver.addKernelInstance<kernel::StdOutKernel>(idb, 8);
    } else {
        outK = pxDriver.addKernelInstance<kernel::FileSink>(idb, 8);
        Value * fName = idb->CreatePointerCast(idb->GetString(TestOutputFile.c_str()), idb->getInt8PtrTy());
        outK->setInitialArguments({fName});
    }
    pxDriver.makeKernelCall(outK, {ResultHexStream}, {});
    
    pxDriver.generatePipelineIR();
    pxDriver.deallocateBuffers();
    idb->CreateRetVoid();
    pxDriver.finalizeObject();
}

int main(int argc, char *argv[]) {
    cl::ParseCommandLineOptions(argc, argv);
    //codegen::SegmentSize = 1;
    ParabixDriver pxDriver("idisa_test");
    pipelineGen(pxDriver);
    
    int32_t fd1 = openFile(Operand1TestFile, llvm::outs());
    int32_t fd2 = openFile(Operand2TestFile, llvm::outs());
    
    auto idisaTestFunction = reinterpret_cast<IDISAtestFunctionType>(pxDriver.getMain());
    idisaTestFunction(fd1, fd2);
    close(fd1);
    close(fd2);
    return 0;
}
