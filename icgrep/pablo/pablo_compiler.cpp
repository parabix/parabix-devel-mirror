/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pablo_compiler.h>
#include <pablo/codegenstate.h>
#include <pablo/printer_pablos.h>
#include <cc/cc_namemap.hpp>
#include <re/re_name.h>
#include <stdexcept>
#include <include/simd-lib/bitblock.hpp>

//#define DUMP_GENERATED_IR
//#define DUMP_OPTIMIZED_IR

extern "C" {
  void wrapped_print_register(BitBlock bit_block) {
      print_register<BitBlock>("", bit_block);
  }
}

#define CREATE_GENERAL_CODE_CATEGORY(SUFFIX) \
SUFFIX * f##SUFFIX = nullptr; \
extern "C" { \
    BitBlock __get_category_##SUFFIX(Basis_bits &basis_bits) { \
        if (f##SUFFIX == nullptr) f##SUFFIX = new SUFFIX(); \
        Struct_##SUFFIX output; \
        f##SUFFIX->do_block(basis_bits, output); \
        return output.cc; \
    } \
}

CREATE_GENERAL_CODE_CATEGORY(Cc)
CREATE_GENERAL_CODE_CATEGORY(Cf)
CREATE_GENERAL_CODE_CATEGORY(Cn)
CREATE_GENERAL_CODE_CATEGORY(Co)
CREATE_GENERAL_CODE_CATEGORY(Cs)
CREATE_GENERAL_CODE_CATEGORY(Ll)
CREATE_GENERAL_CODE_CATEGORY(Lm)
CREATE_GENERAL_CODE_CATEGORY(Lo)
CREATE_GENERAL_CODE_CATEGORY(Lt)
CREATE_GENERAL_CODE_CATEGORY(Lu)
CREATE_GENERAL_CODE_CATEGORY(Mc)
CREATE_GENERAL_CODE_CATEGORY(Me)
CREATE_GENERAL_CODE_CATEGORY(Mn)
CREATE_GENERAL_CODE_CATEGORY(Nd)
CREATE_GENERAL_CODE_CATEGORY(Nl)
CREATE_GENERAL_CODE_CATEGORY(No)
CREATE_GENERAL_CODE_CATEGORY(Pc)
CREATE_GENERAL_CODE_CATEGORY(Pd)
CREATE_GENERAL_CODE_CATEGORY(Pe)
CREATE_GENERAL_CODE_CATEGORY(Pf)
CREATE_GENERAL_CODE_CATEGORY(Pi)
CREATE_GENERAL_CODE_CATEGORY(Po)
CREATE_GENERAL_CODE_CATEGORY(Ps)
CREATE_GENERAL_CODE_CATEGORY(Sc)
CREATE_GENERAL_CODE_CATEGORY(Sk)
CREATE_GENERAL_CODE_CATEGORY(Sm)
CREATE_GENERAL_CODE_CATEGORY(So)
CREATE_GENERAL_CODE_CATEGORY(Zl)
CREATE_GENERAL_CODE_CATEGORY(Zp)
CREATE_GENERAL_CODE_CATEGORY(Zs)

#undef CREATE_GENERAL_CODE_CATEGORY

namespace pablo {

PabloCompiler::PabloCompiler(const cc::CC_NameMap & nameMap, const BasisBitVars & basisBitVars, int bits)
: mBits(bits)
, mBasisBitVars(basisBitVars)
, mMod(new Module("icgrep", getGlobalContext()))
, mBasicBlock(nullptr)
, mExecutionEngine(nullptr)
, mXi64Vect(VectorType::get(IntegerType::get(mMod->getContext(), 64), BLOCK_SIZE / 64))
, mXi128Vect(VectorType::get(IntegerType::get(mMod->getContext(), 128), BLOCK_SIZE / 128))
, mBasisBitsInputPtr(nullptr)
, mCarryQueueIdx(0)
, mCarryQueuePtr(nullptr)
, mNestingDepth(0)
, mCarryQueueSize(0)
, mZeroInitializer(ConstantAggregateZero::get(mXi64Vect))
, mOneInitializer(ConstantVector::getAllOnesValue(mXi64Vect))
, mFunctionType(nullptr)
, mFunc_process_block(nullptr)
, mBasisBitsAddr(nullptr)
, mOutputAddrPtr(nullptr)
, mNameMap(nameMap)
{
    //Create the jit execution engine.up
    InitializeNativeTarget();
    std::string ErrStr;
    mExecutionEngine = EngineBuilder(mMod).setUseMCJIT(true).setErrorStr(&ErrStr).setOptLevel(CodeGenOpt::Level::Less).create();
    if (mExecutionEngine == nullptr) {
        throw std::runtime_error("Could not create ExecutionEngine: " + ErrStr);
    }

    InitializeNativeTargetAsmPrinter();
    InitializeNativeTargetAsmParser();

    DefineTypes();
    DeclareFunctions();
}

PabloCompiler::~PabloCompiler()
{
    delete mMod;
    delete fPs;
    delete fNl;
    delete fNo;
    delete fLo;
    delete fLl;
    delete fLm;
    delete fNd;
    delete fPc;
    delete fLt;
    delete fLu;
    delete fPf;
    delete fPd;
    delete fPe;
    delete fPi;
    delete fPo;
    delete fMe;
    delete fMc;
    delete fMn;
    delete fSk;
    delete fSo;
    delete fSm;
    delete fSc;
    delete fZl;
    delete fCo;
    delete fCn;
    delete fCc;
    delete fCf;
    delete fCs;
    delete fZp;
    delete fZs;

}

LLVM_Gen_RetVal PabloCompiler::compile(PabloBlock & pb)
{
    mCarryQueueSize = 0;
    DeclareCallFunctions(pb.expressions());
    mCarryQueueVector.resize(mCarryQueueSize);

    Function::arg_iterator args = mFunc_process_block->arg_begin();
    mBasisBitsAddr = args++;
    mBasisBitsAddr->setName("basis_bits");
    mCarryQueuePtr = args++;
    mCarryQueuePtr->setName("carry_q");
    mOutputAddrPtr = args++;
    mOutputAddrPtr->setName("output");

    //Create the carry queue.
    mCarryQueueIdx = 0;
    mNestingDepth = 0;
    mBasicBlock = BasicBlock::Create(mMod->getContext(), "parabix_entry", mFunc_process_block,0);

    //The basis bits structure
    for (unsigned i = 0; i < mBits; ++i) {
        IRBuilder<> b(mBasicBlock);
        Value* indices[] = {b.getInt64(0), b.getInt32(i)};
        const String * const name = mBasisBitVars[i]->getName();
        mMarkerMap.insert(std::make_pair(name, b.CreateGEP(mBasisBitsAddr, indices, name->str())));
    }

    //Generate the IR instructions for the function.
    Value * result = compileStatements(pb.expressions());
    SetReturnMarker(result, 0); // matches

    const String * lf = pb.createVar(mNameMap["LineFeed"]->getName())->getName();

    SetReturnMarker(GetMarker(lf), 1); // line feeds

    assert (mCarryQueueIdx == mCarryQueueSize);
    assert (mNestingDepth == 0);
    //Terminate the block
    ReturnInst::Create(mMod->getContext(), mBasicBlock);

    //Un-comment this line in order to display the IR that has been generated by this module.
    #ifdef DUMP_GENERATED_IR
    mMod->dump();
    #endif

    //Create a verifier.  The verifier will print an error message if our module is malformed in any way.
    #ifdef USE_LLVM_3_5
    verifyModule(*mMod, &dbgs());
    #endif
    #ifdef USE_LLVM_3_4
    verifyModule(*mMod, PrintMessageAction);
    #endif

    //Use the pass manager to run optimizations on the function.
    FunctionPassManager fpm(mMod);

#ifdef USE_LLVM_3_5
    mMod->setDataLayout(mExecutionEngine->getDataLayout());
    // Set up the optimizer pipeline.  Start with registering info about how the target lays out data structures.
    fpm.add(new DataLayoutPass(mMod));
#endif

#ifdef USE_LLVM_3_4
    fpm.add(new DataLayout(*mExecutionEngine->getDataLayout()));
#endif

    fpm.add(createPromoteMemoryToRegisterPass()); //Transform to SSA form.
    fpm.add(createBasicAliasAnalysisPass());      //Provide basic AliasAnalysis support for GVN. (Global Value Numbering)
    fpm.add(createCFGSimplificationPass());       //Simplify the control flow graph.
    fpm.add(createInstructionCombiningPass());    //Simple peephole optimizations and bit-twiddling.
    fpm.add(createReassociatePass());             //Reassociate expressions.
    fpm.add(createGVNPass());                     //Eliminate common subexpressions.

    fpm.doInitialization();

    fpm.run(*mFunc_process_block);

#ifdef DUMP_OPTIMIZED_IR
    mMod->dump();
#endif
    mExecutionEngine->finalizeObject();

    LLVM_Gen_RetVal retVal;
    //Return the required size of the carry queue and a pointer to the process_block function.
    retVal.carry_q_size = mCarryQueueSize;
    retVal.process_block_fptr = mExecutionEngine->getPointerToFunction(mFunc_process_block);

    return retVal;
}

void PabloCompiler::DefineTypes()
{
    StructType * structBasisBits = mMod->getTypeByName("struct.Basis_bits");
    if (structBasisBits == nullptr) {
        structBasisBits = StructType::create(mMod->getContext(), "struct.Basis_bits");
    }
    std::vector<Type*>StructTy_struct_Basis_bits_fields;
    for (int i = 0; i < mBits; i++)
    {
        StructTy_struct_Basis_bits_fields.push_back(mXi64Vect);
    }
    if (structBasisBits->isOpaque()) {
        structBasisBits->setBody(StructTy_struct_Basis_bits_fields, /*isPacked=*/false);
    }
    mBasisBitsInputPtr = PointerType::get(structBasisBits, 0);

    std::vector<Type*>functionTypeArgs;
    functionTypeArgs.push_back(mBasisBitsInputPtr);

    //The carry q array.
    //A pointer to the BitBlock vector.
    functionTypeArgs.push_back(PointerType::get(mXi64Vect, 0));

    //The output structure.
    StructType * outputStruct = mMod->getTypeByName("struct.Output");
    if (!outputStruct) {
        outputStruct = StructType::create(mMod->getContext(), "struct.Output");
    }
    if (outputStruct->isOpaque()) {
        std::vector<Type*>fields;
        fields.push_back(mXi64Vect);
        fields.push_back(mXi64Vect);
        outputStruct->setBody(fields, /*isPacked=*/false);
    }
    PointerType* outputStructPtr = PointerType::get(outputStruct, 0);

    //The &output parameter.
    functionTypeArgs.push_back(outputStructPtr);

    mFunctionType = FunctionType::get(
     /*Result=*/Type::getVoidTy(mMod->getContext()),
     /*Params=*/functionTypeArgs,
     /*isVarArg=*/false);
}

void PabloCompiler::DeclareFunctions()
{
    //This function can be used for testing to print the contents of a register from JIT'd code to the terminal window.
    //mFunc_print_register = mMod->getOrInsertFunction("wrapped_print_register", Type::getVoidTy(getGlobalContext()), mXi64Vect, NULL);
    //mExecutionEngine->addGlobalMapping(cast<GlobalValue>(mFunc_print_register), (void *)&wrapped_print_register);
    // to call->  b.CreateCall(mFunc_print_register, unicode_category);

#ifdef USE_UADD_OVERFLOW
    // Type Definitions for llvm.uadd.with.overflow.carryin.i128 or .i256
    std::vector<Type*>StructTy_0_fields;
    StructTy_0_fields.push_back(IntegerType::get(mMod->getContext(), BLOCK_SIZE));
    StructTy_0_fields.push_back(IntegerType::get(mMod->getContext(), 1));
    StructType *StructTy_0 = StructType::get(mMod->getContext(), StructTy_0_fields, /*isPacked=*/false);

    std::vector<Type*>FuncTy_1_args;
    FuncTy_1_args.push_back(IntegerType::get(mMod->getContext(), BLOCK_SIZE));
    FuncTy_1_args.push_back(IntegerType::get(mMod->getContext(), BLOCK_SIZE));
    FuncTy_1_args.push_back(IntegerType::get(mMod->getContext(), 1));
    FunctionType* FuncTy_1 = FunctionType::get(
                                              /*Result=*/StructTy_0,
                                              /*Params=*/FuncTy_1_args,
                                              /*isVarArg=*/false);

    mFunc_llvm_uadd_with_overflow = mMod->getFunction("llvm.uadd.with.overflow.carryin.i"##BLOCK_SIZE);
    if (!mFunc_llvm_uadd_with_overflow) {
        mFunc_llvm_uadd_with_overflow = Function::Create(
          /*Type=*/ FuncTy_1,
          /*Linkage=*/ GlobalValue::ExternalLinkage,
          /*Name=*/ "llvm.uadd.with.overflow.carryin.i"##BLOCK_SIZE, mMod); // (external, no body)
        mFunc_llvm_uadd_with_overflow->setCallingConv(CallingConv::C);
    }
    AttributeSet mFunc_llvm_uadd_with_overflow_PAL;
    {
        SmallVector<AttributeSet, 4> Attrs;
        AttributeSet PAS;
        {
          AttrBuilder B;
          B.addAttribute(Attribute::NoUnwind);
          B.addAttribute(Attribute::ReadNone);
          PAS = AttributeSet::get(mMod->getContext(), ~0U, B);
        }

        Attrs.push_back(PAS);
        mFunc_llvm_uadd_with_overflow_PAL = AttributeSet::get(mMod->getContext(), Attrs);
    }
    mFunc_llvm_uadd_with_overflow->setAttributes(mFunc_llvm_uadd_with_overflow_PAL);
#endif

    //Starts on process_block
    SmallVector<AttributeSet, 4> Attrs;
    AttributeSet PAS;
    {
        AttrBuilder B;
        B.addAttribute(Attribute::ReadOnly);
        B.addAttribute(Attribute::NoCapture);
        PAS = AttributeSet::get(mMod->getContext(), 1U, B);
    }
    Attrs.push_back(PAS);
    {
        AttrBuilder B;
        B.addAttribute(Attribute::NoCapture);
        PAS = AttributeSet::get(mMod->getContext(), 2U, B);
    }
    Attrs.push_back(PAS);
    {
        AttrBuilder B;
        B.addAttribute(Attribute::NoCapture);
        PAS = AttributeSet::get(mMod->getContext(), 3U, B);
    }
    Attrs.push_back(PAS);
    {
        AttrBuilder B;
        B.addAttribute(Attribute::NoUnwind);
        B.addAttribute(Attribute::UWTable);
        PAS = AttributeSet::get(mMod->getContext(), ~0U, B);
    }
    AttributeSet AttrSet = AttributeSet::get(mMod->getContext(), Attrs);

    //Create the function that will be generated.
    mFunc_process_block = mMod->getFunction("process_block");
    if (!mFunc_process_block) {
        mFunc_process_block = Function::Create(
            /*Type=*/mFunctionType,
            /*Linkage=*/GlobalValue::ExternalLinkage,
            /*Name=*/"process_block", mMod);
        mFunc_process_block->setCallingConv(CallingConv::C);
    }
    mFunc_process_block->setAttributes(AttrSet);
}

void PabloCompiler::DeclareCallFunctions(const ExpressionList & stmts) {
    for (PabloAST * stmt : stmts) {
        if (const Assign * assign = dyn_cast<Assign>(stmt)) {
            DeclareCallFunctions(assign->getExpr());
        }
        if (const Next * next = dyn_cast<Next>(stmt)) {
            DeclareCallFunctions(next->getExpr());
        }
        else if (If * ifStatement = dyn_cast<If>(stmt)) {
            const auto preIfCarryCount = mCarryQueueSize;
            DeclareCallFunctions(ifStatement->getCondition());
            DeclareCallFunctions(ifStatement->getBody());
            ifStatement->setInclusiveCarryCount(mCarryQueueSize - preIfCarryCount);
        }
        else if (While * whileStatement = dyn_cast<While>(stmt)) {
            const auto preWhileCarryCount = mCarryQueueSize;
            DeclareCallFunctions(whileStatement->getCondition());
            DeclareCallFunctions(whileStatement->getBody());
            whileStatement->setInclusiveCarryCount(mCarryQueueSize - preWhileCarryCount);
        }
    }
}

void PabloCompiler::DeclareCallFunctions(const PabloAST * expr)
{
    if (const Call * call = dyn_cast<const Call>(expr)) {
        const String * const callee = call->getCallee();
        assert (callee);
        if (mCalleeMap.find(callee) == mCalleeMap.end()) {
            void * callee_ptr = nullptr;
            #define CHECK_GENERAL_CODE_CATEGORY(SUFFIX) \
                if (callee->str() == #SUFFIX) { \
                    callee_ptr = (void*)&__get_category_##SUFFIX; \
                } else
            CHECK_GENERAL_CODE_CATEGORY(Cc)
            CHECK_GENERAL_CODE_CATEGORY(Cf)
            CHECK_GENERAL_CODE_CATEGORY(Cn)
            CHECK_GENERAL_CODE_CATEGORY(Co)
            CHECK_GENERAL_CODE_CATEGORY(Cs)
            CHECK_GENERAL_CODE_CATEGORY(Ll)
            CHECK_GENERAL_CODE_CATEGORY(Lm)
            CHECK_GENERAL_CODE_CATEGORY(Lo)
            CHECK_GENERAL_CODE_CATEGORY(Lt)
            CHECK_GENERAL_CODE_CATEGORY(Lu)
            CHECK_GENERAL_CODE_CATEGORY(Mc)
            CHECK_GENERAL_CODE_CATEGORY(Me)
            CHECK_GENERAL_CODE_CATEGORY(Mn)
            CHECK_GENERAL_CODE_CATEGORY(Nd)
            CHECK_GENERAL_CODE_CATEGORY(Nl)
            CHECK_GENERAL_CODE_CATEGORY(No)
            CHECK_GENERAL_CODE_CATEGORY(Pc)
            CHECK_GENERAL_CODE_CATEGORY(Pd)
            CHECK_GENERAL_CODE_CATEGORY(Pe)
            CHECK_GENERAL_CODE_CATEGORY(Pf)
            CHECK_GENERAL_CODE_CATEGORY(Pi)
            CHECK_GENERAL_CODE_CATEGORY(Po)
            CHECK_GENERAL_CODE_CATEGORY(Ps)
            CHECK_GENERAL_CODE_CATEGORY(Sc)
            CHECK_GENERAL_CODE_CATEGORY(Sk)
            CHECK_GENERAL_CODE_CATEGORY(Sm)
            CHECK_GENERAL_CODE_CATEGORY(So)
            CHECK_GENERAL_CODE_CATEGORY(Zl)
            CHECK_GENERAL_CODE_CATEGORY(Zp)
            CHECK_GENERAL_CODE_CATEGORY(Zs)
            // OTHERWISE ...
            throw std::runtime_error("Unknown unicode category \"" + callee->str() + "\"");
            #undef CHECK_GENERAL_CODE_CATEGORY
            Value * unicodeCategory = mMod->getOrInsertFunction("__get_category_" + callee->str(), mXi64Vect, mBasisBitsInputPtr, NULL);
            if (unicodeCategory == nullptr) {
                throw std::runtime_error("Could not create static method call for unicode category \"" + callee->str() + "\"");
            }
            mExecutionEngine->addGlobalMapping(cast<GlobalValue>(unicodeCategory), callee_ptr);
            mCalleeMap.insert(std::make_pair(callee, unicodeCategory));
        }
    }
    else if (const And * pablo_and = dyn_cast<const And>(expr))
    {
        DeclareCallFunctions(pablo_and->getExpr1());
        DeclareCallFunctions(pablo_and->getExpr2());
    }
    else if (const Or * pablo_or = dyn_cast<const Or>(expr))
    {
        DeclareCallFunctions(pablo_or->getExpr1());
        DeclareCallFunctions(pablo_or->getExpr2());
    }
    else if (const Sel * pablo_sel = dyn_cast<const Sel>(expr))
    {
        DeclareCallFunctions(pablo_sel->getCondition());
        DeclareCallFunctions(pablo_sel->getTrueExpr());
        DeclareCallFunctions(pablo_sel->getFalseExpr());
    }
    else if (const Not * pablo_not = dyn_cast<const Not>(expr))
    {
        DeclareCallFunctions(pablo_not->getExpr());
    }
    else if (const Advance * adv = dyn_cast<const Advance>(expr))
    {
        ++mCarryQueueSize;
        DeclareCallFunctions(adv->getExpr());
    }
    else if (const MatchStar * mstar = dyn_cast<const MatchStar>(expr))
    {
        ++mCarryQueueSize;
        DeclareCallFunctions(mstar->getMarker());
        DeclareCallFunctions(mstar->getCharClass());
    }
    else if (const ScanThru * sthru = dyn_cast<const ScanThru>(expr))
    {
        ++mCarryQueueSize;
        DeclareCallFunctions(sthru->getScanFrom());
        DeclareCallFunctions(sthru->getScanThru());
    }
}

inline Value* PabloCompiler::GetMarker(const String * name) {
    auto itr = mMarkerMap.find(name);
    if (itr == mMarkerMap.end()) {
        IRBuilder<> b(mBasicBlock);
        itr = mMarkerMap.insert(std::make_pair(name, b.CreateAlloca(mXi64Vect))).first;
    }
    return itr->second;
}

void PabloCompiler::SetReturnMarker(Value * marker, const unsigned index) {
    IRBuilder<> b(mBasicBlock);
    Value* marker_bitblock = b.CreateAlignedLoad(marker, BLOCK_SIZE/8, false);
    Value* output_indices[] = {b.getInt64(0), b.getInt32(index)};
    Value* output_struct_GEP = b.CreateGEP(mOutputAddrPtr, output_indices);
    b.CreateAlignedStore(marker_bitblock, output_struct_GEP, BLOCK_SIZE/8, false);
}


Value * PabloCompiler::compileStatements(const ExpressionList & stmts) {
    Value * retVal = nullptr;
    for (PabloAST * statement : stmts) {
        retVal = compileStatement(statement);
    }
    return retVal;
}

Value * PabloCompiler::compileStatement(const PabloAST * stmt)
{
    Value * retVal = nullptr;
    if (const Assign * assign = dyn_cast<const Assign>(stmt))
    {
        Value* expr = compileExpression(assign->getExpr());
        Value* marker = GetMarker(assign->getName());
        IRBuilder<> b(mBasicBlock);
        b.CreateAlignedStore(expr, marker, BLOCK_SIZE/8, false);
        retVal = marker;
    }
    if (const Next * next = dyn_cast<const Next>(stmt))
    {
        IRBuilder<> b(mBasicBlock);
        auto f = mMarkerMap.find(next->getName());
        assert (f != mMarkerMap.end());
        Value * marker = f->second;
        Value * expr = compileExpression(next->getExpr());
        b.CreateAlignedStore(expr, marker, BLOCK_SIZE/8, false);
        retVal = marker;
    }
    else if (const If * ifstmt = dyn_cast<const If>(stmt))
    {
        BasicBlock * ifEntryBlock = mBasicBlock;
        BasicBlock * ifBodyBlock = BasicBlock::Create(mMod->getContext(), "if.body", mFunc_process_block, 0);
        BasicBlock * ifEndBlock = BasicBlock::Create(mMod->getContext(), "if.end", mFunc_process_block, 0);

        int if_start_idx = mCarryQueueIdx;

        Value* if_test_value = compileExpression(ifstmt->getCondition());

        /* Generate the statements into the if body block, and also determine the
           final carry index.  */

        IRBuilder<> bIfBody(ifBodyBlock);
        mBasicBlock = ifBodyBlock;

        ++mNestingDepth;

        Value *  returnMarker = compileStatements(ifstmt->getBody());

        int if_end_idx = mCarryQueueIdx;
        if (if_start_idx < if_end_idx + 1) {
            // Have at least two internal carries.   Accumulate and store.
            int if_accum_idx = mCarryQueueIdx++;

            Value* if_carry_accum_value = genCarryInLoad(if_start_idx);

            for (int c = if_start_idx+1; c < if_end_idx; c++)
            {
                Value* carryq_value = genCarryInLoad(c);
                if_carry_accum_value = bIfBody.CreateOr(carryq_value, if_carry_accum_value);
            }
            genCarryOutStore(if_carry_accum_value, if_accum_idx);

        }
        bIfBody.CreateBr(ifEndBlock);

        IRBuilder<> b_entry(ifEntryBlock);
        mBasicBlock = ifEntryBlock;
        if (if_start_idx < if_end_idx) {
            // Have at least one internal carry.
            int if_accum_idx = mCarryQueueIdx - 1;
            Value* last_if_pending_carries = genCarryInLoad(if_accum_idx);
            if_test_value = b_entry.CreateOr(if_test_value, last_if_pending_carries);
        }
        b_entry.CreateCondBr(genBitBlockAny(if_test_value), ifEndBlock, ifBodyBlock);

        mBasicBlock = ifEndBlock;
        --mNestingDepth;

        retVal = returnMarker;
    }
    else if (const While * whl = dyn_cast<const While>(stmt))
    {
        const auto baseCarryQueueIdx = mCarryQueueIdx;
        if (mNestingDepth == 0) {
            for (auto i = 0; i != whl->getInclusiveCarryCount(); ++i) {
                genCarryInLoad(baseCarryQueueIdx + i);
            }
        }        

        // First compile the initial iteration statements; the calls to genCarryOutStore will update the
        // mCarryQueueVector with the appropriate values. Although we're not actually entering a new basic
        // block yet, increment the nesting depth so that any calls to genCarryInLoad or genCarryOutStore
        // will refer to the previous value.
        ++mNestingDepth;
        compileStatements(whl->getBody());
        // Reset the carry queue index. Note: this ought to be changed in the future. Currently this assumes
        // that compiling the while body twice will generate the equivalent IR. This is not necessarily true
        // but works for now.
        mCarryQueueIdx = baseCarryQueueIdx;

        BasicBlock* whileCondBlock = BasicBlock::Create(mMod->getContext(), "while.cond", mFunc_process_block, 0);
        BasicBlock* whileBodyBlock = BasicBlock::Create(mMod->getContext(), "while.body", mFunc_process_block, 0);
        BasicBlock* whileEndBlock = BasicBlock::Create(mMod->getContext(), "while.end", mFunc_process_block, 0);

        // Note: compileStatements may update the mBasicBlock pointer if the body contains nested loops. Do not
        // assume it is the same one in which we entered the function with.
        IRBuilder<> bEntry(mBasicBlock);
        bEntry.CreateBr(whileCondBlock);

        // CONDITION BLOCK
        IRBuilder<> bCond(whileCondBlock);
        // generate phi nodes for any carry propogating instruction
        std::vector<PHINode*> carryQueuePhiNodes(whl->getInclusiveCarryCount());
        for (auto i = 0; i != whl->getInclusiveCarryCount(); ++i) {
            PHINode * phi = bCond.CreatePHI(mXi64Vect, 2);
            phi->addIncoming(mCarryQueueVector[baseCarryQueueIdx + i], mBasicBlock);
            mCarryQueueVector[baseCarryQueueIdx + i] = mZeroInitializer; // phi; // (use phi for multi-carry mode.)
            carryQueuePhiNodes[i] = phi;
        }

        mBasicBlock = whileCondBlock;
        bCond.CreateCondBr(genBitBlockAny(compileExpression(whl->getCondition())), whileEndBlock, whileBodyBlock);

        // BODY BLOCK
        mBasicBlock = whileBodyBlock;
        retVal = compileStatements(whl->getBody());
        // update phi nodes for any carry propogating instruction
        IRBuilder<> bWhileBody(mBasicBlock);
        for (auto i = 0; i != whl->getInclusiveCarryCount(); ++i) {
            Value * carryOut = bWhileBody.CreateOr(carryQueuePhiNodes[i], mCarryQueueVector[baseCarryQueueIdx + i]);
            carryQueuePhiNodes[i]->addIncoming(carryOut, mBasicBlock);
            mCarryQueueVector[baseCarryQueueIdx + i] = carryQueuePhiNodes[i];
        }

        bWhileBody.CreateBr(whileCondBlock);

        // EXIT BLOCK
        mBasicBlock = whileEndBlock;    
        if (--mNestingDepth == 0) {
            for (auto i = 0; i != whl->getInclusiveCarryCount(); ++i) {
                genCarryOutStore(carryQueuePhiNodes[i], baseCarryQueueIdx + i);
            }
        }
    }
    return retVal;
}

Value * PabloCompiler::compileExpression(const PabloAST * expr)
{
    Value * retVal = nullptr;
    IRBuilder<> b(mBasicBlock);
    if (isa<Ones>(expr)) {
        retVal = mOneInitializer;
    }
    else if (isa<Zeroes>(expr)) {
        retVal = mZeroInitializer;
    }
    else if (const Call* call = dyn_cast<Call>(expr)) {
        //Call the callee once and store the result in the marker map.
        auto mi = mMarkerMap.find(call->getCallee());
        if (mi == mMarkerMap.end()) {
            auto ci = mCalleeMap.find(call->getCallee());
            if (ci == mCalleeMap.end()) {
                throw std::runtime_error("Unexpected error locating static function for \"" + call->getCallee()->str() + "\"");
            }
            Value * unicode_category = b.CreateCall(ci->second, mBasisBitsAddr);
            Value * ptr = b.CreateAlloca(mXi64Vect);
            b.CreateAlignedStore(unicode_category, ptr, BLOCK_SIZE/8, false);
            mi = mMarkerMap.insert(std::make_pair(call->getCallee(), ptr)).first;
        }
        retVal = b.CreateAlignedLoad(mi->second, BLOCK_SIZE/8, false, call->getCallee()->str() );
    }
    else if (const Var * var = dyn_cast<Var>(expr))
    {
        auto f = mMarkerMap.find(var->getName());
        assert (f != mMarkerMap.end());
        retVal = b.CreateAlignedLoad(f->second, BLOCK_SIZE/8, false, var->getName()->str() );
    }
    else if (const And * pablo_and = dyn_cast<And>(expr))
    {
        retVal = b.CreateAnd(compileExpression(pablo_and->getExpr1()), compileExpression(pablo_and->getExpr2()), "and");
    }
    else if (const Or * pablo_or = dyn_cast<Or>(expr))
    {
        retVal = b.CreateOr(compileExpression(pablo_or->getExpr1()), compileExpression(pablo_or->getExpr2()), "or");
    }
    else if (const Sel * pablo_sel = dyn_cast<Sel>(expr))
    {
        Value* ifMask = compileExpression(pablo_sel->getCondition());
        Value* and_if_true_result = b.CreateAnd(ifMask, compileExpression(pablo_sel->getTrueExpr()));
        Value* and_if_false_result = b.CreateAnd(genNot(ifMask), compileExpression(pablo_sel->getFalseExpr()));
        retVal = b.CreateOr(and_if_true_result, and_if_false_result);
    }
    else if (const Not * pablo_not = dyn_cast<Not>(expr))
    {
        retVal = genNot(compileExpression(pablo_not->getExpr()));
    }
    else if (const Advance * adv = dyn_cast<Advance>(expr))
    {
        Value* strm_value = compileExpression(adv->getExpr());
        retVal = genAdvanceWithCarry(strm_value);
    }
    else if (const MatchStar * mstar = dyn_cast<MatchStar>(expr))
    {
        Value* marker = compileExpression(mstar->getMarker());
        Value* cc = compileExpression(mstar->getCharClass());
        Value* marker_and_cc = b.CreateAnd(marker, cc);
        retVal = b.CreateOr(b.CreateXor(genAddWithCarry(marker_and_cc, cc), cc), marker, "matchstar");
    }
    else if (const ScanThru * sthru = dyn_cast<ScanThru>(expr))
    {
        Value* marker_expr = compileExpression(sthru->getScanFrom());
        Value* cc_expr = compileExpression(sthru->getScanThru());
        retVal = b.CreateAnd(genAddWithCarry(marker_expr, cc_expr), genNot(cc_expr), "scanthru");
    }
    return retVal;
}

#ifdef USE_UADD_OVERFLOW
SumWithOverflowPack PabloCompiler::callUaddOverflow(Value* int128_e1, Value* int128_e2, Value* int1_cin) {
    std::vector<Value*> struct_res_params;
    struct_res_params.push_back(int128_e1);
    struct_res_params.push_back(int128_e2);
    struct_res_params.push_back(int1_cin);
    CallInst* struct_res = CallInst::Create(mFunc_llvm_uadd_with_overflow, struct_res_params, "uadd_overflow_res", mBasicBlock);
    struct_res->setCallingConv(CallingConv::C);
    struct_res->setTailCall(false);
    AttributeSet struct_res_PAL;
    struct_res->setAttributes(struct_res_PAL);

    SumWithOverflowPack ret;

    std::vector<unsigned> int128_sum_indices;
    int128_sum_indices.push_back(0);
    ret.sum = ExtractValueInst::Create(struct_res, int128_sum_indices, "sum", mBasicBlock);

    std::vector<unsigned> int1_obit_indices;
    int1_obit_indices.push_back(1);
    ret.obit = ExtractValueInst::Create(struct_res, int1_obit_indices, "obit", mBasicBlock);

    return ret;
}
#endif

Value* PabloCompiler::genAddWithCarry(Value* e1, Value* e2) {
    IRBuilder<> b(mBasicBlock);

    //CarryQ - carry in.
    const int carryIdx = mCarryQueueIdx++;
    Value* carryq_value = genCarryInLoad(carryIdx);

#ifdef USE_UADD_OVERFLOW
    //use llvm.uadd.with.overflow.i128 or i256
    ConstantInt* const_int32_6 = ConstantInt::get(mMod->getContext(), APInt(32, StringRef("0"), 10));
    CastInst* int128_e1 = new BitCastInst(e1, IntegerType::get(mMod->getContext(), BLOCK_SIZE), "e1_128", mBasicBlock);
    CastInst* int128_e2 = new BitCastInst(e2, IntegerType::get(mMod->getContext(), BLOCK_SIZE), "e2_128", mBasicBlock);
    ExtractElementInst * int64_carryq_value = ExtractElementInst::Create(carryq_value, const_int32_6, "carryq_64", mBasicBlock);
    CastInst* int1_carryq_value = new TruncInst(int64_carryq_value, IntegerType::get(mMod->getContext(), 1), "carryq_1", mBasicBlock);
    SumWithOverflowPack sumpack0;
    sumpack0 = callUaddOverflow(int128_e1, int128_e2, int1_carryq_value);
    Value* obit = sumpack0.obit;
    Value* sum = b.CreateBitCast(sumpack0.sum, mXi64Vect, "sum");
    /*obit is the i1 carryout, zero extend and insert it into a v2i64 or v4i64 vector.*/
    ConstantAggregateZero* const_packed_5 = ConstantAggregateZero::get(mXi64Vect);
    CastInst* int64_o0 = new ZExtInst(obit, IntegerType::get(mMod->getContext(), 64), "o0", mBasicBlock);
    InsertElementInst* carry_out = InsertElementInst::Create(const_packed_5, int64_o0, const_int32_6, "carry_out", mBasicBlock);
#else
    //calculate carry through logical ops
    Value* carrygen = b.CreateAnd(e1, e2, "carrygen");
    Value* carryprop = b.CreateOr(e1, e2, "carryprop");
    Value* digitsum = b.CreateAdd(e1, e2, "digitsum");
    Value* partial = b.CreateAdd(digitsum, carryq_value, "partial");
    Value* digitcarry = b.CreateOr(carrygen, b.CreateAnd(carryprop, genNot(partial)));
    Value* mid_carry_in = genShiftLeft64(b.CreateLShr(digitcarry, 63), "mid_carry_in");

    Value* sum = b.CreateAdd(partial, mid_carry_in, "sum");
    Value* carry_out = genShiftHighbitToLow(b.CreateOr(carrygen, b.CreateAnd(carryprop, genNot(sum))), "carry_out");
#endif
    genCarryOutStore(carry_out, carryIdx);
    return sum;
}

Value* PabloCompiler::genCarryInLoad(const unsigned index) {    
    assert (index < mCarryQueueVector.size());
    if (mNestingDepth == 0) {
        IRBuilder<> b(mBasicBlock);
        mCarryQueueVector[index] = b.CreateAlignedLoad(b.CreateGEP(mCarryQueuePtr, b.getInt64(index)), BLOCK_SIZE/8, false);
    }
    return mCarryQueueVector[index];
}

void PabloCompiler::genCarryOutStore(Value* carryOut, const unsigned index ) {
    assert (carryOut);
    assert (index < mCarryQueueVector.size());
    if (mNestingDepth == 0) {        
        IRBuilder<> b(mBasicBlock);
        b.CreateAlignedStore(carryOut, b.CreateGEP(mCarryQueuePtr, b.getInt64(index)), BLOCK_SIZE/8, false);
    }
    mCarryQueueVector[index] = carryOut;
}

inline Value* PabloCompiler::genBitBlockAny(Value* test) {
    IRBuilder<> b(mBasicBlock);
    Value* cast_marker_value_1 = b.CreateBitCast(test, IntegerType::get(mMod->getContext(), BLOCK_SIZE));
    return b.CreateICmpEQ(cast_marker_value_1, ConstantInt::get(IntegerType::get(mMod->getContext(), BLOCK_SIZE), 0));
}

Value* PabloCompiler::genShiftHighbitToLow(Value* e, const Twine &namehint) {
    IRBuilder<> b(mBasicBlock);
    Value* i128_val = b.CreateBitCast(e, IntegerType::get(mMod->getContext(), BLOCK_SIZE));
    return b.CreateBitCast(b.CreateLShr(i128_val, BLOCK_SIZE - 1, namehint), mXi64Vect);
}

Value* PabloCompiler::genShiftLeft64(Value* e, const Twine &namehint) {
    IRBuilder<> b(mBasicBlock);
    Value* i128_val = b.CreateBitCast(e, IntegerType::get(mMod->getContext(), BLOCK_SIZE));
    return b.CreateBitCast(b.CreateShl(i128_val, 64, namehint), mXi64Vect);
}

inline Value* PabloCompiler::genNot(Value* expr) {
    IRBuilder<> b(mBasicBlock);
    return b.CreateXor(expr, mOneInitializer, "not");
}

Value* PabloCompiler::genAdvanceWithCarry(Value* strm_value) {
    IRBuilder<> b(mBasicBlock);
#if (BLOCK_SIZE == 128)
    const auto carryIdx = mCarryQueueIdx++;
    Value* carryq_value = genCarryInLoad(carryIdx);
    Value* srli_1_value = b.CreateLShr(strm_value, 63);
    Value* packed_shuffle;
    Constant* const_packed_1_elems [] = {b.getInt32(0), b.getInt32(2)};
    Constant* const_packed_1 = ConstantVector::get(const_packed_1_elems);
    packed_shuffle = b.CreateShuffleVector(carryq_value, srli_1_value, const_packed_1);

    Constant* const_packed_2_elems[] = {b.getInt64(1), b.getInt64(1)};
    Constant* const_packed_2 = ConstantVector::get(const_packed_2_elems);

    Value* shl_value = b.CreateShl(strm_value, const_packed_2);
    Value* result_value = b.CreateOr(shl_value, packed_shuffle, "advance");

    Value* carry_out = genShiftHighbitToLow(strm_value, "carry_out");
    //CarryQ - carry out:
    genCarryOutStore(carry_out, carryIdx);

    return result_value;
#endif

#if (BLOCK_SIZE == 256)
    return genAddWithCarry(strm_value, strm_value);
#endif

}

}
