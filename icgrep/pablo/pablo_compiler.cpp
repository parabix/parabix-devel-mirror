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
, mOutputPtr(nullptr)
, mCarryQueueIdx(0)
, mptr_carry_q(nullptr)
, mCarryQueueSize(0)
, mZeroInitializer(ConstantAggregateZero::get(mXi64Vect))
, mOneInitializer(ConstantVector::getAllOnesValue(mXi64Vect))
, mFuncTy_0(nullptr)
, mFunc_process_block(nullptr)
, mBasisBitsAddr(nullptr)
, mPtr_carry_q_addr(nullptr)
, mPtr_output_addr(nullptr)
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

LLVM_Gen_RetVal PabloCompiler::compile(const PabloBlock & cg_state)
{
    mCarryQueueSize = 0;
    DeclareCallFunctions(cg_state.expressions());

    Function::arg_iterator args = mFunc_process_block->arg_begin();
    mBasisBitsAddr = args++;
    mBasisBitsAddr->setName("basis_bits");
    mptr_carry_q = args++;
    mptr_carry_q->setName("carry_q");
    Value* ptr_output = args++;
    ptr_output->setName("output");

    //Create the carry queue.
    mCarryQueueIdx = 0;
    mBasicBlock = BasicBlock::Create(mMod->getContext(), "parabix_entry", mFunc_process_block,0);

    //The basis bits structure
    for (unsigned i = 0; i < mBits; ++i) {
        IRBuilder<> b(mBasicBlock);
        Value* indices[] = {b.getInt64(0), b.getInt32(i)};
        const std::string name = mBasisBitVars[i]->getName();
        mMarkerMap.insert(make_pair(name, b.CreateGEP(mBasisBitsAddr, indices, name)));
    }
    mPtr_output_addr = new AllocaInst(mOutputPtr, "output.addr", mBasicBlock);
    new StoreInst(ptr_output, mPtr_output_addr, false, mBasicBlock);

    //Generate the IR instructions for the function.
    SetReturnMarker(compileStatements(cg_state.expressions()), 0); // matches
    SetReturnMarker(GetMarker(mNameMap["LineFeed"]->getName()), 1); // line feeds

    assert (mCarryQueueIdx == mCarryQueueSize);

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
    StructType * StructTy_struct_Basis_bits = mMod->getTypeByName("struct.Basis_bits");
    if (StructTy_struct_Basis_bits == nullptr) {
        StructTy_struct_Basis_bits = StructType::create(mMod->getContext(), "struct.Basis_bits");
    }
    std::vector<Type*>StructTy_struct_Basis_bits_fields;
    for (int i = 0; i < mBits; i++)
    {
        StructTy_struct_Basis_bits_fields.push_back(mXi64Vect);
    }
    if (StructTy_struct_Basis_bits->isOpaque()) {
        StructTy_struct_Basis_bits->setBody(StructTy_struct_Basis_bits_fields, /*isPacked=*/false);
    }
    mBasisBitsInputPtr = PointerType::get(StructTy_struct_Basis_bits, 0);

    std::vector<Type*>FuncTy_0_args;
    FuncTy_0_args.push_back(mBasisBitsInputPtr);

    //The carry q array.
    //A pointer to the BitBlock vector.
    FuncTy_0_args.push_back(PointerType::get(mXi64Vect, 0));

    //The output structure.
    StructType *StructTy_struct_Output = mMod->getTypeByName("struct.Output");
    if (!StructTy_struct_Output) {
        StructTy_struct_Output = StructType::create(mMod->getContext(), "struct.Output");
    }
    std::vector<Type*>StructTy_struct_Output_fields;
    StructTy_struct_Output_fields.push_back(mXi64Vect);
    StructTy_struct_Output_fields.push_back(mXi64Vect);
    if (StructTy_struct_Output->isOpaque()) {
        StructTy_struct_Output->setBody(StructTy_struct_Output_fields, /*isPacked=*/false);
    }
    mOutputPtr = PointerType::get(StructTy_struct_Output, 0);

    //The &output parameter.
    FuncTy_0_args.push_back(mOutputPtr);

    mFuncTy_0 = FunctionType::get(
     /*Result=*/Type::getVoidTy(mMod->getContext()),
     /*Params=*/FuncTy_0_args,
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

    mFunc_llvm_uadd_with_overflow = mMod->getFunction("llvm.uadd.with.overflow.carryin.i##BLOCK_SIZE");
    if (!mFunc_llvm_uadd_with_overflow) {
        mFunc_llvm_uadd_with_overflow = Function::Create(
          /*Type=*/ FuncTy_1,
          /*Linkage=*/ GlobalValue::ExternalLinkage,
          /*Name=*/ "llvm.uadd.with.overflow.carryin.i##BLOCK_SIZE", mMod); // (external, no body)
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
            /*Type=*/mFuncTy_0,
            /*Linkage=*/GlobalValue::ExternalLinkage,
            /*Name=*/"process_block", mMod);
        mFunc_process_block->setCallingConv(CallingConv::C);
    }
    mFunc_process_block->setAttributes(AttrSet);
}

void PabloCompiler::DeclareCallFunctions(const ExpressionList & stmts) {
    for (const PabloAST * stmt : stmts) {
        if (const Assign * an = dyn_cast<const Assign>(stmt)) {
            DeclareCallFunctions(an->getExpr());
        }
        else if (const If * ifstmt = dyn_cast<const If>(stmt)) {
            DeclareCallFunctions(ifstmt->getCondition());
            DeclareCallFunctions(ifstmt->getBody());
        }
        else if (const While * whl = dyn_cast<const While>(stmt)) {
            DeclareCallFunctions(whl->getCondition());
            DeclareCallFunctions(whl->getBody());
        }
    }
}

void PabloCompiler::DeclareCallFunctions(const PabloAST * expr)
{
    if (const Call * pablo_call = dyn_cast<const Call>(expr)) {
        const std::string callee = pablo_call->getCallee();
        if (mCalleeMap.find(callee) == mCalleeMap.end()) {
            void * callee_ptr = nullptr;
            #define CHECK_GENERAL_CODE_CATEGORY(SUFFIX) \
                if (callee == #SUFFIX) { \
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
            throw std::runtime_error("Unknown unicode category \"" + callee + "\"");
            #undef CHECK_GENERAL_CODE_CATEGORY
            Value * get_unicode_category = mMod->getOrInsertFunction("__get_category_" + callee, mXi64Vect, mBasisBitsInputPtr, NULL);
            if (get_unicode_category == nullptr) {
                throw std::runtime_error("Could not create static method call for unicode category \"" + callee + "\"");
            }
            mExecutionEngine->addGlobalMapping(cast<GlobalValue>(get_unicode_category), callee_ptr);
            mCalleeMap.insert(std::make_pair(callee, get_unicode_category));
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
        DeclareCallFunctions(mstar->getExpr1());
        DeclareCallFunctions(mstar->getExpr2());
    }
    else if (const ScanThru * sthru = dyn_cast<const ScanThru>(expr))
    {
        ++mCarryQueueSize;
        DeclareCallFunctions(sthru->getScanFrom());
        DeclareCallFunctions(sthru->getScanThru());
    }
}

Value* PabloCompiler::GetMarker(const std::string & name)
{    
    auto itr = mMarkerMap.find(name);
    if (itr == mMarkerMap.end()) {
        IRBuilder<> b(mBasicBlock);
        itr = mMarkerMap.insert(make_pair(name, b.CreateAlloca(mXi64Vect))).first;
    }
    return itr->second;
}

void PabloCompiler::SetReturnMarker(Value * marker, const unsigned index)
{
    IRBuilder<> b(mBasicBlock);
    Value* marker_bitblock = b.CreateLoad(marker);
    Value* output_struct = b.CreateLoad(mPtr_output_addr);
    Value* output_indices[] = {b.getInt64(0), b.getInt32(index)};
    Value* output_struct_GEP = b.CreateGEP(output_struct, output_indices);
    b.CreateStore(marker_bitblock, output_struct_GEP);
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
        Value * expr = compileExpression(assign->getExpr());
        Value * marker = nullptr;
        IRBuilder<> b(mBasicBlock);
        auto f = mMarkerMap.find(assign->getName());
        if (f == mMarkerMap.end()) {
            marker = b.CreateAlloca(mXi64Vect, 0, assign->getName());
            mMarkerMap.insert(std::make_pair(assign->getName(), marker));
        }
        else {
            marker = f->second;
        }
        b.CreateStore(expr, marker);
        retVal = marker;
    }
    if (const Next * next = dyn_cast<const Next>(stmt))
    {
        IRBuilder<> b(mBasicBlock);
        auto f = mMarkerMap.find(next->getName());
        assert (f != mMarkerMap.end());
        Value * marker = f->second;
        Value * expr = compileExpression(next->getExpr());
        b.CreateStore(expr, marker);
        retVal = marker;
    }
    else if (const If * ifstmt = dyn_cast<const If>(stmt))
    {
        BasicBlock * ifEntryBlock = mBasicBlock;
        BasicBlock * ifBodyBlock = BasicBlock::Create(mMod->getContext(), "if.body",mFunc_process_block, 0);
        BasicBlock * ifEndBlock = BasicBlock::Create(mMod->getContext(), "if.end",mFunc_process_block, 0);

        int if_start_idx = mCarryQueueIdx;

        Value* if_test_value = compileExpression(ifstmt->getCondition());

        /* Generate the statements into the if body block, and also determine the
           final carry index.  */

        IRBuilder<> b_ifbody(ifBodyBlock);
        mBasicBlock = ifBodyBlock;

        Value *  returnMarker = compileStatements(ifstmt->getBody());

        int if_end_idx = mCarryQueueIdx;
        if (if_start_idx < if_end_idx + 1) {
            // Have at least two internal carries.   Accumulate and store.
            int if_accum_idx = mCarryQueueIdx++;

            Value* if_carry_accum_value = genCarryInLoad(mptr_carry_q, if_start_idx);

            for (int c = if_start_idx+1; c < if_end_idx; c++)
            {
                Value* carryq_value = genCarryInLoad(mptr_carry_q, c);
                if_carry_accum_value = b_ifbody.CreateOr(carryq_value, if_carry_accum_value);
            }
            genCarryOutStore(if_carry_accum_value, mptr_carry_q, if_accum_idx);

        }
        b_ifbody.CreateBr(ifEndBlock);

        IRBuilder<> b_entry(ifEntryBlock);
        mBasicBlock = ifEntryBlock;
        if (if_start_idx < if_end_idx) {
            // Have at least one internal carry.
            int if_accum_idx = mCarryQueueIdx - 1;
            Value* last_if_pending_carries = genCarryInLoad(mptr_carry_q, if_accum_idx);
            if_test_value = b_entry.CreateOr(if_test_value, last_if_pending_carries);
        }
        b_entry.CreateCondBr(genBitBlockAny(if_test_value), ifEndBlock, ifBodyBlock);

        mBasicBlock = ifEndBlock;

        retVal = returnMarker;
    }
    else if (const While* whl = dyn_cast<const While>(stmt))
    {
        int idx = mCarryQueueIdx;

        //With this call to the while body we will account for all of the carry in values.
        Value * returnMarker = compileStatements(whl->getBody());

        BasicBlock*  whileCondBlock = BasicBlock::Create(mMod->getContext(), "while.cond", mFunc_process_block, 0);
        BasicBlock*  whileBodyBlock = BasicBlock::Create(mMod->getContext(), "while.body", mFunc_process_block, 0);
        BasicBlock*  whileEndBlock = BasicBlock::Create(mMod->getContext(), "while.end", mFunc_process_block, 0);

        IRBuilder<> b(mBasicBlock);
        b.CreateBr(whileCondBlock);
        mBasicBlock = whileCondBlock;
        IRBuilder<> b_cond(whileCondBlock);

        Value* expression_marker_value = compileExpression(whl->getCondition());
        Value* int_tobool1 = genBitBlockAny(expression_marker_value);

        b_cond.CreateCondBr(int_tobool1, whileEndBlock, whileBodyBlock);

        mBasicBlock = whileBodyBlock;
        mCarryQueueIdx = 0;
        //Store the current carry queue.
        Value* ptr_last_carry_q = mptr_carry_q;

        IRBuilder<> b_wb1(mBasicBlock);
        //Create and initialize a new carry queue.
        Value * ptr_while_carry_q = b_wb1.CreateAlloca(mXi64Vect, b_wb1.getInt64(mCarryQueueSize - idx));
        for (int i = 0; i < (mCarryQueueSize - idx); i++) {
            genCarryOutStore(mZeroInitializer, ptr_while_carry_q, i);
        }

        //Point mptr_carry_q to the new local carry queue.
        mptr_carry_q = ptr_while_carry_q;

        returnMarker = compileStatements(whl->getBody());

        IRBuilder<> b_wb2(mBasicBlock);
        //Copy back to the last carry queue the carries from the execution of the while statement list.
        for (int c = 0; c < (mCarryQueueSize - idx); c++)
        {
            Value* new_carryq_value = b_wb2.CreateOr(genCarryInLoad(mptr_carry_q, c), genCarryInLoad(ptr_last_carry_q, idx + c));
            genCarryOutStore(new_carryq_value, ptr_last_carry_q, idx + c);
        }

        b_wb2.CreateBr(whileCondBlock);

        mBasicBlock = whileEndBlock;
        mptr_carry_q = ptr_last_carry_q;
        mCarryQueueIdx += idx;

        retVal = returnMarker;
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
                throw std::runtime_error("Unexpected error locating static function for \"" + call->getCallee() + "\"");
            }
            Value * unicode_category = b.CreateCall(ci->second, mBasisBitsAddr);
            Value * ptr = b.CreateAlloca(mXi64Vect);
            b.CreateStore(unicode_category, ptr);
            mi = mMarkerMap.insert(std::make_pair(call->getCallee(), ptr)).first;
        }
        retVal = b.CreateLoad(mi->second);
    }
    else if (const Var * var = dyn_cast<Var>(expr))
    {
        auto f = mMarkerMap.find(var->getName());
        assert (f != mMarkerMap.end());
        retVal = b.CreateLoad(f->second, false, var->getName());
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
        Value* expr_value = compileExpression(pablo_not->getExpr());
        retVal = b.CreateXor(expr_value, mOneInitializer, "not");
    }
    else if (const Advance * adv = dyn_cast<Advance>(expr))
    {
        Value* strm_value = compileExpression(adv->getExpr());
        retVal = genAdvanceWithCarry(strm_value);
    }
    else if (const MatchStar * mstar = dyn_cast<MatchStar>(expr))
    {
        Value* marker_expr = compileExpression(mstar->getExpr1());
        Value* cc_expr = compileExpression(mstar->getExpr2());
        Value* marker_and_cc = b.CreateAnd(marker_expr, cc_expr);
        retVal = b.CreateOr(b.CreateXor(genAddWithCarry(marker_and_cc, cc_expr), cc_expr), marker_expr, "matchstar");
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
    int this_carry_idx = mCarryQueueIdx++;
    Value* carryq_value = genCarryInLoad(mptr_carry_q, this_carry_idx);

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
    genCarryOutStore(carry_out, mptr_carry_q, this_carry_idx);
    return sum;
}

Value* PabloCompiler::genCarryInLoad(Value* ptr_carry_q, int n) {
    IRBuilder<> b(mBasicBlock);
    Value* carryq_idx = b.getInt64(n);
    Value* carryq_GEP = b.CreateGEP(ptr_carry_q, carryq_idx);
    return b.CreateLoad(carryq_GEP);
}

Value* PabloCompiler::genCarryOutStore(Value* carryout, Value* ptr_carry_q, int n ) {
    IRBuilder<> b(mBasicBlock);
    Value* carryq_idx = b.getInt64(n);
    Value* carryq_GEP = b.CreateGEP(ptr_carry_q, carryq_idx);
    return b.CreateStore(carryout, carryq_GEP);
}

Value* PabloCompiler::genBitBlockAny(Value* e) {
    IRBuilder<> b(mBasicBlock);
    Value* cast_marker_value_1 = b.CreateBitCast(e, IntegerType::get(mMod->getContext(), BLOCK_SIZE));
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

Value* PabloCompiler::genNot(Value* e, const Twine &namehint) {
    IRBuilder<> b(mBasicBlock);
    return b.CreateXor(e, mOneInitializer, namehint);
}

Value* PabloCompiler::genAdvanceWithCarry(Value* strm_value) {
    IRBuilder<> b(mBasicBlock);
#if (BLOCK_SIZE == 128)
    int this_carry_idx = mCarryQueueIdx;
    mCarryQueueIdx++;

    Value* carryq_value = genCarryInLoad(mptr_carry_q, this_carry_idx);

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
    genCarryOutStore(carry_out, mptr_carry_q, this_carry_idx);

    return result_value;
#endif

#if (BLOCK_SIZE == 256)
    return genAddWithCarry(strm_value, strm_value);
#endif

}

}
