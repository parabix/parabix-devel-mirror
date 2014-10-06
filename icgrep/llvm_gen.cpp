/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "llvm_gen.h"
//Pablo Expressions
#include <pablo/codegenstate.h>

using namespace pablo;

Ps* fPs = NULL; Nl* fNl = NULL; No* fNo = NULL; Lo* fLo = NULL; Ll* fLl = NULL; Lm* fLm = NULL; Nd* fNd = NULL;
Pc* fPc = NULL; Lt* fLt = NULL; Lu* fLu = NULL; Pf* fPf = NULL; Pd* fPd = NULL; Pe* fPe = NULL; Pi* fPi = NULL;
Po* fPo = NULL; Me* fMe = NULL; Mc* fMc = NULL; Mn* fMn = NULL; Sk* fSk = NULL; So* fSo = NULL; Sm* fSm = NULL;
Sc* fSc = NULL; Zl* fZl = NULL; Co* fCo = NULL; Cn* fCn = NULL; Cc* fCc = NULL; Cf* fCf = NULL; Cs* fCs = NULL;
Zp* fZp = NULL; Zs* fZs = NULL;

extern "C" {
  void wrapped_print_register(BitBlock bit_block) {
      print_register<BitBlock>("", bit_block);
  }
}

extern "C" {
    BitBlock wrapped_get_category_Ps(Basis_bits &basis_bits){
        if (fPs == nullptr) fPs = new Ps();
        Struct_Ps ps_output;
        fPs->do_block(basis_bits, ps_output);

        return ps_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Nl(Basis_bits &basis_bits){
        if (fNl == nullptr) fNl = new Nl();
        Struct_Nl nl_output;
        fNl->do_block(basis_bits, nl_output);

        return nl_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_No(Basis_bits &basis_bits){
        if (fNo == nullptr) fNo = new No();
        Struct_No no_output;
        fNo->do_block(basis_bits, no_output);

        return no_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Lo(Basis_bits &basis_bits){
        if (fLo == nullptr) fLo = new Lo();
        Struct_Lo lo_output;
        fLo->do_block(basis_bits, lo_output);

        return lo_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Ll(Basis_bits &basis_bits){
        if (fLl == nullptr) fLl = new Ll();
        Struct_Ll ll_output;
        fLl->do_block(basis_bits, ll_output);

        return ll_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Lm(Basis_bits &basis_bits){
        if (fLm == nullptr) fLm = new Lm();
        Struct_Lm lm_output;
        fLm->do_block(basis_bits, lm_output);

        return lm_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Nd(Basis_bits &basis_bits){
        if (fNd == nullptr) fNd = new Nd();
        Struct_Nd nd_output;
        fNd->do_block(basis_bits, nd_output);

        return nd_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Pc(Basis_bits &basis_bits){
        if (fPc == nullptr) fPc = new Pc();
        Struct_Pc pc_output;
        fPc->do_block(basis_bits, pc_output);

        return pc_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Lt(Basis_bits &basis_bits){
        if (fLt == nullptr) fLt = new Lt();
        Struct_Lt lt_output;
        fLt->do_block(basis_bits, lt_output);

        return lt_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Lu(Basis_bits &basis_bits){
        if (fLu == nullptr) fLu = new Lu();
        Struct_Lu lu_output;
        fLu->do_block(basis_bits, lu_output);

        return lu_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Pf(Basis_bits &basis_bits){
        if (fPf == nullptr) fPf = new Pf();
        Struct_Pf pf_output;
        fPf->do_block(basis_bits, pf_output);

        return pf_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Pd(Basis_bits &basis_bits){
        if (fPd == nullptr) fPd = new Pd();
        Struct_Pd pd_output;
        fPd->do_block(basis_bits, pd_output);

        return pd_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Pe(Basis_bits &basis_bits){
        if (fPe == nullptr) fPe = new Pe();
        Struct_Pe pe_output;
        fPe->do_block(basis_bits, pe_output);

        return pe_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Pi(Basis_bits &basis_bits){
        if (fPi == nullptr) fPi = new Pi();
        Struct_Pi pi_output;
        fPi->do_block(basis_bits, pi_output);

        return pi_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Po(Basis_bits &basis_bits){
        if (fPo == nullptr) fPo = new Po();
        Struct_Po po_output;
        fPo->do_block(basis_bits, po_output);

        return po_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Me(Basis_bits &basis_bits){
        if (fMe == nullptr) fMe = new Me();
        Struct_Me me_output;
        fMe->do_block(basis_bits, me_output);

        return me_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Mc(Basis_bits &basis_bits){
        if (fMc == nullptr) fMc = new Mc();
        Struct_Mc mc_output;
        fMc->do_block(basis_bits, mc_output);

        return mc_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Mn(Basis_bits &basis_bits){
        if (fMn == nullptr) fMn = new Mn();
        Struct_Mn mn_output;
        fMn->do_block(basis_bits, mn_output);

        return mn_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Sk(Basis_bits &basis_bits){
        if (fSk == nullptr) fSk = new Sk();
        Struct_Sk sk_output;
        fSk->do_block(basis_bits, sk_output);

        return sk_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_So(Basis_bits &basis_bits){
        if (fSo == nullptr) fSo = new So();
        Struct_So so_output;
        fSo->do_block(basis_bits, so_output);

        return so_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Sm(Basis_bits &basis_bits){
        if (fSm == nullptr) fSm = new Sm();
        Struct_Sm sm_output;
        fSm->do_block(basis_bits, sm_output);

        return sm_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Sc(Basis_bits &basis_bits){
        if (fSc == nullptr) fSc = new Sc();
        Struct_Sc sc_output;
        fSc->do_block(basis_bits, sc_output);

        return sc_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Zl(Basis_bits &basis_bits){
        if (fZl == nullptr) fZl = new Zl();
        Struct_Zl zl_output;
        fZl->do_block(basis_bits, zl_output);

        return zl_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Co(Basis_bits &basis_bits){
        if (fCo == nullptr) fCo = new Co();
        Struct_Co co_output;
        fCo->do_block(basis_bits, co_output);

        return co_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Cn(Basis_bits &basis_bits){
        if (fCn == nullptr) fCn = new Cn();
        Struct_Cn cn_output;
        fCn->do_block(basis_bits, cn_output);

        return cn_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Cc(Basis_bits &basis_bits){
        if (fCc == nullptr) fCc = new Cc();
        Struct_Cc cc_output;
        fCc->do_block(basis_bits, cc_output);

        return cc_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Cf(Basis_bits &basis_bits){
        if (fCf == nullptr) fCf = new Cf();
        Struct_Cf cf_output;
        fCf->do_block(basis_bits, cf_output);

        return cf_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Cs(Basis_bits &basis_bits){
        if (fCs == nullptr) fCs = new Cs();
        Struct_Cs cs_output;
        fCs->do_block(basis_bits, cs_output);

        return cs_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Zp(Basis_bits &basis_bits){
        if (fZp == nullptr) fZp = new Zp();
        Struct_Zp zp_output;
        fZp->do_block(basis_bits, zp_output);

        return zp_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Zs(Basis_bits &basis_bits){
        if (fZs == nullptr) fZs = new Zs();
        Struct_Zs zs_output;
        fZs->do_block(basis_bits, zs_output);

        return zs_output.cc;
    }
}

LLVM_Generator::LLVM_Generator(std::map<std::string, std::string> name_map, std::string basis_pattern, int bits)
{
    m_name_map = name_map;
    mBasis_Pattern = basis_pattern;
    mBits = bits;
}

LLVM_Generator::~LLVM_Generator()
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

LLVM_Gen_RetVal LLVM_Generator::Generate_LLVMIR(const CodeGenState & cg_state)
{
    //Create the module.
    MakeLLVMModule();

    //Create the jit execution engine.up
    InitializeNativeTarget();
    std::string ErrStr;

    mExecutionEngine = EngineBuilder(mMod).setUseMCJIT(true).setErrorStr(&ErrStr).setOptLevel(CodeGenOpt::Level::Less).create();
    if (!mExecutionEngine)
    {
        std::cout << "\nCould not create ExecutionEngine: " + ErrStr << std::endl;
        exit(1);
    }

    InitializeNativeTargetAsmPrinter();
    InitializeNativeTargetAsmParser();

    DefineTypes();
    DeclareFunctions();
    DeclareCallFunctions(cg_state.expressions());

    Function::arg_iterator args = mFunc_process_block->arg_begin();
    Value* ptr_basis_bits = args++;
    ptr_basis_bits->setName("basis_bits");
    mptr_carry_q = args++;
    mptr_carry_q->setName("carry_q");
    Value* ptr_output = args++;
    ptr_output->setName("output");

    //Create the carry queue.
    mCarryQueueIdx = 0;
    mCarryQueueSize += LLVM_Generator_Helper::CarryCount_PabloStatements(cg_state.expressions());
    mBasicBlock = BasicBlock::Create(mMod->getContext(), "parabix_entry", mFunc_process_block,0);

    //The basis bits structure
    mPtr_basis_bits_addr = new AllocaInst(mStruct_Basis_Bits_Ptr1, "basis_bits.addr", mBasicBlock);
    StoreInst* void_14 = new StoreInst(ptr_basis_bits, mPtr_basis_bits_addr, false, mBasicBlock);

    for (int i = 0; i < mBits; i++)
    {
        StoreBitBlockMarkerPtr(mBasis_Pattern + std::to_string(i), i);
    }

    //The output structure.
    mPtr_output_addr = new AllocaInst(mStruct_Output_Ptr1, "output.addr", mBasicBlock);
    StoreInst* void_16 = new StoreInst(ptr_output, mPtr_output_addr, false, mBasicBlock);

    //Generate the IR instructions for the function.
    Generate_PabloStatements(cg_state.expressions());

    Assign * final = dyn_cast<Assign>(cg_state.expressions().back());
    assert (final);

    SetReturnMarker(final->getName(), 0);
    SetReturnMarker(m_name_map.find("LineFeed")->second, 1);

    //Terminate the block
    ReturnInst::Create(mMod->getContext(), mBasicBlock);

    #ifndef NDEBUG
    //Create a verifier.  The verifier will print an error message if our module is malformed in any way.
    #ifdef USE_LLVM_3_5
    verifyModule(*mMod, &dbgs());
    #endif
    #ifdef USE_LLVM_3_4
    verifyModule(*mMod, PrintMessageAction);
    #endif
    #endif

    //Un-comment this line in order to display the IR that has been generated by this module.
#ifdef DUMP_GENERATED_IR
    mMod->dump();
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

void LLVM_Generator::DefineTypes()
{
    //The BitBlock vector.
    mXi64Vect = VectorType::get(IntegerType::get(mMod->getContext(), 64), BLOCK_SIZE / 64);
    mXi128Vect = VectorType::get(IntegerType::get(mMod->getContext(), 128), BLOCK_SIZE / 128);
    //A pointer to the BitBlock vector.
    mXi64Vect_Ptr1 = PointerType::get(mXi64Vect, 0);

    //Constant definitions.
    mConst_int64_neg1 = ConstantInt::get(mMod->getContext(), APInt(64, StringRef("-1"), 10));

    mConst_Aggregate_Xi64_0 = ConstantAggregateZero::get(mXi64Vect);
    std::vector<Constant*> const_packed_27_elems;
    for (int i = 0; i < BLOCK_SIZE / 64; ++i)
      const_packed_27_elems.push_back(mConst_int64_neg1);
    mConst_Aggregate_Xi64_neg1 = ConstantVector::get(const_packed_27_elems);


    StructType *StructTy_struct_Basis_bits = mMod->getTypeByName("struct.Basis_bits");
    if (!StructTy_struct_Basis_bits) {
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

    mStruct_Basis_Bits_Ptr1 = PointerType::get(StructTy_struct_Basis_bits, 0);

    std::vector<Type*>FuncTy_0_args;
    FuncTy_0_args.push_back(mStruct_Basis_Bits_Ptr1);

    //The carry q array.
    FuncTy_0_args.push_back(mXi64Vect_Ptr1);

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
    mStruct_Output_Ptr1 = PointerType::get(StructTy_struct_Output, 0);

    //The &output parameter.
    FuncTy_0_args.push_back(mStruct_Output_Ptr1);

    mFuncTy_0 = FunctionType::get(
     /*Result=*/Type::getVoidTy(mMod->getContext()),
     /*Params=*/FuncTy_0_args,
     /*isVarArg=*/false);
}

void LLVM_Generator::DeclareFunctions()
{
    //This function can be used for testing to print the contents of a register from JIT'd code to the terminal window.
    //mFunc_print_register = mMod->getOrInsertFunction("wrapped_print_register", Type::getVoidTy(getGlobalContext()), mXi64Vect, NULL);
    //mExecutionEngine->addGlobalMapping(cast<GlobalValue>(mFunc_print_register), (void *)&wrapped_print_register);
    // to call->  b.CreateCall(mFunc_print_register, unicode_category);

#ifdef USE_UADD_OVERFLOW
    // Type Definitions for llvm.uadd.with.overflow.i128 or .i256
    std::vector<Type*>StructTy_0_fields;
    StructTy_0_fields.push_back(IntegerType::get(mMod->getContext(), BLOCK_SIZE));
    StructTy_0_fields.push_back(IntegerType::get(mMod->getContext(), 1));
    StructType *StructTy_0 = StructType::get(mMod->getContext(), StructTy_0_fields, /*isPacked=*/false);

    std::vector<Type*>FuncTy_1_args;
    FuncTy_1_args.push_back(IntegerType::get(mMod->getContext(), BLOCK_SIZE));
    FuncTy_1_args.push_back(IntegerType::get(mMod->getContext(), BLOCK_SIZE));
    FunctionType* FuncTy_1 = FunctionType::get(
                                              /*Result=*/StructTy_0,
                                              /*Params=*/FuncTy_1_args,
                                              /*isVarArg=*/false);

    mFunc_llvm_uadd_with_overflow = mMod->getFunction("llvm.uadd.with.overflow.i" + std::to_string(BLOCK_SIZE));
    if (!mFunc_llvm_uadd_with_overflow) {
        mFunc_llvm_uadd_with_overflow = Function::Create(
          /*Type=*/FuncTy_1,
          /*Linkage=*/GlobalValue::ExternalLinkage,
          /*Name=*/"llvm.uadd.with.overflow.i" + std::to_string(BLOCK_SIZE), mMod); // (external, no body)
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

void LLVM_Generator::DeclareCallFunctions(const ExpressionList & stmts) {
    for (PabloE * stmt : stmts) {
        if (const Assign * an = dyn_cast<const Assign>(stmt))
        {
            DeclareCallFunctions(an->getExpr());
        }
        else if (If * ifstmt = dyn_cast<If>(stmt))
        {
            DeclareCallFunctions(ifstmt->getExpr());
            DeclareCallFunctions(ifstmt->getPSList());
        }
        else if (While * whl = dyn_cast<While>(stmt))
        {
            DeclareCallFunctions(whl->getCondition());
            DeclareCallFunctions(whl->getPSList());
        }
    }
}

void LLVM_Generator::DeclareCallFunctions(const PabloE * expr)
{
    if (const Call * pablo_call = dyn_cast<const Call>(expr))
    {
        std::string callee = "wrapped_get_category_" + pablo_call->getCallee();
        if (mMarkerMap.find(callee) == mMarkerMap.end())
        {
            Value* func_get_unicode_category = mMod->getOrInsertFunction(callee, mXi64Vect, mStruct_Basis_Bits_Ptr1, NULL);
            void* addr;
            if (pablo_call->getCallee() == "Ps")
            {
                addr = (void *)&wrapped_get_category_Ps;
            }
            else if (pablo_call->getCallee() == "Nl")
            {
                addr = (void *)&wrapped_get_category_Nl;
            }
            else if (pablo_call->getCallee() == "No")
            {
                addr = (void *)&wrapped_get_category_No;
            }
            else if (pablo_call->getCallee() == "Lo")
            {
                addr = (void *)&wrapped_get_category_Lo;
            }
            else if (pablo_call->getCallee() == "Ll")
            {
                addr = (void *)&wrapped_get_category_Ll;
            }
            else if (pablo_call->getCallee() == "Lm")
            {
                addr = (void *)&wrapped_get_category_Lm;
            }
            else if (pablo_call->getCallee() == "Nd")
            {
                addr = (void *)&wrapped_get_category_Nd;
            }
            else if (pablo_call->getCallee() == "Pc")
            {
                addr = (void *)&wrapped_get_category_Pc;
            }
            else if (pablo_call->getCallee() == "Lt")
            {
                addr = (void *)&wrapped_get_category_Lt;
            }
            else if (pablo_call->getCallee() == "Lu")
            {
                addr = (void *)&wrapped_get_category_Lu;
            }
            else if (pablo_call->getCallee() == "Pf")
            {
                addr = (void *)&wrapped_get_category_Pf;
            }
            else if (pablo_call->getCallee() == "Pd")
            {
                addr = (void *)&wrapped_get_category_Pd;
            }
            else if (pablo_call->getCallee() == "Pe")
            {
                addr = (void *)&wrapped_get_category_Pe;
            }
            else if (pablo_call->getCallee() == "Pi")
            {
                addr = (void *)&wrapped_get_category_Pi;
            }
            else if (pablo_call->getCallee() == "Po")
            {
                addr = (void *)&wrapped_get_category_Po;
            }
            else if (pablo_call->getCallee() == "Me")
            {
                addr = (void *)&wrapped_get_category_Me;
            }
            else if (pablo_call->getCallee() == "Mc")
            {
                addr = (void *)&wrapped_get_category_Mc;
            }
            else if (pablo_call->getCallee() == "Mn")
            {
                addr = (void *)&wrapped_get_category_Mn;
            }
            else if (pablo_call->getCallee() == "Sk")
            {
                addr = (void *)&wrapped_get_category_Sk;
            }
            else if (pablo_call->getCallee() == "So")
            {
                addr = (void *)&wrapped_get_category_So;
            }
            else if (pablo_call->getCallee() == "Sm")
            {
                addr = (void *)&wrapped_get_category_Sm;
            }
            else if (pablo_call->getCallee() == "Sc")
            {
                addr = (void *)&wrapped_get_category_Sc;
            }
            else if (pablo_call->getCallee() == "Zl")
            {
                addr = (void *)&wrapped_get_category_Zl;
            }
            else if (pablo_call->getCallee() == "Co")
            {
                addr = (void *)&wrapped_get_category_Co;
            }
            else if (pablo_call->getCallee() == "Cn")
            {
                addr = (void *)&wrapped_get_category_Cn;
            }
            else if (pablo_call->getCallee() == "Cc")
            {
                addr = (void *)&wrapped_get_category_Cc;
            }
            else if (pablo_call->getCallee() == "Cf")
            {
                addr = (void *)&wrapped_get_category_Cf;
            }
            else if (pablo_call->getCallee() == "Cs")
            {
                addr = (void *)&wrapped_get_category_Cs;
            }
            else if (pablo_call->getCallee() == "Zp")
            {
                addr = (void *)&wrapped_get_category_Zp;
            }
            else if (pablo_call->getCallee() == "Zs")
            {
                addr = (void *)&wrapped_get_category_Zs;
            }

            mExecutionEngine->addGlobalMapping(cast<GlobalValue>(func_get_unicode_category), addr);
            mMarkerMap.insert(make_pair(callee, func_get_unicode_category));
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
        DeclareCallFunctions(adv->getExpr());
    }
    else if (const MatchStar * mstar = dyn_cast<const MatchStar>(expr))
    {
        DeclareCallFunctions(mstar->getExpr1());
        DeclareCallFunctions(mstar->getExpr2());
    }
    else if (const ScanThru * sthru = dyn_cast<const ScanThru>(expr))
    {
        DeclareCallFunctions(sthru->getScanFrom());
        DeclareCallFunctions(sthru->getScanThru());
    }
}

void LLVM_Generator::MakeLLVMModule()
{
    mMod = new Module("icgrep", getGlobalContext());

}

void LLVM_Generator::StoreBitBlockMarkerPtr(std::string name, int index)
{
    IRBuilder<> b(mBasicBlock);

    Value* basis_bits_struct = b.CreateLoad(mPtr_basis_bits_addr);
    Value* struct_indices[] = {b.getInt64(0), b.getInt32(index)};
    Value* basis_bits_struct_GEP = b.CreateGEP(basis_bits_struct, struct_indices, name);
    mMarkerMap.insert(make_pair(name, basis_bits_struct_GEP));
}

Value* LLVM_Generator::GetMarker(std::string name)
{
    IRBuilder<> b(mBasicBlock);
    auto itr = mMarkerMap.find(name);
    if (itr == mMarkerMap.end()) {
        Value* ptr = b.CreateAlloca(mXi64Vect);
        b.CreateStore(mConst_Aggregate_Xi64_0, ptr);
        itr = mMarkerMap.insert(make_pair(name, ptr)).first;
    }
    return itr->second;
}

void LLVM_Generator::SetReturnMarker(std::string marker, int output_idx)
{
    IRBuilder<> b(mBasicBlock);

    Value* marker_bitblock = b.CreateLoad(GetMarker(marker));
    Value* output_struct = b.CreateLoad(mPtr_output_addr);
    Value* output_indices[] = {b.getInt64(0), b.getInt32(output_idx)};
    Value* output_struct_GEP = b.CreateGEP(output_struct, output_indices, "return" + marker);
    Value* store_marker = b.CreateStore(marker_bitblock, output_struct_GEP);
}

std::string LLVM_Generator::Generate_PabloStatements(const ExpressionList & stmts) {
    std::string retVal = "";
    for (auto it = stmts.begin(); it != stmts.end(); ++it) {
        retVal = Generate_PabloS(*it);
    }
    return retVal;
}

std::string LLVM_Generator::Generate_PabloS(PabloE *stmt)
{
    std::string retVal = "";

    if (Assign * assign = dyn_cast<Assign>(stmt))
    {
        IRBuilder<> b(mBasicBlock);

        b.CreateStore(Generate_PabloE(assign->getExpr()), GetMarker(assign->getName()));

        retVal = assign->getName();
    }
    else if (If * ifstmt = dyn_cast<If>(stmt))
    {
        BasicBlock*  ifEntryBlock = mBasicBlock;
        BasicBlock*  ifBodyBlock = BasicBlock::Create(mMod->getContext(), "if.body",mFunc_process_block, 0);
        BasicBlock*  ifEndBlock = BasicBlock::Create(mMod->getContext(), "if.end",mFunc_process_block, 0);

        int if_start_idx = mCarryQueueIdx;

        Value* if_test_value = Generate_PabloE(ifstmt->getExpr());

        /* Generate the statements into the if body block, and also determine the
           final carry index.  */

        IRBuilder<> b_ifbody(ifBodyBlock);
        mBasicBlock = ifBodyBlock;
        std::string returnMarker = Generate_PabloStatements(ifstmt->getPSList());
        int if_end_idx = mCarryQueueIdx;

        if (if_start_idx < if_end_idx + 1) {
            // Have at least two internal carries.   Accumulate and store.
            int if_accum_idx = mCarryQueueIdx;
            mCarryQueueIdx++;

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
    else if (While* whl = dyn_cast<While>(stmt))
    {
        int idx = mCarryQueueIdx;

        //With this call to the while body we will account for all of the carry in values.
        std::string returnMarker = Generate_PabloStatements(whl->getPSList());

        BasicBlock*  whileCondBlock = BasicBlock::Create(mMod->getContext(), "while.cond", mFunc_process_block, 0);
        BasicBlock*  whileBodyBlock = BasicBlock::Create(mMod->getContext(), "while.body",mFunc_process_block, 0);
        BasicBlock*  whileEndBlock = BasicBlock::Create(mMod->getContext(), "while.end",mFunc_process_block, 0);

        IRBuilder<> b(mBasicBlock);
        b.CreateBr(whileCondBlock);
        mBasicBlock = whileCondBlock;
        IRBuilder<> b_cond(whileCondBlock);

        Value* expression_marker_value = Generate_PabloE(whl->getCondition());
        Value* int_tobool1 = genBitBlockAny(expression_marker_value);

        b_cond.CreateCondBr(int_tobool1, whileEndBlock, whileBodyBlock);

        mBasicBlock = whileBodyBlock;
        mCarryQueueIdx = 0;
        //Store the current carry queue.
        Value* ptr_last_carry_q = mptr_carry_q;

        IRBuilder<> b_wb1(mBasicBlock);
        //Create and initialize a new carry queue.
        Value* ptr_while_carry_q = b_wb1.CreateAlloca(mXi64Vect, b_wb1.getInt64(mCarryQueueSize - idx));
        for (int i=0; i < (mCarryQueueSize - idx); i++)
        {
            genCarryOutStore(mConst_Aggregate_Xi64_0, ptr_while_carry_q, i);
        }

        //Point mptr_carry_q to the new local carry queue.
        mptr_carry_q = ptr_while_carry_q;

        returnMarker = Generate_PabloStatements(whl->getPSList());

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

Value* LLVM_Generator::Generate_PabloE(PabloE * expr)
{
    Value* retVal = 0;

    if (All* all = dyn_cast<All>(expr))
    {
        IRBuilder<> b(mBasicBlock);
        Value* ptr_all = b.CreateAlloca(mXi64Vect);
        b.CreateStore((all->getValue() == 0 ? mConst_Aggregate_Xi64_0 : mConst_Aggregate_Xi64_neg1), ptr_all);
        Value* all_value = b.CreateLoad(ptr_all);
        retVal = all_value;
    }
    else if (Call* call = dyn_cast<Call>(expr))
    {
        IRBuilder<> b(mBasicBlock);

        //Call the callee once and store the result in the marker map.
        if (mMarkerMap.find(call->getCallee()) == mMarkerMap.end())
        {
            std::map<std::string, Value*>::iterator itFuncGet = mMarkerMap.find("wrapped_get_category_" + call->getCallee());
            Value* basis_bits_struct = b.CreateLoad(mPtr_basis_bits_addr);
            Value* unicode_category = b.CreateCall(itFuncGet->second, basis_bits_struct);
            Value* ptr = b.CreateAlloca(mXi64Vect);
            Value* void_1 = b.CreateStore(unicode_category, ptr);

            mMarkerMap.insert(make_pair(call->getCallee(), ptr));
        }
        std::map<std::string, Value*>::iterator itGet = mMarkerMap.find(call->getCallee());
        Value * var_value = b.CreateLoad(itGet->second);


        retVal = var_value;
    }
    else if (Var * var = dyn_cast<Var>(expr))
    {
        IRBuilder<> b(mBasicBlock);

        Value* var_value = b.CreateLoad(GetMarker(var->getName()), false, var->getName());

        retVal = var_value;
    }
    else if (And * pablo_and = dyn_cast<And>(expr))
    {
        IRBuilder<> b(mBasicBlock);

        Value* and_result = b.CreateAnd(Generate_PabloE(pablo_and->getExpr1()), Generate_PabloE(pablo_and->getExpr2()), "and_inst");

        retVal = and_result;
    }
    else if (Or * pablo_or = dyn_cast<Or>(expr))
    {
        IRBuilder<> b(mBasicBlock);

        Value* or_result = b.CreateOr(Generate_PabloE(pablo_or->getExpr1()), Generate_PabloE(pablo_or->getExpr2()), "or_inst");

        retVal = or_result;
    }
    else if (Sel * pablo_sel = dyn_cast<Sel>(expr))
    {
        IRBuilder<>b(mBasicBlock);
        Value* ifMask = Generate_PabloE(pablo_sel->getCondition());
        Value* and_if_true_result = b.CreateAnd(ifMask, Generate_PabloE(pablo_sel->getTrueExpr()));
        Value* and_if_false_result = b.CreateAnd(genNot(ifMask), Generate_PabloE(pablo_sel->getFalseExpr()));
        Value* or_result = b.CreateOr(and_if_true_result, and_if_false_result);

        retVal = or_result;
    }
    else if (Not * pablo_not = dyn_cast<Not>(expr))
    {
        IRBuilder<> b(mBasicBlock);

        Value* expr_value = Generate_PabloE(pablo_not->getExpr());
        Value* xor_rslt = b.CreateXor(expr_value, mConst_Aggregate_Xi64_neg1, "xor_inst");

        retVal = xor_rslt;
    }
    else if (CharClass * cc = dyn_cast<CharClass>(expr))
    {
        IRBuilder<> b(mBasicBlock);

        Value* character_class = b.CreateLoad(GetMarker(cc->getCharClass()));

        retVal = character_class;
    }
    else if (Advance * adv = dyn_cast<Advance>(expr))
    {
        IRBuilder<> b(mBasicBlock);
        Value* strm_value = Generate_PabloE(adv->getExpr());
        retVal = genAdvanceWithCarry(strm_value);
    }
    else if (MatchStar * mstar = dyn_cast<MatchStar>(expr))
    {
        IRBuilder<> b(mBasicBlock);
        Value* strm_value = Generate_PabloE(mstar->getExpr1());
        Value* cc_value = Generate_PabloE(mstar->getExpr2());
        retVal = genMatchStar(strm_value, cc_value);
    }
    else if (ScanThru * sthru = dyn_cast<ScanThru>(expr))
    {
        IRBuilder<> b(mBasicBlock);
        Value* strm_value = Generate_PabloE(sthru->getScanFrom());
        Value* scanthru_value = Generate_PabloE(sthru->getScanThru());
        retVal = genScanThru(strm_value, scanthru_value);
    }

    return retVal;
}


Value* LLVM_Generator::genMatchStar(Value* marker_expr, Value* cc_expr) {
    IRBuilder<> b(mBasicBlock);
    Value* marker_and_cc = b.CreateAnd(marker_expr, cc_expr);
    return b.CreateOr(b.CreateXor(genAddWithCarry(marker_and_cc, cc_expr), cc_expr), marker_expr, "matchstar_rslt");
}

Value* LLVM_Generator::genScanThru(Value* marker_expr, Value* cc_expr) {
    IRBuilder<> b(mBasicBlock);
    return b.CreateAnd(genAddWithCarry(marker_expr, cc_expr), genNot(cc_expr), "scanthru_rslt");
}

#ifdef USE_UADD_OVERFLOW
SumWithOverflowPack LLVM_Generator::callUaddOverflow(Value* int128_e1, Value* int128_e2) {
    std::vector<Value*> struct_res_params;
    struct_res_params.push_back(int128_e1);
    struct_res_params.push_back(int128_e2);
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

Value* LLVM_Generator::genAddWithCarry(Value* e1, Value* e2) {
    IRBuilder<> b(mBasicBlock);

    //CarryQ - carry in.
    int this_carry_idx = mCarryQueueIdx;
    mCarryQueueIdx++;

    Value* carryq_value = genCarryInLoad(mptr_carry_q, this_carry_idx);

#ifdef USE_UADD_OVERFLOW
    //use llvm.uadd.with.overflow.i128 or i256

    CastInst* int128_e1 = new BitCastInst(e1, IntegerType::get(mMod->getContext(), BLOCK_SIZE), "e1_128", mBasicBlock);
    CastInst* int128_e2 = new BitCastInst(e2, IntegerType::get(mMod->getContext(), BLOCK_SIZE), "e2_128", mBasicBlock);
    CastInst* int128_carryq_value = new BitCastInst(carryq_value, IntegerType::get(mMod->getContext(), BLOCK_SIZE), "carryq_128", mBasicBlock);

    SumWithOverflowPack sumpack0, sumpack1;

    sumpack0 = callUaddOverflow(int128_e1, int128_e2);
    sumpack1 = callUaddOverflow(sumpack0.sum, int128_carryq_value);

    Value* obit = b.CreateOr(sumpack0.obit, sumpack1.obit, "carry_bit");
    Value* ret_sum = b.CreateBitCast(sumpack1.sum, mXi64Vect, "ret_sum");

    /*obit is the i1 carryout, zero extend and insert it into a v2i64 or v4i64 vector.*/
    ConstantAggregateZero* const_packed_5 = ConstantAggregateZero::get(mXi64Vect);
    ConstantInt* const_int32_6 = ConstantInt::get(mMod->getContext(), APInt(32, StringRef("0"), 10));
    CastInst* int64_o0 = new ZExtInst(obit, IntegerType::get(mMod->getContext(), 64), "o0", mBasicBlock);
    InsertElementInst* carry_out = InsertElementInst::Create(const_packed_5, int64_o0, const_int32_6, "carry_out", mBasicBlock);

    Value* void_1 = genCarryOutStore(carry_out, mptr_carry_q, this_carry_idx);
    return ret_sum;
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
    Value* void_1 = genCarryOutStore(carry_out, mptr_carry_q, this_carry_idx);

    return sum;
#endif
}

Value* LLVM_Generator::genCarryInLoad(Value* ptr_carry_q, int n) {
    IRBuilder<> b(mBasicBlock);
    Value* carryq_idx = b.getInt64(n);
    Value* carryq_GEP = b.CreateGEP(ptr_carry_q, carryq_idx);
    return b.CreateLoad(carryq_GEP);
}

Value* LLVM_Generator::genCarryOutStore(Value* carryout, Value* ptr_carry_q, int n ) {
    IRBuilder<> b(mBasicBlock);
    Value* carryq_idx = b.getInt64(n);
    Value* carryq_GEP = b.CreateGEP(ptr_carry_q, carryq_idx);
    return b.CreateStore(carryout, carryq_GEP);
}

Value* LLVM_Generator::genBitBlockAny(Value* e) {
    IRBuilder<> b(mBasicBlock);
    Value* cast_marker_value_1 = b.CreateBitCast(e, IntegerType::get(mMod->getContext(), BLOCK_SIZE));
    return b.CreateICmpEQ(cast_marker_value_1, ConstantInt::get(IntegerType::get(mMod->getContext(), BLOCK_SIZE), 0));
}

Value* LLVM_Generator::genShiftHighbitToLow(Value* e, const Twine &namehint) {
    IRBuilder<> b(mBasicBlock);
    Value* i128_val = b.CreateBitCast(e, IntegerType::get(mMod->getContext(), BLOCK_SIZE));
    return b.CreateBitCast(b.CreateLShr(i128_val, BLOCK_SIZE - 1, namehint), mXi64Vect);
}

Value* LLVM_Generator::genShiftLeft64(Value* e, const Twine &namehint) {
    IRBuilder<> b(mBasicBlock);
    Value* i128_val = b.CreateBitCast(e, IntegerType::get(mMod->getContext(), BLOCK_SIZE));
    return b.CreateBitCast(b.CreateShl(i128_val, 64, namehint), mXi64Vect);
}

Value* LLVM_Generator::genNot(Value* e, const Twine &namehint) {
    IRBuilder<> b(mBasicBlock);
    return b.CreateXor(e, mConst_Aggregate_Xi64_neg1, namehint);
}

Value* LLVM_Generator::genAdvanceWithCarry(Value* strm_value) {
    IRBuilder<> b(mBasicBlock);

#if (BLOCK_SIZE == 128)
    int this_carry_idx = mCarryQueueIdx;
    mCarryQueueIdx++;

    Value* carryq_value = genCarryInLoad(mptr_carry_q, this_carry_idx);

    Value* srli_1_value = b.CreateLShr(strm_value, 63);

    Value* packed_shuffle;
    Constant* const_packed_1_elems [] = {b.getInt32(0), b.getInt32(2)};
    Constant* const_packed_1 = ConstantVector::get(const_packed_1_elems);
    packed_shuffle = b.CreateShuffleVector(carryq_value, srli_1_value, const_packed_1, "packed_shuffle nw");

    Constant* const_packed_2_elems[] = {b.getInt64(1), b.getInt64(1)};
    Constant* const_packed_2 = ConstantVector::get(const_packed_2_elems);

    Value* shl_value = b.CreateShl(strm_value, const_packed_2, "shl_value");
    Value* result_value = b.CreateOr(shl_value, packed_shuffle, "or.result_value");

    Value* carry_out = genShiftHighbitToLow(strm_value, "carry_out");
    //CarryQ - carry out:
    Value* void_1 = genCarryOutStore(carry_out, mptr_carry_q, this_carry_idx);

    return result_value;
#endif

#if (BLOCK_SIZE == 256)
    return genAddWithCarry(strm_value, strm_value);
#endif
}


