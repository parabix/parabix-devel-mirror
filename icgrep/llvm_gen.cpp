/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "llvm_gen.h"

Ps* ps = NULL; Nl* nl = NULL; No* no = NULL; Lo* lo = NULL; Ll* ll = NULL; Lm* lm = NULL; Nd* nd = NULL;
Pc* pc = NULL; Lt* lt = NULL; Lu* lu = NULL; Pf* pf = NULL; Pd* pd = NULL; Pe* pe = NULL; Pi* pi = NULL;
Po* po = NULL; Me* me = NULL; Mc* mc = NULL; Mn* mn = NULL; Sk* sk = NULL; So* so = NULL; Sm* sm = NULL;
Sc* sc = NULL; Zl* zl = NULL; Co* co = NULL; Cn* cn = NULL; Cc* cc = NULL; Cf* cf = NULL; Cs* cs = NULL;
Zp* zp = NULL; Zs* zs = NULL;

extern "C" {
  void wrapped_print_register(BitBlock bit_block) {
      print_register<BitBlock>("", bit_block);
  }
}

extern "C" {
    BitBlock wrapped_get_category_Ps(Basis_bits &basis_bits){
        if (ps == NULL) ps = new Ps();
        Struct_Ps ps_output;
        ps->do_block(basis_bits, ps_output);

        return ps_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Nl(Basis_bits &basis_bits){
        if (nl == NULL) nl = new Nl();
        Struct_Nl nl_output;
        nl->do_block(basis_bits, nl_output);

        return nl_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_No(Basis_bits &basis_bits){
        if (no == NULL) no = new No();
        Struct_No no_output;
        no->do_block(basis_bits, no_output);

        return no_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Lo(Basis_bits &basis_bits){
        if (lo == NULL) lo = new Lo();
        Struct_Lo lo_output;
        lo->do_block(basis_bits, lo_output);

        return lo_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Ll(Basis_bits &basis_bits){
        if (ll == NULL) ll = new Ll();
        Struct_Ll ll_output;
        ll->do_block(basis_bits, ll_output);

        return ll_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Lm(Basis_bits &basis_bits){
        if (lm == NULL) lm = new Lm();
        Struct_Lm lm_output;
        lm->do_block(basis_bits, lm_output);

        return lm_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Nd(Basis_bits &basis_bits){
        if (nd == NULL) nd = new Nd();
        Struct_Nd nd_output;
        nd->do_block(basis_bits, nd_output);

        return nd_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Pc(Basis_bits &basis_bits){
        if (pc == NULL) pc = new Pc();
        Struct_Pc pc_output;
        pc->do_block(basis_bits, pc_output);

        return pc_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Lt(Basis_bits &basis_bits){
        if (lt == NULL) lt = new Lt();
        Struct_Lt lt_output;
        lt->do_block(basis_bits, lt_output);

        return lt_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Lu(Basis_bits &basis_bits){
        if (lu == NULL) lu = new Lu();
        Struct_Lu lu_output;
        lu->do_block(basis_bits, lu_output);

        return lu_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Pf(Basis_bits &basis_bits){
        if (pf == NULL) pf = new Pf();
        Struct_Pf pf_output;
        pf->do_block(basis_bits, pf_output);

        return pf_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Pd(Basis_bits &basis_bits){
        if (pd == NULL) pd = new Pd();
        Struct_Pd pd_output;
        pd->do_block(basis_bits, pd_output);

        return pd_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Pe(Basis_bits &basis_bits){
        if (pe == NULL) pe = new Pe();
        Struct_Pe pe_output;
        pe->do_block(basis_bits, pe_output);

        return pe_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Pi(Basis_bits &basis_bits){
        if (pi == NULL) pi = new Pi();
        Struct_Pi pi_output;
        pi->do_block(basis_bits, pi_output);

        return pi_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Po(Basis_bits &basis_bits){
        if (po == NULL) po = new Po();
        Struct_Po po_output;
        po->do_block(basis_bits, po_output);

        return po_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Me(Basis_bits &basis_bits){
        if (me == NULL) me = new Me();
        Struct_Me me_output;
        me->do_block(basis_bits, me_output);

        return me_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Mc(Basis_bits &basis_bits){
        if (mc == NULL) mc = new Mc();
        Struct_Mc mc_output;
        mc->do_block(basis_bits, mc_output);

        return mc_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Mn(Basis_bits &basis_bits){
        if (mn == NULL) mn = new Mn();
        Struct_Mn mn_output;
        mn->do_block(basis_bits, mn_output);

        return mn_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Sk(Basis_bits &basis_bits){
        if (sk == NULL) sk = new Sk();
        Struct_Sk sk_output;
        sk->do_block(basis_bits, sk_output);

        return sk_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_So(Basis_bits &basis_bits){
        if (so == NULL) so = new So();
        Struct_So so_output;
        so->do_block(basis_bits, so_output);

        return so_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Sm(Basis_bits &basis_bits){
        if (sm == NULL) sm = new Sm();
        Struct_Sm sm_output;
        sm->do_block(basis_bits, sm_output);

        return sm_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Sc(Basis_bits &basis_bits){
        if (sc == NULL) sc = new Sc();
        Struct_Sc sc_output;
        sc->do_block(basis_bits, sc_output);

        return sc_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Zl(Basis_bits &basis_bits){
        if (zl == NULL) zl = new Zl();
        Struct_Zl zl_output;
        zl->do_block(basis_bits, zl_output);

        return zl_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Co(Basis_bits &basis_bits){
        if (co == NULL) co = new Co();
        Struct_Co co_output;
        co->do_block(basis_bits, co_output);

        return co_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Cn(Basis_bits &basis_bits){
        if (cn == NULL) cn = new Cn();
        Struct_Cn cn_output;
        cn->do_block(basis_bits, cn_output);

        return cn_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Cc(Basis_bits &basis_bits){
        if (cc == NULL) cc = new Cc();
        Struct_Cc cc_output;
        cc->do_block(basis_bits, cc_output);

        return cc_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Cf(Basis_bits &basis_bits){
        if (cf == NULL) cf = new Cf();
        Struct_Cf cf_output;
        cf->do_block(basis_bits, cf_output);

        return cf_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Cs(Basis_bits &basis_bits){
        if (cs == NULL) cs = new Cs();
        Struct_Cs cs_output;
        cs->do_block(basis_bits, cs_output);

        return cs_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Zp(Basis_bits &basis_bits){
        if (zp == NULL) zp = new Zp();
        Struct_Zp zp_output;
        zp->do_block(basis_bits, zp_output);

        return zp_output.cc;
    }
}

extern "C" {
    BitBlock wrapped_get_category_Zs(Basis_bits &basis_bits){
        if (zs == NULL) zs = new Zs();
        Struct_Zs zs_output;
        zs->do_block(basis_bits, zs_output);

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

    if (ps != NULL) delete ps;
    if (nl != NULL) delete nl;
    if (no != NULL) delete no;
    if (lo != NULL) delete lo;
    if (ll != NULL) delete ll;
    if (lm != NULL) delete lm;
    if (nd != NULL) delete nd;
    if (pc != NULL) delete pc;
    if (lt != NULL) delete lt;
    if (lu != NULL) delete lu;
    if (pf != NULL) delete pf;
    if (pd != NULL) delete pd;
    if (pe != NULL) delete pe;
    if (pi != NULL) delete pi;
    if (po != NULL) delete po;
    if (me != NULL) delete me;
    if (mc != NULL) delete mc;
    if (mn != NULL) delete mn;
    if (sk != NULL) delete sk;
    if (so != NULL) delete so;
    if (sm != NULL) delete sm;
    if (sc != NULL) delete sc;
    if (zl != NULL) delete zl;
    if (co != NULL) delete co;
    if (cn != NULL) delete cn;
    if (cc != NULL) delete cc;
    if (cf != NULL) delete cf;
    if (cs != NULL) delete cs;
    if (zp != NULL) delete zp;
    if (zs != NULL) delete zs;

}

LLVM_Gen_RetVal LLVM_Generator::Generate_LLVMIR(CodeGenState cg_state, CodeGenState subexpression_cg_state, std::list<PabloS*> cc_cgo_stmtsl)
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
    DeclareCallFunctions(cg_state.stmtsl);

    Function::arg_iterator args = mFunc_process_block->arg_begin();
    Value* ptr_basis_bits = args++;
    ptr_basis_bits->setName("basis_bits");
    mptr_carry_q = args++;
    mptr_carry_q->setName("carry_q");
    Value* ptr_output = args++;
    ptr_output->setName("output");

    //Create the carry queue.
    mCarryQueueIdx = 0;
    mCarryQueueSize = LLVM_Generator_Helper::CarryCount_PabloStatements(subexpression_cg_state.stmtsl);
    mCarryQueueSize += LLVM_Generator_Helper::CarryCount_PabloStatements(cg_state.stmtsl);
    /* The following may be needed if carry-generating operations are ever inserted
       by the character class compiler.
    mCarryQueueSize += LLVM_Generator_Helper::CarryCount_PabloStatements(cc_cgo_stmtsl);
    */

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

    Generate_PabloStatements(cc_cgo_stmtsl);
    Generate_PabloStatements(subexpression_cg_state.stmtsl);
    Generate_PabloStatements(cg_state.stmtsl);
    SetReturnMarker(cg_state.newsym, 0);
    SetReturnMarker(m_name_map.find("LineFeed")->second, 1);

    //Terminate the block
    ReturnInst::Create(mMod->getContext(), mBasicBlock);

    //Create a verifier.  The verifier will print an error message if our module is malformed in any way.
#ifdef USE_LLVM_3_5
    verifyModule(*mMod, &dbgs());
#endif

#ifdef USE_LLVM_3_4
    verifyModule(*mMod, PrintMessageAction);
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

#define bitBlockExprType m64x2Vect

void LLVM_Generator::DefineTypes()
{
    //The BitBlock vector.
    m64x2Vect = VectorType::get(IntegerType::get(mMod->getContext(), 64), 2);
    m128x1Vect = VectorType::get(IntegerType::get(mMod->getContext(), 128), 1);
    //A pointer to the BitBlock vector.
    m64x2Vect_Ptr1 = PointerType::get(m64x2Vect, 0);

    //Constant definitions.
    mConst_int64_neg1 = ConstantInt::get(mMod->getContext(), APInt(64, StringRef("-1"), 10));

    mConst_Aggregate_64x2_0 = ConstantAggregateZero::get(m64x2Vect);
    std::vector<Constant*> const_packed_27_elems;
    const_packed_27_elems.push_back(mConst_int64_neg1);
    const_packed_27_elems.push_back(mConst_int64_neg1);
    mConst_Aggregate_64x2_neg1 = ConstantVector::get(const_packed_27_elems);


    StructType *StructTy_struct_Basis_bits = mMod->getTypeByName("struct.Basis_bits");
    if (!StructTy_struct_Basis_bits) {
        StructTy_struct_Basis_bits = StructType::create(mMod->getContext(), "struct.Basis_bits");
    }
    std::vector<Type*>StructTy_struct_Basis_bits_fields;
    for (int i = 0; i < mBits; i++)
    {
        StructTy_struct_Basis_bits_fields.push_back(m64x2Vect);
    }
    if (StructTy_struct_Basis_bits->isOpaque()) {
        StructTy_struct_Basis_bits->setBody(StructTy_struct_Basis_bits_fields, /*isPacked=*/false);
    }

    mStruct_Basis_Bits_Ptr1 = PointerType::get(StructTy_struct_Basis_bits, 0);

    std::vector<Type*>FuncTy_0_args;
    FuncTy_0_args.push_back(mStruct_Basis_Bits_Ptr1);

    //The carry q array.
    FuncTy_0_args.push_back(m64x2Vect_Ptr1);

    //The output structure.
    StructType *StructTy_struct_Output = mMod->getTypeByName("struct.Output");
    if (!StructTy_struct_Output) {
        StructTy_struct_Output = StructType::create(mMod->getContext(), "struct.Output");
    }
    std::vector<Type*>StructTy_struct_Output_fields;
    StructTy_struct_Output_fields.push_back(m64x2Vect);
    StructTy_struct_Output_fields.push_back(m64x2Vect);
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
    //mFunc_print_register = mMod->getOrInsertFunction("wrapped_print_register", Type::getVoidTy(getGlobalContext()), m64x2Vect, NULL);
    //mExecutionEngine->addGlobalMapping(cast<GlobalValue>(mFunc_print_register), (void *)&wrapped_print_register);
    // to call->  b.CreateCall(mFunc_print_register, unicode_category);

    // Type Definitions for llvm.uadd.with.overflow.i128
    std::vector<Type*>StructTy_0_fields;
    StructTy_0_fields.push_back(IntegerType::get(mMod->getContext(), 128));
    StructTy_0_fields.push_back(IntegerType::get(mMod->getContext(), 1));
    StructType *StructTy_0 = StructType::get(mMod->getContext(), StructTy_0_fields, /*isPacked=*/false);

    std::vector<Type*>FuncTy_1_args;
    FuncTy_1_args.push_back(IntegerType::get(mMod->getContext(), 128));
    FuncTy_1_args.push_back(IntegerType::get(mMod->getContext(), 128));
    FunctionType* FuncTy_1 = FunctionType::get(
                                              /*Result=*/StructTy_0,
                                              /*Params=*/FuncTy_1_args,
                                              /*isVarArg=*/false);

    mFunc_llvm_uadd_with_overflow_i128 = mMod->getFunction("llvm.uadd.with.overflow.i128");
    if (!mFunc_llvm_uadd_with_overflow_i128) {
        mFunc_llvm_uadd_with_overflow_i128 = Function::Create(
          /*Type=*/FuncTy_1,
          /*Linkage=*/GlobalValue::ExternalLinkage,
          /*Name=*/"llvm.uadd.with.overflow.i128", mMod); // (external, no body)
        mFunc_llvm_uadd_with_overflow_i128->setCallingConv(CallingConv::C);
    }
    AttributeSet mFunc_llvm_uadd_with_overflow_i128_PAL;
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
        mFunc_llvm_uadd_with_overflow_i128_PAL = AttributeSet::get(mMod->getContext(), Attrs);
    }
    mFunc_llvm_uadd_with_overflow_i128->setAttributes(mFunc_llvm_uadd_with_overflow_i128_PAL);

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

void LLVM_Generator::DeclareCallFunctions(std::list<PabloS*> stmts)
{
    std::list<PabloS*>::iterator it;
    for (it = stmts.begin(); it != stmts.end(); ++it)
    {
        DeclareCallFunctions_PabloS(*it);
    }
}

void LLVM_Generator::DeclareCallFunctions_PabloS(PabloS* stmt)
{
    if (Assign* an = dynamic_cast<Assign*>(stmt))
    {
        DeclareCallFunctions_PabloE(an->getExpr());
    }
    if (While* whl = dynamic_cast<While*>(stmt))
    {
        DeclareCallFunctions_PabloE(whl->getExpr());
        DeclareCallFunctions(whl->getPSList());
    }
}

void LLVM_Generator::DeclareCallFunctions_PabloE(PabloE* expr)
{
    if (Call* pablo_call = dynamic_cast<Call*>(expr))
    {
        std::string callee = "wrapped_get_category_" + pablo_call->getCallee();
        if (mMarkerMap.find(callee) == mMarkerMap.end())
        {
            Value* func_get_unicode_category = mMod->getOrInsertFunction(callee, m64x2Vect, mStruct_Basis_Bits_Ptr1, NULL);
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
    else if (And* pablo_and = dynamic_cast<And*>(expr))
    {
        DeclareCallFunctions_PabloE(pablo_and->getExpr1());
        DeclareCallFunctions_PabloE(pablo_and->getExpr2());
    }
    else if (Or* pablo_or = dynamic_cast<Or*>(expr))
    {
        DeclareCallFunctions_PabloE(pablo_or->getExpr1());
        DeclareCallFunctions_PabloE(pablo_or->getExpr2());
    }
    else if (Sel* pablo_sel = dynamic_cast<Sel*>(expr))
    {
        DeclareCallFunctions_PabloE(pablo_sel->getIf_expr());
        DeclareCallFunctions_PabloE(pablo_sel->getT_expr());
        DeclareCallFunctions_PabloE(pablo_sel->getF_expr());
    }
    else if (Not* pablo_not = dynamic_cast<Not*>(expr))
    {
        DeclareCallFunctions_PabloE(pablo_not->getExpr());
    }
    else if (Advance* adv = dynamic_cast<Advance*>(expr))
    {
        DeclareCallFunctions_PabloE(adv->getExpr());
    }
    else if (MatchStar* mstar = dynamic_cast<MatchStar*>(expr))
    {
        DeclareCallFunctions_PabloE(mstar->getExpr1());
        DeclareCallFunctions_PabloE(mstar->getExpr2());
    }
    else if (ScanThru* sthru = dynamic_cast<ScanThru*>(expr))
    {
        DeclareCallFunctions_PabloE(sthru->getScanFrom());
        DeclareCallFunctions_PabloE(sthru->getScanThru());
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

    if (mMarkerMap.find(name) == mMarkerMap.end())
    {
        Value* ptr = b.CreateAlloca(m64x2Vect);
        Value* void_1 = b.CreateStore(mConst_Aggregate_64x2_0, ptr);
        mMarkerMap.insert(make_pair(name, ptr));
    }
    std::map<std::string, Value*>::iterator itGet = mMarkerMap.find(name);

    return itGet->second;
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

std::string LLVM_Generator::Generate_PabloStatements(std::list<PabloS*> stmts)
{
    std::string retVal = "";

    std::list<PabloS*>::iterator it;
    for (it = stmts.begin(); it != stmts.end(); ++it)
    {
        retVal = Generate_PabloS(*it);
    }

    return retVal;
}

std::string LLVM_Generator::Generate_PabloS(PabloS *stmt)
{
    std::string retVal = "";

    if (Assign* assign = dynamic_cast<Assign*>(stmt))
    {
        IRBuilder<> b(mBasicBlock);

        b.CreateStore(Generate_PabloE(assign->getExpr()), GetMarker(assign->getM()));

        retVal = assign->getM();
    }
    else if (If* ifstmt = dynamic_cast<If*>(stmt))
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
            Value* void_1 = genCarryOutStore(if_carry_accum_value, mptr_carry_q, if_accum_idx);

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
    else if (While* whl = dynamic_cast<While*>(stmt))
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

        Value* expression_marker_value = Generate_PabloE(whl->getExpr());
        // Use an i128 compare for simplicity and speed.
        Value* cast_marker_value_1 = b_cond.CreateBitCast(expression_marker_value, IntegerType::get(mMod->getContext(), 128));
        Value* int_tobool1 = b_cond.CreateICmpEQ(cast_marker_value_1, ConstantInt::get(IntegerType::get(mMod->getContext(), 128), 0));
        b_cond.CreateCondBr(int_tobool1, whileEndBlock, whileBodyBlock);

        mBasicBlock = whileBodyBlock;
        mCarryQueueIdx = 0;
        //Store the current carry queue.
        Value* ptr_last_carry_q = mptr_carry_q;

        IRBuilder<> b_wb1(mBasicBlock);
        //Create and initialize a new carry queue.
        Value* ptr_while_carry_q = b_wb1.CreateAlloca(m64x2Vect, b_wb1.getInt64(mCarryQueueSize - idx));
        for (int i=0; i<(mCarryQueueSize-idx); i++)
        {
            Value* void_1 = genCarryOutStore(mConst_Aggregate_64x2_0, ptr_while_carry_q, i);
        }

        //Point mptr_carry_q to the new local carry queue.
        mptr_carry_q = ptr_while_carry_q;

        returnMarker = Generate_PabloStatements(whl->getPSList());

        IRBuilder<> b_wb2(mBasicBlock);
        //Copy back to the last carry queue the carries from the execution of the while statement list.
        for (int c=0; c<(mCarryQueueSize-idx); c++)
        {
            Value* new_carryq_value = b_wb2.CreateOr(genCarryInLoad(mptr_carry_q, c), genCarryInLoad(ptr_last_carry_q, idx + c));
            Value* void_1 = genCarryOutStore(new_carryq_value, ptr_last_carry_q, idx + c);
        }

        b_wb2.CreateBr(whileCondBlock);

        mBasicBlock = whileEndBlock;
        mptr_carry_q = ptr_last_carry_q;
        mCarryQueueIdx += idx;

        retVal = returnMarker;
    }

    return retVal;
}

Value* LLVM_Generator::Generate_PabloE(PabloE *expr)
{
    Value* retVal = 0;

    if (All* all = dynamic_cast<All*>(expr))
    {
        IRBuilder<> b(mBasicBlock);

        if ((all->getNum() != 0) && (all->getNum() != 1))
            std::cout << "\nErr: 'All' can only be set to 1 or 0.\n" << std::endl;
        Value* ptr_all = b.CreateAlloca(m64x2Vect);
        Value* void_1 = b.CreateStore((all->getNum() == 0 ? mConst_Aggregate_64x2_0 : mConst_Aggregate_64x2_neg1), ptr_all);
        Value* all_value = b.CreateLoad(ptr_all);

        retVal = all_value;
    }
    else if (Call* call = dynamic_cast<Call*>(expr))
    {
        IRBuilder<> b(mBasicBlock);

        //Call the callee once and store the result in the marker map.
        if (mMarkerMap.find(call->getCallee()) == mMarkerMap.end())
        {
            std::map<std::string, Value*>::iterator itFuncGet = mMarkerMap.find("wrapped_get_category_" + call->getCallee());
            Value* basis_bits_struct = b.CreateLoad(mPtr_basis_bits_addr);
            Value* unicode_category = b.CreateCall(itFuncGet->second, basis_bits_struct);
            Value* ptr = b.CreateAlloca(m64x2Vect);
            Value* void_1 = b.CreateStore(unicode_category, ptr);

            mMarkerMap.insert(make_pair(call->getCallee(), ptr));
        }
        std::map<std::string, Value*>::iterator itGet = mMarkerMap.find(call->getCallee());
        Value * var_value = b.CreateLoad(itGet->second);


        retVal = var_value;
    }
    else if (Var* var = dynamic_cast<Var*>(expr))
    {
        IRBuilder<> b(mBasicBlock);

        Value* var_value = b.CreateLoad(GetMarker(var->getVar()), false, var->getVar());

        retVal = var_value;
    }
    else if (And* pablo_and = dynamic_cast<And*>(expr))
    {
        IRBuilder<> b(mBasicBlock);

        Value* and_result = b.CreateAnd(Generate_PabloE(pablo_and->getExpr1()), Generate_PabloE(pablo_and->getExpr2()), "and_inst");

        retVal = and_result;
    }
    else if (Or* pablo_or = dynamic_cast<Or*>(expr))
    {
        IRBuilder<> b(mBasicBlock);

        Value* or_result = b.CreateOr(Generate_PabloE(pablo_or->getExpr1()), Generate_PabloE(pablo_or->getExpr2()), "or_inst");

        retVal = or_result;
    }
    else if (Sel* pablo_sel = dynamic_cast<Sel*>(expr))
    {
        IRBuilder<>b(mBasicBlock);
        Value* ifMask = Generate_PabloE(pablo_sel->getIf_expr());
        Value* and_if_true_result = b.CreateAnd(ifMask, Generate_PabloE(pablo_sel->getT_expr()));
        Value* and_if_false_result = b.CreateAnd(genNot(ifMask), Generate_PabloE(pablo_sel->getF_expr()));
        Value* or_result = b.CreateOr(and_if_true_result, and_if_false_result);

        retVal = or_result;
    }
    else if (Not* pablo_not = dynamic_cast<Not*>(expr))
    {
        IRBuilder<> b(mBasicBlock);

        Constant* const_packed_elems [] = {b.getInt64(-1), b.getInt64(-1)};
        Constant* const_packed = ConstantVector::get(const_packed_elems);
        Value* expr_value = Generate_PabloE(pablo_not->getExpr());
        Value* xor_rslt = b.CreateXor(expr_value, const_packed, "xor_inst");

        retVal = xor_rslt;
    }
    else if (CharClass* cc = dynamic_cast<CharClass*>(expr))
    {
        IRBuilder<> b(mBasicBlock);

        Value* character_class = b.CreateLoad(GetMarker(cc->getCharClass()));

        retVal = character_class;
    }
    else if (Advance* adv = dynamic_cast<Advance*>(expr))
    {
        IRBuilder<> b(mBasicBlock);

        int this_carry_idx = mCarryQueueIdx;
        mCarryQueueIdx++;

        Value* carryq_value = genCarryInLoad(mptr_carry_q, this_carry_idx);

        Value* strm_value = Generate_PabloE(adv->getExpr());
        Value* srli_1_value = b.CreateLShr(strm_value, 63);

        Value* packed_shuffle;
        Constant* const_packed_1_elems [] = {b.getInt32(0), b.getInt32(2)};
        Constant* const_packed_1 = ConstantVector::get(const_packed_1_elems);
        packed_shuffle = b.CreateShuffleVector(carryq_value, srli_1_value, const_packed_1, "packed_shuffle nw");

        Constant* const_packed_2_elems[] = {b.getInt64(1), b.getInt64(1)};
        Constant* const_packed_2 = ConstantVector::get(const_packed_2_elems);

        Value* shl_value = b.CreateShl(strm_value, const_packed_2, "shl_value");
        Value* result_value = b.CreateOr(shl_value, packed_shuffle, "or.result_value");

        Value* carry_out = genShiftRight127(strm_value, "carry_out");
        //CarryQ - carry out:
        Value* void_1 = genCarryOutStore(carry_out, mptr_carry_q, this_carry_idx);

        retVal = result_value;
    }
    else if (MatchStar* mstar = dynamic_cast<MatchStar*>(expr))
    {
        IRBuilder<> b(mBasicBlock);
        Value* strm_value = Generate_PabloE(mstar->getExpr1());
        Value* cc_value = Generate_PabloE(mstar->getExpr2());
        retVal = genMatchStar(strm_value, cc_value);
    }
    else if (ScanThru* sthru = dynamic_cast<ScanThru*>(expr))
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

SumWithOverflowPack LLVM_Generator::genUaddOverflow(Value* int128_e1, Value* int128_e2) {
    std::vector<Value*> struct_res_params;
    struct_res_params.push_back(int128_e1);
    struct_res_params.push_back(int128_e2);
    CallInst* struct_res = CallInst::Create(mFunc_llvm_uadd_with_overflow_i128, struct_res_params, "uadd_overflow_res", mBasicBlock);
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


Value* LLVM_Generator::genAddWithCarry(Value* e1, Value* e2) {
    IRBuilder<> b(mBasicBlock);

    //CarryQ - carry in.
    int this_carry_idx = mCarryQueueIdx;
    mCarryQueueIdx++;

    Value* carryq_value = genCarryInLoad(mptr_carry_q, this_carry_idx);

    //new code chunk, use llvm.uadd.with.overflow.i128
    CastInst* int128_e1 = new BitCastInst(e1, IntegerType::get(mMod->getContext(), 128), "e1_128", mBasicBlock);
    CastInst* int128_e2 = new BitCastInst(e2, IntegerType::get(mMod->getContext(), 128), "e2_128", mBasicBlock);
    CastInst* int128_carryq_value = new BitCastInst(carryq_value, IntegerType::get(mMod->getContext(), 128), "carryq_128", mBasicBlock);

    SumWithOverflowPack sumpack0, sumpack1;

    sumpack0 = genUaddOverflow(int128_e1, int128_e2);
    sumpack1 = genUaddOverflow(sumpack0.sum, int128_carryq_value);

    Value* obit = b.CreateOr(sumpack0.obit, sumpack1.obit, "carry_bit");
    Value* ret_sum = b.CreateBitCast(sumpack1.sum, m64x2Vect, "ret_sum");

    /*obit is the i1 carryout, zero extend and insert it into a v2i64 vector.*/
    ConstantAggregateZero* const_packed_5 = ConstantAggregateZero::get(m64x2Vect);
    ConstantInt* const_int32_6 = ConstantInt::get(mMod->getContext(), APInt(32, StringRef("0"), 10));
    CastInst* int64_o0 = new ZExtInst(obit, IntegerType::get(mMod->getContext(), 64), "o0", mBasicBlock);
    InsertElementInst* carry_out = InsertElementInst::Create(const_packed_5, int64_o0, const_int32_6, "carry_out", mBasicBlock);

    Value* void_1 = genCarryOutStore(carry_out, mptr_carry_q, this_carry_idx);

    return ret_sum;
    //new code chunk ends

    //Old code chunk, calculate carry through logical ops
    //Value* carrygen = b.CreateAnd(e1, e2, "carrygen");
    //Value* carryprop = b.CreateOr(e1, e2, "carryprop");
    //Value* digitsum = b.CreateAdd(e1, e2, "digitsum");
    //Value* partial = b.CreateAdd(digitsum, carryq_value, "partial");
    //Value* digitcarry = b.CreateOr(carrygen, b.CreateAnd(carryprop, genNot(partial)));
    //Value* mid_carry_in = genShiftLeft64(b.CreateLShr(digitcarry, 63), "mid_carry_in");

    //Value* sum = b.CreateAdd(partial, mid_carry_in, "sum");
    //Value* carry_out = genShiftRight127(b.CreateOr(carrygen, b.CreateAnd(carryprop, genNot(sum))), "carry_out");
    //Value* void_1 = genCarryOutStore(carry_out, mptr_carry_q, this_carry_idx);

    //return sum;
    //Old code chunk ends
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
    Value* cast_marker_value_1 = b.CreateBitCast(e, IntegerType::get(mMod->getContext(), 128));
    return b.CreateICmpEQ(cast_marker_value_1, ConstantInt::get(IntegerType::get(mMod->getContext(), 128), 0));
}

Value* LLVM_Generator::genShiftRight127(Value* e, const Twine &namehint) {
    IRBuilder<> b(mBasicBlock);
    Value* i128_val = b.CreateBitCast(e, IntegerType::get(mMod->getContext(), 128));
    return b.CreateBitCast(b.CreateLShr(i128_val, 127, namehint), bitBlockExprType);
}

Value* LLVM_Generator::genShiftLeft64(Value* e, const Twine &namehint) {
    IRBuilder<> b(mBasicBlock);
    Value* i128_val = b.CreateBitCast(e, IntegerType::get(mMod->getContext(), 128));
    return b.CreateBitCast(b.CreateShl(i128_val, 64, namehint), bitBlockExprType);
}

Value* LLVM_Generator::genNot(Value* e, const Twine &namehint) {
    IRBuilder<> b(mBasicBlock);
    return b.CreateXor(e, mConst_Aggregate_64x2_neg1, namehint);
}




