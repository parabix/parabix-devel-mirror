/*
 *  Copyright (c) 2014-16 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pablo_compiler.h>
#include <pablo/pablo_toolchain.h>
#include <pablo/codegenstate.h>
#include <pablo/carry_manager.h>
#include <pablo/printer_pablos.h>
#include <pablo/function.h>
#include <re/re_name.h>
#include <stdexcept>
#include <sstream>
#include <IDISA/idisa_builder.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/IRBuilder.h>
#include <iostream>
#include <hrtime.h>
#include <llvm/Support/Debug.h>

namespace pablo {

PabloCompiler::PabloCompiler(IDISA::IDISA_Builder * b, PabloKernel * k, PabloFunction * const function)
: mMod(b->getModule())
, iBuilder(b)
, mBitBlockType(b->getBitBlockType())
, mCarryManager(nullptr)
, mPabloFunction(function)
, mPabloBlock(nullptr)
, mKernelBuilder(k)
, mWhileDepth(0)
, mIfDepth(0)
, mFunction(nullptr)
, mMaxWhileDepth(0) {
    
}


Type * PabloCompiler::initializeKernelData() {
    Examine(mPabloFunction);
    
    mCarryManager = make_unique<CarryManager>(iBuilder);
    Type * carryDataType = mCarryManager->initializeCarryData(mPabloFunction);
    return carryDataType;
}
    
void PabloCompiler::compile(Function * doBlockFunction) {

    // Make sure that we generate code into the right module.
    mMod = iBuilder->getModule();
    mFunction = doBlockFunction;
    #ifdef PRINT_TIMING_INFORMATION
    const timestamp_t pablo_compilation_start = read_cycle_counter();
    #endif

    //Generate Kernel//
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));
    mSelf = mKernelBuilder->getParameter(doBlockFunction, "self");
    mCarryManager->initializeCodeGen(mKernelBuilder, mSelf);
      
    Value * blockNo = mKernelBuilder->getScalarField(mSelf, blockNoScalar);
    std::string inputName = mKernelBuilder->mStreamSetInputs[0].ssName;
    Value * inputSet_ptr  = mKernelBuilder->getStreamSetBlockPtr(mSelf, inputName, blockNo);

    Value * outputSet_ptr = nullptr;
    if (mPabloFunction->getNumOfResults() > 0) {
        std::string outputName = mKernelBuilder->mStreamSetOutputs[0].ssName;
        outputSet_ptr = mKernelBuilder->getStreamSetBlockPtr(mSelf, outputName, blockNo);
    }

    PabloBlock * const entryBlock = mPabloFunction->getEntryBlock();
    mMarkerMap.emplace(entryBlock->createZeroes(), iBuilder->allZeroes());
    mMarkerMap.emplace(entryBlock->createOnes(), iBuilder->allOnes());
    for (unsigned j = 0; j < mPabloFunction->getNumOfParameters(); ++j) {
        Value * inputVal = iBuilder->CreateGEP(inputSet_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(j)}); 
        const Var * const var = mPabloFunction->getParameter(j);
        if (DebugOptionIsSet(DumpTrace)) {
            iBuilder->CallPrintRegister(var->getName()->to_string(), iBuilder->CreateBlockAlignedLoad(inputVal));
        }
        mMarkerMap.emplace(var, inputVal);
    }
    
    compileBlock(entryBlock);
    
    for (unsigned j = 0; j < mPabloFunction->getNumOfResults(); ++j) {
        const auto f = mMarkerMap.find(mPabloFunction->getResult(j));
        if (LLVM_UNLIKELY(f == mMarkerMap.end())) {
            throw std::runtime_error("PabloCompiler: result " + std::to_string(j) + " was not assigned a value!");
        }
        iBuilder->CreateBlockAlignedStore(f->second, outputSet_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(j)});
    }
    
    #ifdef PRINT_TIMING_INFORMATION
    const timestamp_t pablo_compilation_end = read_cycle_counter();
    std::cerr << "PABLO COMPILATION TIME: " << (pablo_compilation_end - pablo_compilation_start) << std::endl;
    #endif
}

inline void PabloCompiler::Examine(const PabloFunction * const function) {
    mWhileDepth = 0;
    mIfDepth = 0;
    mMaxWhileDepth = 0;
    Examine(function->getEntryBlock());
}

void PabloCompiler::Examine(const PabloBlock * const block) {
    unsigned maxOffset = 0;
    for (const Statement * stmt : *block) {
         boost::container::flat_set<unsigned> offsets;
        if (LLVM_UNLIKELY(isa<Lookahead>(stmt))) {
            const Lookahead * const la = cast<Lookahead>(stmt);
            assert (isa<Var>(la->getExpr()));
            if (la->getAmount() > maxOffset) maxOffset = la->getAmount();
        } else {
            if (LLVM_UNLIKELY(isa<If>(stmt))) {
                Examine(cast<If>(stmt)->getBody());
            } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
                mMaxWhileDepth = std::max(mMaxWhileDepth, ++mWhileDepth);
                Examine(cast<While>(stmt)->getBody());
                --mWhileDepth;
            }
        }
        mKernelBuilder->setLookAhead(maxOffset);
    }
}

void PabloCompiler::compileBlock(const PabloBlock * const block) {
    mPabloBlock = block;
    for (const Statement * statement : *block) {
        compileStatement(statement);
    }
    mPabloBlock = block->getPredecessor ();
}

void PabloCompiler::compileIf(const If * ifStatement) {        
    //
    //  The If-ElseZero stmt:
    //  if <predicate:expr> then <body:stmt>* elsezero <defined:var>* endif
    //  If the value of the predicate is nonzero, then determine the values of variables
    //  <var>* by executing the given statements.  Otherwise, the value of the
    //  variables are all zero.  Requirements: (a) no variable that is defined within
    //  the body of the if may be accessed outside unless it is explicitly
    //  listed in the variable list, (b) every variable in the defined list receives
    //  a value within the body, and (c) the logical consequence of executing
    //  the statements in the event that the predicate is zero is that the
    //  values of all defined variables indeed work out to be 0.
    //
    //  Simple Implementation with Phi nodes:  a phi node in the if exit block
    //  is inserted for each variable in the defined variable list.  It receives
    //  a zero value from the ifentry block and the defined value from the if
    //  body.
    //

    BasicBlock * const ifEntryBlock = iBuilder->GetInsertBlock();
    BasicBlock * const ifBodyBlock = BasicBlock::Create(mMod->getContext(), "if.body", mFunction, 0);
    BasicBlock * const ifEndBlock = BasicBlock::Create(mMod->getContext(), "if.end", mFunction, 0);
    
    PabloBlock * ifBody = ifStatement->getBody();
    
    Value * const condition = compileExpression(ifStatement->getCondition());
    
    mCarryManager->enterScope(ifBody);
    iBuilder->CreateCondBr(mCarryManager->generateSummaryTest(condition), ifBodyBlock, ifEndBlock);
    
    // Entry processing is complete, now handle the body of the if.
    iBuilder->SetInsertPoint(ifBodyBlock);
    
    compileBlock(ifBody);
    BasicBlock * ifExitBlock = iBuilder->GetInsertBlock();

    if (mCarryManager->hasCarries()) {
        mCarryManager->storeCarryOutSummary();
    }
    mCarryManager->addOuterSummaryToNestedSummary();

    iBuilder->CreateBr(ifEndBlock);
    //End Block
    iBuilder->SetInsertPoint(ifEndBlock);
    for (const PabloAST * node : ifStatement->getDefined()) {
        const Assign * assign = cast<Assign>(node);
        PHINode * phi = iBuilder->CreatePHI(mBitBlockType, 2, assign->getName()->value());
        auto f = mMarkerMap.find(assign);
        assert (f != mMarkerMap.end());
        phi->addIncoming(iBuilder->allZeroes(), ifEntryBlock);
        phi->addIncoming(f->second, ifExitBlock);
        f->second = phi;
        assert (mMarkerMap[assign] == phi);
    }
    // Create the phi Node for the summary variable, if needed.
    mCarryManager->buildCarryDataPhisAfterIfBody(ifEntryBlock, ifExitBlock);
    mCarryManager->leaveScope();
}

void PabloCompiler::compileWhile(const While * whileStatement) {

    PabloBlock * const whileBody = whileStatement->getBody();
    
    BasicBlock * whileEntryBlock = iBuilder->GetInsertBlock();
    BasicBlock * whileBodyBlock = BasicBlock::Create(mMod->getContext(), "while.body", mFunction, 0);
    BasicBlock * whileEndBlock = BasicBlock::Create(mMod->getContext(), "while.end", mFunction, 0);

    mCarryManager->enterScope(whileBody);
    mCarryManager->ensureCarriesLoadedRecursive();

    const auto & nextNodes = whileStatement->getVariants();
    std::vector<PHINode *> nextPhis;
    nextPhis.reserve(nextNodes.size());
#ifdef ENABLE_BOUNDED_WHILE
    PHINode * bound_phi = nullptr;  // Needed for bounded while loops.
#endif
    // On entry to the while structure, proceed to execute the first iteration
    // of the loop body unconditionally.   The while condition is tested at the end of
    // the loop.

    iBuilder->CreateBr(whileBodyBlock);
    iBuilder->SetInsertPoint(whileBodyBlock);

    //
    // There are 3 sets of Phi nodes for the while loop.
    // (1) Carry-ins: (a) incoming carry data first iterations, (b) zero thereafter
    // (2) Carry-out accumulators: (a) zero first iteration, (b) |= carry-out of each iteration
    // (3) Next nodes: (a) values set up before loop, (b) modified values calculated in loop.
#ifdef ENABLE_BOUNDED_WHILE
    // (4) The loop bound, if any.
#endif

    mCarryManager->initializeWhileEntryCarryDataPhis(whileEntryBlock);

    // for any Next nodes in the loop body, initialize to (a) pre-loop value.
    for (const Next * n : nextNodes) {
        PHINode * phi = iBuilder->CreatePHI(mBitBlockType, 2, n->getName()->value());
        auto f = mMarkerMap.find(n->getInitial());        
        assert (f != mMarkerMap.end());
        phi->addIncoming(f->second, whileEntryBlock);
        f->second = phi;
        assert(mMarkerMap[n->getInitial()] == phi);
        nextPhis.push_back(phi);
    }
#ifdef ENABLE_BOUNDED_WHILE
    if (whileStatement->getBound()) {
        bound_phi = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "while_bound");
        bound_phi->addIncoming(ConstantInt::get(iBuilder->getSizeTy(), whileStatement->getBound()), whileEntryBlock);
    }
#endif
    //
    // Now compile the loop body proper.  Carry-out accumulated values
    // and iterated values of Next nodes will be computed.
    ++mWhileDepth;
    compileBlock(whileBody);

    // After the whileBody has been compiled, we may be in a different basic block.
    BasicBlock * whileExitBlock = iBuilder->GetInsertBlock();

    if (mCarryManager->hasCarries()) {
        mCarryManager->storeCarryOutSummary();
    }
    mCarryManager->finalizeWhileBlockCarryDataPhis(whileExitBlock);

    // Terminate the while loop body with a conditional branch back.
    Value * cond_expr = iBuilder->bitblock_any(compileExpression(whileStatement->getCondition()));
#ifdef ENABLE_BOUNDED_WHILE
    if (whileStatement->getBound()) {
        Value * new_bound = iBuilder->CreateSub(bound_phi, ConstantInt::get(iBuilder->getSizeTy(), 1));
        bound_phi->addIncoming(new_bound, whileExitBlock);
        cond_expr = iBuilder->CreateAnd(cond_expr, iBuilder->CreateICmpUGT(new_bound, ConstantInt::getNullValue(iBuilder->getSizeTy())));
    }
#endif    
    iBuilder->CreateCondBr(cond_expr, whileBodyBlock, whileEndBlock);

    // and for any Next nodes in the loop body
    for (unsigned i = 0; i < nextNodes.size(); i++) {
        const Next * n = nextNodes[i];
        const auto f = mMarkerMap.find(n->getExpr());
        if (LLVM_UNLIKELY(f == mMarkerMap.end())) {
            throw std::runtime_error("Next node expression was not compiled!");
        }
        nextPhis[i]->addIncoming(f->second, whileExitBlock);
    }

    iBuilder->SetInsertPoint(whileEndBlock);
    --mWhileDepth;

    mCarryManager->ensureCarriesStoredRecursive();
    mCarryManager->leaveScope();
}


void PabloCompiler::compileStatement(const Statement * stmt) {
    Value * expr = nullptr;
    if (const Assign * assign = dyn_cast<const Assign>(stmt)) {
        expr = compileExpression(assign->getExpression());
    } else if (const Next * next = dyn_cast<const Next>(stmt)) {
        expr = compileExpression(next->getExpr());
    } else if (const If * ifStatement = dyn_cast<const If>(stmt)) {
        compileIf(ifStatement);
        return;
    } else if (const While * whileStatement = dyn_cast<const While>(stmt)) {
        compileWhile(whileStatement);
        return;
//    } else if (const Call* call = dyn_cast<Call>(stmt)) {
//        // Call the callee once and store the result in the marker map.
//        if (LLVM_UNLIKELY(mMarkerMap.count(call) == 0)) {
//            return;
//        }

//        const Prototype * proto = call->getPrototype();
//        const String * callee = proto->getName();

//        Type * inputType = StructType::get(mMod->getContext(), std::vector<Type *>{proto->getNumOfParameters(), mBitBlockType});
//        Type * outputType = StructType::get(mMod->getContext(), std::vector<Type *>{proto->getNumOfResults(), mBitBlockType});
//        FunctionType * functionType = FunctionType::get(Type::getVoidTy(mMod->getContext()), std::vector<Type *>{PointerType::get(inputType, 0), PointerType::get(outputType, 0)}, false);

//        //Starts on process_block
//        SmallVector<AttributeSet, 3> Attrs;
//        Attrs.push_back(AttributeSet::get(mMod->getContext(), 1U, std::vector<Attribute::AttrKind>({ Attribute::ReadOnly, Attribute::NoCapture })));
//        Attrs.push_back(AttributeSet::get(mMod->getContext(), 2U, std::vector<Attribute::AttrKind>({ Attribute::ReadNone, Attribute::NoCapture })));
//        AttributeSet AttrSet = AttributeSet::get(mMod->getContext(), Attrs);

//        Function * externalFunction = cast<Function>(mMod->getOrInsertFunction(callee->value(), functionType, AttrSet));
//        if (LLVM_UNLIKELY(externalFunction == nullptr)) {
//            throw std::runtime_error("Could not create static method call for external function \"" + callee->to_string() + "\"");
//        }
//        externalFunction->setCallingConv(llvm::CallingConv::C);

//        AllocaInst * outputStruct = iBuilder->CreateAlloca(outputType);
//        iBuilder->CreateCall2(externalFunction, mInputAddressPtr, outputStruct);
//        Value * outputPtr = iBuilder->CreateGEP(outputStruct, std::vector<Value *>({ iBuilder->getInt32(0), iBuilder->getInt32(0) }));

//        expr = iBuilder->CreateBlockAlignedLoad(outputPtr);
    } else if (const And * pablo_and = dyn_cast<And>(stmt)) {
        expr = iBuilder->simd_and(compileExpression(pablo_and->getOperand(0)), compileExpression(pablo_and->getOperand(1)));
    } else if (const Or * pablo_or = dyn_cast<Or>(stmt)) {
        expr = iBuilder->simd_or(compileExpression(pablo_or->getOperand(0)), compileExpression(pablo_or->getOperand(1)));
    } else if (const Xor * pablo_xor = dyn_cast<Xor>(stmt)) {
        expr = iBuilder->simd_xor(compileExpression(pablo_xor->getOperand(0)), compileExpression(pablo_xor->getOperand(1)));
    } else if (const Sel * sel = dyn_cast<Sel>(stmt)) {
        Value* ifMask = compileExpression(sel->getCondition());
        Value* ifTrue = iBuilder->simd_and(ifMask, compileExpression(sel->getTrueExpr()));
        Value* ifFalse = iBuilder->simd_and(iBuilder->simd_not(ifMask), compileExpression(sel->getFalseExpr()));
        expr = iBuilder->simd_or(ifTrue, ifFalse);
    } else if (const Not * pablo_not = dyn_cast<Not>(stmt)) {
        expr = iBuilder->simd_not(compileExpression(pablo_not->getExpr()));
    } else if (const Advance * adv = dyn_cast<Advance>(stmt)) {
        Value * const strm_value = compileExpression(adv->getExpr());
        expr = mCarryManager->advanceCarryInCarryOut(adv->getLocalIndex(), adv->getAmount(), strm_value);
    } else if (const MatchStar * mstar = dyn_cast<MatchStar>(stmt)) {
        Value * const marker = compileExpression(mstar->getMarker());
        Value * const cc = compileExpression(mstar->getCharClass());
        Value * const marker_and_cc = iBuilder->simd_and(marker, cc);
        Value * const sum = mCarryManager->addCarryInCarryOut(mstar->getLocalCarryIndex(), marker_and_cc, cc);
        expr = iBuilder->simd_or(iBuilder->simd_xor(sum, cc), marker);
    } else if (const ScanThru * sthru = dyn_cast<ScanThru>(stmt)) {
        Value * const  marker_expr = compileExpression(sthru->getScanFrom());
        Value * const  cc_expr = compileExpression(sthru->getScanThru());
        Value * const  sum = mCarryManager->addCarryInCarryOut(sthru->getLocalCarryIndex(), marker_expr, cc_expr);
        expr = iBuilder->simd_and(sum, iBuilder->simd_not(cc_expr));
    } else if (const InFile * e = dyn_cast<InFile>(stmt)) {
        Value * EOFmask = mKernelBuilder->getScalarField(mSelf, "EOFmask");
        expr = iBuilder->simd_xor(compileExpression(e->getExpr()), EOFmask);
    } else if (const AtEOF * e = dyn_cast<AtEOF>(stmt)) {
        Value * EOFbit = mKernelBuilder->getScalarField(mSelf, "EOFbit");
		expr = iBuilder->simd_and(compileExpression(e->getExpr()), EOFbit);
    } else if (const Count * c = dyn_cast<Count>(stmt)) {
        Value * const to_count = compileExpression(c->getExpr());
        std::string counter = c->getName()->to_string();
        Value * countSoFar = mKernelBuilder->getScalarField(mSelf, counter);
        unsigned counterSize = countSoFar->getType()->getIntegerBitWidth();
        Value * fieldCounts = iBuilder->simd_popcount(counterSize, to_count);
        for (unsigned i = 0; i < iBuilder->getBitBlockWidth()/counterSize; ++i) {
            countSoFar = iBuilder->CreateAdd(countSoFar, iBuilder->mvmd_extract(counterSize, fieldCounts, i));
        }
        mKernelBuilder->setScalarField(mSelf, counter, countSoFar);
        expr = iBuilder->bitCast(iBuilder->CreateZExt(countSoFar, iBuilder->getIntNTy(iBuilder->getBitBlockWidth())));
    } else if (const Lookahead * l = dyn_cast<Lookahead>(stmt)) {
        PabloAST * const var = l->getExpr();
        if (LLVM_UNLIKELY(!isa<Var>(var))) {
            throw std::runtime_error("Lookahead operations may only be applied to input streams");
        }
        unsigned index = 0;
        for (; index < mPabloFunction->getNumOfParameters(); ++index) {
            if (mPabloFunction->getParameter(index) == var) {
                break;
            }
        }
        if (LLVM_UNLIKELY(index >= mPabloFunction->getNumOfParameters())) {
            throw std::runtime_error("Lookahead has an illegal Var operand");
        }
        const unsigned bit_shift = (l->getAmount() % iBuilder->getBitBlockWidth());
        const unsigned block_shift = (l->getAmount() / iBuilder->getBitBlockWidth());
        std::string inputName = mKernelBuilder->mStreamSetInputs[0].ssName;
        Value * blockNo = mKernelBuilder->getScalarField(mSelf, blockNoScalar);
        Value * lookAhead_blockPtr  = mKernelBuilder->getStreamSetBlockPtr(mSelf, inputName, iBuilder->CreateAdd(blockNo, ConstantInt::get(iBuilder->getSizeTy(), block_shift)));
        Value * lookAhead_inputPtr = iBuilder->CreateGEP(lookAhead_blockPtr, {iBuilder->getInt32(0), iBuilder->getInt32(index)});
        Value * lookAhead = iBuilder->CreateBlockAlignedLoad(lookAhead_inputPtr);
        if (bit_shift == 0) {  // Simple case with no intra-block shifting.
            expr = lookAhead;  
        }
        else { // Need to form shift result from two adjacent blocks.
            Value * lookAhead_blockPtr1  = mKernelBuilder->getStreamSetBlockPtr(mSelf, inputName, iBuilder->CreateAdd(blockNo, ConstantInt::get(iBuilder->getSizeTy(), block_shift + 1)));
            Value * lookAhead_inputPtr1 = iBuilder->CreateGEP(lookAhead_blockPtr1, {iBuilder->getInt32(0), iBuilder->getInt32(index)});
            Value * lookAhead1 = iBuilder->CreateBlockAlignedLoad(lookAhead_inputPtr1);
            if (LLVM_UNLIKELY((bit_shift % 8) == 0)) { // Use a single whole-byte shift, if possible.
                expr = iBuilder->mvmd_dslli(8, lookAhead1, lookAhead, (bit_shift / 8));
            }
            else {
                Type  * const streamType = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());
                Value * b1 = iBuilder->CreateBitCast(lookAhead1, streamType);
                Value * b0 = iBuilder->CreateBitCast(lookAhead, streamType);
                Value * result = iBuilder->CreateOr(iBuilder->CreateShl(b1, iBuilder->getBitBlockWidth() - bit_shift), iBuilder->CreateLShr(b0, bit_shift));
                expr = iBuilder->CreateBitCast(result, mBitBlockType);
            }
        }
    } else {
        std::string tmp;
        llvm::raw_string_ostream msg(tmp);
        msg << "Internal error: ";
        PabloPrinter::print(stmt, msg);
        msg << " is not a recognized statement in the Pablo compiler.";
        throw std::runtime_error(msg.str());
    }
    mMarkerMap.emplace(stmt, expr);
    if (DebugOptionIsSet(DumpTrace)) {
        iBuilder->CallPrintRegister(stmt->getName()->to_string(), expr);
    }
    
}

Value * PabloCompiler::compileExpression(const PabloAST * expr) {
    if (LLVM_UNLIKELY(isa<Ones>(expr))) {
        return iBuilder->allOnes();
    } else if (LLVM_UNLIKELY(isa<Zeroes>(expr))) {
        return iBuilder->allZeroes();
    }
    auto f = mMarkerMap.find(expr);
    if (LLVM_UNLIKELY(f == mMarkerMap.end())) {
        std::string tmp;
        llvm::raw_string_ostream out(tmp);
        out << "\"";
        PabloPrinter::print(expr, out);
        out << "\" was used before definition!";
        throw std::runtime_error(out.str());
    }
    Value * result = f->second;
    if (LLVM_UNLIKELY(isa<Var>(expr))) {
        assert (isa<GetElementPtrInst>(result));
        result = iBuilder->CreateBlockAlignedLoad(result);
    }
    return result;
}

}
