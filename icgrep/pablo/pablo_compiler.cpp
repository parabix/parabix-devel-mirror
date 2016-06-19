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

#define DSSLI_FIELDWIDTH 64

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


Type * PabloCompiler::initializeCarryData() {
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

    Examine(mPabloFunction);
    
    //Generate Kernel//
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));
    mSelf = mKernelBuilder->getParameter(doBlockFunction, "self");
    mCarryManager->initializeCodeGen(mKernelBuilder, mSelf);
      
    Value * inputSet_ptr = mKernelBuilder->getParameter(doBlockFunction, "inputs");
    
    Value * outputSet_ptr = nullptr;
    if (mPabloFunction->getNumOfResults() > 0) {
        outputSet_ptr = mKernelBuilder->getParameter(doBlockFunction, "outputs");
    }
    for (unsigned j = 0; j < mPabloFunction->getNumOfParameters(); ++j) {
        Value * inputVal = iBuilder->CreateGEP(inputSet_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(j)}); 
        //Value * inputVal = iBuilder->CreateBlockAlignedLoad(inputSet_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(j)});
        const Var * const var = mPabloFunction->getParameter(j);
        if (DebugOptionIsSet(DumpTrace)) {
            iBuilder->CallPrintRegister(var->getName()->to_string(), iBuilder->CreateBlockAlignedLoad(inputVal));
        }
        mMarkerMap.insert(std::make_pair(var, inputVal));
    }
    
    compileBlock(mPabloFunction->getEntryBlock());
    
    for (unsigned j = 0; j < mPabloFunction->getNumOfResults(); ++j) {
        const auto f = mMarkerMap.find(mPabloFunction->getResult(j));
        if (LLVM_UNLIKELY(f == mMarkerMap.end())) {
            throw std::runtime_error("PabloCompiler: result " + std::to_string(j) + " was not assigned a value!");
        }
        iBuilder->CreateBlockAlignedStore(f->second, outputSet_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(j)});
    }
    mKernelBuilder->setScalarField(mSelf, "BlockNo", iBuilder->CreateAdd(mKernelBuilder->getScalarField(mSelf, "BlockNo"), iBuilder->getInt64(1)));
    iBuilder->CreateRetVoid();

    
    #ifdef PRINT_TIMING_INFORMATION
    const timestamp_t pablo_compilation_end = read_cycle_counter();
    std::cerr << "PABLO COMPILATION TIME: " << (pablo_compilation_end - pablo_compilation_start) << std::endl;
    #endif

}

inline void PabloCompiler::Examine(const PabloFunction * const function) {
    mWhileDepth = 0;
    mIfDepth = 0;
    mMaxWhileDepth = 0;
    LookaheadOffsetMap offsetMap;
    Examine(function->getEntryBlock(), offsetMap);
    mInputStreamOffset.clear();
    for (const auto & oi : offsetMap) {
        for (const auto offset : oi.second) {
            mInputStreamOffset.insert(offset / iBuilder->getBitBlockWidth());
        }
    }
}

void PabloCompiler::Examine(const PabloBlock * const block, LookaheadOffsetMap & offsetMap) {
    for (const Statement * stmt : *block) {
         boost::container::flat_set<unsigned> offsets;
        if (LLVM_UNLIKELY(isa<Lookahead>(stmt))) {
            const Lookahead * const la = cast<Lookahead>(stmt);
            assert (isa<Var>(la->getExpr()));
            offsets.insert(la->getAmount());
            offsets.insert(la->getAmount() + iBuilder->getBitBlockWidth() - 1);
        } else {
            for (unsigned i = 0; i < stmt->getNumOperands(); ++i) {
                const PabloAST * expr = stmt->getOperand(i);
                if (isa<Var>(expr)) {
                    offsets.insert(0);
                } else if (LLVM_LIKELY(isa<Statement>(expr) && !isa<Assign>(expr) && !isa<Next>(expr))) {
                    const auto f = offsetMap.find(expr);
                    assert (f != offsetMap.end());
                    const auto & o = f->second;
                    offsets.insert(o.begin(), o.end());
                }
            }
            if (LLVM_UNLIKELY(isa<If>(stmt))) {
                Examine(cast<If>(stmt)->getBody(), offsetMap);
            } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
                mMaxWhileDepth = std::max(mMaxWhileDepth, ++mWhileDepth);
                Examine(cast<While>(stmt)->getBody(), offsetMap);
                --mWhileDepth;
            }
        }
        offsetMap.emplace(stmt, offsets);
    }
}

void PabloCompiler::compileBlock(const PabloBlock * const block) {
    mPabloBlock = block;
    for (const Statement * statement : *block) {
        compileStatement(statement);
    }
    mPabloBlock = block->getParent();
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
        mMarkerMap[assign] = phi;
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

    mCarryManager->initializeWhileEntryCarryDataPhis(whileEntryBlock);

    // for any Next nodes in the loop body, initialize to (a) pre-loop value.
    for (const Next * n : nextNodes) {
        PHINode * phi = iBuilder->CreatePHI(mBitBlockType, 2, n->getName()->value());
        auto f = mMarkerMap.find(n->getInitial());
        assert (f != mMarkerMap.end());
        phi->addIncoming(f->second, whileEntryBlock);
        mMarkerMap[n->getInitial()] = phi;
        nextPhis.push_back(phi);
    }

    //
    // Now compile the loop body proper.  Carry-out accumulated values
    // and iterated values of Next nodes will be computed.
    ++mWhileDepth;
    compileBlock(whileBody);

    BasicBlock * whileExitBlock = iBuilder->GetInsertBlock();

    if (mCarryManager->hasCarries()) {
        mCarryManager->storeCarryOutSummary();
    }
    mCarryManager->finalizeWhileBlockCarryDataPhis(whileExitBlock);

    // Terminate the while loop body with a conditional branch back.
    iBuilder->CreateCondBr(iBuilder->bitblock_any(compileExpression(whileStatement->getCondition())), whileBodyBlock, whileEndBlock);

    // and for any Next nodes in the loop body
    for (unsigned i = 0; i < nextNodes.size(); i++) {
        const Next * n = nextNodes[i];
        auto f = mMarkerMap.find(n->getExpr());
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
        Value * EOFmark = mKernelBuilder->getScalarField(mSelf, "EOFmark");
        Value * infileMask = iBuilder->simd_add(iBuilder->getBitBlockWidth(), EOFmark, iBuilder->allOnes());
        expr = iBuilder->simd_and(compileExpression(e->getExpr()), infileMask);
    } else if (const AtEOF * e = dyn_cast<AtEOF>(stmt)) {
        Value * EOFmark = mKernelBuilder->getScalarField(mSelf, "EOFmark");
		expr = iBuilder->simd_and(compileExpression(e->getExpr()), EOFmark);
    } else if (const Count * c = dyn_cast<Count>(stmt)) {
        Value * const to_count = compileExpression(c->getExpr());
        std::string counter = c->getName()->to_string();
        Value * countSoFar = mKernelBuilder->getScalarField(mSelf, counter);
        Value * fieldCounts = iBuilder->simd_popcount(64, to_count);
        for (unsigned i = 0; i < iBuilder->getBitBlockWidth()/64; ++i) {
            countSoFar = iBuilder->CreateAdd(countSoFar, iBuilder->mvmd_extract(64, fieldCounts, i));
        }
        mKernelBuilder->setScalarField(mSelf, counter, countSoFar);
        expr = iBuilder->bitCast(iBuilder->CreateZExt(countSoFar, iBuilder->getIntNTy(iBuilder->getBitBlockWidth())));
    } else if (const Lookahead * l = dyn_cast<Lookahead>(stmt)) {
        PabloAST * const var = l->getExpr();
        if (LLVM_UNLIKELY(!isa<Var>(var))) {
            throw std::runtime_error("Lookahead input type must be a Var object");
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
        const unsigned offset0 = (l->getAmount() / iBuilder->getBitBlockWidth());
        const unsigned offset1 = ((l->getAmount() + iBuilder->getBitBlockWidth() - 1) / iBuilder->getBitBlockWidth());
        const unsigned shift = (l->getAmount() % iBuilder->getBitBlockWidth());
        Value * const v0 = nullptr;//iBuilder->CreateBlockAlignedLoad(mKernelBuilder->getInputStream(index, offset0));
        Value * const v1 = nullptr;//iBuilder->CreateBlockAlignedLoad(mKernelBuilder->getInputStream(index, offset1));
        if (LLVM_UNLIKELY((shift % 8) == 0)) { // Use a single whole-byte shift, if possible.
            expr = iBuilder->mvmd_dslli(8, v1, v0, (shift / 8));
        } else if (LLVM_LIKELY(shift < DSSLI_FIELDWIDTH)) {
            Value * ahead = iBuilder->mvmd_dslli(DSSLI_FIELDWIDTH, v1, v0, 1);
            ahead = iBuilder->simd_slli(DSSLI_FIELDWIDTH, ahead, DSSLI_FIELDWIDTH - shift);
            Value * value = iBuilder->simd_srli(DSSLI_FIELDWIDTH, v0, shift);
            expr = iBuilder->simd_or(value, ahead);
        } else {
            Type  * const streamType = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());
            Value * b0 = iBuilder->CreateBitCast(v0, streamType);
            Value * b1 = iBuilder->CreateBitCast(v1, streamType);
            Value * result = iBuilder->CreateOr(iBuilder->CreateShl(b1, iBuilder->getBitBlockWidth() - shift), iBuilder->CreateLShr(b0, shift));
            expr = iBuilder->CreateBitCast(result, mBitBlockType);
        }
    } else {
        std::string tmp;
        llvm::raw_string_ostream msg(tmp);
        msg << "Internal error: ";
        PabloPrinter::print(stmt, msg);
        msg << " is not a recognized statement in the Pablo compiler.";
        throw std::runtime_error(msg.str());
    }
    mMarkerMap[stmt] = expr;
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
        std::string o;
        llvm::raw_string_ostream str(o);
        str << "\"";
        PabloPrinter::print(expr, str);
        str << "\" was used before definition!";
        throw std::runtime_error(str.str());
    }
    Value * result = f->second;
    if (LLVM_UNLIKELY(isa<Var>(expr))) {
        assert (isa<GetElementPtrInst>(result));
        result = iBuilder->CreateBlockAlignedLoad(result);
    }
    return result;
}

}
