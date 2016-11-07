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

PabloCompiler::PabloCompiler(PabloKernel * k, PabloFunction * const function)
: iBuilder(k->getBuilder())
, mBitBlockType(iBuilder->getBitBlockType())
, mCarryManager(nullptr)
, mPabloFunction(function)
, mKernelBuilder(k)
, mWhileDepth(0)
, mIfDepth(0)
, mFunction(nullptr)
, mMaxWhileDepth(0) {
    
}

Type * PabloCompiler::initializeKernelData() {
    Examine(mPabloFunction);    
    mCarryManager = std::unique_ptr<CarryManager>(new CarryManager(iBuilder));
    Type * carryDataType = mCarryManager->initializeCarryData(mPabloFunction);
    return carryDataType;
}
    
void PabloCompiler::verifyParameter(const Var * var, const Value * param) {
    if (LLVM_UNLIKELY(&(param->getContext()) != &(iBuilder->getContext()))) {
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "Cannot compile ";
        mPabloFunction->print(out);
        out << ": LLVM Context for ";
        var->print(out);
        out << " differs from that of the kernel.";
        throw std::runtime_error(out.str());
    }
}

void PabloCompiler::compile(Function * doBlockFunction) {

    // Make sure that we generate code into the right module.
    mFunction = doBlockFunction;
    #ifdef PRINT_TIMING_INFORMATION
    const timestamp_t pablo_compilation_start = read_cycle_counter();
    #endif

    //Generate Kernel//
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));
    mSelf = mKernelBuilder->getParameter(doBlockFunction, "self");

    mCarryManager->initializeCodeGen(mKernelBuilder, mSelf);
      
    PabloBlock * const entryBlock = mPabloFunction->getEntryBlock(); assert (entryBlock);
    mMarkerMap.emplace(entryBlock->createZeroes(), iBuilder->allZeroes());
    mMarkerMap.emplace(entryBlock->createOnes(), iBuilder->allOnes());

    Value * const blockNo = mKernelBuilder->getScalarField(mSelf, blockNoScalar);

    for (unsigned i = 0, j = 0; i < mPabloFunction->getNumOfParameters(); ++i) {
        Var * var = mPabloFunction->getParameter(i);
        std::string name = var->getName()->to_string();
        Value * input = nullptr;
        if (var->getType()->isSingleValueType()) {
            input = mKernelBuilder->getScalarFieldPtr(mSelf, name);
        } else {
            input = mKernelBuilder->getStreamSetBlockPtr(mSelf, name, blockNo);
            input = iBuilder->CreateGEP(input, {iBuilder->getInt32(0), iBuilder->getInt32(j++)});
        }
        verifyParameter(var, input);
        mMarkerMap.emplace(var, input);
    }

    for (unsigned i = 0, j = 0; i < mPabloFunction->getNumOfResults(); ++i) {
        Var * var = mPabloFunction->getResult(i);
        std::string name = var->getName()->to_string();
        Value * output = nullptr;
        if (var->getType()->isSingleValueType()) {
            output = mKernelBuilder->getScalarFieldPtr(mSelf, name);
        } else {
            output = mKernelBuilder->getStreamSetBlockPtr(mSelf, name, blockNo);
            output = iBuilder->CreateGEP(output, {iBuilder->getInt32(0), iBuilder->getInt32(j++)});
        }
        verifyParameter(var, output);
        mMarkerMap.emplace(var, output);
    }

    compileBlock(entryBlock);

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
    for (const Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<Lookahead>(stmt))) {
            const Lookahead * const la = cast<Lookahead>(stmt);
            assert (isa<Var>(la->getExpr()));
            if (LLVM_LIKELY(la->getAmount() > mKernelBuilder->getLookAhead())) {
                mKernelBuilder->setLookAhead(la->getAmount());
            }
        } else {
            if (LLVM_UNLIKELY(isa<If>(stmt))) {
                Examine(cast<If>(stmt)->getBody());
            } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
                mMaxWhileDepth = std::max(mMaxWhileDepth, ++mWhileDepth);
                Examine(cast<While>(stmt)->getBody());
                --mWhileDepth;
            }
        }
    }    
}

inline void PabloCompiler::compileBlock(const PabloBlock * const block) {
    for (const Statement * statement : *block) {
        compileStatement(statement);
    }
}

static const llvm::StringRef EmptyString;

inline const llvm::StringRef & getName(const PabloAST * expr) {
    if (expr->getName()) {
        return expr->getName()->value();
    }
    return EmptyString;
}

void PabloCompiler::compileIf(const If * const ifStatement) {
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

    Module * const mod = iBuilder->getModule();
    BasicBlock * const ifEntryBlock = iBuilder->GetInsertBlock();
    BasicBlock * const ifBodyBlock = BasicBlock::Create(mod->getContext(), "if.body", mFunction, 0);
    BasicBlock * const ifEndBlock = BasicBlock::Create(mod->getContext(), "if.end", mFunction, 0);
    
    std::vector<std::pair<const Var *, Value *>> incoming;

    for (const Var * var : ifStatement->getEscaped()) {
        auto f = mMarkerMap.find(var);
        if (LLVM_UNLIKELY(f == mMarkerMap.end())) {
            std::string tmp;
            raw_string_ostream out(tmp);
            PabloPrinter::print(var, out);
            out << " is uninitialized prior to entering ";
            PabloPrinter::print(ifStatement, out);
            llvm::report_fatal_error(out.str());
        }
        incoming.emplace_back(var, f->second);
    }

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
    for (const auto i : incoming) {
        const Var * var; Value * value;
        std::tie(var, value) = i;

        auto f = mMarkerMap.find(var);
        if (LLVM_UNLIKELY(f == mMarkerMap.end() || f->second == value)) {
            std::string tmp;
            raw_string_ostream out(tmp);
            PabloPrinter::print(var, out);
            out << " was not assigned a value.";
            llvm::report_fatal_error(out.str());
        }

        PHINode * phi = iBuilder->CreatePHI(mBitBlockType, 2, getName(var));
        phi->addIncoming(value, ifEntryBlock);
        phi->addIncoming(f->second, ifExitBlock);
        f->second = phi;

        assert (mMarkerMap[var] == phi);
    }
    // Create the phi Node for the summary variable, if needed.
    mCarryManager->buildCarryDataPhisAfterIfBody(ifEntryBlock, ifExitBlock);
    mCarryManager->leaveScope();
}

void PabloCompiler::compileWhile(const While * const whileStatement) {

    PabloBlock * const whileBody = whileStatement->getBody();
    
    BasicBlock * whileEntryBlock = iBuilder->GetInsertBlock();

    Module * const mod = iBuilder->getModule();
    BasicBlock * whileBodyBlock = BasicBlock::Create(mod->getContext(), "while.body", mFunction, 0);
    BasicBlock * whileEndBlock = BasicBlock::Create(mod->getContext(), "while.end", mFunction, 0);

    mCarryManager->enterScope(whileBody);
    mCarryManager->ensureCarriesLoadedRecursive();

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

    std::vector<std::pair<const Var *, PHINode *>> variants;

    // for any Next nodes in the loop body, initialize to (a) pre-loop value.
    for (const auto var : whileStatement->getEscaped()) {
        PHINode * phi = iBuilder->CreatePHI(mBitBlockType, 2, getName(var));
        auto f = mMarkerMap.find(var);
        if (LLVM_UNLIKELY(f == mMarkerMap.end())) {
            std::string tmp;
            raw_string_ostream out(tmp);
            PabloPrinter::print(var, out);
            out << " was not assigned a value.";
            llvm::report_fatal_error(out.str());
        }
        phi->addIncoming(f->second, whileEntryBlock);
        f->second = phi;
        assert(mMarkerMap[var] == phi);
        variants.emplace_back(var, phi);
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

    // and for any variant nodes in the loop body
    for (const auto variant : variants) {
        const Var * var; PHINode * phi;
        std::tie(var, phi) = variant;
        const auto f = mMarkerMap.find(var);
        if (LLVM_UNLIKELY(f == mMarkerMap.end() || f->second == phi)) {
            std::string tmp;
            raw_string_ostream out(tmp);
            PabloPrinter::print(var, out);
            out << " was not assigned a value.";
            llvm::report_fatal_error(out.str());
        }
        phi->addIncoming(f->second, whileExitBlock);
        f->second = phi;
    }

    iBuilder->SetInsertPoint(whileEndBlock);
    --mWhileDepth;

    mCarryManager->ensureCarriesStoredRecursive();
    mCarryManager->leaveScope();
}

void PabloCompiler::compileStatement(const Statement * stmt) {

    if (LLVM_UNLIKELY(isa<If>(stmt))) {
        compileIf(cast<If>(stmt));
    } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
        compileWhile(cast<While>(stmt));
    } else {
        const PabloAST * expr = stmt;
        Value * value = nullptr;
        if (LLVM_UNLIKELY(isa<Assign>(stmt))) {

            expr = cast<Assign>(stmt)->getVariable();
            value = compileExpression(cast<Assign>(stmt)->getValue());

            bool storeInstRequired = false;
            if (LLVM_LIKELY(isa<Var>(expr))) {
                for (unsigned i = 0; i < mPabloFunction->getNumOfResults(); ++i) {
                    if (expr == mPabloFunction->getResult(i)) {
                        storeInstRequired = true;
                        break;
                    }
                }
            }

            if (LLVM_UNLIKELY(storeInstRequired || isa<Extract>(expr))) {

                const auto f = mMarkerMap.find(expr);
                if (LLVM_UNLIKELY(f == mMarkerMap.end())) {
                    std::string tmp;
                    raw_string_ostream out(tmp);
                    PabloPrinter::print(expr, out);
                    out << " was not defined prior to ";
                    PabloPrinter::print(stmt, out);
                    throw std::runtime_error(out.str());
                }
                Value * const ptr = f->second;

                assert (&(value->getContext()) == &(ptr->getContext()));

                if (isa<Count>(cast<Assign>(stmt)->getValue())) {
                    Value * count = iBuilder->CreateLoad(ptr);
                    Value * accum = iBuilder->CreateAdd(value, count);
                    iBuilder->CreateStore(accum, ptr);
                } else {
                    iBuilder->CreateBlockAlignedStore(value, ptr);
                }
            }

        } else if (const Extract * extract = dyn_cast<Extract>(stmt)) {
            Value * array = compileExpression(extract->getArray(), false);
            Value * index = compileExpression(extract->getIndex());
            value = iBuilder->CreateGEP(array, index, getName(stmt));
        } else if (const And * pablo_and = dyn_cast<And>(stmt)) {
            value = iBuilder->simd_and(compileExpression(pablo_and->getOperand(0)), compileExpression(pablo_and->getOperand(1)));
        } else if (const Or * pablo_or = dyn_cast<Or>(stmt)) {
            value = iBuilder->simd_or(compileExpression(pablo_or->getOperand(0)), compileExpression(pablo_or->getOperand(1)));
        } else if (const Xor * pablo_xor = dyn_cast<Xor>(stmt)) {
            value = iBuilder->simd_xor(compileExpression(pablo_xor->getOperand(0)), compileExpression(pablo_xor->getOperand(1)));
        } else if (const Sel * sel = dyn_cast<Sel>(stmt)) {
            Value* ifMask = compileExpression(sel->getCondition());
            Value* ifTrue = iBuilder->simd_and(ifMask, compileExpression(sel->getTrueExpr()));
            Value* ifFalse = iBuilder->simd_and(iBuilder->simd_not(ifMask), compileExpression(sel->getFalseExpr()));
            value = iBuilder->simd_or(ifTrue, ifFalse);
        } else if (const Not * pablo_not = dyn_cast<Not>(stmt)) {
            value = iBuilder->simd_not(compileExpression(pablo_not->getExpr()));
        } else if (const Advance * adv = dyn_cast<Advance>(stmt)) {
            Value * const strm_value = compileExpression(adv->getExpr());
            value = mCarryManager->advanceCarryInCarryOut(adv->getLocalIndex(), adv->getAmount(), strm_value);
        } else if (const MatchStar * mstar = dyn_cast<MatchStar>(stmt)) {
            Value * const marker = compileExpression(mstar->getMarker());
            Value * const cc = compileExpression(mstar->getCharClass());
            Value * const marker_and_cc = iBuilder->simd_and(marker, cc);
            Value * const sum = mCarryManager->addCarryInCarryOut(mstar->getLocalCarryIndex(), marker_and_cc, cc);
            value = iBuilder->simd_or(iBuilder->simd_xor(sum, cc), marker);
        } else if (const ScanThru * sthru = dyn_cast<ScanThru>(stmt)) {
            Value * const  marker_expr = compileExpression(sthru->getScanFrom());
            Value * const  cc_expr = compileExpression(sthru->getScanThru());
            Value * const  sum = mCarryManager->addCarryInCarryOut(sthru->getLocalCarryIndex(), marker_expr, cc_expr);
            value = iBuilder->simd_and(sum, iBuilder->simd_not(cc_expr));
        } else if (const InFile * e = dyn_cast<InFile>(stmt)) {
            Value * EOFmask = mKernelBuilder->getScalarField(mSelf, "EOFmask");
            value = iBuilder->simd_xor(compileExpression(e->getExpr()), EOFmask);
        } else if (const AtEOF * e = dyn_cast<AtEOF>(stmt)) {
            Value * EOFbit = mKernelBuilder->getScalarField(mSelf, "EOFbit");
            value = iBuilder->simd_and(compileExpression(e->getExpr()), EOFbit);
        } else if (const Count * c = dyn_cast<Count>(stmt)) {
            Value * const to_count = compileExpression(c->getExpr());
            const unsigned counterSize = 64;
            Value * fieldCounts = iBuilder->simd_popcount(counterSize, to_count);
            value = iBuilder->mvmd_extract(counterSize, fieldCounts, 0);
            for (unsigned i = 1; i < (iBuilder->getBitBlockWidth() / counterSize); ++i) {
                Value * temp = iBuilder->mvmd_extract(counterSize, fieldCounts, i);
                value = iBuilder->CreateAdd(value, temp);
            }
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
            std::string inputName = var->getName()->to_string();;
            Value * blockNo = mKernelBuilder->getScalarField(mSelf, blockNoScalar);
            Value * lookAhead_blockPtr  = mKernelBuilder->getStreamSetBlockPtr(mSelf, inputName, iBuilder->CreateAdd(blockNo, ConstantInt::get(iBuilder->getSizeTy(), block_shift)));
            Value * lookAhead_inputPtr = iBuilder->CreateGEP(lookAhead_blockPtr, {iBuilder->getInt32(0), iBuilder->getInt32(index)});
            Value * lookAhead = iBuilder->CreateBlockAlignedLoad(lookAhead_inputPtr);
            if (bit_shift == 0) {  // Simple case with no intra-block shifting.
                value = lookAhead;
            } else { // Need to form shift result from two adjacent blocks.
                Value * lookAhead_blockPtr1  = mKernelBuilder->getStreamSetBlockPtr(mSelf, inputName, iBuilder->CreateAdd(blockNo, ConstantInt::get(iBuilder->getSizeTy(), block_shift + 1)));
                Value * lookAhead_inputPtr1 = iBuilder->CreateGEP(lookAhead_blockPtr1, {iBuilder->getInt32(0), iBuilder->getInt32(index)});
                Value * lookAhead1 = iBuilder->CreateBlockAlignedLoad(lookAhead_inputPtr1);
                if (LLVM_UNLIKELY((bit_shift % 8) == 0)) { // Use a single whole-byte shift, if possible.
                    value = iBuilder->mvmd_dslli(8, lookAhead1, lookAhead, (bit_shift / 8));
                } else {
                    Type  * const streamType = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());
                    Value * b1 = iBuilder->CreateBitCast(lookAhead1, streamType);
                    Value * b0 = iBuilder->CreateBitCast(lookAhead, streamType);
                    Value * result = iBuilder->CreateOr(iBuilder->CreateShl(b1, iBuilder->getBitBlockWidth() - bit_shift), iBuilder->CreateLShr(b0, bit_shift));
                    value = iBuilder->CreateBitCast(result, mBitBlockType);
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

        mMarkerMap[expr] = value;

        if (DebugOptionIsSet(DumpTrace)) {
            iBuilder->CallPrintRegister(stmt->getName()->to_string(), value);
        }
    }
}

Value * PabloCompiler::compileExpression(const PabloAST * expr, const bool ensureLoaded) const {
    if (LLVM_UNLIKELY(isa<Ones>(expr))) {
        return iBuilder->allOnes();
    } else if (LLVM_UNLIKELY(isa<Zeroes>(expr))) {
        return iBuilder->allZeroes();
    } else if (LLVM_UNLIKELY(isa<Integer>(expr))) {
        return iBuilder->getInt64(cast<Integer>(expr)->value());
    }
    const auto f = mMarkerMap.find(expr);
    if (LLVM_UNLIKELY(f == mMarkerMap.end())) {
        std::string tmp;
        llvm::raw_string_ostream out(tmp);
        PabloPrinter::print(expr, out);
        out << " was used before definition!";
        throw std::runtime_error(out.str());
    }
    Value * value = f->second;
    if (LLVM_UNLIKELY(isa<GetElementPtrInst>(value) && ensureLoaded)) {
        value = iBuilder->CreateBlockAlignedLoad(value);
    }
    return value;
}

}
