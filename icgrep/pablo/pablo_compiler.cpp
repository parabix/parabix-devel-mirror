/*
 *  Copyright (c) 2014-16 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pablo_compiler.h"
#include <pablo/pablo_kernel.h>
#include <pablo/pablo_toolchain.h>
#include <pablo/codegenstate.h>
#include <pablo/boolean.h>
#include <pablo/arithmetic.h>
#include <pablo/branch.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_lookahead.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_infile.h>
#include <pablo/pe_count.h>
#include <pablo/pe_integer.h>
#include <pablo/pe_string.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_repeat.h>
#include <pablo/pe_pack.h>
#include <pablo/pe_var.h>
#include <pablo/ps_assign.h>
#include <pablo/ps_terminate.h>
#ifdef USE_CARRYPACK_MANAGER
#include <pablo/carrypack_manager.h>
#else
#include <pablo/carry_manager.h>
#endif
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Type.h>
#include <llvm/Support/raw_os_ostream.h>
#include <llvm/ADT/STLExtras.h> // for make_unique

using namespace llvm;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;

inline static unsigned getAlignment(const Type * const type) {
    return type->getPrimitiveSizeInBits() / 8;
}

inline static unsigned getAlignment(const Value * const expr) {
    return getAlignment(expr->getType());
}

inline static unsigned getPointerElementAlignment(const Value * const ptr) {
    return getAlignment(ptr->getType()->getPointerElementType());
}

void PabloCompiler::initializeKernelData(const std::unique_ptr<kernel::KernelBuilder> & b) {
    mBranchCount = 0;
    examineBlock(b, mKernel->getEntryScope());
    mCarryManager->initializeCarryData(b, mKernel);
    if (CompileOptionIsSet(PabloCompilationFlags::EnableProfiling)) {
        const auto count = (mBranchCount * 2) + 1;
        mKernel->addInternalScalar(ArrayType::get(mKernel->getSizeTy(), count), "profile");
        mBasicBlock.reserve(count);
    }
}

void PabloCompiler::releaseKernelData(const std::unique_ptr<kernel::KernelBuilder> & b) {
    mCarryManager->releaseCarryData(b);
}

void PabloCompiler::clearCarryData(const std::unique_ptr<kernel::KernelBuilder> & b) {
    mCarryManager->clearCarryData(b);
}

void PabloCompiler::compile(const std::unique_ptr<kernel::KernelBuilder> & b) {
    mCarryManager->initializeCodeGen(b);
    PabloBlock * const entryBlock = mKernel->getEntryScope(); assert (entryBlock);
    mMarker.emplace(entryBlock->createZeroes(), b->allZeroes());
    mMarker.emplace(entryBlock->createOnes(), b->allOnes());
    mBranchCount = 0;
    addBranchCounter(b);
    compileBlock(b, entryBlock);
    mCarryManager->finalizeCodeGen(b);
}

void PabloCompiler::examineBlock(const std::unique_ptr<kernel::KernelBuilder> & b, const PabloBlock * const block) {
    for (const Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<Lookahead>(stmt))) {
            const Lookahead * const la = cast<Lookahead>(stmt);
            PabloAST * input = la->getExpression();
            if (isa<Extract>(input)) {
                input = cast<Extract>(input)->getArray();
            }
            bool notFound = true;
            if (LLVM_LIKELY(isa<Var>(input))) {
                for (unsigned i = 0; i < mKernel->getNumOfInputs(); ++i) {
                    if (input == mKernel->getInput(i)) {
                        const auto & binding = mKernel->getInputStreamSetBinding(i);
                        if (LLVM_UNLIKELY(!binding.hasLookahead() || binding.getLookahead() < la->getAmount())) {
                            std::string tmp;
                            raw_string_ostream out(tmp);
                            input->print(out);
                            out << " must have a lookahead attribute of at least " << la->getAmount();
                            report_fatal_error(out.str());
                        }
                        notFound = false;
                        break;
                    }
                }
            }
            if (LLVM_UNLIKELY(notFound)) {
                report_fatal_error("Lookahead " + stmt->getName() + " can only be performed on an input streamset");
            }
        } else if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            ++mBranchCount;
            examineBlock(b, cast<Branch>(stmt)->getBody());
        } else if (LLVM_UNLIKELY(isa<Count>(stmt))) {
            mKernel->addInternalScalar(stmt->getType(), stmt->getName().str());
        }
    }    
}

void PabloCompiler::addBranchCounter(const std::unique_ptr<kernel::KernelBuilder> & b) {
    if (CompileOptionIsSet(PabloCompilationFlags::EnableProfiling)) {        
        Value * ptr = b->getScalarFieldPtr("profile");
        assert (mBasicBlock.size() < ptr->getType()->getPointerElementType()->getArrayNumElements());
        ptr = b->CreateGEP(ptr, {b->getInt32(0), b->getInt32(mBasicBlock.size())});
        const auto alignment = getPointerElementAlignment(ptr);
        Value * value = b->CreateAlignedLoad(ptr, alignment, false, "branchCounter");
        value = b->CreateAdd(value, ConstantInt::get(cast<IntegerType>(value->getType()), 1));
        b->CreateAlignedStore(value, ptr, alignment);
        mBasicBlock.push_back(b->GetInsertBlock());
    }
}

inline void PabloCompiler::compileBlock(const std::unique_ptr<kernel::KernelBuilder> & b, const PabloBlock * const block) {
    for (const Statement * statement : *block) {
        compileStatement(b, statement);
    }
}

void PabloCompiler::compileIf(const std::unique_ptr<kernel::KernelBuilder> & b, const If * const ifStatement) {
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

    BasicBlock * const ifEntryBlock = b->GetInsertBlock();
    ++mBranchCount;
    BasicBlock * const ifBodyBlock = b->CreateBasicBlock("if.body_" + std::to_string(mBranchCount));
    BasicBlock * const ifEndBlock = b->CreateBasicBlock("if.end_" + std::to_string(mBranchCount));
    
    std::vector<std::pair<const Var *, Value *>> incoming;

    for (const Var * var : ifStatement->getEscaped()) {
        if (LLVM_UNLIKELY(var->isKernelParameter())) {
            Value * marker = nullptr;
            if (var->isScalar()) {
                marker = b->getScalarFieldPtr(var->getName());
            } else if (var->isReadOnly()) {
                marker = b->getInputStreamBlockPtr(var->getName(), b->getInt32(0));
            } else if (var->isReadNone()) {
                marker = b->getOutputStreamBlockPtr(var->getName(), b->getInt32(0));
            }
            mMarker[var] = marker;
        } else {
            auto f = mMarker.find(var);
            if (LLVM_UNLIKELY(f == mMarker.end())) {
                std::string tmp;
                raw_string_ostream out(tmp);
                var->print(out);
                out << " is uninitialized prior to entering ";
                ifStatement->print(out);
                report_fatal_error(out.str());
            }
            incoming.emplace_back(var, f->second);
        }
    }

    const PabloBlock * ifBody = ifStatement->getBody();
    
    mCarryManager->enterIfScope(b, ifBody);

    Value * condition = compileExpression(b, ifStatement->getCondition());
    if (condition->getType() == b->getBitBlockType()) {
        condition = b->bitblock_any(mCarryManager->generateSummaryTest(b, condition));
    }
    
    b->CreateCondBr(condition, ifBodyBlock, ifEndBlock);
    
    // Entry processing is complete, now handle the body of the if.
    b->SetInsertPoint(ifBodyBlock);

    mCarryManager->enterIfBody(b, ifEntryBlock);

    addBranchCounter(b);

    compileBlock(b, ifBody);

    mCarryManager->leaveIfBody(b, b->GetInsertBlock());

    BasicBlock * ifExitBlock = b->GetInsertBlock();

    b->CreateBr(ifEndBlock);

    ifEndBlock->moveAfter(ifExitBlock);

    //End Block
    b->SetInsertPoint(ifEndBlock);

    mCarryManager->leaveIfScope(b, ifEntryBlock, ifExitBlock);

    for (const auto i : incoming) {
        const Var * var; Value * incoming;
        std::tie(var, incoming) = i;

        auto f = mMarker.find(var);
        if (LLVM_UNLIKELY(f == mMarker.end())) {
            std::string tmp;
            raw_string_ostream out(tmp);
            out << "PHINode creation error: ";
            var->print(out);
            out << " was not assigned an outgoing value.";
            report_fatal_error(out.str());
        }

        Value * const outgoing = f->second;
        if (LLVM_UNLIKELY(incoming == outgoing)) {
            continue;
        }

        if (LLVM_UNLIKELY(incoming->getType() != outgoing->getType())) {
            std::string tmp;
            raw_string_ostream out(tmp);
            out << "PHINode creation error: incoming type of ";
            var->print(out);
            out << " (";
            incoming->getType()->print(out);
            out << ") differs from the outgoing type (";
            outgoing->getType()->print(out);
            out << ") within ";
            ifStatement->print(out);
            report_fatal_error(out.str());
        }

        PHINode * phi = b->CreatePHI(incoming->getType(), 2, var->getName());
        phi->addIncoming(incoming, ifEntryBlock);
        phi->addIncoming(outgoing, ifExitBlock);
        f->second = phi;
    }

    addBranchCounter(b);
}

void PabloCompiler::compileWhile(const std::unique_ptr<kernel::KernelBuilder> & b, const While * const whileStatement) {

    const PabloBlock * const whileBody = whileStatement->getBody();

    BasicBlock * whileEntryBlock = b->GetInsertBlock();

    const auto escaped = whileStatement->getEscaped();

#ifdef ENABLE_BOUNDED_WHILE
    PHINode * bound_phi = nullptr;  // Needed for bounded while loops.
#endif
    // On entry to the while structure, proceed to execute the first iteration
    // of the loop body unconditionally. The while condition is tested at the end of
    // the loop.

    for (const Var * var : escaped) {
        if (LLVM_UNLIKELY(var->isKernelParameter())) {
            Value * marker = nullptr;
            if (var->isScalar()) {
                marker = b->getScalarFieldPtr(var->getName());
            } else if (var->isReadOnly()) {
                marker = b->getInputStreamBlockPtr(var->getName(), b->getInt32(0));
            } else if (var->isReadNone()) {
                marker = b->getOutputStreamBlockPtr(var->getName(), b->getInt32(0));
            }
            mMarker[var] = marker;
        }
    }

    mCarryManager->enterLoopScope(b, whileBody);

    BasicBlock * whileBodyBlock = b->CreateBasicBlock("while.body_" + std::to_string(mBranchCount));
    BasicBlock * whileEndBlock = b->CreateBasicBlock("while.end_" + std::to_string(mBranchCount));
    ++mBranchCount;

    b->CreateBr(whileBodyBlock);

    b->SetInsertPoint(whileBodyBlock);

    //
    // There are 3 sets of Phi nodes for the while loop.
    // (1) Carry-ins: (a) incoming carry data first iterations, (b) zero thereafter
    // (2) Carry-out accumulators: (a) zero first iteration, (b) |= carry-out of each iteration
    // (3) Next nodes: (a) values set up before loop, (b) modified values calculated in loop.
#ifdef ENABLE_BOUNDED_WHILE
    // (4) The loop bound, if any.
#endif

    std::vector<std::pair<const Var *, PHINode *>> variants;

    // for any Next nodes in the loop body, initialize to (a) pre-loop value.
    for (const auto var : escaped) {
        auto f = mMarker.find(var);
        if (LLVM_UNLIKELY(f == mMarker.end())) {
            std::string tmp;
            raw_string_ostream out(tmp);
            out << "PHINode creation error: ";
            var->print(out);
            out << " is uninitialized prior to entering ";
            whileStatement->print(out);
            report_fatal_error(out.str());
        }
        Value * entryValue = f->second;
        PHINode * phi = b->CreatePHI(entryValue->getType(), 2, var->getName());
        phi->addIncoming(entryValue, whileEntryBlock);
        f->second = phi;
        assert(mMarker[var] == phi);
        variants.emplace_back(var, phi);
    }
#ifdef ENABLE_BOUNDED_WHILE
    if (whileStatement->getBound()) {
        bound_phi = b->CreatePHI(b->getSizeTy(), 2, "while_bound");
        bound_phi->addIncoming(b->getSize(whileStatement->getBound()), whileEntryBlock);
    }
#endif

    mCarryManager->enterLoopBody(b, whileEntryBlock);

    addBranchCounter(b);

    compileBlock(b, whileBody);

    // After the whileBody has been compiled, we may be in a different basic block.

    mCarryManager->leaveLoopBody(b, b->GetInsertBlock());


#ifdef ENABLE_BOUNDED_WHILE
    if (whileStatement->getBound()) {
        Value * new_bound = b->CreateSub(bound_phi, b->getSize(1));
        bound_phi->addIncoming(new_bound, whileExitBlock);
        condition = b->CreateAnd(condition, b->CreateICmpUGT(new_bound, ConstantInt::getNullValue(b->getSizeTy())));
    }
#endif

    BasicBlock * const whileExitBlock = b->GetInsertBlock();

    // and for any variant nodes in the loop body
    for (const auto variant : variants) {
        const Var * var; PHINode * incomingPhi;
        std::tie(var, incomingPhi) = variant;
        const auto f = mMarker.find(var);
        if (LLVM_UNLIKELY(f == mMarker.end())) {
            std::string tmp;
            raw_string_ostream out(tmp);
            out << "PHINode creation error: ";
            var->print(out);
            out << " is no longer assigned a value.";
            report_fatal_error(out.str());
        }

        Value * const outgoingValue = f->second;

        if (LLVM_UNLIKELY(incomingPhi->getType() != outgoingValue->getType())) {
            std::string tmp;
            raw_string_ostream out(tmp);
            out << "PHINode creation error: incoming type of ";
            var->print(out);
            out << " (";
            incomingPhi->getType()->print(out);
            out << ") differs from the outgoing type (";
            outgoingValue->getType()->print(out);
            out << ") within ";
            whileStatement->print(out);
            report_fatal_error(out.str());
        }

        incomingPhi->addIncoming(outgoingValue, whileExitBlock);
    }

    // Terminate the while loop body with a conditional branch back.
    Value * condition = compileExpression(b, whileStatement->getCondition());
    if (condition->getType() == b->getBitBlockType()) {
        condition = b->bitblock_any(mCarryManager->generateSummaryTest(b, condition));
    }

    b->CreateCondBr(condition, whileBodyBlock, whileEndBlock);

    whileEndBlock->moveAfter(whileExitBlock);

    b->SetInsertPoint(whileEndBlock);

    mCarryManager->leaveLoopScope(b, whileEntryBlock, whileExitBlock);

    addBranchCounter(b);
}

void PabloCompiler::compileStatement(const std::unique_ptr<kernel::KernelBuilder> & b, const Statement * const stmt) {

    if (LLVM_UNLIKELY(isa<If>(stmt))) {
        compileIf(b, cast<If>(stmt));
    } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
        compileWhile(b, cast<While>(stmt));
    } else {
        const PabloAST * expr = stmt;
        Value * value = nullptr;
        if (isa<And>(stmt)) {
            Value * const op0 = compileExpression(b, stmt->getOperand(0));
            Value * const op1 = compileExpression(b, stmt->getOperand(1));
            value = b->simd_and(op0, op1);
        } else if (isa<Or>(stmt)) {
            Value * const op0 = compileExpression(b, stmt->getOperand(0));
            Value * const op1 = compileExpression(b, stmt->getOperand(1));
            value = b->simd_or(op0, op1);
        } else if (isa<Xor>(stmt)) {
            Value * const op0 = compileExpression(b, stmt->getOperand(0));
            Value * const op1 = compileExpression(b, stmt->getOperand(1));
            value = b->simd_xor(op0, op1);
        } else if (const Sel * sel = dyn_cast<Sel>(stmt)) {
            Value* ifMask = compileExpression(b, sel->getCondition());
            Value* ifTrue = b->simd_and(ifMask, compileExpression(b, sel->getTrueExpr()));
            Value* ifFalse = b->simd_and(b->simd_not(ifMask), compileExpression(b, sel->getFalseExpr()));
            value = b->simd_or(ifTrue, ifFalse);
        } else if (isa<Not>(stmt)) {
            value = b->simd_not(compileExpression(b, stmt->getOperand(0)));
        } else if (isa<Advance>(stmt)) {
            const Advance * const adv = cast<Advance>(stmt);
            // If our expr is an Extract op on a mutable Var then we need to pass the index value to the carry
            // manager so that it properly selects the correct carry bit.
            value = mCarryManager->advanceCarryInCarryOut(b, adv, compileExpression(b, adv->getExpression()));
        } else if (isa<IndexedAdvance>(stmt)) {
            const IndexedAdvance * const adv = cast<IndexedAdvance>(stmt);
            Value * strm = compileExpression(b, adv->getExpression());
            Value * index_strm = compileExpression(b, adv->getIndex());
            // If our expr is an Extract op on a mutable Var then we need to pass the index value to the carry
            // manager so that it properly selects the correct carry bit.
            value = mCarryManager->indexedAdvanceCarryInCarryOut(b, adv, strm, index_strm);
        } else if (const MatchStar * mstar = dyn_cast<MatchStar>(stmt)) {
            Value * const marker = compileExpression(b, mstar->getMarker());
            Value * const cc = compileExpression(b, mstar->getCharClass());
            Value * const marker_and_cc = b->simd_and(marker, cc);
            Value * const sum = mCarryManager->addCarryInCarryOut(b, mstar, marker_and_cc, cc);
            value = b->simd_or(b->simd_xor(sum, cc), marker);
        } else if (const ScanThru * sthru = dyn_cast<ScanThru>(stmt)) {
            Value * const from = compileExpression(b, sthru->getScanFrom());
            Value * const thru = compileExpression(b, sthru->getScanThru());
            Value * const sum = mCarryManager->addCarryInCarryOut(b, sthru, from, thru);
            value = b->simd_and(sum, b->simd_not(thru));
        } else if (const ScanTo * sthru = dyn_cast<ScanTo>(stmt)) {
            Value * const marker_expr = compileExpression(b, sthru->getScanFrom());
            Value * const to = b->simd_xor(compileExpression(b, sthru->getScanTo()), b->getScalarField("EOFmask"));
            Value * const sum = mCarryManager->addCarryInCarryOut(b, sthru, marker_expr, b->simd_not(to));
            value = b->simd_and(sum, to);
        } else if (const AdvanceThenScanThru * sthru = dyn_cast<AdvanceThenScanThru>(stmt)) {
            Value * const from = compileExpression(b, sthru->getScanFrom());
            Value * const thru = compileExpression(b, sthru->getScanThru());
            Value * const sum = mCarryManager->addCarryInCarryOut(b, sthru, from, b->simd_or(from, thru));
            value = b->simd_and(sum, b->simd_not(thru));
        } else if (const AdvanceThenScanTo * sthru = dyn_cast<AdvanceThenScanTo>(stmt)) {
            Value * const from = compileExpression(b, sthru->getScanFrom());
            Value * const to = b->simd_xor(compileExpression(b, sthru->getScanTo()), b->getScalarField("EOFmask"));
            Value * const sum = mCarryManager->addCarryInCarryOut(b, sthru, from, b->simd_or(from, b->simd_not(to)));
            value = b->simd_and(sum, to);
        } else if (const TerminateAt * s = dyn_cast<TerminateAt>(stmt)) {
            Value * signal_strm = compileExpression(b, s->getExpr());
            llvm::errs() << "Here\n";
            b->CallPrintRegister("signal_strm", signal_strm);
            BasicBlock * signalCallBack = b->CreateBasicBlock("signalCallBack");
            BasicBlock * postSignal = b->CreateBasicBlock("postSignal");
            b->CreateCondBr(b->bitblock_any(signal_strm), signalCallBack, postSignal);
            b->SetInsertPoint(signalCallBack);
            // Perhaps check for handler address and skip call back if none???
            Value * handler = b->getScalarField("handler_address");
            Function * const dispatcher = b->getModule()->getFunction("signal_dispatcher"); assert (dispatcher);
            b->CreateCall(dispatcher, {handler, ConstantInt::get(b->getInt32Ty(), s->getSignalCode())});
            //Value * rel_position = b->createCountForwardZeroes(signal_strm);
            //Value * position = b->CreateAdd(b->getProcessedItemCount(), rel_position);
            b->setTerminationSignal();
            b->CreateBr(postSignal);
            b->SetInsertPoint(postSignal);
            value = signal_strm;
        } else if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
            expr = cast<Assign>(stmt)->getVariable();
            value = compileExpression(b, cast<Assign>(stmt)->getValue());
            if (isa<Extract>(expr) || (isa<Var>(expr) && cast<Var>(expr)->isKernelParameter())) {
                Value * const ptr = compileExpression(b, expr, false);                
                Type * const elemTy = ptr->getType()->getPointerElementType();
                b->CreateAlignedStore(b->CreateZExt(value, elemTy), ptr, getAlignment(elemTy));
                value = ptr;
            }
        } else if (const InFile * e = dyn_cast<InFile>(stmt)) {
            Value * EOFmask = b->getScalarField("EOFmask");
            value = b->simd_and(compileExpression(b, e->getExpr()), b->simd_not(EOFmask));
        } else if (const AtEOF * e = dyn_cast<AtEOF>(stmt)) {
            Value * EOFbit = b->getScalarField("EOFbit");
            value = b->simd_and(compileExpression(b, e->getExpr()), EOFbit);
        } else if (const Count * c = dyn_cast<Count>(stmt)) {
            Value * EOFbit = b->getScalarField("EOFbit");
            Value * EOFmask = b->getScalarField("EOFmask");
            Value * const to_count = b->simd_and(b->simd_or(b->simd_not(EOFmask), EOFbit), compileExpression(b, c->getExpr()));
            Value * const ptr = b->getScalarFieldPtr(stmt->getName().str());
            const auto alignment = getPointerElementAlignment(ptr);
            Value * const countSoFar = b->CreateAlignedLoad(ptr, alignment, c->getName() + "_accumulator");
            const auto fieldWidth = b->getSizeTy()->getBitWidth();
            Value * bitBlockCount = b->simd_popcount(b->getBitBlockWidth(), to_count);
            value = b->CreateAdd(b->mvmd_extract(fieldWidth, bitBlockCount, 0), countSoFar, "countSoFar");
            b->CreateAlignedStore(value, ptr, alignment);
        } else if (const Lookahead * l = dyn_cast<Lookahead>(stmt)) {
            PabloAST * stream = l->getExpression();
            Value * index = nullptr;
            if (LLVM_UNLIKELY(isa<Extract>(stream))) {                
                index = compileExpression(b, cast<Extract>(stream)->getIndex(), true);
                stream = cast<Extract>(stream)->getArray();
            } else {
                index = b->getInt32(0);
            }
            const auto bit_shift = (l->getAmount() % b->getBitBlockWidth());
            const auto block_shift = (l->getAmount() / b->getBitBlockWidth());
            Value * ptr = b->getInputStreamBlockPtr(cast<Var>(stream)->getName(), index, b->getSize(block_shift));
            // Value * base = b->CreatePointerCast(b->getBaseAddress(cast<Var>(stream)->getName()), ptr->getType());
            Value * lookAhead = b->CreateBlockAlignedLoad(ptr);
            if (LLVM_UNLIKELY(bit_shift == 0)) {  // Simple case with no intra-block shifting.
                value = lookAhead;
            } else { // Need to form shift result from two adjacent blocks.
                Value * ptr1 = b->getInputStreamBlockPtr(cast<Var>(stream)->getName(), index, b->getSize(block_shift + 1));
                Value * lookAhead1 = b->CreateBlockAlignedLoad(ptr1);
                if (LLVM_UNLIKELY((bit_shift % 8) == 0)) { // Use a single whole-byte shift, if possible.
                    value = b->mvmd_dslli(8, lookAhead1, lookAhead, (bit_shift / 8));
                } else {
                    Type  * const streamType = b->getIntNTy(b->getBitBlockWidth());
                    Value * b1 = b->CreateBitCast(lookAhead1, streamType);
                    Value * b0 = b->CreateBitCast(lookAhead, streamType);
                    Value * result = b->CreateOr(b->CreateShl(b1, b->getBitBlockWidth() - bit_shift), b->CreateLShr(b0, bit_shift));
                    value = b->CreateBitCast(result, b->getBitBlockType());
                }
            }
        } else if (const Repeat * const s = dyn_cast<Repeat>(stmt)) {
            value = compileExpression(b, s->getValue());
            Type * const ty = s->getType();
            if (LLVM_LIKELY(ty->isVectorTy())) {
                const auto fw = s->getFieldWidth()->value();
                value = b->CreateZExtOrTrunc(value, b->getIntNTy(fw));
                value = b->simd_fill(fw, value);
            } else {
                value = b->CreateZExtOrTrunc(value, ty);
            }
        } else if (const PackH * const p = dyn_cast<PackH>(stmt)) {
            const auto sourceWidth = p->getValue()->getType()->getVectorElementType()->getIntegerBitWidth();
            const auto packWidth = p->getFieldWidth()->value();
            assert (sourceWidth == packWidth);
            Value * const base = compileExpression(b, p->getValue(), false);
            const auto result_packs = sourceWidth/2;
            if (LLVM_LIKELY(result_packs > 1)) {
                value = b->CreateAlloca(ArrayType::get(b->getBitBlockType(), result_packs));
            }
            Constant * const ZERO = b->getInt32(0);
            for (unsigned i = 0; i < result_packs; ++i) {
                Value * A = b->CreateLoad(b->CreateGEP(base, {ZERO, b->getInt32(i * 2)}));
                Value * B = b->CreateLoad(b->CreateGEP(base, {ZERO, b->getInt32(i * 2 + 1)}));
                Value * P = b->bitCast(b->hsimd_packh(packWidth, A, B));
                if (LLVM_UNLIKELY(result_packs == 1)) {
                    value = P;
                    break;
                }
                b->CreateStore(P, b->CreateGEP(value, {ZERO, b->getInt32(i)}));
            }
        } else if (const PackL * const p = dyn_cast<PackL>(stmt)) {
            const auto sourceWidth = p->getValue()->getType()->getVectorElementType()->getIntegerBitWidth();
            const auto packWidth = p->getFieldWidth()->value();
            assert (sourceWidth == packWidth);
            Value * const base = compileExpression(b, p->getValue(), false);
            const auto result_packs = sourceWidth/2;
            if (LLVM_LIKELY(result_packs > 1)) {
                value = b->CreateAlloca(ArrayType::get(b->getBitBlockType(), result_packs));
            }
            Constant * const ZERO = b->getInt32(0);
            for (unsigned i = 0; i < result_packs; ++i) {
                Value * A = b->CreateLoad(b->CreateGEP(base, {ZERO, b->getInt32(i * 2)}));
                Value * B = b->CreateLoad(b->CreateGEP(base, {ZERO, b->getInt32(i * 2 + 1)}));
                Value * P = b->bitCast(b->hsimd_packl(packWidth, A, B));
                if (LLVM_UNLIKELY(result_packs == 1)) {
                    value = P;
                    break;
                }
                b->CreateStore(P, b->CreateGEP(value, {ZERO, b->getInt32(i)}));
            }
        } else {
            std::string tmp;
            raw_string_ostream out(tmp);
            out << "PabloCompiler: ";
            stmt->print(out);
            out << " was not recognized by the compiler";
            report_fatal_error(out.str());
        }
        assert (expr);
        assert (value);
        mMarker[expr] = value;
        if (DebugOptionIsSet(DumpTrace)) {
            std::string tmp;
            raw_string_ostream name(tmp);
            expr->print(name);
            if (value->getType()->isVectorTy()) {
                b->CallPrintRegister(name.str(), value);
            } else if (value->getType()->isIntegerTy()) {
                b->CallPrintInt(name.str(), value);
            }
        }
    }
}

unsigned getIntegerBitWidth(const Type * ty) {
    if (ty->isArrayTy()) {
        assert (ty->getArrayNumElements() == 1);
        ty = ty->getArrayElementType();
    }
    if (ty->isVectorTy()) {
        assert (ty->getVectorNumElements() == 0);
        ty = ty->getVectorElementType();
    }
    return ty->getIntegerBitWidth();
}

Value * PabloCompiler::compileExpression(const std::unique_ptr<kernel::KernelBuilder> & b, const PabloAST * const expr, const bool ensureLoaded) {
    const auto f = mMarker.find(expr);    
    Value * value = nullptr;
    if (LLVM_LIKELY(f != mMarker.end())) {
        value = f->second;
    } else {
        if (isa<Integer>(expr)) {
            value = ConstantInt::get(cast<Integer>(expr)->getType(), cast<Integer>(expr)->value());
        } else if (isa<Zeroes>(expr)) {
            value = b->allZeroes();
        } else if (LLVM_UNLIKELY(isa<Ones>(expr))) {
            value = b->allOnes();
        } else if (isa<Extract>(expr)) {
            const Extract * const extract = cast<Extract>(expr);
            const Var * const var = cast<Var>(extract->getArray());
            Value * const index = compileExpression(b, extract->getIndex());
            value = getPointerToVar(b, var, index);
        } else if (LLVM_UNLIKELY(isa<Var>(expr))) {
            const Var * const var = cast<Var>(expr);
            if (LLVM_LIKELY(var->isKernelParameter() && var->isScalar())) {
                value = b->getScalarFieldPtr(var->getName());
            } else { // use before def error
                std::string tmp;
                raw_string_ostream out(tmp);
                out << "PabloCompiler: ";
                expr->print(out);
                out << " is not a scalar value or was used before definition";
                report_fatal_error(out.str());
            }
        } else if (LLVM_UNLIKELY(isa<Operator>(expr))) {
            const Operator * const op = cast<Operator>(expr);
            const PabloAST * lh = op->getLH();
            const PabloAST * rh = op->getRH();
            if ((isa<Var>(lh) || isa<Extract>(lh)) || (isa<Var>(rh) || isa<Extract>(rh))) {
                if (getIntegerBitWidth(lh->getType()) != getIntegerBitWidth(rh->getType())) {
                    llvm::report_fatal_error("Integer types must be identical!");
                }
                const unsigned intWidth = std::min(getIntegerBitWidth(lh->getType()), getIntegerBitWidth(rh->getType()));
                const unsigned maskWidth = b->getBitBlockWidth() / intWidth;
                IntegerType * const maskTy = b->getIntNTy(maskWidth);
                VectorType * const vTy = VectorType::get(b->getIntNTy(intWidth), maskWidth);

                Value * baseLhv = nullptr;
                Value * lhvStreamIndex = nullptr;
                if (isa<Var>(lh)) {
                    lhvStreamIndex = b->getInt32(0);
                } else if (isa<Extract>(lh)) {
                    lhvStreamIndex = compileExpression(b, cast<Extract>(lh)->getIndex());
                    lh = cast<Extract>(lh)->getArray();
                } else {
                    baseLhv = compileExpression(b, lh);
                }

                Value * baseRhv = nullptr;
                Value * rhvStreamIndex = nullptr;
                if (isa<Var>(rh)) {
                    rhvStreamIndex = b->getInt32(0);
                } else if (isa<Extract>(rh)) {
                    rhvStreamIndex = compileExpression(b, cast<Extract>(rh)->getIndex());
                    rh = cast<Extract>(rh)->getArray();
                } else {
                    baseRhv = compileExpression(b, rh);
                }

                const TypeId typeId = op->getClassTypeId();

                if (LLVM_UNLIKELY(typeId == TypeId::Add || typeId == TypeId::Subtract)) {

                    value = b->CreateAlloca(vTy, b->getInt32(intWidth));

                    for (unsigned i = 0; i < intWidth; ++i) {
                        llvm::Constant * const index = b->getInt32(i);
                        Value * lhv = nullptr;
                        if (baseLhv) {
                            lhv = baseLhv;
                        } else {
                            lhv = getPointerToVar(b, cast<Var>(lh), lhvStreamIndex, index);
                            lhv = b->CreateBlockAlignedLoad(lhv);
                        }
                        lhv = b->CreateBitCast(lhv, vTy);

                        Value * rhv = nullptr;
                        if (baseRhv) {
                            rhv = baseRhv;
                        } else {
                            rhv = getPointerToVar(b, cast<Var>(rh), rhvStreamIndex, index);
                            rhv = b->CreateBlockAlignedLoad(rhv);
                        }
                        rhv = b->CreateBitCast(rhv, vTy);

                        Value * result = nullptr;
                        if (typeId == TypeId::Add) {
                            result = b->CreateAdd(lhv, rhv);
                        } else { // if (typeId == TypeId::Subtract) {
                            result = b->CreateSub(lhv, rhv);
                        }
                        b->CreateAlignedStore(result, b->CreateGEP(value, {b->getInt32(0), b->getInt32(i)}), getAlignment(result));
                    }

                } else {

                    value = UndefValue::get(VectorType::get(maskTy, intWidth));

                    for (unsigned i = 0; i < intWidth; ++i) {
                        llvm::Constant * const index = b->getInt32(i);
                        Value * lhv = nullptr;
                        if (baseLhv) {
                            lhv = baseLhv;
                        } else {
                            lhv = getPointerToVar(b, cast<Var>(lh), lhvStreamIndex, index);
                            lhv = b->CreateBlockAlignedLoad(lhv);
                        }
                        lhv = b->CreateBitCast(lhv, vTy);

                        Value * rhv = nullptr;
                        if (baseRhv) {
                            rhv = baseRhv;
                        } else {
                            rhv = getPointerToVar(b, cast<Var>(rh), rhvStreamIndex, index);
                            rhv = b->CreateBlockAlignedLoad(rhv);
                        }
                        rhv = b->CreateBitCast(rhv, vTy);
                        Value * comp = nullptr;
                        switch (typeId) {
                            case TypeId::GreaterThanEquals:
                            case TypeId::LessThan:
                                comp = b->simd_ult(intWidth, lhv, rhv);
                                break;
                            case TypeId::Equals:
                            case TypeId::NotEquals:
                                comp = b->simd_eq(intWidth, lhv, rhv);
                                break;
                            case TypeId::LessThanEquals:
                            case TypeId::GreaterThan:
                                comp = b->simd_ugt(intWidth, lhv, rhv);
                                break;
                            default: llvm_unreachable("invalid vector operator id");
                        }
                        Value * const mask = b->CreateZExtOrTrunc(b->hsimd_signmask(intWidth, comp), maskTy);
                        value = b->mvmd_insert(maskWidth, value, mask, i);
                    }
                    value = b->CreateBitCast(value, b->getBitBlockType());
                    switch (typeId) {
                        case TypeId::GreaterThanEquals:
                        case TypeId::LessThanEquals:
                        case TypeId::NotEquals:
                            value = b->CreateNot(value);
                        default: break;
                    }
                }

            } else {
                Value * const lhv = compileExpression(b, lh);
                Value * const rhv = compileExpression(b, rh);
                switch (op->getClassTypeId()) {
                    case TypeId::Add:
                        value = b->CreateAdd(lhv, rhv); break;
                    case TypeId::Subtract:
                        value = b->CreateSub(lhv, rhv); break;
                    case TypeId::LessThan:
                        value = b->CreateICmpULT(lhv, rhv); break;
                    case TypeId::LessThanEquals:
                        value = b->CreateICmpULE(lhv, rhv); break;
                    case TypeId::Equals:
                        value = b->CreateICmpEQ(lhv, rhv); break;
                    case TypeId::GreaterThanEquals:
                        value = b->CreateICmpUGE(lhv, rhv); break;
                    case TypeId::GreaterThan:
                        value = b->CreateICmpUGT(lhv, rhv); break;
                    case TypeId::NotEquals:
                        value = b->CreateICmpNE(lhv, rhv); break;
                    default: llvm_unreachable("invalid scalar operator id");
                }
            }
        } else { // use before def error
            std::string tmp;
            raw_string_ostream out(tmp);
            out << "PabloCompiler: ";
            expr->print(out);
            out << " was used before definition";
            report_fatal_error(out.str());
        }
        assert (value);
        // mMarker.insert({expr, value});
    }
    if (LLVM_UNLIKELY(value->getType()->isPointerTy() && ensureLoaded)) {
        value = b->CreateAlignedLoad(value, getPointerElementAlignment(value));
    }
    return value;
}

Value * PabloCompiler::getPointerToVar(const std::unique_ptr<kernel::KernelBuilder> & b, const Var * var, Value * index1, Value * index2)  {
    assert (var && index1);
    if (LLVM_LIKELY(var->isKernelParameter())) {
        if (LLVM_UNLIKELY(var->isScalar())) {
            std::string tmp;
            raw_string_ostream out(tmp);
            out << mKernel->getName();
            out << ": cannot index scalar value ";
            var->print(out);
            report_fatal_error(out.str());
        } else if (var->isReadOnly()) {
            if (index2) {
                return b->getInputStreamPackPtr(var->getName(), index1, index2);
            } else {
                return b->getInputStreamBlockPtr(var->getName(), index1);
            }
        } else if (var->isReadNone()) {
            if (index2) {
                return b->getOutputStreamPackPtr(var->getName(), index1, index2);
            } else {
                return b->getOutputStreamBlockPtr(var->getName(), index1);
            }
        } else {
            std::string tmp;
            raw_string_ostream out(tmp);
            out << mKernel->getName();
            out << ": stream ";
            var->print(out);
            out << " cannot be read from or written to";
            report_fatal_error(out.str());
        }
    } else {
        Value * const ptr = compileExpression(b, var, false);
        std::vector<Value *> offsets;
        offsets.push_back(ConstantInt::getNullValue(index1->getType()));
        offsets.push_back(index1);
        if (index2) offsets.push_back(index2);
        return b->CreateGEP(ptr, offsets);
    }
}

PabloCompiler::PabloCompiler(PabloKernel * const kernel)
: mKernel(kernel)
, mCarryManager(make_unique<CarryManager>())
, mBranchCount(0) {
    assert ("PabloKernel cannot be null!" && kernel);
}

PabloCompiler::~PabloCompiler() {
}

}
