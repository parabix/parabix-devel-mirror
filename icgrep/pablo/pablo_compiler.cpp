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
#include <pablo/pe_var.h>
#include <pablo/ps_assign.h>
#ifdef USE_CARRYPACK_MANAGER
#include <pablo/carrypack_manager.h>
#else
#include <pablo/carry_manager.h>
#endif
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_os_ostream.h>

using namespace llvm;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;

inline static unsigned getAlignment(const Value * const type) {
    return type->getType()->getPrimitiveSizeInBits() / 8;
}

inline static unsigned getPointerElementAlignment(const Value * const ptr) {
    return ptr->getType()->getPointerElementType()->getPrimitiveSizeInBits() / 8;
}

void PabloCompiler::initializeKernelData(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    assert ("PabloCompiler does not have a IDISA iBuilder" && iBuilder);
    mBranchCount = 0;
    examineBlock(iBuilder, mKernel->getEntryBlock());
    mCarryManager->initializeCarryData(iBuilder, mKernel);
    if (CompileOptionIsSet(PabloCompilationFlags::EnableProfiling)) {
        const auto count = (mBranchCount * 2) + 1;
        mKernel->addScalar(ArrayType::get(mKernel->getSizeTy(), count), "profile");
        mBasicBlock.reserve(count);
    }
}

void PabloCompiler::releaseKernelData(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    assert ("PabloCompiler does not have a IDISA iBuilder" && iBuilder);
    mCarryManager->releaseCarryData(iBuilder);
}

void PabloCompiler::compile(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    assert ("PabloCompiler does not have a IDISA iBuilder" && iBuilder);
    mCarryManager->initializeCodeGen(iBuilder);
    PabloBlock * const entryBlock = mKernel->getEntryBlock(); assert (entryBlock);
    mMarker.emplace(entryBlock->createZeroes(), iBuilder->allZeroes());
    mMarker.emplace(entryBlock->createOnes(), iBuilder->allOnes());
    mBranchCount = 0;
    addBranchCounter(iBuilder);
    compileBlock(iBuilder, entryBlock);
    mCarryManager->finalizeCodeGen(iBuilder);
}

void PabloCompiler::examineBlock(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const block) {
    for (const Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<Lookahead>(stmt))) {
            const Lookahead * const la = cast<Lookahead>(stmt);
            PabloAST * input = la->getExpression();
            if (LLVM_UNLIKELY(isa<Extract>(input))) {
                input = cast<Extract>(input)->getArray();
            }
            bool notFound = true;
            if (LLVM_LIKELY(isa<Var>(input))) {
                for (unsigned i = 0; i < mKernel->getNumOfInputs(); ++i) {
                    if (input == mKernel->getInput(i)) {
                        if (LLVM_LIKELY(mKernel->getLookAhead(i) < la->getAmount())) {
                            mKernel->setLookAhead(i, la->getAmount());
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
            examineBlock(iBuilder, cast<Branch>(stmt)->getBody());
        } else if (LLVM_UNLIKELY(isa<Count>(stmt))) {
            mAccumulator.insert(std::make_pair(stmt, iBuilder->getInt32(mKernel->addUnnamedScalar(stmt->getType()))));
        }
    }    
}

void PabloCompiler::addBranchCounter(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    if (CompileOptionIsSet(PabloCompilationFlags::EnableProfiling)) {        
        Value * ptr = iBuilder->getScalarFieldPtr("profile");
        assert (mBasicBlock.size() < ptr->getType()->getPointerElementType()->getArrayNumElements());
        ptr = iBuilder->CreateGEP(ptr, {iBuilder->getInt32(0), iBuilder->getInt32(mBasicBlock.size())});
        const auto alignment = getPointerElementAlignment(ptr);
        Value * value = iBuilder->CreateAlignedLoad(ptr, alignment, false, "branchCounter");
        value = iBuilder->CreateAdd(value, ConstantInt::get(cast<IntegerType>(value->getType()), 1));
        iBuilder->CreateAlignedStore(value, ptr, alignment);
        mBasicBlock.push_back(iBuilder->GetInsertBlock());
    }
}

inline void PabloCompiler::compileBlock(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const block) {
    for (const Statement * statement : *block) {
        compileStatement(iBuilder, statement);
    }
}

void PabloCompiler::compileIf(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const If * const ifStatement) {
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
    ++mBranchCount;
    BasicBlock * const ifBodyBlock = iBuilder->CreateBasicBlock("if.body_" + std::to_string(mBranchCount));
    BasicBlock * const ifEndBlock = iBuilder->CreateBasicBlock("if.end_" + std::to_string(mBranchCount));
    
    std::vector<std::pair<const Var *, Value *>> incoming;

    for (const Var * var : ifStatement->getEscaped()) {
        if (LLVM_UNLIKELY(var->isKernelParameter())) {
            Value * marker = nullptr;
            if (var->isScalar()) {
                marker = iBuilder->getScalarFieldPtr(var->getName());
            } else if (var->isReadOnly()) {
                marker = iBuilder->getInputStreamBlockPtr(var->getName(), iBuilder->getInt32(0));
            } else if (var->isReadNone()) {
                marker = iBuilder->getOutputStreamBlockPtr(var->getName(), iBuilder->getInt32(0));
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
    
    mCarryManager->enterIfScope(iBuilder, ifBody);

    Value * condition = compileExpression(iBuilder, ifStatement->getCondition());
    if (condition->getType() == iBuilder->getBitBlockType()) {
        condition = iBuilder->bitblock_any(mCarryManager->generateSummaryTest(iBuilder, condition));
    }
    
    iBuilder->CreateCondBr(condition, ifBodyBlock, ifEndBlock);
    
    // Entry processing is complete, now handle the body of the if.
    iBuilder->SetInsertPoint(ifBodyBlock);

    mCarryManager->enterIfBody(iBuilder, ifEntryBlock);

    addBranchCounter(iBuilder);

    compileBlock(iBuilder, ifBody);

    mCarryManager->leaveIfBody(iBuilder, iBuilder->GetInsertBlock());

    BasicBlock * ifExitBlock = iBuilder->GetInsertBlock();

    iBuilder->CreateBr(ifEndBlock);

    ifEndBlock->moveAfter(ifExitBlock);

    //End Block
    iBuilder->SetInsertPoint(ifEndBlock);

    mCarryManager->leaveIfScope(iBuilder, ifEntryBlock, ifExitBlock);

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

        PHINode * phi = iBuilder->CreatePHI(incoming->getType(), 2, var->getName());
        phi->addIncoming(incoming, ifEntryBlock);
        phi->addIncoming(outgoing, ifExitBlock);
        f->second = phi;
    }

    addBranchCounter(iBuilder);
}

void PabloCompiler::compileWhile(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const While * const whileStatement) {

    const PabloBlock * const whileBody = whileStatement->getBody();

    BasicBlock * whileEntryBlock = iBuilder->GetInsertBlock();

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
                marker = iBuilder->getScalarFieldPtr(var->getName());
            } else if (var->isReadOnly()) {
                marker = iBuilder->getInputStreamBlockPtr(var->getName(), iBuilder->getInt32(0));
            } else if (var->isReadNone()) {
                marker = iBuilder->getOutputStreamBlockPtr(var->getName(), iBuilder->getInt32(0));
            }
            mMarker[var] = marker;
        }
    }

    mCarryManager->enterLoopScope(iBuilder, whileBody);

    BasicBlock * whileBodyBlock = iBuilder->CreateBasicBlock("while.body_" + std::to_string(mBranchCount));
    BasicBlock * whileEndBlock = iBuilder->CreateBasicBlock("while.end_" + std::to_string(mBranchCount));
    ++mBranchCount;

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
        PHINode * phi = iBuilder->CreatePHI(entryValue->getType(), 2, var->getName());
        phi->addIncoming(entryValue, whileEntryBlock);
        f->second = phi;
        assert(mMarker[var] == phi);
        variants.emplace_back(var, phi);
    }
#ifdef ENABLE_BOUNDED_WHILE
    if (whileStatement->getBound()) {
        bound_phi = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "while_bound");
        bound_phi->addIncoming(iBuilder->getSize(whileStatement->getBound()), whileEntryBlock);
    }
#endif

    mCarryManager->enterLoopBody(iBuilder, whileEntryBlock);

    addBranchCounter(iBuilder);

    compileBlock(iBuilder, whileBody);

    // After the whileBody has been compiled, we may be in a different basic block.

    mCarryManager->leaveLoopBody(iBuilder, iBuilder->GetInsertBlock());


#ifdef ENABLE_BOUNDED_WHILE
    if (whileStatement->getBound()) {
        Value * new_bound = iBuilder->CreateSub(bound_phi, iBuilder->getSize(1));
        bound_phi->addIncoming(new_bound, whileExitBlock);
        condition = iBuilder->CreateAnd(condition, iBuilder->CreateICmpUGT(new_bound, ConstantInt::getNullValue(iBuilder->getSizeTy())));
    }
#endif

    BasicBlock * const whileExitBlock = iBuilder->GetInsertBlock();

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
    Value * condition = compileExpression(iBuilder, whileStatement->getCondition());
    if (condition->getType() == iBuilder->getBitBlockType()) {
        condition = iBuilder->bitblock_any(mCarryManager->generateSummaryTest(iBuilder, condition));
    }

    iBuilder->CreateCondBr(condition, whileBodyBlock, whileEndBlock);

    whileEndBlock->moveAfter(whileExitBlock);

    iBuilder->SetInsertPoint(whileEndBlock);

    mCarryManager->leaveLoopScope(iBuilder, whileEntryBlock, whileExitBlock);

    addBranchCounter(iBuilder);
}

void PabloCompiler::compileStatement(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const Statement * const stmt) {

    if (LLVM_UNLIKELY(isa<If>(stmt))) {
        compileIf(iBuilder, cast<If>(stmt));
    } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
        compileWhile(iBuilder, cast<While>(stmt));
    } else {
        const PabloAST * expr = stmt;
        Value * value = nullptr;
        if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
            value = compileExpression(iBuilder, cast<Assign>(stmt)->getValue());
            expr = cast<Assign>(stmt)->getVariable();
            Value * ptr = nullptr;
            if (LLVM_LIKELY(isa<Var>(expr))) {
                const Var * var = cast<Var>(expr);
                if (LLVM_UNLIKELY(var->isReadOnly())) {
                    std::string tmp;
                    raw_string_ostream out(tmp);
                    out << mKernel->getName();
                    out << " cannot assign value to ";
                    var->print(out);
                    out << ": ";
                    var->print(out);
                    out << " is read only";
                    report_fatal_error(out.str());
                }
                if (var->isKernelParameter()) {
                    if (var->isScalar()) {
                        ptr = iBuilder->getScalarFieldPtr(var->getName());
                    } else {
                        ptr = iBuilder->getOutputStreamBlockPtr(var->getName(), iBuilder->getInt32(0));
                    }
                }
            } else if (isa<Extract>(expr)) {
                const auto f = mMarker.find(expr);
                if (LLVM_UNLIKELY(f == mMarker.end())) {
                    std::string tmp;
                    raw_string_ostream out(tmp);
                    out << mKernel->getName();
                    out << " cannot assign value to ";
                    expr->print(out);
                    out << ": ";
                    expr->print(out);
                    out << " does not dominate ";
                    stmt->print(out);
                    report_fatal_error(out.str());
                }
                ptr = f->second;
                assert (ptr);
            }
            if (ptr) {
                iBuilder->CreateAlignedStore(value, ptr, getAlignment(value));
                value = ptr;
            }
        } else if (const Extract * extract = dyn_cast<Extract>(stmt)) {
            Value * index = compileExpression(iBuilder, extract->getIndex());
            Var * const array = dyn_cast<Var>(extract->getArray());
            if (LLVM_LIKELY(array && array->isKernelParameter())) {
                if (array->isReadOnly()) {
                    value = iBuilder->getInputStreamBlockPtr(array->getName(), index);
                } else if (array->isReadNone()) {
                    value = iBuilder->getOutputStreamBlockPtr(array->getName(), index);
                } else {
                    std::string tmp;
                    raw_string_ostream out(tmp);
                    out << mKernel->getName();
                    out << " stream ";
                    expr->print(out);
                    out << " cannot be read or written to";
                    report_fatal_error(out.str());
                }
            } else {
                Value * ptr = compileExpression(iBuilder, extract->getArray(), false);
                value = iBuilder->CreateGEP(ptr, {ConstantInt::getNullValue(index->getType()), index}, "extract");
            }
        } else if (isa<And>(stmt)) {
            value = compileExpression(iBuilder, stmt->getOperand(0));
            for (unsigned i = 1; i < stmt->getNumOperands(); ++i) {
                value = iBuilder->simd_and(value, compileExpression(iBuilder, stmt->getOperand(1)));
            }
        } else if (isa<Or>(stmt)) {
            value = compileExpression(iBuilder, stmt->getOperand(0));
            for (unsigned i = 1; i < stmt->getNumOperands(); ++i) {
                value = iBuilder->simd_or(value, compileExpression(iBuilder, stmt->getOperand(1)));
            }
        } else if (isa<Xor>(stmt)) {
            value = compileExpression(iBuilder, stmt->getOperand(0));
            for (unsigned i = 1; i < stmt->getNumOperands(); ++i) {
                value = iBuilder->simd_xor(value, compileExpression(iBuilder, stmt->getOperand(1)));
            }
        } else if (const Sel * sel = dyn_cast<Sel>(stmt)) {
            Value* ifMask = compileExpression(iBuilder, sel->getCondition());
            Value* ifTrue = iBuilder->simd_and(ifMask, compileExpression(iBuilder, sel->getTrueExpr()));
            Value* ifFalse = iBuilder->simd_and(iBuilder->simd_not(ifMask), compileExpression(iBuilder, sel->getFalseExpr()));
            value = iBuilder->simd_or(ifTrue, ifFalse);
        } else if (isa<Not>(stmt)) {
            value = iBuilder->simd_not(compileExpression(iBuilder, stmt->getOperand(0)));
        } else if (isa<Advance>(stmt)) {
            const Advance * const adv = cast<Advance>(stmt);
            // If our expr is an Extract op on a mutable Var then we need to pass the index value to the carry
            // manager so that it properly selects the correct carry bit.
            value = mCarryManager->advanceCarryInCarryOut(iBuilder, adv, compileExpression(iBuilder, adv->getExpression()));
        } else if (const MatchStar * mstar = dyn_cast<MatchStar>(stmt)) {
            Value * const marker = compileExpression(iBuilder, mstar->getMarker());
            Value * const cc = compileExpression(iBuilder, mstar->getCharClass());
            Value * const marker_and_cc = iBuilder->simd_and(marker, cc);
            Value * const sum = mCarryManager->addCarryInCarryOut(iBuilder, mstar, marker_and_cc, cc);
            value = iBuilder->simd_or(iBuilder->simd_xor(sum, cc), marker);
        } else if (const ScanThru * sthru = dyn_cast<ScanThru>(stmt)) {
            Value * const from = compileExpression(iBuilder, sthru->getScanFrom());
            Value * const thru = compileExpression(iBuilder, sthru->getScanThru());
            Value * const sum = mCarryManager->addCarryInCarryOut(iBuilder, sthru, from, thru);
            value = iBuilder->simd_and(sum, iBuilder->simd_not(thru));
        } else if (const ScanTo * sthru = dyn_cast<ScanTo>(stmt)) {
            Value * const marker_expr = compileExpression(iBuilder, sthru->getScanFrom());
            Value * const to = iBuilder->simd_xor(compileExpression(iBuilder, sthru->getScanTo()), iBuilder->getScalarField("EOFmask"));
            Value * const sum = mCarryManager->addCarryInCarryOut(iBuilder, sthru, marker_expr, iBuilder->simd_not(to));
            value = iBuilder->simd_and(sum, to);
        } else if (const AdvanceThenScanThru * sthru = dyn_cast<AdvanceThenScanThru>(stmt)) {
            Value * const from = compileExpression(iBuilder, sthru->getScanFrom());
            Value * const thru = compileExpression(iBuilder, sthru->getScanThru());
            Value * const sum = mCarryManager->addCarryInCarryOut(iBuilder, sthru, from, iBuilder->simd_or(from, thru));
            value = iBuilder->simd_and(sum, iBuilder->simd_not(thru));
        } else if (const AdvanceThenScanTo * sthru = dyn_cast<AdvanceThenScanTo>(stmt)) {
            Value * const from = compileExpression(iBuilder, sthru->getScanFrom());
            Value * const to = iBuilder->simd_xor(compileExpression(iBuilder, sthru->getScanTo()), iBuilder->getScalarField("EOFmask"));
            Value * const sum = mCarryManager->addCarryInCarryOut(iBuilder, sthru, from, iBuilder->simd_or(from, iBuilder->simd_not(to)));
            value = iBuilder->simd_and(sum, to);
        } else if (const InFile * e = dyn_cast<InFile>(stmt)) {
            Value * EOFmask = iBuilder->getScalarField("EOFmask");
            value = iBuilder->simd_and(compileExpression(iBuilder, e->getExpr()), iBuilder->simd_not(EOFmask));
        } else if (const AtEOF * e = dyn_cast<AtEOF>(stmt)) {
            Value * EOFbit = iBuilder->getScalarField("EOFbit");
            value = iBuilder->simd_and(compileExpression(iBuilder, e->getExpr()), EOFbit);
        } else if (const Count * c = dyn_cast<Count>(stmt)) {
            Value * EOFbit = iBuilder->getScalarField("EOFbit");
            Value * EOFmask = iBuilder->getScalarField("EOFmask");
            Value * const to_count = iBuilder->simd_and(iBuilder->simd_or(iBuilder->simd_not(EOFmask), EOFbit), compileExpression(iBuilder, c->getExpr()));
            const unsigned counterSize = iBuilder->getSizeTy()->getBitWidth();
            const auto f = mAccumulator.find(c);
            if (LLVM_UNLIKELY(f == mAccumulator.end())) {
                report_fatal_error("Unknown accumulator: " + c->getName().str());
            }
            Value * ptr = iBuilder->getScalarFieldPtr(f->second);
            const auto alignment = getPointerElementAlignment(ptr);
            Value * countSoFar = iBuilder->CreateAlignedLoad(ptr, alignment, c->getName() + "_accumulator");
            auto fields = (iBuilder->getBitBlockWidth() / counterSize);
            Value * fieldCounts = iBuilder->simd_popcount(counterSize, to_count);
            while (fields > 1) {
                fields = fields/2;
                fieldCounts = iBuilder->CreateAdd(fieldCounts, iBuilder->mvmd_srli(counterSize, fieldCounts, fields));
            }
            value = iBuilder->CreateAdd(iBuilder->mvmd_extract(counterSize, fieldCounts, 0), countSoFar, "countSoFar");
            iBuilder->CreateAlignedStore(value, ptr, alignment);
        } else if (const Lookahead * l = dyn_cast<Lookahead>(stmt)) {
            Var * var = nullptr;
            PabloAST * stream = l->getExpression();
            Value * index = iBuilder->getInt32(0);
            if (LLVM_UNLIKELY(isa<Extract>(stream))) {
                var = cast<Var>(cast<Extract>(stream)->getArray());
            }
            const auto bit_shift = (l->getAmount() % iBuilder->getBitBlockWidth());
            const auto block_shift = (l->getAmount() / iBuilder->getBitBlockWidth());

            Value * ptr = iBuilder->getAdjustedInputStreamBlockPtr(iBuilder->getSize(block_shift), var->getName(), index);
            Value * lookAhead = iBuilder->CreateBlockAlignedLoad(ptr);
            if (bit_shift == 0) {  // Simple case with no intra-block shifting.
                value = lookAhead;
            } else { // Need to form shift result from two adjacent blocks.
                Value * ptr = iBuilder->getAdjustedInputStreamBlockPtr(iBuilder->getSize(block_shift + 1), var->getName(), index);
                Value * lookAhead1 = iBuilder->CreateBlockAlignedLoad(ptr);
                if (LLVM_UNLIKELY((bit_shift % 8) == 0)) { // Use a single whole-byte shift, if possible.
                    value = iBuilder->mvmd_dslli(8, lookAhead1, lookAhead, (bit_shift / 8));
                } else {
                    Type  * const streamType = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());
                    Value * b1 = iBuilder->CreateBitCast(lookAhead1, streamType);
                    Value * b0 = iBuilder->CreateBitCast(lookAhead, streamType);
                    Value * result = iBuilder->CreateOr(iBuilder->CreateShl(b1, iBuilder->getBitBlockWidth() - bit_shift), iBuilder->CreateLShr(b0, bit_shift));
                    value = iBuilder->CreateBitCast(result, iBuilder->getBitBlockType());
                }
            }

        } else {
            std::string tmp;
            raw_string_ostream out(tmp);
            out << "PabloCompiler: ";
            stmt->print(out);
            out << " was not recognized by the compiler";
            report_fatal_error(out.str());
        }

        mMarker[expr] = value;
        if (DebugOptionIsSet(DumpTrace)) {
            const String & name = isa<Var>(expr) ? cast<Var>(expr)->getName() : cast<Statement>(expr)->getName();
            if (value->getType()->isVectorTy()) {
                iBuilder->CallPrintRegister(name.str(), value);
            } else if (value->getType()->isIntegerTy()) {
                iBuilder->CallPrintInt(name.str(), value);
            }
        }
    }
}

Value * PabloCompiler::compileExpression(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloAST * expr, const bool ensureLoaded) const {
    if (LLVM_UNLIKELY(isa<Ones>(expr))) {
        return iBuilder->allOnes();
    } else if (LLVM_UNLIKELY(isa<Zeroes>(expr))) {
        return iBuilder->allZeroes();
    } else if (LLVM_UNLIKELY(isa<Integer>(expr))) {
        return ConstantInt::get(cast<Integer>(expr)->getType(), cast<Integer>(expr)->value());
    } else if (LLVM_UNLIKELY(isa<Operator>(expr))) {
        const Operator * op = cast<Operator>(expr);
        Value * lh = compileExpression(iBuilder, op->getLH());
        Value * rh = compileExpression(iBuilder, op->getRH());
        if (LLVM_UNLIKELY(lh->getType() != rh->getType())) {
            std::string tmp;
            raw_string_ostream out(tmp);
            out << "Operator creation error: left hand type of ";
            expr->print(out);
            out << " (";
            lh->getType()->print(out);
            out << ") differs from right hand type (";
            rh->getType()->print(out);
            out << ")";
            report_fatal_error(out.str());
        }
        switch (op->getClassTypeId()) {
            case TypeId::Add:
                return iBuilder->CreateAdd(lh, rh);
            case TypeId::Subtract:
                return iBuilder->CreateSub(lh, rh);
            case TypeId::LessThan:
                return iBuilder->CreateICmpSLT(lh, rh);
            case TypeId::LessThanEquals:
                return iBuilder->CreateICmpSLE(lh, rh);
            case TypeId::Equals:
                return iBuilder->CreateICmpEQ(lh, rh);
            case TypeId::GreaterThanEquals:
                return iBuilder->CreateICmpSGE(lh, rh);
            case TypeId::GreaterThan:
                return iBuilder->CreateICmpSGT(lh, rh);
            case TypeId::NotEquals:
                return iBuilder->CreateICmpNE(lh, rh);
            default: break;
        }
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "PabloCompiler: ";
        expr->print(out);
        out << " is not a valid Operator";
        report_fatal_error(out.str());
    }
    const auto f = mMarker.find(expr);
    if (LLVM_UNLIKELY(f == mMarker.end())) {
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "PabloCompiler: ";
        expr->print(out);
        out << " was used before definition!";
        report_fatal_error(out.str());
    }
    Value * value = f->second;
    if (LLVM_UNLIKELY(isa<GetElementPtrInst>(value) && ensureLoaded)) {
        value = iBuilder->CreateAlignedLoad(value, getPointerElementAlignment(value));
    }
    return value;
}

PabloCompiler::PabloCompiler(PabloKernel * const kernel)
: mKernel(kernel)
, mCarryManager(new CarryManager)
, mBranchCount(0) {
    assert ("PabloKernel cannot be null!" && kernel);
}

PabloCompiler::~PabloCompiler() {
    delete mCarryManager;
}

}
