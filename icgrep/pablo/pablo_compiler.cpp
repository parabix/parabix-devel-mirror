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
#include <pablo/prototype.h>
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

using TypeId = PabloAST::ClassTypeId;

void PabloCompiler::initializeKernelData() {
    Examine();
    mCarryManager->initializeCarryData(mKernel);
}
    
void PabloCompiler::compile(Value * const self, Function * function) {

    // Make sure that we generate code into the right module.
    mSelf = self;
    mFunction = function;

    //Generate Kernel//
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", function, 0));

    mCarryManager->initializeCodeGen(self, function);
      
    PabloBlock * const entryBlock = mKernel->getEntryBlock(); assert (entryBlock);
    mMarkerMap.emplace(entryBlock->createZeroes(), iBuilder->allZeroes());
    mMarkerMap.emplace(entryBlock->createOnes(), iBuilder->allOnes());

    Value * const blockNo = mKernel->getScalarField(mSelf, blockNoScalar);

    for (unsigned i = 0; i < mKernel->getNumOfInputs(); ++i) {
        Var * var = mKernel->getInput(i);
        std::string name = var->getName()->to_string();
        Value * input = nullptr;
        if (var->getType()->isSingleValueType()) {
            input = mKernel->getScalarFieldPtr(mSelf, name);
        } else {
            input = mKernel->getStreamSetBlockPtr(mSelf, name, blockNo);
        }
        mMarkerMap.emplace(var, input);
    }

    for (unsigned i = 0; i < mKernel->getNumOfOutputs(); ++i) {
        Var * var = mKernel->getOutput(i);
        std::string name = var->getName()->to_string();
        Value * output = nullptr;
        if (var->getType()->isSingleValueType()) {
            output = mKernel->getScalarFieldPtr(mSelf, name);
        } else {
            output = mKernel->getStreamSetBlockPtr(mSelf, name, blockNo);
        }
        mMarkerMap.emplace(var, output);
    }

    compileBlock(entryBlock);

    #ifdef PRINT_TIMING_INFORMATION
    const timestamp_t pablo_compilation_end = read_cycle_counter();
    std::cerr << "PABLO COMPILATION TIME: " << (pablo_compilation_end - pablo_compilation_start) << std::endl;
    #endif
}

inline void PabloCompiler::Examine() {
    Examine(mKernel->getEntryBlock());
}

void PabloCompiler::Examine(const PabloBlock * const block) {
    for (const Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<Lookahead>(stmt))) {
            const Lookahead * const la = cast<Lookahead>(stmt);
            assert (isa<Var>(la->getExpr()));
            if (LLVM_LIKELY(la->getAmount() > mKernel->getLookAhead())) {
                mKernel->setLookAhead(la->getAmount());
            }
        } else if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            Examine(cast<Branch>(stmt)->getBody());
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

    BasicBlock * const ifEntryBlock = iBuilder->GetInsertBlock();
    BasicBlock * const ifBodyBlock = BasicBlock::Create(mFunction->getContext(), "if.body", mFunction);
    BasicBlock * const ifEndBlock = BasicBlock::Create(mFunction->getContext(), "if.end", mFunction);
    
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
    
    mCarryManager->enterIfScope(ifBody);

    Value * condition = compileExpression(ifStatement->getCondition());
    if (condition->getType() == iBuilder->getBitBlockType()) {
        condition = iBuilder->bitblock_any(mCarryManager->generateSummaryTest(condition));
    }
    
    iBuilder->CreateCondBr(condition, ifBodyBlock, ifEndBlock);
    
    // Entry processing is complete, now handle the body of the if.
    iBuilder->SetInsertPoint(ifBodyBlock);

    mCarryManager->enterIfBody(ifEntryBlock);

    compileBlock(ifBody);

    BasicBlock * ifExitBlock = iBuilder->GetInsertBlock();    

    mCarryManager->leaveIfBody(ifExitBlock);

    iBuilder->CreateBr(ifEndBlock);
    //End Block
    iBuilder->SetInsertPoint(ifEndBlock);

    mCarryManager->leaveIfScope(ifEntryBlock, ifExitBlock);

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

        Value * const next = f->second;

        assert (value->getType() == next->getType());

        PHINode * phi = iBuilder->CreatePHI(value->getType(), 2, getName(var));
        phi->addIncoming(value, ifEntryBlock);
        phi->addIncoming(next, ifExitBlock);
        f->second = phi;

        assert (mMarkerMap[var] == phi);
    }    
}

void PabloCompiler::compileWhile(const While * const whileStatement) {

    PabloBlock * const whileBody = whileStatement->getBody();
    
    BasicBlock * whileEntryBlock = iBuilder->GetInsertBlock();

    BasicBlock * whileBodyBlock = BasicBlock::Create(iBuilder->getContext(), "while.body", mFunction);

    BasicBlock * whileEndBlock = BasicBlock::Create(iBuilder->getContext(), "while.end", mFunction);

    const auto escaped = whileStatement->getEscaped();

#ifdef ENABLE_BOUNDED_WHILE
    PHINode * bound_phi = nullptr;  // Needed for bounded while loops.
#endif
    // On entry to the while structure, proceed to execute the first iteration
    // of the loop body unconditionally.   The while condition is tested at the end of
    // the loop.

    mCarryManager->enterLoopScope(whileBody);

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
        auto f = mMarkerMap.find(var);
        if (LLVM_UNLIKELY(f == mMarkerMap.end())) {
            std::string tmp;
            raw_string_ostream out(tmp);
            PabloPrinter::print(var, out);
            out << " is uninitialized prior to entering ";
            PabloPrinter::print(whileStatement, out);
            llvm::report_fatal_error(out.str());
        }
        Value * entryValue = f->second;
        PHINode * phi = iBuilder->CreatePHI(entryValue->getType(), 2, getName(var));
        phi->addIncoming(entryValue, whileEntryBlock);
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

    mCarryManager->enterLoopBody(whileEntryBlock);

    //
    // Now compile the loop body proper.  Carry-out accumulated values
    // and iterated values of Next nodes will be computed.
    compileBlock(whileBody);

    // After the whileBody has been compiled, we may be in a different basic block.
    BasicBlock * whileExitBlock = iBuilder->GetInsertBlock();

    mCarryManager->leaveLoopBody(whileExitBlock);

    // Terminate the while loop body with a conditional branch back.
    Value * condition = compileExpression(whileStatement->getCondition());
    if (condition->getType() == iBuilder->getBitBlockType()) {
        condition = iBuilder->bitblock_any(condition);
    }
#ifdef ENABLE_BOUNDED_WHILE
    if (whileStatement->getBound()) {
        Value * new_bound = iBuilder->CreateSub(bound_phi, iBuilder->getSize(1));
        bound_phi->addIncoming(new_bound, whileExitBlock);
        condition = iBuilder->CreateAnd(condition, iBuilder->CreateICmpUGT(new_bound, ConstantInt::getNullValue(iBuilder->getSizeTy())));
    }
#endif

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
        Value * exitValue = f->second;
        assert (phi->getType() == exitValue->getType());
        phi->addIncoming(exitValue, whileExitBlock);
        f->second = phi;
    }

    iBuilder->CreateCondBr(condition, whileBodyBlock, whileEndBlock);

    iBuilder->SetInsertPoint(whileEndBlock);

    mCarryManager->leaveLoopScope(whileEntryBlock, whileExitBlock);

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
                for (unsigned i = 0; i < mKernel->getNumOfOutputs(); ++i) {
                    if (expr == mKernel->getOutput(i)) {
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
                    out << "PabloCompiler: use-before-definition error: ";
                    expr->print(out);
                    out << " does not dominate ";
                    stmt->print(out);
                    llvm::report_fatal_error(out.str());
                }
                Value * const ptr = f->second;

                assert (&(value->getContext()) == &(ptr->getContext()));

                if (isa<Count>(cast<Assign>(stmt)->getValue())) {
                    Value * count = iBuilder->CreateLoad(ptr);
                    value = iBuilder->CreateTruncOrBitCast(value, count->getType());
                    value = iBuilder->CreateAdd(value, count);
                }

                const Type * const type = value->getType();
                if (isa<VectorType>(type) || isa<IntegerType>(type)) {
                    const auto bitWidth = isa<VectorType>(type)
                            ? cast<VectorType>(type)->getBitWidth()
                            : cast<IntegerType>(type)->getBitWidth();
                    iBuilder->CreateAlignedStore(value, ptr, bitWidth / 8);
                } else {
                    iBuilder->CreateStore(value, ptr);
                }
            }

        } else if (const Extract * extract = dyn_cast<Extract>(stmt)) {
            Value * array = compileExpression(extract->getArray(), false);
            Value * index = compileExpression(extract->getIndex());
            value = iBuilder->CreateGEP(array, {ConstantInt::getNullValue(index->getType()), index}, getName(stmt));
        } else if (isa<And>(stmt)) {
            value = compileExpression(stmt->getOperand(0));
            for (unsigned i = 1; i < stmt->getNumOperands(); ++i) {
                value = iBuilder->simd_and(value, compileExpression(stmt->getOperand(1)));
            }
        } else if (isa<Or>(stmt)) {
            value = compileExpression(stmt->getOperand(0));
            for (unsigned i = 1; i < stmt->getNumOperands(); ++i) {
                value = iBuilder->simd_or(value, compileExpression(stmt->getOperand(1)));
            }
        } else if (isa<Xor>(stmt)) {
            value = compileExpression(stmt->getOperand(0));
            for (unsigned i = 1; i < stmt->getNumOperands(); ++i) {
                value = iBuilder->simd_xor(value, compileExpression(stmt->getOperand(1)));
            }
        } else if (const Sel * sel = dyn_cast<Sel>(stmt)) {
            Value* ifMask = compileExpression(sel->getCondition());
            Value* ifTrue = iBuilder->simd_and(ifMask, compileExpression(sel->getTrueExpr()));
            Value* ifFalse = iBuilder->simd_and(iBuilder->simd_not(ifMask), compileExpression(sel->getFalseExpr()));
            value = iBuilder->simd_or(ifTrue, ifFalse);
        } else if (isa<Not>(stmt)) {
            value = iBuilder->simd_not(compileExpression(stmt->getOperand(0)));
        } else if (isa<Advance>(stmt)) {
            const Advance * const adv = cast<Advance>(stmt);
            // If our expr is an Extract op on a mutable Var then we need to pass the index value to the carry
            // manager so that it properly selects the correct carry bit.
            value = mCarryManager->advanceCarryInCarryOut(adv, compileExpression(adv->getExpression()));
        } else if (const MatchStar * mstar = dyn_cast<MatchStar>(stmt)) {
            Value * const marker = compileExpression(mstar->getMarker());
            Value * const cc = compileExpression(mstar->getCharClass());
            Value * const marker_and_cc = iBuilder->simd_and(marker, cc);
            Value * const sum = mCarryManager->addCarryInCarryOut(mstar, marker_and_cc, cc);
            value = iBuilder->simd_or(iBuilder->simd_xor(sum, cc), marker);
        } else if (const ScanThru * sthru = dyn_cast<ScanThru>(stmt)) {
            Value * const marker_expr = compileExpression(sthru->getScanFrom());
            Value * const cc_expr = compileExpression(sthru->getScanThru());
            Value * const sum = mCarryManager->addCarryInCarryOut(sthru, marker_expr, cc_expr);
            value = iBuilder->simd_and(sum, iBuilder->simd_not(cc_expr));
        } else if (const InFile * e = dyn_cast<InFile>(stmt)) {
            Value * EOFmask = mKernel->getScalarField(mSelf, "EOFmask");
            value = iBuilder->simd_xor(compileExpression(e->getExpr()), EOFmask);
        } else if (const AtEOF * e = dyn_cast<AtEOF>(stmt)) {
            Value * EOFbit = mKernel->getScalarField(mSelf, "EOFbit");
            value = iBuilder->simd_and(compileExpression(e->getExpr()), EOFbit);
        } else if (const Count * c = dyn_cast<Count>(stmt)) {
            Value * const to_count = compileExpression(c->getExpr());
            const unsigned counterSize = iBuilder->getSizeTy()->getBitWidth();
            Value * const partial = iBuilder->simd_popcount(counterSize, to_count);
            if (LLVM_UNLIKELY(counterSize <= 1)) {
                value = partial;
            } else {
                value = iBuilder->mvmd_extract(counterSize, partial, 0);
                const auto fields = (iBuilder->getBitBlockWidth() / counterSize);
                for (unsigned i = 1; i < fields; ++i) {
                    Value * temp = iBuilder->mvmd_extract(counterSize, partial, i);
                    value = iBuilder->CreateAdd(value, temp);
                }
            }
        } else if (const Lookahead * l = dyn_cast<Lookahead>(stmt)) {
            PabloAST * const var = l->getExpr();
            if (LLVM_UNLIKELY(!isa<Var>(var))) {
                throw std::runtime_error("Lookahead operations may only be applied to input streams");
            }
            unsigned index = 0;
            for (; index < mKernel->getNumOfInputs(); ++index) {
                if (mKernel->getInput(index) == var) {
                    break;
                }
            }
            if (LLVM_UNLIKELY(index >= mKernel->getNumOfInputs())) {
                throw std::runtime_error("Lookahead has an illegal Var operand");
            }
            const unsigned bit_shift = (l->getAmount() % iBuilder->getBitBlockWidth());
            const unsigned block_shift = (l->getAmount() / iBuilder->getBitBlockWidth());
            std::string inputName = var->getName()->to_string();;
            Value * blockNo = mKernel->getScalarField(mSelf, blockNoScalar);
            Value * lookAhead_blockPtr  = mKernel->getStreamSetBlockPtr(mSelf, inputName, iBuilder->CreateAdd(blockNo, ConstantInt::get(iBuilder->getSizeTy(), block_shift)));
            Value * lookAhead_inputPtr = iBuilder->CreateGEP(lookAhead_blockPtr, {iBuilder->getInt32(0), iBuilder->getInt32(index)});
            Value * lookAhead = iBuilder->CreateBlockAlignedLoad(lookAhead_inputPtr);
            if (bit_shift == 0) {  // Simple case with no intra-block shifting.
                value = lookAhead;
            } else { // Need to form shift result from two adjacent blocks.
                Value * lookAhead_blockPtr1  = mKernel->getStreamSetBlockPtr(mSelf, inputName, iBuilder->CreateAdd(blockNo, ConstantInt::get(iBuilder->getSizeTy(), block_shift + 1)));
                Value * lookAhead_inputPtr1 = iBuilder->CreateGEP(lookAhead_blockPtr1, {iBuilder->getInt32(0), iBuilder->getInt32(index)});
                Value * lookAhead1 = iBuilder->CreateBlockAlignedLoad(lookAhead_inputPtr1);
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
            llvm::raw_string_ostream msg(tmp);
            msg << "Internal error: ";
            PabloPrinter::print(stmt, msg);
            msg << " is not a recognized statement in the Pablo compiler.";
            throw std::runtime_error(msg.str());
        }

        mMarkerMap[expr] = value;
        if (DebugOptionIsSet(DumpTrace)) {
            assert (expr->getName());
            iBuilder->CallPrintRegister(expr->getName()->to_string(), value);
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
    } else if (LLVM_UNLIKELY(isa<Operator>(expr))) {
        const Operator * op = cast<Operator>(expr);
        Value * lh = compileExpression(op->getLH());
        Value * rh = compileExpression(op->getRH());
        assert (lh->getType() == rh->getType());
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
            default:
                break;
        }
        std::string tmp;
        raw_string_ostream out(tmp);
        expr->print(out);
        out << " is not a valid Operator";
        llvm::report_fatal_error(out.str());
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

PabloCompiler::PabloCompiler(PabloKernel * kernel)
: iBuilder(kernel->getBuilder())
, mCarryManager(new CarryManager(iBuilder))
, mKernel(kernel)
, mFunction(nullptr) {

}

PabloCompiler::~PabloCompiler() {
    delete mCarryManager;
}

}
