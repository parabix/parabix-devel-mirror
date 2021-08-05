#include <kernel/util/directorysearch.h>

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/syscall.h>
#include <kernel/core/kernel_builder.h>
#include <llvm/IR/Module.h>
#include <codegen/TypeBuilder.h>

#define FIRST_NONSYSTEM_INODE 11

using namespace llvm;

namespace kernel {

void DirectorySearch::linkExternalMethods(BuilderRef b) {
    fOpen = b->LinkFunction("open", open);
    fSysCall = b->LinkFunction("syscall", syscall);
}

void DirectorySearch::generateInitializeMethod(BuilderRef b) {

    Value * const buffer = b->CreateCacheAlignedMalloc(b->getSize(PATH_MAX * 2));
    b->setScalarField("buffer", buffer);
    Value * const rootPath = b->getScalarField("rootPath");

    std::vector<Value *> args(2);
    args[0] = rootPath;
    args[1] = b->getInt32(O_RDONLY | O_DIRECTORY);
    Value * const fd = b->CreateCall(fOpen->getFunctionType(), fOpen, args);
    b->setScalarField("fileDescriptor", fd);
    BasicBlock * const invalid = b->CreateBasicBlock("invalidDirectory");
    BasicBlock * const exit = b->CreateBasicBlock("exit");

    Value * const isInvalid = b->CreateICmpEQ(fd, ConstantInt::getAllOnesValue(fd->getType()));
    b->CreateUnlikelyCondBr(isInvalid, invalid, exit);

    b->setInsertionPoint(invalid);
    b->setTerminationSignal();
    b->CreateBr(exit);

    b->setInsertionPoint(exit);
    Value * const rootPathLength = b->CreateStrlenCall(rootPath);
    b->setScalarField("currentPathLength", rootPathLength);
    addToOutputStream(b, rootPath, rootPathLength, "directoryNames", b->getSize(0));
}

StructType * getDirEntTy(llvm::LLVMContext & c) {
    std::vector<Type *> fields;
    #ifdef __USE_LARGEFILE64
    fields.push_back(TypeBuilder<ino64_t>::get(c)); /* 64-bit inode number */
    #define d_ino 0
    fields.push_back(TypeBuilder<off64_t>::get(c));  /* 64-bit offset to next structure */
    #define d_off 1
    fields.push_back(TypeBuilder<unsigned short>::get(c)); /* Size of this dirent */
    #define d_reclen 2
    fields.push_back(TypeBuilder<unsigned char>::get(c)); /* File type */
    #define d_type 3
    fields.push_back(TypeBuilder<char *>::get(c)); /* Filename (null-terminated). length is actually (d_reclen - 2 - offsetof(struct linux_dirent, d_name)) */
    #define d_name 4
    #else
    fields.push_back(TypeBuilder<unsigned long>::get(c)); /* Inode number */
    #define d_ino 0
    fields.push_back(TypeBuilder<unsigned long>::get(c)); /* Offset to next linux_dirent */
    #define d_off 1
    fields.push_back(TypeBuilder<unsigned short>::get(c)); /* Length of this linux_dirent */
    #define d_reclen 2
    fields.push_back(TypeBuilder<char *>::get(c)); /* Filename (null-terminated) */
    #define d_name 3
    fields.push_back(TypeBuilder<char>::get(c)); // Zero padding byte
    fields.push_back(TypeBuilder<char>::get(c)); // File type (only since Linux 2.6.4); offset is (d_reclen - 1)
    #define d_type 5
    #endif
    return StructType::get(c, fields, true);
}

#ifdef __USE_LARGEFILE64
#define GETDENTNO SYS_getdents64
#define NON_NAME_PADDING_BYTES 0
#else
#define GETDENTNO SYS_getdents
#define NON_NAME_PADDING_BYTES 2
#endif

void DirectorySearch::addToOutputStream(BuilderRef b, Value * const name, Value * const nameLength, StringRef field, Value * const consumed) {
    Value * const produced = b->getProducedItemCount(field);
    StreamSetBuffer * buffer = getStreamSetBuffer(field);
    Value * const writable = buffer->getLinearlyWritableItems(b, produced, consumed);
    Value * const hasSpace = b->CreateICmpULT(nameLength, writable);
    BasicBlock * expandBuffer = b->CreateBasicBlock(field + "Expand");
    BasicBlock * writeName = b->CreateBasicBlock(field + "Write");
    b->CreateLikelyCondBr(hasSpace, writeName, expandBuffer);
    // expand the output stream buffer if we cannot fit the name into it.
    b->setInsertPoint(expandBuffer);
    Value * const unconsumed = b->CreateSub(produced, consumed);
    Value * size = b->CreateZExtOrTrunc(nameLength, unconsumed->getType());
    size = b->CreateMul(size, ConstantInt::get(size->getType(), 16));
    size = b->CreateAdd(size, unconsumed);
    buffer->setCapacity(b, size);
    b->CreateBr(writeName);
    // write the name to the buffer
    b->setInsertPoint(writeName);
    Value * const ptr = b->getRawOutputPointer(field, produced);
    b->CreateMemCpy(ptr, name, nameLength, 1);
}


void DirectorySearch::generateDoSegmentMethod(BuilderRef b) {

    BasicBlock * const readDirectory = b->CreateBasicBlock("readDirectory");
    BasicBlock * const processDirEnts = b->CreateBasicBlock("processBuffer");
    BasicBlock * const checkEntry = b->CreateBasicBlock("checkEntry");

    BasicBlock * const addFile = b->CreateBasicBlock("addFile");
    BasicBlock * const nonFile = b->CreateBasicBlock("addFile");

    BasicBlock * const addDirectory = b->CreateBasicBlock("addDirectory");
    BasicBlock * const nextEntry = b->CreateBasicBlock("nextEntry");
    BasicBlock * const finishedDirectory = b->CreateBasicBlock("finishedDirectory");

    BasicBlock * const filledSegment = b->CreateBasicBlock("filledSegment");



    std::vector<Value *> args(4);
    IntegerType * const sysNoTy = b->getIntNTy(sizeof(long int) * 8);
    args[0] = ConstantInt::get(sysNoTy, GETDENTNO);
    args[1] = b->getScalarField("fileDescriptor");
    args[2] = b->getScalarField("buffer");
    args[3] = b->getScalarField("size");

    // check whether there is any pending data in the buffer
    StructType * const dirEntryTy = getDirEntTy(b->getContext());
    Type * const recordLengthTy = dirEntryTy->getStructElementType(d_reclen);
    Value * const pendingOffset = b->getScalarField("pendingOffset");
    Value * const pendingBytes = b->getScalarField("pendingBytes");

    Value * initialDirectoryOffset = nullptr;
    if (mRecursive) {
        initialDirectoryOffset = b->getScalarField("currentDirectoryOffset");
    } else {
        initialDirectoryOffset = b->getSize(0);
    }
    Value * const initialDirectoryLength = b->getScalarField("currentPathLength");

    BasicBlock * const entryBlock = b->getInsertBlock();
    Value * const initiallyExhausted = b->CreateICmpEQ(pendingOffset, pendingBytes);
    b->CreateCondBr(initiallyExhausted, readDirectory, processDirEnts);

    // read the directory entries from the OS
    b->setInsertPoint(readDirectory);
    Value * baseDirectoryLength = initialDirectoryLength;
    if (mRecursive) {
        PHINode * const baseDirectoryLengthPhi = b->CreatePHI(b->getSizeTy(), 2);
        baseDirectoryLength->addIncoming(initialDirectoryLength, entryBlock);
        baseDirectoryLength = baseDirectoryLengthPhi;


    }


    Value * const bytesRead = b->CreateCall(fSysCall->getFunctionType(), fSysCall, args);
    // TODO: if -1, resize?
    Value * const hasData = b->CreateIsNotNull(bytesRead);
    BasicBlock * const readDirectoryExit = b->getInsertBlock();
    b->CreateLikelyCondBr(hasData, processDirEnts, finishedDirectory);

    // begin processing the directory entries
    b->setInsertPoint(processDirEnts);
    PHINode * const offset = b->CreatePHI(recordLengthTy, 3);
    offset->addIncoming(ConstantInt::get(recordLengthTy, 0), readDirectory);
    offset->addIncoming(pendingOffset, entryBlock);

    PHINode * const numBytes = b->CreatePHI(recordLengthTy, 3);
    numBytes->addIncoming(bytesRead, readDirectory);
    numBytes->addIncoming(pendingBytes, entryBlock);

    PHINode * const currentDirectoryOffset = nullptr;
    if (mRecursive) {
        currentDirectoryOffset = b->CreatePHI(b->getSizeTy(), 3);
        currentDirectoryOffset->addIncoming(initialDirectoryOffset, readDirectory);
        currentDirectoryOffset->addIncoming(initialDirectoryOffset, entryBlock);
    }

    PHINode * const currentDirectoryLength = b->CreatePHI(b->getSizeTy(), 3);
    currentDirectoryLength->addIncoming(baseDirectoryLength, readDirectory);
    currentDirectoryLength->addIncoming(initialDirectoryLength, entryBlock);


    // parse the entry
    Value * const bufferOffset = b->CreateGEP(buffer, offset);
    Value * const dirEntry = b->CreatePointerCast(bufferOffset, dirEntryTy);
    Value * const iNodePtr = b->CreateGEP(dirEntry, {b->getInt32(0), b->getInt32(d_ino)});
    Value * const iNode = b->CreateLoad(iNodePtr);
    Constant * const firstNonSystemINode = ConstantInt::get(iNode->getType(), FIRST_NONSYSTEM_INODE);
    Value * const nonSystemEntry = b->CreateICmpULT(iNode, firstNonSystemINode);
    Value * const recLenPtr = b->CreateGEP(dirEntry, {b->getInt32(0), b->getInt32(d_reclen)});
    Value * const recLen = b->CreateLoad(recLenPtr);
    b->CreateCondBr(nonSystemEntry, checkEntry, nextEntry);

    // not a system file; check the entry
    b->setInsertPoint(checkEntry);
    // does this entry refer to a file or directory?
    Value * const typePtr = b->CreateGEP(dirEntry, {b->getInt32(0), b->getInt32(d_type)});
    Value * const type = b->CreateLoad(typePtr);
    Constant * const FILE_FLAG = ConstantInt::get(type->getType(), DT_REG);
    Constant * const DIR_FLAG = ConstantInt::get(type->getType(), DT_DIR);
    Constant * const FLAG_NOT_SET = ConstantInt::get(type->getType(), 0);

    Value * const name = b->CreateGEP(dirEntry, {b->getInt32(0), b->getInt32(d_name)});
    // unless we want to include all hidden files, check if this file is hidden
    if (!mIncludeHidden) {
        BasicBlock * const checkFile = b->CreateBasicBlock("notHidden");
        Value * const notHidden = b->CreateICmpNE(b->CreateLoad(name), b->getInt8('.'));
        b->CreateLikelyCondBr(notHidden, checkFile, nextEntry);

        b->setInsertPoint(checkFile);
    }

    // compute the name length
    DataLayout DL(b->getModule());
    const StructLayout * const dirEntLayout = DL.getStructLayout(dirEntryTy);
    const auto nameOffset = dirEntLayout->getElementOffset(d_name);
    Constant * const nonNameBytes = ConstantInt::get(recLen->getType(), nameOffset + NON_NAME_PADDING_BYTES);
    Value * const nameLength = b->CreateSub(recLen, nonNameBytes);

    // is this entry a file?
    Value * const isFile = b->CreateICmpNE(b->CreateICmpAnd(type, FILE_FLAG), FLAG_NOT_SET);
    b->CreateLikelyCondBr(isFile, addFile, nonFile);

    // add file to the file stream
    b->setInsertPoint(addFile);
    addToOutputStream(b, name, nameLength, "fileNames", b->getConsumedItemCount(field));
    // TODO: add the directory index and test whether we can store any more this segment?
    b->CreateBr(nextEntry);

    // not a file; is it a directory?
    b->setInsertPoint(nonFile);
    Value * const isDirectory = b->CreateICmpNE(b->CreateICmpAnd(type, DIR_FLAG), FLAG_NOT_SET);
    b->CreateLikelyCondBr(isDirectory, addDirectory, nextEntry);
    // add directory name to the directory stream
    b->setInsertPoint(isDirectory);
    // need to add current path prefix
    addToOutputStream(b, name, nameLength, "directoryNames", b->getSize(0));
    b->CreateBr(nextEntry);

    // iterate to the next file until we've exhausted the buffer
    b->setInsertPoint(nextEntry);
    Value * const nextOffset = b->CreateAdd(offset, recLen);
    offset->addIncoming(nextOffset, nextEntry);
    numBytes->addIncoming(numBytes, nextEntry);

    Value * const exhaustedBuffer = b->CreateICmpULT(nextOffset, numBytes);
    b->CreateUnlikelyCondBr(exhaustedBuffer, processDirEnts, readDirectory);

    b->setInsertPoint(finishedDirectory);
    // if this is recursive, check for a pending directory in the buffer
    if (mRecursive) {
        PHINode * const directoryOffset = b->CreatePHI(currentDirectoryOffset->getType(), 2);
        directoryOffset->addIncoming(currentDirectoryOffset, readDirectoryExit);
        PHINode * const pathLength = b->CreatePHI(currentDirectoryLength->getType(), 2);
        pathLength->addIncoming(currentDirectoryLength, readDirectoryExit);
        Value * const nextDirectoryOffset = b->CreateAdd(directoryOffset, pathLength);
        Value * const directoryNameLength = b->getProducedItemCount("directoryNames");
        Value * const hasPendingFolder = b->CreateICmpNE(nextDirectoryOffset, directoryNameLength);
        BasicBlock * const nextDirectory = b->CreateBasicBlock("nextDirectory");
        BasicBlock * const checkedAllDirectories = b->CreateBasicBlock("checkedAllDirectories");
        b->CreateCondBr(hasPendingFolder, nextDirectory, checkedAllDirectories);

        // open the next directory; if it's valid, resume parsing the directory entries; otherwise
        // check the subsequent folder.
        b->setInsertPoint(nextDirectory);
        Value * const nextPath = b->getRawOutputPointer("directoryNames", nextDirectoryOffset);
        Value * const nextPathLength = b->CreateStrlenCall(nextPath);
        std::vector<Value *> args(2);
        args[0] = nextPath;
        args[1] = b->getInt32(O_RDONLY | O_DIRECTORY);
        Value * const fd = b->CreateCall(fOpen->getFunctionType(), fOpen, args);
        b->setScalarField("fileDescriptor", fd);
        Value * const valid = b->CreateICmpNE(fd, ConstantInt::getAllOnesValue(fd->getType()));
        BasicBlock * const nextDirectoryExit = b->getInsertBlock();
        directoryOffset->addIncoming(nextDirectoryOffset, nextDirectoryExit);
        pathLength->addIncoming(nextPathLength, nextDirectoryExit);
        cast<PHINode>(baseDirectoryLength)->addIncoming(nextPathLength, nextDirectoryExit);
        b->CreateLikelyCondBr(valid, readDirectory, finishedDirectory);

        b->setInsertPoint(checkedAllDirectories);
    }
    b->setTerminationSignal();
    filledSegment->moveAfter(b->getInsertBlock());
    b->CreateBr(filledSegment);

    b->setInsertPoint(filledSegment);
}

void DirectorySearch::generateFinalizeMethod(BuilderRef b) {


}

DirectorySearch::DirectorySearch(BuilderRef b,
                                 Scalar * const rootPath,
                                 StreamSet * const directoryNameStream,
                                 StreamSet * const fileDirectoryStream, // stores an offset number to the directoryNameStream
                                 StreamSet * const fileNameStream,
                                 const unsigned filesPerSegment, const bool recursive = true, const bool includeHidden = false)
: SegmentOrientedKernel(b, "DirectorySearch" + (recursive ? "R" : "") + (includeHidden ? "H" :"")
// input streams
,{}
// output stream
,{Binding{"directoryNames", directoryNameStream, UnknownRate(), ManagedBuffer()}
, Binding{"fileDirectoryOffset", fileDirectoryStream, FixedRate()}
, Binding{"fileNames", fileNameStream, UnknownRate(), ManagedBuffer()}
}
// input scalar
,{Binding{"rootPath", rootPath}}
// output scalar
,{}
// internal scalars
,{Binding{b->getInt32Ty(), "fileDescriptor"}
, Binding{b->getIntNTy(sizeof(unsigned short) * 8), "pendingOffset"}
, Binding{b->getIntNTy(sizeof(unsigned short) * 8), "pendingBytes"}
, Binding{b->getInt8PtrTy(), "buffer"}
, Binding{b->getSizeTy(), "size"}
, Binding{b->getSizeTy(), "currentPathLength"}
})
, mRecursive(recursive)
, mIncludeHidden(includeHidden) {
    if (recursive) {
        addInternalScalar(b->getSizeTy(), "currentDirectoryOffset");
    }
    setStride(filesPerSegment);
}

}
