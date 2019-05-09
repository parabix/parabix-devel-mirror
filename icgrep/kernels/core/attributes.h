#ifndef ATTRIBUTES_H
#define ATTRIBUTES_H

#include <vector>
#include <llvm/ADT/StringRef.h>
#include <llvm/Support/Compiler.h>
#include <assert.h>

namespace llvm { class raw_ostream; }

namespace kernel {

struct Attribute {

    friend struct AttributeSet;

    enum class KindId {

        None,

        /** INPUT STREAM ATTRIBUTES **/

        LookAhead,

        // A LookAhead(n) attribute on an input stream set S declares that the kernel
        // looks ahead n positions in the input stream.  That is, processing of item
        // S[i, j] may be defined in terms of S[i, j+n].

        // Guarantee required: the pipeline compiler must ensure that, when
        // the kernel is called with an available item count of N for the
        // input stream, that the corresponding input data buffer has
        // valid items in the buffer up to and including position N+n,
        // and that the blocks containing the items from N-1 through N+n are
        // linearly contiguous.  When the final doMultiBlock call is made,
        // the pipeline compiler must ensure that the n items at lookahead
        // positions are zero-initialized.

        // (Note: this guarantee anticipates using a single buffer pointer
        // for both normal stream access and lookahead access.   This avoids
        // the cost of extra parameters per doMultiBlock call, but requires
        // that the corresponding circular buffer have a "lookahead extension area"
        // that holds a copy of the data at the physical start of buffer).

        LookBehind, /// NOT DONE

        // LookBehind(n) ensures that up to n positions prior to the current processed
        // item of an input must be stored in linear (contiguous) memory.

        // NOTE: If n is 0, all data will be linear (at the cost of additional
        // memcopies in order to preserve this property)

        Principal,

        // One input stream can be declared as the principal input buffer for a kernel.
        // If a kernel has a principal input stream, when processing the final stride,
        // a MultiBlockKernel assumes the item count of the principle is the correct
        // one and zero extends / truncates all other input streams to match it.

        Deferred,

        // Normally, the processed item count of fixed rate streams is automatically
        // updated by the MultiBlock kernel. However, some streams behave like Fixed
        // rate streams (in that they will always eventually process a Fixed amount of
        // data) but the kernel processes the data in unpredictable chunks. Rather than
        // declaring those as Unknown or Bounded rates, marking their rate calculation
        // as Deferred provides the pipeline with a stronger guarantee when it comes to
        // buffer size calculations.

        ZeroExtended,

        // If the available item count of an input stream is less than some other input
        // stream(s), the stream will be zero-extended to the length of the larger stream.
        // If this option is not set and the kernel does not have a MustExplicitlyTerminate
        // attribute, it will end once any input has been exhausted.

        // NOTE: zero-extended streams are not considered by the pipeline when ascertaining
        // whether it is entering the final segment. At least one input stream must not be
        // zero-extended and a stream may not have both Principal and ZeroExtend attributes.

        IndependentRegionBegin, IndependentRegionEnd, /// NOT DONE

        // Some kernels can divide their processing into concrete non-overlapping regions
        // between a beginning and ending position. This is a hard guarantee that regardless
        // of the computations between the start of the stream and the beginning of the first
        // independent region or between the *beginning* of any two independent regions, A,
        // B, the calculations that occur prior to the beginning of B do not affect the
        // calculations after it --- even if A is started at an arbitrary position with a
        // zeroed-out kernel state.

        // If a kernel K is processed simultaneously by two threads, K_0 and K_1, and K_1 is
        // waiting K_0 to finish and update it's kernel state for K_1 to resume at, K_1 can
        // compute what its state will be and begin processing before K_0 is finished. This
        // requires a the pipeline to intervene and call an optimized "output-less" instance
        // of the kernel prior to calling B.

        RegionSelector, /// NOT DONE

        // If we are only interested in processing data within a selected region, this
        // attribute instructs the pipeline "skip" any data that is not within a selected
        // region. Unless we can bound the size of a region, all I/O buffers associated with
        // this kernel will backed by dynamic buffers.

        // NOTE: For any non-selected region, the processed/produced item counts will be
        // aligned as expected for *countable* rate streams (with zeros written to the
        // skipped regions.) Non-countable rate streams will process/produce 0 items.

        SupressNonRegionZeroFill, /// NOT DONE

        // The I/O stream will *not* be zero-filled to maintain alignment with the input
        // when "skipping" a non-selected region.

        RequiresPopCountArray, RequiresNegatedPopCountArray,

        // Marks whether a particular input stream requires a popcount or negated popcount
        // array for its own internal processing.

        /** OUTPUT STREAM ATTRIBUTES **/

        Add,

        // An Add(K) attribute states that K items will be added to this stream after
        // processing the final block.

        RoundUpTo,

        // A RoundUpTo(k) attribute indicates the final item count of this stream will
        // be rounded up to the nearest multiple of k

        ManagedBuffer,

        // Generally, kernels do not require knowledge about how many items are consumed
        // from their produced streams or who is consuming them and instead rely on the
        // pipeline to manage their output buffers for them. The major exception are source
        // kernels since they produce data "autonomously" and may manage their memory
        // internally. Thus this attribute instructs both the kernel compiler and pipeline
        // that a particular output stream needs both the consumed item count and a pointer
        // to each of its consumers logical segment number for its internal logic.

        Expandable, /// NOT DONE

        // Indicates that the number of stream sets in this buffer can increase.

        /** INPUT/OUTPUT STREAM ATTRIBUTES **/

        Misaligned,

        // Assume that we cannot statically compute the alignment of this stream set and
        // perform any operations accordingly

        BlockSize,

        // Typically a kernel assumes that each stream of a stream set is a linear sequence
        // of items. The BlockSize(K) attribute informs the kernel is actually divided into
        // (BlockWidth / K) elements and each "stream" actually contains K items of the
        // first stream followed by K elements of the second stream and so on. The notion
        // of produced/processed item count changes to suite. I.e., when typical kernels
        // report that they've processed/produced up to the i-th position, it means:

        //                                         v
        //                 ...|AAAAAAAA AAAAAAAA AA*              |...
        //                 ...|BBBBBBBB BBBBBBBB BB*              |...
        //                 ...|CCCCCCCC CCCCCCCC CC*              |...
        //                 ...|DDDDDDDD DDDDDDDD DD*              |...

        // However, if (BlockWidth / K) is 4, the same i-th position above is actually:

        //                 ...|AAAAAAAA|BBBBBBBB|CCCCCCCC|DDDDDDDD|...
        //                 ...|AAAAAAAA|BBBBBBBB|CCCCCCCC|DDDDDDDD|...
        //                 ...|AA*     |BB*     |CC*     |DD*     |...
        //                 ...|        |        |        |        |...

        // (Note: this replaces the concept of swizzling and anticipates that the pipeline
        // will take on the role of automatically inserting the swizzling code necessary).

        ReverseAdapter, /// NOT DONE

        // Conceptually, reversing a stream S is simple: {S_1,...,S_n} -> {S_n,...,S_1}.
        // However, this means all of the input data must be computed and stored prior to
        // executing the first iteration of this kernel. In practice, this is unnecessary
        // as in the context of text parsing, we're almost always searching for the true
        // starting position of something ambigious after we've found its end position by
        // some prior kernel.


        SliceOffset, /// NOT DONE

        // Given a SliceOffset of k, the k-th stream set be the base (zeroth) stream set
        // for the kernel. Internally, this stores a scalar in the kernel state and loads
        // it once at the start of each segment.

        /** KERNEL ATTRIBUTES **/

        CanTerminateEarly,

        // Indicates that this kernel can call setTerminationSignal() to terminate the
        // kernel prior to processing all of its input streams.

        MustExplicitlyTerminate,

        // This kernel terminates *only* after the programmer calls setTerminationSignal()
        // and will be called even when there is no new input data when the prior kernels
        // in the pipeline have also terminated.

        SideEffecting,

        // Mark this kernel as side-effecting, which will prevent the pipeline compiler
        // from wrongly removing it from the pipeline. All sink kernels that produce a
        // result through StdOut/StdErr should be marked as SideEffecting.

        Family,

        // Marks that this kernel is belongs to a named family and whether an input scalar
        // should be surpressed from the generated "main" function because it will be bound
        // within it to the appropriate handle/function pointers.

        InternallySynchronized,

        // Indicates this kernel does not require multithreading synchronization to ensure
        // correct behaviour. This *only* affects the locks surrounding the kernel in the
        // outer pipeline.

        // NOTE: this means either the kernel must either internally handle synchronization
        // or that any interleaving of segments is valid.

        InfrequentlyUsed,

        // This kernel is infrequently used and should be compiled with O1 instead of O3.

        /** COUNT **/

        __Count
    };

    KindId getKind() const {
        return mKind;
    }

    bool isAdd() const {
        return mKind == KindId::Add;
    }

    bool isPrincipal() const {
        return mKind == KindId::Principal;
    }

    bool isRoundUpTo() const {
        return mKind == KindId::RoundUpTo;
    }

    bool isBlockSize() const {
        return mKind == KindId::BlockSize;
    }

    unsigned amount() const {
        return mAmount;
    }

    bool operator == (const Attribute & other) const {
        return mKind == other.mKind && mAmount == other.mAmount;
    }

    bool operator != (const Attribute & other) const {
        return !(*this == other);
    }

    void print(llvm::raw_ostream & out) const noexcept;

    Attribute(const KindId kind, const unsigned k) : mKind(kind), mAmount(k) { }

private:

    const KindId    mKind;
    unsigned        mAmount;
};

#if 0

struct IntegerAttribute : public Attribute {
    IntegerAttribute(const KindId kind, const unsigned k) : Attribute(kind, 0), mValue(k) { }
private:
    const unsigned mValue;
};

struct StringAttribute : public Attribute {
    StringAttribute(const KindId kind, const llvm::StringRef k) : Attribute(kind, 0), mValue(k) { }
private:
    const std::string mValue;
};

#endif

struct AttributeSet : public std::vector<Attribute> {

    using AttributeId = Attribute::KindId;

    const AttributeSet & getAttributes() const {
        return *this;
    }

    Attribute & findOrAddAttribute(const AttributeId id) {
        if (Attribute * const attr = __findAttribute(id)) {
            return *attr;
        } else {
            return addAttribute(Attribute(id, 0));
        }
    }

    Attribute & findAttribute(const AttributeId id) const LLVM_READNONE {
        return *__findAttribute(id);
    }

    Attribute & addAttribute(Attribute attribute);

    bool hasAttributes() const LLVM_READNONE {
        return !empty();
    }

    bool hasAttribute(const AttributeId id) const LLVM_READNONE {
        return __findAttribute(id) != nullptr;
    }

    AttributeSet() = default;

    AttributeSet(const AttributeSet &) = default;

    AttributeSet(Attribute && attr) { emplace_back(std::move(attr)); }

    AttributeSet(std::initializer_list<Attribute> attrs) : std::vector<Attribute>(attrs) { }

protected:

    void print(llvm::raw_ostream & out) const noexcept;

private:

    Attribute * __findAttribute(const AttributeId id) const;

};

inline Attribute Add1() {
    return Attribute(Attribute::KindId::Add, 1);
}

inline Attribute RoundUpTo(const unsigned k) {
    return Attribute(Attribute::KindId::RoundUpTo, k);
}

inline Attribute ManagedBuffer() {
    return Attribute(Attribute::KindId::ManagedBuffer, 0);
}

inline Attribute Principal() {
    return Attribute(Attribute::KindId::Principal, 0);
}

inline Attribute ZeroExtended() {
    return Attribute(Attribute::KindId::ZeroExtended, 0);
}

inline Attribute LookAhead(const unsigned k) {
    return Attribute(Attribute::KindId::LookAhead, k);
}

inline Attribute LookBehind(const unsigned k) {
    return Attribute(Attribute::KindId::LookBehind, k);
}

inline Attribute Deferred() {
    return Attribute(Attribute::KindId::Deferred, 0);
}

inline Attribute BlockSize(const unsigned k) {
    assert (k && ((k & (k - 1)) == 0));
    return Attribute(Attribute::KindId::BlockSize, k);
}

inline Attribute Swizzled() {
    return BlockSize(64);
}

inline Attribute Misaligned() {
    return Attribute(Attribute::KindId::Misaligned, 0);
}

inline Attribute IndependentRegionBegin(const unsigned streamIndex) {
    return Attribute(Attribute::KindId::IndependentRegionBegin, streamIndex);
}

inline Attribute IndependentRegionEnd(const unsigned streamIndex = 0) {
    return Attribute(Attribute::KindId::IndependentRegionEnd, streamIndex);
}

inline Attribute RegionSelector(const unsigned streamIndex = 0) {
    return Attribute(Attribute::KindId::RegionSelector, streamIndex);
}

inline Attribute SupressNonRegionZeroFill() {
    return Attribute(Attribute::KindId::SupressNonRegionZeroFill, 0);
}

inline Attribute CanTerminateEarly() {
    return Attribute(Attribute::KindId::CanTerminateEarly, 0);
}

inline Attribute MustExplicitlyTerminate() {
    return Attribute(Attribute::KindId::MustExplicitlyTerminate, 0);
}

inline Attribute SideEffecting() {
    return Attribute(Attribute::KindId::SideEffecting, 0);
}

inline Attribute Family() {
    return Attribute(Attribute::KindId::Family, 0);
}

inline Attribute InternallySynchronized() {
    return Attribute(Attribute::KindId::InternallySynchronized, 0);
}

inline Attribute InfrequentlyUsed() {
    return Attribute(Attribute::KindId::InfrequentlyUsed, 0);
}

inline Attribute RequiresPopCountArray() {
    return Attribute(Attribute::KindId::RequiresPopCountArray, 0);
}

inline Attribute RequiresNegatedPopCountArray() {
    return Attribute(Attribute::KindId::RequiresNegatedPopCountArray, 0);
}




}
#endif // ATTRIBUTES_H
