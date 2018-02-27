#ifndef ATTRIBUTES_H
#define ATTRIBUTES_H

#include <vector>

namespace kernel {

struct Attribute {

    enum class KindId {

        /** INPUT STREAM ATTRIBUTES **/

        LookAhead, /// NOT DONE

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

        // A LookBehind(n) attribute on an input stream S declares that the kernel
        // requires access to input items up to n positions prior to the current
        // processed item position.

        // (Notes: this may allow more efficient advances within n (without saving state).
        // However, a lookbehind extension area prior to the normal buffer base address
        // is necessary, which must be initially zero-filled.)

        // A LookBehind(n) attribute on an output stream S declares that the kernel
        // requires access to up to n previously generated output items.
        // (Example: lz4d lookbehind(65536)).

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

        ConditionalRegionBegin, ConditionalRegionEnd, /// NOT DONE

        // Some kernels have clearly demarcated regions in which a MultiBlock kernel will
        // produce useful outputs for only the inputs within those regions. This attribute
        // instructs the kernel to "zero-fill" the output of any non-selected regions,
        // skipping strides entirely whenever possible.

        // If the same regions are also independent, we can avoid the overhead of "masking
        // out" the input streams. Otherwise a MultiBlock will use temporary buffers for all
        // uses of the streams and zero out any non-regions from the data.

        AlwaysConsume,

        // Always consume the input (i.e., use the lowerbound to determine whether to there
        // is enough data to execute a stride rather than the upper bound.)

        /** OUTPUT STREAM ATTRIBUTES **/

        Add,

        // An Add(K) attribute states that K items will be added to this stream after
        // processing the final block.

        RoundUpTo,

        // A RoundUpTo(k) attribute indicates the final item count of this stream will
        // be rounded up to the nearest multiple of k

        /** INPUT/OUTPUT STREAM ATTRIBUTES **/

        Misaligned,

        // Assume that we cannot statically compute the alignment of this stream set and
        // perform any operations accordingly

        BlockSize, /// NOT DONE

        // A BlockSize(K) attribute, where K=2^k for some value of k>=4 declares
        // that the layout of stream data items within the corresponding input
        // or output buffer is arranged in blocks of K items each.   In each
        // block, the data buffer contains K items of the first stream in the
        // set, followed by K items of the next stream in the set and so on,
        // up to and including K items of the last stream in the set.

        // (Note: this replaces the concept of swizzling and anticipates that
        // the pipeline will take on the role of automatically inserting the
        // swizzling code necessary).

        ReverseRegionBegin, ReverseRegionEnd, /// NOT DONE

        // Conceptually, reversing a stream S is simple: {S_1,...,S_n} -> {S_n,...,S_1}.
        // However, this means all of the input data must be computed and stored prior to
        // executing this kernel. In practice, this is unnecessary as in the context of
        // text parsing, we're almost always searching for the true starting position of
        // something ambigious after we've found its end position in some prior kernel.


        Swizzled,

        // Whether the input streamset is in swizzled form


//        Here is a revised definition of SegmentedReverse:

//        Given a stream of data bits S that is considered to be divided into
//        segments, and a marker stream S having a one bit at the final position
//        of each segment, the R = SegmentedReverse(S, M) when

//        R_{i} = S_{l + (h - i)}
//              where l = the maximum j such that j <= i and either j = 0 or M_{j-1} = 1
//          and where h = the minimum j such that j >= i and either j = length(S) -  or M_j = 1
//          (l and h are the low and high positions of the segment containing i)

//        This is an invertible operation, so we can apply R to a kernel's input
//        and then to its output to get a SegmentedReverse version of a kernel

//        A kernel which computes segmented reverse is feasible, but seems complex
//        to implement, and probably too slow.  I have played around with several
//        ways of tackling it, no good method yet.

//        If there are multiple segments within a block, we could instead use
//        the following:

//        BlockSegmentedReverse

//        B_{i} = S_{L + (H - i)}
//             where l = the maximum j such that j <= i and either j = 0 or M_{j-1} = 1
//                   h = the minimum j such that j >= i and either j = length(S) -  or M_j = 1
//                   L = l if l div BlockSize < h divBlockSize, otherwise (i div BlockSize) * BlockSize
//                   H = h if l div BlockSize < h divBlockSize, otherwise L + BlockSize - 1

//        An alternative way of looking at this is to eliminate all but the first
//        and last marker positions within a block.

//        The main point is that, if we apply B to inputs, perform the kernel
//        and the apply B to outputs, we get the same result if we applied R
//        (assuming that the kernel computations do not cross boundaries in M).

//        This will be more efficient to compute, but still involves overhead
//        for shifting and combining streams.

//        I think it may be better to focus on the ReverseKernel adapter, that
//        handles the reverse operations for both input and output.   This actually
//        gives more flexibility, because, in a multiblock scenario, we can process
//        the longest sequence of blocks such that both the beginning and end blocks
//        have a one bit.   If there are any interior blocks with one bits, then
//        they will be handled automatically without special shifting and masking.

//        By the way, in my designs, I am wanting to have a callable Multiblock
//        function, so that the Multiblock function for a Reversed Kernel just
//        does a little work before calling the Multiblock function of the base kernel.
//        That seems to have disappeared in the current system.


        /** KERNEL ATTRIBUTES **/

        SelectMinimumInputLength, /// NOT DONE

        // If a kernel has multiple input streams and their final item count differs,
        // a MultiBlock kernel will select the *minimum* input item count as it's
        // principle item length and truncate the streams to fit

        // NOTE: this is the default if a kernel does not have SelectMaximumInputLength
        // set and no PrincipalInputStream was declared.

        SelectMaximumInputLength, /// NOT DONE

        // If a kernel has multiple input streams and their final item count differs,
        // a MultiBlock kernel will select the *maximum* input item count as it's
        // principle item length and zero-extend the streams accordingly.

        CanTerminateEarly,

        // Indicates that this kernel can call setTerminationSignal() to terminate the
        // kernel prior to processing all of its input streams.

        MustExplicitlyTerminate,

        // This kernel terminates *only* after the programmer calls setTerminationSignal()
        // and will be called even when there is no new input data when the prior kernels
        // in the pipeline have also terminated.

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

    void setAmount(const unsigned amount) {
        mAmount = amount;
    }

    bool operator == (const Attribute & other) const {
        return mKind == other.mKind && mAmount == other.mAmount;
    }

    bool operator != (const Attribute & other) const {
        return !(*this == other);
    }

protected:

    friend struct AttributeSet;
    friend struct Binding;
    friend Attribute Add1();
    friend Attribute Principal();
    friend Attribute AlwaysConsume();
    friend Attribute RoundUpTo(const unsigned);
    friend Attribute LookAhead(const unsigned);
    friend Attribute LookBehind(const unsigned);
    friend Attribute Deferred();
    friend Attribute Misaligned();
    friend Attribute ConditionalRegionBegin();
    friend Attribute ConditionalRegionEnd();
    friend Attribute Swizzled();
    friend Attribute CanTerminateEarly();
    friend Attribute MustExplicitlyTerminate();

    Attribute(const KindId kind, const unsigned k) : mKind(kind), mAmount(k) { }

private:

    const KindId    mKind;
    unsigned        mAmount;
};

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

    Attribute & findAttribute(const AttributeId id) const {
        return *__findAttribute(id);
    }

    Attribute & addAttribute(Attribute attribute);

    bool hasAttributes() const {
        return !empty();
    }

    bool hasAttribute(const AttributeId id) const {
        return __findAttribute(id) != nullptr;
    }

    AttributeSet() = default;

    AttributeSet(Attribute && attr) { emplace_back(std::move(attr)); }

    AttributeSet(std::initializer_list<Attribute> attrs) : std::vector<Attribute>(attrs) { }

private:

    Attribute * __findAttribute(const AttributeId id) const;

};


inline Attribute Add1() {
    return Attribute(Attribute::KindId::Add, 1);
}

inline Attribute RoundUpTo(const unsigned k) {
    return Attribute(Attribute::KindId::RoundUpTo, k);
}

inline Attribute AlwaysConsume() {
    return Attribute(Attribute::KindId::AlwaysConsume, 0);
}

inline Attribute Principal() {
    return Attribute(Attribute::KindId::Principal, 0);
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

inline Attribute Misaligned() {
    return Attribute(Attribute::KindId::Misaligned, 0);
}

inline Attribute ConditionalRegionBegin() {
    return Attribute(Attribute::KindId::ConditionalRegionBegin, 0);
}

inline Attribute ConditionalRegionEnd() {
    return Attribute(Attribute::KindId::ConditionalRegionEnd, 0);
}

inline Attribute CanTerminateEarly() {
    return Attribute(Attribute::KindId::CanTerminateEarly, 0);
}

inline Attribute MustExplicitlyTerminate() {
    return Attribute(Attribute::KindId::MustExplicitlyTerminate, 0);
}

inline Attribute Swizzled() {
    return Attribute(Attribute::KindId::Swizzled, 0);
}

}

#endif // ATTRIBUTES_H
