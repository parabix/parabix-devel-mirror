#ifndef ATTRIBUTES_H
#define ATTRIBUTES_H

namespace kernel {

struct Attribute {

    friend struct Binding;

    enum class KindId {

        /** INPUT STREAM ATTRIBUTES **/

        BlockSize,

        // A BlockSize(K) attribute, where K=2^k for some value of k>=4 declares
        // that the layout of stream data items within the corresponding input
        // or output buffer is arranged in blocks of K items each.   In each
        // block, the data buffer contains K items of the first stream in the
        // set, followed by K items of the next stream in the set and so on,
        // up to and including K items of the last stream in the set.

        // (Note: this replaces the concept of swizzling and anticipates that
        // the pipeline will take on the role of automatically inserting the
        // swizzling code necessary).

        LookAhead,

        // A LookAhead(n) attribute on an input stream set S declares that the kernel
        // looks ahead n positions in the input stream.   That is,
        // processing of item S[i, j] may be defined in terms of S[i, j+n].

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

        LookBehind,

        // A LookBehind(n) attribute on an input stream S declares that the kernel
        // requires access to input items up to n positions prior to the current
        // processed item position.

        // (Notes: this may allow more efficient advances within n (without saving state).
        // However, a lookbehind extension area prior to the normal buffer base address
        // is necessary, which must be initially zero-filled.)

        // A LookBehind(n) attribute on an output stream S declares that the kernel
        // requires access to up to n previously generated output items.
        // (Example: lz4d lookbehind(65536)).

        Principle,

        // One input stream can be declared as the principle input buffer for a kernel.
        // If a kernel has a principle input stream, when processing the final stride,
        // a MultiBlockKernel assumes the item count of the principle is the correct
        // one and zero extends / truncates all other input streams to match it.

        /** OUTPUT STREAM ATTRIBUTES **/

        Add,

        // An Add(K) attribute states that K bits will be added to this stream after
        // processing the final block.

        RoundUpTo,

        // A RoundUpTo(k) attribute indicates the final item count of this stream will
        // be rounded up to the nearest multiple of k

        /** KERNEL ATTRIBUTES **/

        SelectMinimumInputLength,

        // If a kernel has multiple input streams and their final item count differs,
        // a MultiBlock kernel will select the *minimum* input item count as it's
        // principle item length and truncate the streams to fit

        // NOTE: this is the default if a kernel does not have SelectMaximumInputLength
        // set and no PrincipleInputStream was declared.

        SelectMaximumInputLength,

        // If a kernel has multiple input streams and their final item count differs,
        // a MultiBlock kernel will select the *maximum* input item count as it's
        // principle item length and zero-extend the streams accordingly.

    };

    bool isAdd() const {
        return mKind == KindId::Add;
    }

    bool isPrinciple() const {
        return mKind == KindId::Principle;
    }

    bool isRoundUpTo() const {
        return mKind == KindId::RoundUpTo;
    }

    unsigned getAmount() const {
        return mK;
    }

    bool operator == (const Attribute & other) const {
        return mKind == other.mKind && mK == other.mK;
    }

    bool operator != (const Attribute & other) const {
        return !(*this == other);
    }

protected:

    KindId getKind() const {
        return mKind;
    }

    friend Attribute Add1();
    friend Attribute Principle();
    friend Attribute RoundUpTo(const unsigned);
    friend Attribute LookBehind(const unsigned);

    Attribute(const KindId kind, const unsigned k) : mKind(kind), mK(k) { }

private:

    const KindId    mKind;
    const unsigned  mK;

};

inline Attribute Add1() {
    return Attribute(Attribute::KindId::Add, 1);
}

inline Attribute RoundUpTo(const unsigned k) {
    return Attribute(Attribute::KindId::RoundUpTo, k);
}

inline Attribute Principle() {
    return Attribute(Attribute::KindId::Principle, 0);
}

inline Attribute LookBehind(const unsigned k) {
    return Attribute(Attribute::KindId::LookBehind, k);
}


}


#endif // ATTRIBUTES_H
