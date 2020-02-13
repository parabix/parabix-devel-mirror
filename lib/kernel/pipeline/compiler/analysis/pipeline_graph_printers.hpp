#ifndef PIPELINE_GRAPH_PRINTER_HPP
#define PIPELINE_GRAPH_PRINTER_HPP

#include "../pipeline_compiler.hpp"
#include <boost/algorithm/string/replace.hpp>

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printGraph
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Graph>
void printGraph(const Graph & G, raw_ostream & out, const StringRef name = "G") {

    out << "digraph \"" << name << "\" {\n";
    for (auto v : make_iterator_range(vertices(G))) {
        out << "v" << v << " [label=\"" << v << "\"];\n";
    }
    for (auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);
        out << "v" << s << " -> v" << t << ";\n";
    }

    out << "}\n\n";
    out.flush();
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printRelationshipGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void printRelationshipGraph(const RelationshipGraph & G, raw_ostream & out, const StringRef name = "G") {


    auto write = [](const Rational & v, llvm::raw_ostream & out)  {
        if (LLVM_LIKELY(v.denominator() == 1)) {
            out << v.numerator();
        } else {
            out << '(' << v.numerator() << '/' << v.denominator() << ')';
        }
    };

    out << "digraph " << name << " {\n";
    for (const auto v : make_iterator_range(vertices(G))) {
        out << "v" << v << " [label=\"" << v << ' ';
        const RelationshipNode & rn = G[v];
        switch (rn.Type) {
            case RelationshipNode::IsNil:
                out << "<nil>";
                break;
            case RelationshipNode::IsKernel:
                out << "Kernel:";
                if (rn.Kernel) {
                    out << rn.Kernel->getName();
                }
                break;
            case RelationshipNode::IsBinding: {
                    const Binding & binding = rn.Binding;
                    out << "Binding:";
                    using KindId = ProcessingRate::KindId;
                    const ProcessingRate & rate = binding.getRate();
                    switch (rate.getKind()) {
                        case KindId::Fixed:
                            out << 'F';
                            write(rate.getLowerBound(), out);
                            break;
                        case KindId::Greedy:
                            out << 'G';
                            write(rate.getLowerBound(), out);
                            break;
                        case KindId::Bounded:
                            out << 'B';
                            write(rate.getLowerBound(), out);
                            out << '-';
                            write(rate.getUpperBound(), out);
                            break;
                        case KindId::Unknown:
                            out << 'U';
                            write(rate.getLowerBound(), out);
                            break;
                        case KindId::PopCount:
                            out << "Pop";
                            break;
                        case KindId::NegatedPopCount:
                            out << "Neg";
                            break;
                        case KindId::Relative:
                            out << 'R';
                            break;
                        case KindId::PartialSum:
                            out << 'P';
                            break;
                        case KindId::__Count: llvm_unreachable("");
                    }
                }
                break;
            case RelationshipNode::IsCallee:
                assert (&rn.Callee);
                out << "Callee:" << rn.Callee.get().Name;
                break;
            case RelationshipNode::IsRelationship:
                assert (rn.Relationship);
                if (isa<StreamSet>(rn.Relationship)) {
                    out << "StreamSet: ";
                } else if (isa<ScalarConstant>(rn.Relationship)) {
                    out << "Constant: ";
                } else if (isa<Scalar>(rn.Relationship)) {
                    out << "Scalar: ";
                } else {
                    out << "<Unknown Relationship>: ";
                }
                rn.Relationship->getType()->print(errs());

//                out << " ";
//                out.write_hex(reinterpret_cast<uintptr_t>(rn.Relationship));
        }
        out << "\"];\n";
        out.flush();
    }

    for (const auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);
        out << "v" << s << " -> v" << t << " [";
        const RelationshipType & rt = G[e];
        if (rt.Reason != ReasonType::OrderingConstraint) {
            out  << "label=\"";
            switch (rt.Type) {
                case PortType::Input:
                    out << 'I';
                    break;
                case PortType::Output:
                    out << 'O';
                    break;
            }
            out << ':' << rt.Number;
            switch (rt.Reason) {
                case ReasonType::Explicit:
                    break;
                case ReasonType::ImplicitPopCount:
                    out << " (popcount)";
                    break;
                case ReasonType::ImplicitRegionSelector:
                    out << " (region)";
                    break;
                case ReasonType::Reference:
                    out << " (ref)";
                    break;
                default:
                    llvm_unreachable("invalid or unhandled reason type!");
                    break;
            }
            out << "\"";
        }


        switch (rt.Reason) {
            case ReasonType::None:
            case ReasonType::Explicit:
                break;
            case ReasonType::ImplicitPopCount:
            case ReasonType::ImplicitRegionSelector:
                out << " color=blue";
                break;
            case ReasonType::Reference:
                out << " color=gray";
                break;
            case ReasonType::OrderingConstraint:
                out << " color=red";
                break;
        }
        out << "];\n";
        out.flush();
    }
    out << "}\n\n";
    out.flush();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printBufferGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printBufferGraph(raw_ostream & out) const {

    using BufferId = StreamSetBuffer::BufferKind;

    auto print_rational = [&out](const Rational & r) -> raw_ostream & {
        if (r.denominator() > 1) {
            const auto n = r.numerator() / r.denominator();
            const auto p = r.numerator() % r.denominator();
            out << n << '+' << p << '/' << r.denominator();
        } else {
            out << r.numerator();
        }
        return out;
    };

    auto rate_range = [&out, print_rational](const Rational & a, const Rational & b) -> raw_ostream & {
        print_rational(a);
        out << ",";
        print_rational(b);
        return out;
    };

    auto printStreamSet = [&](const unsigned streamSet) {
        out << "v" << streamSet << " [shape=record,";
        const BufferNode & bn = mBufferGraph[streamSet];
        if (bn.NonLocal) {
            out << "style=bold,";
        }
        out << "label=\"" << streamSet << "|{";

        const StreamSetBuffer * const buffer = bn.Buffer;
        if (buffer == nullptr) {
            out << '?';
        } else {
            char bufferType = '?';
            switch (bn.Type) {
                case BufferType::Internal:
                    switch (buffer->getBufferKind()) {
                        case BufferId::StaticBuffer:
                            bufferType = 'S'; break;
                        case BufferId::DynamicBuffer:
                            bufferType = 'D'; break;
                        default: llvm_unreachable("unknown streamset type");
                    }
                    break;
                case BufferType::ManagedByKernel:
                    bufferType = 'M';
                    break;
                case BufferType::External:
                    bufferType = 'E';
                    break;
                case BufferType::UnownedExternal:
                    bufferType = 'U';
                    break;
                default: llvm_unreachable("unknown buffer type id");
            }
            out << bufferType;
            if (buffer->isLinear()) {
                out << 'L';
            }
            Type * ty = buffer->getBaseType();
            out << ':'
                << ty->getArrayNumElements() << 'x';
            ty = ty->getArrayElementType();
            ty = ty->getVectorElementType();
            out << ty->getIntegerBitWidth();
        }

        out << "|{";

        if (buffer && buffer->getBufferKind() != BufferId::ExternalBuffer) {
            switch (buffer->getBufferKind()) {
                case BufferId::StaticBuffer:
                    out << cast<StaticBuffer>(buffer)->getCapacity();
                    break;
                case BufferId::DynamicBuffer:
                    out << cast<DynamicBuffer>(buffer)->getInitialCapacity();
                    break;
                default: llvm_unreachable("unknown buffer type");
            }
        }

        if (bn.LookBehind) {
            out << "|LB:" << bn.LookBehind;
        }
        if (bn.CopyBack) {
            out << "|CB:" << bn.CopyBack;
        }
        if (bn.LookAhead) {
            out << "|LA:" << bn.LookAhead;
        }

        out << "}}\"];\n";

    };

    auto currentPartition = PartitionCount;
    bool closePartition = false;

    auto checkClosePartitionLabel = [&]() {
        if (closePartition) {
            out << "}\n";
            closePartition = false;
        }
    };

    auto checkOpenPartitionLabel = [&](const unsigned kernel) {
        const auto partitionId = KernelPartitionId[kernel];
        if (partitionId != currentPartition) {
            checkClosePartitionLabel();
            if (LLVM_LIKELY(partitionId != -1U)) {
                out << "subgraph cluster" << partitionId << " {\n"
                       "label=\"Partition #" << partitionId  << "\";"
                       "fontcolor=\"red\";"
                       "style=\"rounded,dashed,bold\";"
                       "color=\"red\";"
                       "\n";
                closePartition = true;
            }
        }
        currentPartition = partitionId;
    };

    auto printKernel = [&](const unsigned kernel, const StringRef name) {
        checkOpenPartitionLabel(kernel);
        out << "v" << kernel << " [label=\"[" <<
                kernel << "] " << name << "\\n"
                //" Partition: " << KernelPartitionId[v] << "\\n"
                " Expected:  ["; print_rational(MinimumNumOfStrides[kernel]) << ',';
                                print_rational(MaximumNumOfStrides[kernel]) << "]\\n"
                "\" shape=rect,style=rounded,peripheries=2"
                "];\n";

        for (const auto e : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const auto streamSet = target(e, mBufferGraph);
            printStreamSet(streamSet);
        }
    };

    out << "digraph \"" << mTarget->getName() << "\" {\n"
           "rankdir=tb;"
           "nodesep=0.5;"
           "ranksep=0.5;"
           "newrank=true;"
           // "compound=true;"
           "\n";

    printKernel(PipelineInput, "P_{in}");
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        auto name = kernel->getName().str();
        boost::replace_all(name, "\"", "\\\"");
        printKernel(i, name);
    }
    printKernel(PipelineOutput, "P_{out}");
    checkClosePartitionLabel();


    for (auto e : make_iterator_range(edges(mBufferGraph))) {
        const auto s = source(e, mBufferGraph);
        const auto t = target(e, mBufferGraph);

        const BufferRateData & pd = mBufferGraph[e];

        bool isLocal = true;
        bool isChecked = false;
        // is this edge from a buffer to a kernel?
        if (s >= FirstStreamSet) {
            const auto p = parent(s, mBufferGraph);
            const auto pId = KernelPartitionId[p];
            const auto tId = KernelPartitionId[t];
            // does this use of the buffer cross a partition boundary?
            if (pId != tId) {
                isLocal = false;
                // does the consuming partition need to check this buffer?
                for (const auto f : make_iterator_range(in_edges(tId, mPartitioningGraph))) {
                    const PartitioningGraphEdge & E = mPartitioningGraph[f];
                    if (E.Kernel == t && E.Port == pd.Port) {
                        isChecked = true;
                        break;
                    }
                }
            }
        }

        out << "v" << s << " -> v" << t <<
               " [";
        out << "label=\"#" << pd.Port.Number << ": ";
        const Binding & binding = pd.Binding;
        const ProcessingRate & rate = binding.getRate();
        switch (rate.getKind()) {
            case RateId::Fixed:
                out << "F(";
                print_rational(pd.Minimum);
                out << ")";
                break;
            case RateId::Bounded:
                out << "B(";
                rate_range(pd.Minimum, pd.Maximum);
                out << ")";
                break;
            case RateId::Greedy:
                out << "G(";
                print_rational(rate.getLowerBound());
                out << ",*)";
                break;
            case RateId::PartialSum:
                out << "P(";
                print_rational(rate.getUpperBound());
                out << ")";
                break;
            default: llvm_unreachable("unknown or unhandled rate type in buffer graph");
        }
        if (binding.hasAttribute(AttrId::Principal)) {
            out << " [P]";
        }
        if (binding.hasAttribute(AttrId::ZeroExtended)) {
            if (pd.ZeroExtended) {
                out << " [Z]";
            } else {
                out << " [z&#x336;]";
            }
        }
        std::string name = binding.getName();
        boost::replace_all(name, "\"", "\\\"");
        out << "\\n" << name << "\"";
        if (isLocal) {
            out << " style=dashed";
        } else if (isChecked) {
            out << " style=bold";
        }
        out << "];\n";
    }

    out << "}\n\n";
    out.flush();
}

}

#endif // PIPELINE_GRAPH_PRINTER_HPP
