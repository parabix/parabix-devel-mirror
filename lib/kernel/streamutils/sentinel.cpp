#include <kernel/streamutils/sentinel.h>
#include <kernel/core/kernel_builder.h>
#include <pablo/builder.hpp>
#include <pablo/pe_ones.h>          // for Ones
#include <pablo/pe_var.h>           // for Var
#include <pablo/pe_infile.h>

using namespace pablo;

namespace kernel {

AddSentinel::AddSentinel(const std::unique_ptr<kernel::KernelBuilder> & kb, StreamSet * input, StreamSet * output)
: PabloKernel(kb, "AddSentinel" + std::to_string(input->getNumElements()) + "x" + std::to_string(input->getFieldWidth()),
{Binding{"input", input}},
{Binding{"output", output, FixedRate(), Add1()}}) {
    assert (input->getNumElements() == output->getNumElements());
}

void AddSentinel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * EOFbit = pb.createAtEOF(pb.createAdvance(pb.createOnes(), 1));
    std::vector<PabloAST *> inputs = getInputStreamSet("input");
    Var * const outputs = getOutputStreamVar("output");
    for (unsigned i = 0; i < inputs.size(); i++) {
        PabloAST * extended = pb.createOr(inputs[i], EOFbit, "addSentinel");
        pb.createAssign(pb.createExtract(outputs, pb.getInteger(i)), extended);
    }
}
}
