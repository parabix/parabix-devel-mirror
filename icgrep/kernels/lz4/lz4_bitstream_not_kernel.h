
#ifndef ICGREP_LZ4_BITSTREAM_NOT_KERNEL_H
#define ICGREP_LZ4_BITSTREAM_NOT_KERNEL_H


#include <kernels/kernel.h>

#include <pablo/pablo_kernel.h>

namespace IDISA { class IDISA_Builder; }
namespace llvm { class Value; }

namespace kernel {

    // TODO: This kernel is just a workaround, it will be removed later.
    class LZ4BitStreamNotKernel : public pablo::PabloKernel  {
    public:
        LZ4BitStreamNotKernel(const std::unique_ptr<KernelBuilder> & b);
    protected:
        void generatePabloMethod() override;
    };

}


#endif //ICGREP_LZ4_BITSTREAM_NOT_KERNEL_H
