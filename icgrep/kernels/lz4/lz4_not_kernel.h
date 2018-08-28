
#ifndef ICGREP_LZ4_NOT_KERNEL_H
#define ICGREP_LZ4_NOT_KERNEL_H


#include <pablo/pablo_kernel.h>  // for PabloKernel
#include <cc/alphabet.h>

namespace IDISA { class IDISA_Builder; }
namespace re { class RE; }
namespace cc { class Alphabet; }
namespace kernel {

    class LZ4NotKernel: public pablo::PabloKernel {
    public:
        LZ4NotKernel(const std::unique_ptr<kernel::KernelBuilder> & kb);
    protected:
        void generatePabloMethod() override;
    };

}



#endif //ICGREP_LZ4_NOT_KERNEL_H
