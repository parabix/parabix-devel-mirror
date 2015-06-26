#include <UCD/unicode_set.h>
//#include <UCD/Blocks.h>
//#include <UCD/DerivedCoreProperties.h>
//#include <UCD/DerivedGeneralCategory.h>
//#include <UCD/EastAsianWidth.h>
//#include <UCD/HangulSyllableType.h>
//#include <UCD/LineBreak.h>
//#include <UCD/PropList.h>
//#include <UCD/ScriptExtensions.h>
#include <UCD/Scripts.h>
#include <iostream>
#include <llvm/Support/raw_os_ostream.h>

using namespace UCD;

// int main(int, char *) {
void run_unicode_tests() {

    llvm::raw_os_ostream out(std::cerr);

    SC_ns::aghb_Set.dump(out);

    int i = 0;

    for (auto interval : SC_ns::aghb_Set) {
        std::cerr << std::get<0>(interval) << ',' << std::get<1>(interval) << std::endl;
        if (++i == 5) {
            break;
        }
    }
}
