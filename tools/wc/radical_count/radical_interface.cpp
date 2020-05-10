#include "radical_interface.h"
#include <unicode/data/kRSKangXi.h>

using namespace std;
using namespace BS;
using namespace UCD::KRS_ns;

    BS::map<string, const UCD::UnicodeSet*> UnicodeSetTable::_unicodeset_radical_table
    {
        {"1",&_1_Set},                       {"2",&_2_Set},                       {"3",&_3_Set},                       {"4",&_4_Set},                       {"5",&_5_Set},                       {"6",&_6_Set},
        {"7",&_7_Set},                       {"8",&_8_Set},                       {"9",&_9_Set},                       {"10",&_10_Set},                     {"11",&_11_Set},                     {"12",&_12_Set},
        {"13",&_13_Set},                     {"14",&_14_Set},                     {"15",&_15_Set},                     {"16",&_16_Set},                     {"17",&_17_Set},                     {"18",&_18_Set},
        {"19",&_19_Set},                     {"20",&_20_Set},                     {"21",&_21_Set},                     {"22",&_22_Set},                     {"23",&_23_Set},                     {"24",&_24_Set},
        {"25",&_25_Set},                     {"26",&_26_Set},                     {"27",&_27_Set},                     {"28",&_28_Set},                     {"29",&_29_Set},                     {"30",&_30_Set},
        {"31",&_31_Set},                     {"32",&_32_Set},                     {"33",&_33_Set},                     {"34",&_34_Set},                     {"35",&_35_Set},                     {"36",&_36_Set},
        {"37",&_37_Set},                     {"38",&_38_Set},                     {"39",&_39_Set},                     {"40",&_40_Set},                     {"41",&_41_Set},                     {"42",&_42_Set},
        {"43",&_43_Set},                     {"44",&_44_Set},                     {"45",&_45_Set},                     {"46",&_46_Set},                     {"47",&_47_Set},                     {"48",&_48_Set},
        {"49",&_49_Set},                     {"50",&_50_Set},                     {"51",&_51_Set},                     {"52",&_52_Set},                     {"53",&_53_Set},                     {"54",&_54_Set},
        {"55",&_55_Set},                     {"56",&_56_Set},                     {"57",&_57_Set},                     {"58",&_58_Set},                     {"59",&_59_Set},                     {"60",&_60_Set},
        {"61",&_61_Set},                     {"62",&_62_Set},                     {"63",&_63_Set},                     {"64",&_64_Set},                     {"65",&_65_Set},                     {"66",&_66_Set},
        {"67",&_67_Set},                     {"68",&_68_Set},                     {"69",&_69_Set},                     {"70",&_70_Set},                     {"71",&_71_Set},                     {"72",&_72_Set},
        {"73",&_73_Set},                     {"74",&_74_Set},                     {"75",&_75_Set},                     {"76",&_76_Set},                     {"77",&_77_Set},                     {"78",&_78_Set},
        {"79",&_79_Set},                     {"80",&_80_Set},                     {"81",&_81_Set},                     {"82",&_82_Set},                     {"83",&_83_Set},                     {"84",&_84_Set},
        {"85",&_85_Set},                     {"86",&_86_Set},                     {"87",&_87_Set},                     {"88",&_88_Set},                     {"89",&_89_Set},                     {"90",&_90_Set},
        {"91",&_91_Set},                     {"92",&_92_Set},                     {"93",&_93_Set},                     {"94",&_94_Set},                     {"95",&_95_Set},                     {"96",&_96_Set},
        {"97",&_97_Set},                     {"98",&_98_Set},                     {"99",&_99_Set},                     {"100",&_100_Set},                   {"101",&_101_Set},                   {"102",&_102_Set},
        {"103",&_103_Set},                   {"104",&_104_Set},                   {"105",&_105_Set},                   {"106",&_106_Set},                   {"107",&_107_Set},                   {"108",&_108_Set},
        {"109",&_109_Set},                   {"110",&_110_Set},                   {"111",&_111_Set},                   {"112",&_112_Set},                   {"113",&_113_Set},                   {"114",&_114_Set},
        {"115",&_115_Set},                   {"116",&_116_Set},                   {"117",&_117_Set},                   {"118",&_118_Set},                   {"119",&_119_Set},                   {"120",&_120_Set},
        {"121",&_121_Set},                   {"122",&_122_Set},                   {"123",&_123_Set},                   {"124",&_124_Set},                   {"125",&_125_Set},                   {"126",&_126_Set},
        {"127",&_127_Set},                   {"128",&_128_Set},                   {"129",&_129_Set},                   {"130",&_130_Set},                   {"131",&_131_Set},                   {"132",&_132_Set},
        {"133",&_133_Set},                   {"134",&_134_Set},                   {"135",&_135_Set},                   {"136",&_136_Set},                   {"137",&_137_Set},                   {"138",&_138_Set},
        {"139",&_139_Set},                   {"140",&_140_Set},                   {"141",&_141_Set},                   {"142",&_142_Set},                   {"143",&_143_Set},                   {"144",&_144_Set},
        {"145",&_145_Set},                   {"146",&_146_Set},                   {"147",&_147_Set},                   {"148",&_148_Set},                   {"149",&_149_Set},                   {"150",&_150_Set},
        {"151",&_151_Set},                   {"152",&_152_Set},                   {"153",&_153_Set},                   {"154",&_154_Set},                   {"155",&_155_Set},                   {"156",&_156_Set},
        {"157",&_157_Set},                   {"158",&_158_Set},                   {"159",&_159_Set},                   {"160",&_160_Set},                   {"161",&_161_Set},                   {"162",&_162_Set},
        {"163",&_163_Set},                   {"164",&_164_Set},                   {"165",&_165_Set},                   {"166",&_166_Set},                   {"167",&_167_Set},                   {"168",&_168_Set},
        {"169",&_169_Set},                   {"170",&_170_Set},                   {"171",&_171_Set},                   {"172",&_172_Set},                   {"173",&_173_Set},                   {"174",&_174_Set},
        {"175",&_175_Set},                   {"176",&_176_Set},                   {"177",&_177_Set},                   {"178",&_178_Set},                   {"179",&_179_Set},                   {"180",&_180_Set},
        {"181",&_181_Set},                   {"182",&_182_Set},                   {"183",&_183_Set},                   {"184",&_184_Set},                   {"185",&_185_Set},                   {"186",&_186_Set},
        {"187",&_187_Set},                   {"188",&_188_Set},                   {"189",&_189_Set},                   {"190",&_190_Set},                   {"191",&_191_Set},                   {"192",&_192_Set},
        {"193",&_193_Set},                   {"194",&_194_Set},                   {"195",&_195_Set},                   {"196",&_196_Set},                   {"197",&_197_Set},                   {"198",&_198_Set},
        {"199",&_199_Set},                   {"200",&_200_Set},                   {"201",&_201_Set},                   {"202",&_202_Set},                   {"203",&_203_Set},                   {"204",&_204_Set},
        {"205",&_205_Set},                   {"206",&_206_Set},                   {"207",&_207_Set},                   {"208",&_208_Set},                   {"209",&_209_Set},                   {"210",&_210_Set},
        {"211",&_211_Set},                   {"212",&_212_Set},                   {"213",&_213_Set},                   {"214",&_214_Set}
    };


