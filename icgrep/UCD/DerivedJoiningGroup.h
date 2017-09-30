#ifndef DERIVEDJOININGGROUP_H
#define DERIVEDJOININGGROUP_H
/*
 *  Copyright (c) 2017 International Characters, Inc.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters, Inc.
 *
 *  This file is generated by UCD_properties.py - manual edits may be lost.
 */

#include "PropertyObjects.h"
#include "PropertyValueAliases.h"
#include "unicode_set.h"

namespace UCD {
  namespace JG_ns {
    const unsigned independent_prop_values = 89;
    /** Code Point Ranges for No_Joining_Group
    [0000, 061f], [0621, 0621], [0640, 0640], [064b, 066d], [0670, 0670],
    [0674, 0674], [06d4, 06d4], [06d6, 06ed], [06f0, 06f9], [06fd, 06fe],
    [0700, 070f], [0711, 0711], [0730, 074c], [0780, 089f], [08ad, 08ad],
    [08b5, 08b5], [08be, 10abf], [10ac6, 10ac6], [10ac8, 10ac8],
    [10acb, 10acc], [10ae2, 10ae3], [10ae5, 10aea], [10af0, 10ffff]**/
    const UnicodeSet no_joining_group_Set 
        {{{Full, 49}, {Mixed, 3}, {Empty, 2}, {Mixed, 5}, {Empty, 1},
          {Full, 9}, {Mixed, 1}, {Full, 2064}, {Mixed, 2}, {Full, 32680}},
         {0x00000002, 0xfffff801, 0x00113fff, 0xffd00000, 0x63ff3fff,
          0x0002ffff, 0xffff0000, 0x00001fff, 0xc0202000, 0x00001940,
          0xffff07ec}};
    /** Code Point Ranges for Ain
    [0639, 063a], [06a0, 06a0], [06fc, 06fc], [075d, 075f], [08b3, 08b3]**/
    const UnicodeSet ain_Set 
        {{{Empty, 49}, {Mixed, 1}, {Empty, 3}, {Mixed, 1}, {Empty, 1},
          {Mixed, 1}, {Empty, 2}, {Mixed, 1}, {Empty, 10}, {Mixed, 1},
          {Empty, 34746}},
         {0x06000000, 0x00000001, 0x10000000, 0xe0000000, 0x00080000}};
    /** Code Point Ranges for Alaph
    [0710, 0710]**/
    const UnicodeSet alaph_Set 
        {{{Empty, 56}, {Mixed, 1}, {Empty, 34759}},
         {0x00010000}};
    /** Code Point Ranges for Alef
    [0622, 0623], [0625, 0625], [0627, 0627], [0671, 0673], [0675, 0675],
    [0773, 0774]**/
    const UnicodeSet alef_Set 
        {{{Empty, 49}, {Mixed, 1}, {Empty, 1}, {Mixed, 1}, {Empty, 7},
          {Mixed, 1}, {Empty, 34756}},
         {0x000000ac, 0x002e0000, 0x00180000}};
    /** Code Point Ranges for Beh
    [0628, 0628], [062a, 062b], [066e, 066e], [0679, 0680], [0750, 0756],
    [08a0, 08a1], [08b6, 08b8]**/
    const UnicodeSet beh_Set 
        {{{Empty, 49}, {Mixed, 1}, {Empty, 1}, {Mixed, 2}, {Empty, 5},
          {Mixed, 1}, {Empty, 10}, {Mixed, 1}, {Empty, 34746}},
         {0x00000d00, 0xfe004000, 0x00000001, 0x007f0000, 0x01c00003}};
    /** Code Point Ranges for Beth
    [0712, 0712], [072d, 072d]**/
    const UnicodeSet beth_Set 
        {{{Empty, 56}, {Mixed, 2}, {Empty, 34758}},
         {0x00040000, 0x00002000}};
    /** Code Point Ranges for Dal
    [062f, 0630], [0688, 0690], [06ee, 06ee], [0759, 075a], [08ae, 08ae]**/
    const UnicodeSet dal_Set 
        {{{Empty, 49}, {Mixed, 1}, {Empty, 2}, {Mixed, 1}, {Empty, 2},
          {Mixed, 1}, {Empty, 2}, {Mixed, 1}, {Empty, 10}, {Mixed, 1},
          {Empty, 34746}},
         {0x00018000, 0x0001ff00, 0x00004000, 0x06000000, 0x00004000}};
    /** Code Point Ranges for Dalath_Rish
    [0715, 0716], [072a, 072a], [072f, 072f]**/
    const UnicodeSet dalath_rish_Set 
        {{{Empty, 56}, {Mixed, 2}, {Empty, 34758}},
         {0x00600000, 0x00008400}};
    /** Code Point Ranges for E
    [0725, 0725]**/
    const UnicodeSet e_Set 
        {{{Empty, 57}, {Mixed, 1}, {Empty, 34758}},
         {0x00000020}};
    /** Code Point Ranges for Feh
    [0641, 0641], [06a1, 06a6], [0760, 0761], [08a4, 08a4]**/
    const UnicodeSet feh_Set 
        {{{Empty, 50}, {Mixed, 1}, {Empty, 2}, {Mixed, 1}, {Empty, 5},
          {Mixed, 1}, {Empty, 9}, {Mixed, 1}, {Empty, 34746}},
         {0x00000002, 0x0000007e, 0x00000003, 0x00000010}};
    /** Code Point Ranges for Final_Semkath
    [0724, 0724]**/
    const UnicodeSet final_semkath_Set 
        {{{Empty, 57}, {Mixed, 1}, {Empty, 34758}},
         {0x00000010}};
    /** Code Point Ranges for Gaf
    [063b, 063c], [06a9, 06a9], [06ab, 06ab], [06af, 06b4], [0762, 0764],
    [08b0, 08b0]**/
    const UnicodeSet gaf_Set 
        {{{Empty, 49}, {Mixed, 1}, {Empty, 3}, {Mixed, 1}, {Empty, 5},
          {Mixed, 1}, {Empty, 9}, {Mixed, 1}, {Empty, 34746}},
         {0x18000000, 0x001f8a00, 0x0000001c, 0x00010000}};
    /** Code Point Ranges for Gamal
    [0713, 0714], [072e, 072e]**/
    const UnicodeSet gamal_Set 
        {{{Empty, 56}, {Mixed, 2}, {Empty, 34758}},
         {0x00180000, 0x00004000}};
    /** Code Point Ranges for Hah
    [062c, 062e], [0681, 0687], [06bf, 06bf], [0757, 0758], [076e, 076f],
    [0772, 0772], [077c, 077c], [08a2, 08a2]**/
    const UnicodeSet hah_Set 
        {{{Empty, 49}, {Mixed, 1}, {Empty, 2}, {Mixed, 2}, {Empty, 4},
          {Mixed, 2}, {Empty, 9}, {Mixed, 1}, {Empty, 34746}},
         {0x00007000, 0x000000fe, 0x80000000, 0x01800000, 0x1004c000,
          0x00000004}};
    /** Code Point Ranges for Teh_Marbuta_Goal
    [06c3, 06c3]**/
    const UnicodeSet teh_marbuta_goal_Set 
        {{{Empty, 54}, {Mixed, 1}, {Empty, 34761}},
         {0x00000008}};
    /** Code Point Ranges for He
    [0717, 0717]**/
    const UnicodeSet he_Set 
        {{{Empty, 56}, {Mixed, 1}, {Empty, 34759}},
         {0x00800000}};
    /** Code Point Ranges for Heh
    [0647, 0647]**/
    const UnicodeSet heh_Set 
        {{{Empty, 50}, {Mixed, 1}, {Empty, 34765}},
         {0x00000080}};
    /** Code Point Ranges for Heh_Goal
    [06c1, 06c2]**/
    const UnicodeSet heh_goal_Set 
        {{{Empty, 54}, {Mixed, 1}, {Empty, 34761}},
         {0x00000006}};
    /** Code Point Ranges for Heth
    [071a, 071a]**/
    const UnicodeSet heth_Set 
        {{{Empty, 56}, {Mixed, 1}, {Empty, 34759}},
         {0x04000000}};
    /** Code Point Ranges for Kaf
    [0643, 0643], [06ac, 06ae], [077f, 077f], [08b4, 08b4]**/
    const UnicodeSet kaf_Set 
        {{{Empty, 50}, {Mixed, 1}, {Empty, 2}, {Mixed, 1}, {Empty, 5},
          {Mixed, 1}, {Empty, 9}, {Mixed, 1}, {Empty, 34746}},
         {0x00000008, 0x00007000, 0x80000000, 0x00100000}};
    /** Code Point Ranges for Kaph
    [071f, 071f]**/
    const UnicodeSet kaph_Set 
        {{{Empty, 56}, {Mixed, 1}, {Empty, 34759}},
         {0x80000000}};
    /** Code Point Ranges for Knotted_Heh
    [06be, 06be], [06ff, 06ff]**/
    const UnicodeSet knotted_heh_Set 
        {{{Empty, 53}, {Mixed, 1}, {Empty, 1}, {Mixed, 1}, {Empty, 34760}},
         {0x40000000, 0x80000000}};
    /** Code Point Ranges for Lam
    [0644, 0644], [06b5, 06b8], [076a, 076a], [08a6, 08a6]**/
    const UnicodeSet lam_Set 
        {{{Empty, 50}, {Mixed, 1}, {Empty, 2}, {Mixed, 1}, {Empty, 5},
          {Mixed, 1}, {Empty, 9}, {Mixed, 1}, {Empty, 34746}},
         {0x00000010, 0x01e00000, 0x00000400, 0x00000040}};
    /** Code Point Ranges for Lamadh
    [0720, 0720]**/
    const UnicodeSet lamadh_Set 
        {{{Empty, 57}, {Mixed, 1}, {Empty, 34758}},
         {0x00000001}};
    /** Code Point Ranges for Meem
    [0645, 0645], [0765, 0766], [08a7, 08a7]**/
    const UnicodeSet meem_Set 
        {{{Empty, 50}, {Mixed, 1}, {Empty, 8}, {Mixed, 1}, {Empty, 9},
          {Mixed, 1}, {Empty, 34746}},
         {0x00000020, 0x00000060, 0x00000080}};
    /** Code Point Ranges for Mim
    [0721, 0721]**/
    const UnicodeSet mim_Set 
        {{{Empty, 57}, {Mixed, 1}, {Empty, 34758}},
         {0x00000002}};
    /** Code Point Ranges for Noon
    [0646, 0646], [06b9, 06bc], [0767, 0769]**/
    const UnicodeSet noon_Set 
        {{{Empty, 50}, {Mixed, 1}, {Empty, 2}, {Mixed, 1}, {Empty, 5},
          {Mixed, 1}, {Empty, 34756}},
         {0x00000040, 0x1e000000, 0x00000380}};
    /** Code Point Ranges for Nun
    [0722, 0722]**/
    const UnicodeSet nun_Set 
        {{{Empty, 57}, {Mixed, 1}, {Empty, 34758}},
         {0x00000004}};
    /** Code Point Ranges for Pe
    [0726, 0726]**/
    const UnicodeSet pe_Set 
        {{{Empty, 57}, {Mixed, 1}, {Empty, 34758}},
         {0x00000040}};
    /** Code Point Ranges for Qaf
    [0642, 0642], [066f, 066f], [06a7, 06a8], [08a5, 08a5]**/
    const UnicodeSet qaf_Set 
        {{{Empty, 50}, {Mixed, 2}, {Empty, 1}, {Mixed, 1}, {Empty, 15},
          {Mixed, 1}, {Empty, 34746}},
         {0x00000004, 0x00008000, 0x00000180, 0x00000020}};
    /** Code Point Ranges for Qaph
    [0729, 0729]**/
    const UnicodeSet qaph_Set 
        {{{Empty, 57}, {Mixed, 1}, {Empty, 34758}},
         {0x00000200}};
    /** Code Point Ranges for Reh
    [0631, 0632], [0691, 0699], [06ef, 06ef], [075b, 075b], [076b, 076c],
    [0771, 0771], [08aa, 08aa], [08b2, 08b2], [08b9, 08b9]**/
    const UnicodeSet reh_Set 
        {{{Empty, 49}, {Mixed, 1}, {Empty, 2}, {Mixed, 1}, {Empty, 2},
          {Mixed, 1}, {Empty, 2}, {Mixed, 2}, {Empty, 9}, {Mixed, 1},
          {Empty, 34746}},
         {0x00060000, 0x03fe0000, 0x00008000, 0x08000000, 0x00021800,
          0x02040400}};
    /** Code Point Ranges for Reversed_Pe
    [0727, 0727]**/
    const UnicodeSet reversed_pe_Set 
        {{{Empty, 57}, {Mixed, 1}, {Empty, 34758}},
         {0x00000080}};
    /** Code Point Ranges for Sad
    [0635, 0636], [069d, 069e], [06fb, 06fb], [08af, 08af]**/
    const UnicodeSet sad_Set 
        {{{Empty, 49}, {Mixed, 1}, {Empty, 2}, {Mixed, 1}, {Empty, 2},
          {Mixed, 1}, {Empty, 13}, {Mixed, 1}, {Empty, 34746}},
         {0x00600000, 0x60000000, 0x08000000, 0x00008000}};
    /** Code Point Ranges for Sadhe
    [0728, 0728]**/
    const UnicodeSet sadhe_Set 
        {{{Empty, 57}, {Mixed, 1}, {Empty, 34758}},
         {0x00000100}};
    /** Code Point Ranges for Seen
    [0633, 0634], [069a, 069c], [06fa, 06fa], [075c, 075c], [076d, 076d],
    [0770, 0770], [077d, 077e]**/
    const UnicodeSet seen_Set 
        {{{Empty, 49}, {Mixed, 1}, {Empty, 2}, {Mixed, 1}, {Empty, 2},
          {Mixed, 1}, {Empty, 2}, {Mixed, 2}, {Empty, 34756}},
         {0x00180000, 0x1c000000, 0x04000000, 0x10000000, 0x60012000}};
    /** Code Point Ranges for Semkath
    [0723, 0723]**/
    const UnicodeSet semkath_Set 
        {{{Empty, 57}, {Mixed, 1}, {Empty, 34758}},
         {0x00000008}};
    /** Code Point Ranges for Shin
    [072b, 072b]**/
    const UnicodeSet shin_Set 
        {{{Empty, 57}, {Mixed, 1}, {Empty, 34758}},
         {0x00000800}};
    /** Code Point Ranges for Swash_Kaf
    [06aa, 06aa]**/
    const UnicodeSet swash_kaf_Set 
        {{{Empty, 53}, {Mixed, 1}, {Empty, 34762}},
         {0x00000400}};
    /** Code Point Ranges for Tah
    [0637, 0638], [069f, 069f], [08a3, 08a3]**/
    const UnicodeSet tah_Set 
        {{{Empty, 49}, {Mixed, 1}, {Empty, 2}, {Mixed, 1}, {Empty, 16},
          {Mixed, 1}, {Empty, 34746}},
         {0x01800000, 0x80000000, 0x00000008}};
    /** Code Point Ranges for Taw
    [072c, 072c]**/
    const UnicodeSet taw_Set 
        {{{Empty, 57}, {Mixed, 1}, {Empty, 34758}},
         {0x00001000}};
    /** Code Point Ranges for Teh_Marbuta
    [0629, 0629], [06c0, 06c0], [06d5, 06d5]**/
    const UnicodeSet teh_marbuta_Set 
        {{{Empty, 49}, {Mixed, 1}, {Empty, 4}, {Mixed, 1}, {Empty, 34761}},
         {0x00000200, 0x00200001}};
    /** Code Point Ranges for Teth
    [071b, 071c]**/
    const UnicodeSet teth_Set 
        {{{Empty, 56}, {Mixed, 1}, {Empty, 34759}},
         {0x18000000}};
    /** Code Point Ranges for Waw
    [0624, 0624], [0648, 0648], [0676, 0677], [06c4, 06cb], [06cf, 06cf],
    [0778, 0779], [08ab, 08ab]**/
    const UnicodeSet waw_Set 
        {{{Empty, 49}, {Mixed, 3}, {Empty, 2}, {Mixed, 1}, {Empty, 4},
          {Mixed, 1}, {Empty, 9}, {Mixed, 1}, {Empty, 34746}},
         {0x00000010, 0x00000100, 0x00c00000, 0x00008ff0, 0x03000000,
          0x00000800}};
    /** Code Point Ranges for Syriac_Waw
    [0718, 0718]**/
    const UnicodeSet syriac_waw_Set 
        {{{Empty, 56}, {Mixed, 1}, {Empty, 34759}},
         {0x01000000}};
    /** Code Point Ranges for Yeh
    [0620, 0620], [0626, 0626], [0649, 064a], [0678, 0678], [06d0, 06d1],
    [0777, 0777], [08a8, 08a9], [08ba, 08ba]**/
    const UnicodeSet yeh_Set 
        {{{Empty, 49}, {Mixed, 3}, {Empty, 2}, {Mixed, 1}, {Empty, 4},
          {Mixed, 1}, {Empty, 9}, {Mixed, 1}, {Empty, 34746}},
         {0x00000041, 0x00000600, 0x01000000, 0x00030000, 0x00800000,
          0x04000300}};
    /** Code Point Ranges for Yeh_Barree
    [06d2, 06d3]**/
    const UnicodeSet yeh_barree_Set 
        {{{Empty, 54}, {Mixed, 1}, {Empty, 34761}},
         {0x000c0000}};
    /** Code Point Ranges for Yeh_With_Tail
    [06cd, 06cd]**/
    const UnicodeSet yeh_with_tail_Set 
        {{{Empty, 54}, {Mixed, 1}, {Empty, 34761}},
         {0x00002000}};
    /** Code Point Ranges for Yudh
    [071d, 071d]**/
    const UnicodeSet yudh_Set 
        {{{Empty, 56}, {Mixed, 1}, {Empty, 34759}},
         {0x20000000}};
    /** Code Point Ranges for Yudh_He
    [071e, 071e]**/
    const UnicodeSet yudh_he_Set 
        {{{Empty, 56}, {Mixed, 1}, {Empty, 34759}},
         {0x40000000}};
    /** Code Point Ranges for Zain
    [0719, 0719]**/
    const UnicodeSet zain_Set 
        {{{Empty, 56}, {Mixed, 1}, {Empty, 34759}},
         {0x02000000}};
    /** Code Point Ranges for Zhain
    [074d, 074d]**/
    const UnicodeSet zhain_Set 
        {{{Empty, 58}, {Mixed, 1}, {Empty, 34757}},
         {0x00002000}};
    /** Code Point Ranges for Khaph
    [074e, 074e]**/
    const UnicodeSet khaph_Set 
        {{{Empty, 58}, {Mixed, 1}, {Empty, 34757}},
         {0x00004000}};
    /** Code Point Ranges for Fe
    [074f, 074f]**/
    const UnicodeSet fe_Set 
        {{{Empty, 58}, {Mixed, 1}, {Empty, 34757}},
         {0x00008000}};
    /** Code Point Ranges for Burushaski_Yeh_Barree
    [077a, 077b]**/
    const UnicodeSet burushaski_yeh_barree_Set 
        {{{Empty, 59}, {Mixed, 1}, {Empty, 34756}},
         {0x0c000000}};
    /** Code Point Ranges for Farsi_Yeh
    [063d, 063f], [06cc, 06cc], [06ce, 06ce], [0775, 0776]**/
    const UnicodeSet farsi_yeh_Set 
        {{{Empty, 49}, {Mixed, 1}, {Empty, 4}, {Mixed, 1}, {Empty, 4},
          {Mixed, 1}, {Empty, 34756}},
         {0xe0000000, 0x00005000, 0x00600000}};
    /** Code Point Ranges for Nya
    [06bd, 06bd]**/
    const UnicodeSet nya_Set 
        {{{Empty, 53}, {Mixed, 1}, {Empty, 34762}},
         {0x20000000}};
    /** Code Point Ranges for Rohingya_Yeh
    [08ac, 08ac]**/
    const UnicodeSet rohingya_yeh_Set 
        {{{Empty, 69}, {Mixed, 1}, {Empty, 34746}},
         {0x00001000}};
    /** Code Point Ranges for Straight_Waw
    [08b1, 08b1]**/
    const UnicodeSet straight_waw_Set 
        {{{Empty, 69}, {Mixed, 1}, {Empty, 34746}},
         {0x00020000}};
    /** Code Point Ranges for Manichaean_Aleph
    [10ac0, 10ac0]**/
    const UnicodeSet manichaean_aleph_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x00000001}};
    /** Code Point Ranges for Manichaean_Ayin
    [10ad9, 10ada]**/
    const UnicodeSet manichaean_ayin_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x06000000}};
    /** Code Point Ranges for Manichaean_Beth
    [10ac1, 10ac2]**/
    const UnicodeSet manichaean_beth_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x00000006}};
    /** Code Point Ranges for Manichaean_Daleth
    [10ac5, 10ac5]**/
    const UnicodeSet manichaean_daleth_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x00000020}};
    /** Code Point Ranges for Manichaean_Dhamedh
    [10ad4, 10ad4]**/
    const UnicodeSet manichaean_dhamedh_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x00100000}};
    /** Code Point Ranges for Manichaean_Five
    [10aec, 10aec]**/
    const UnicodeSet manichaean_five_Set 
        {{{Empty, 2135}, {Mixed, 1}, {Empty, 32680}},
         {0x00001000}};
    /** Code Point Ranges for Manichaean_Gimel
    [10ac3, 10ac4]**/
    const UnicodeSet manichaean_gimel_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x00000018}};
    /** Code Point Ranges for Manichaean_Heth
    [10acd, 10acd]**/
    const UnicodeSet manichaean_heth_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x00002000}};
    /** Code Point Ranges for Manichaean_Hundred
    [10aef, 10aef]**/
    const UnicodeSet manichaean_hundred_Set 
        {{{Empty, 2135}, {Mixed, 1}, {Empty, 32680}},
         {0x00008000}};
    /** Code Point Ranges for Manichaean_Kaph
    [10ad0, 10ad2]**/
    const UnicodeSet manichaean_kaph_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x00070000}};
    /** Code Point Ranges for Manichaean_Lamedh
    [10ad3, 10ad3]**/
    const UnicodeSet manichaean_lamedh_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x00080000}};
    /** Code Point Ranges for Manichaean_Mem
    [10ad6, 10ad6]**/
    const UnicodeSet manichaean_mem_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x00400000}};
    /** Code Point Ranges for Manichaean_Nun
    [10ad7, 10ad7]**/
    const UnicodeSet manichaean_nun_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x00800000}};
    /** Code Point Ranges for Manichaean_One
    [10aeb, 10aeb]**/
    const UnicodeSet manichaean_one_Set 
        {{{Empty, 2135}, {Mixed, 1}, {Empty, 32680}},
         {0x00000800}};
    /** Code Point Ranges for Manichaean_Pe
    [10adb, 10adc]**/
    const UnicodeSet manichaean_pe_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x18000000}};
    /** Code Point Ranges for Manichaean_Qoph
    [10ade, 10ae0]**/
    const UnicodeSet manichaean_qoph_Set 
        {{{Empty, 2134}, {Mixed, 2}, {Empty, 32680}},
         {0xc0000000, 0x00000001}};
    /** Code Point Ranges for Manichaean_Resh
    [10ae1, 10ae1]**/
    const UnicodeSet manichaean_resh_Set 
        {{{Empty, 2135}, {Mixed, 1}, {Empty, 32680}},
         {0x00000002}};
    /** Code Point Ranges for Manichaean_Sadhe
    [10add, 10add]**/
    const UnicodeSet manichaean_sadhe_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x20000000}};
    /** Code Point Ranges for Manichaean_Samekh
    [10ad8, 10ad8]**/
    const UnicodeSet manichaean_samekh_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x01000000}};
    /** Code Point Ranges for Manichaean_Taw
    [10ae4, 10ae4]**/
    const UnicodeSet manichaean_taw_Set 
        {{{Empty, 2135}, {Mixed, 1}, {Empty, 32680}},
         {0x00000010}};
    /** Code Point Ranges for Manichaean_Ten
    [10aed, 10aed]**/
    const UnicodeSet manichaean_ten_Set 
        {{{Empty, 2135}, {Mixed, 1}, {Empty, 32680}},
         {0x00002000}};
    /** Code Point Ranges for Manichaean_Teth
    [10ace, 10ace]**/
    const UnicodeSet manichaean_teth_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x00004000}};
    /** Code Point Ranges for Manichaean_Thamedh
    [10ad5, 10ad5]**/
    const UnicodeSet manichaean_thamedh_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x00200000}};
    /** Code Point Ranges for Manichaean_Twenty
    [10aee, 10aee]**/
    const UnicodeSet manichaean_twenty_Set 
        {{{Empty, 2135}, {Mixed, 1}, {Empty, 32680}},
         {0x00004000}};
    /** Code Point Ranges for Manichaean_Waw
    [10ac7, 10ac7]**/
    const UnicodeSet manichaean_waw_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x00000080}};
    /** Code Point Ranges for Manichaean_Yodh
    [10acf, 10acf]**/
    const UnicodeSet manichaean_yodh_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x00008000}};
    /** Code Point Ranges for Manichaean_Zayin
    [10ac9, 10aca]**/
    const UnicodeSet manichaean_zayin_Set 
        {{{Empty, 2134}, {Mixed, 1}, {Empty, 32681}},
         {0x00000600}};
    /** Code Point Ranges for African_Feh
    [08bb, 08bb]**/
    const UnicodeSet african_feh_Set 
        {{{Empty, 69}, {Mixed, 1}, {Empty, 34746}},
         {0x08000000}};
    /** Code Point Ranges for African_Qaf
    [08bc, 08bc]**/
    const UnicodeSet african_qaf_Set 
        {{{Empty, 69}, {Mixed, 1}, {Empty, 34746}},
         {0x10000000}};
    /** Code Point Ranges for African_Noon
    [08bd, 08bd]**/
    const UnicodeSet african_noon_Set 
        {{{Empty, 69}, {Mixed, 1}, {Empty, 34746}},
         {0x20000000}};
    static EnumeratedPropertyObject property_object
        {jg,
         JG_ns::independent_prop_values,
         JG_ns::enum_names,
         JG_ns::value_names,
         JG_ns::aliases_only_map,
         {&no_joining_group_Set, &ain_Set, &alaph_Set, &alef_Set, &beh_Set,
        &beth_Set, &dal_Set, &dalath_rish_Set, &e_Set, &feh_Set,
        &final_semkath_Set, &gaf_Set, &gamal_Set, &hah_Set,
        &teh_marbuta_goal_Set, &he_Set, &heh_Set, &heh_goal_Set, &heth_Set,
        &kaf_Set, &kaph_Set, &knotted_heh_Set, &lam_Set, &lamadh_Set,
        &meem_Set, &mim_Set, &noon_Set, &nun_Set, &pe_Set, &qaf_Set,
        &qaph_Set, &reh_Set, &reversed_pe_Set, &sad_Set, &sadhe_Set,
        &seen_Set, &semkath_Set, &shin_Set, &swash_kaf_Set, &tah_Set,
        &taw_Set, &teh_marbuta_Set, &teth_Set, &waw_Set, &syriac_waw_Set,
        &yeh_Set, &yeh_barree_Set, &yeh_with_tail_Set, &yudh_Set,
        &yudh_he_Set, &zain_Set, &zhain_Set, &khaph_Set, &fe_Set,
        &burushaski_yeh_barree_Set, &farsi_yeh_Set, &nya_Set,
        &rohingya_yeh_Set, &straight_waw_Set, &manichaean_aleph_Set,
        &manichaean_ayin_Set, &manichaean_beth_Set, &manichaean_daleth_Set,
        &manichaean_dhamedh_Set, &manichaean_five_Set,
        &manichaean_gimel_Set, &manichaean_heth_Set,
        &manichaean_hundred_Set, &manichaean_kaph_Set,
        &manichaean_lamedh_Set, &manichaean_mem_Set, &manichaean_nun_Set,
        &manichaean_one_Set, &manichaean_pe_Set, &manichaean_qoph_Set,
        &manichaean_resh_Set, &manichaean_sadhe_Set, &manichaean_samekh_Set,
        &manichaean_taw_Set, &manichaean_ten_Set, &manichaean_teth_Set,
        &manichaean_thamedh_Set, &manichaean_twenty_Set,
        &manichaean_waw_Set, &manichaean_yodh_Set, &manichaean_zayin_Set,
        &african_feh_Set, &african_qaf_Set, &african_noon_Set
         }};
    }
}

#endif
