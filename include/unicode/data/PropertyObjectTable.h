#ifndef PROPERTYOBJECTTABLE_H
#define PROPERTYOBJECTTABLE_H
/*
 *  Copyright (c) 2018 International Characters, Inc.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters, Inc.
 *
 *  This file is generated by UCD_properties.py - manual edits may be lost.
 */

#include "PropertyAliases.h"
#include "PropertyObjects.h"
#include <array>
#include "BidiBrackets.h"
#include "BidiMirroring.h"
#include "Blocks.h"
#include "CaseFolding.h"
#include "CompositionExclusions.h"
#include "DerivedAge.h"
#include "DerivedBidiClass.h"
#include "DerivedBinaryProperties.h"
#include "DerivedCombiningClass.h"
#include "DerivedCoreProperties.h"
#include "DerivedDecompositionType.h"
#include "DerivedGeneralCategory.h"
#include "DerivedJoiningGroup.h"
#include "DerivedJoiningType.h"
#include "DerivedNormalizationProps.h"
#include "DerivedNumericType.h"
#include "EastAsianWidth.h"
#include "GraphemeBreakProperty.h"
#include "HangulSyllableType.h"
#include "IndicPositionalCategory.h"
#include "IndicSyllabicCategory.h"
#include "Jamo.h"
#include "LineBreak.h"
#include "NameAliases.h"
#include "PropList.h"
#include "ScriptExtensions.h"
#include "Scripts.h"
#include "SentenceBreakProperty.h"
#include "SpecialCasing.h"
#include "UnicodeData.h"
#include "VerticalOrientation.h"
#include "WordBreakProperty.h"
#include "emoji-data.h"

namespace UCD {

  const std::array<PropertyObject *, 127> property_object_table = {{
    new UnsupportedPropertyObject(cjkAccountingNumeric, PropertyObject::ClassTypeId::NumericProperty),
    new UnsupportedPropertyObject(cjkOtherNumeric, PropertyObject::ClassTypeId::NumericProperty),
    new UnsupportedPropertyObject(cjkPrimaryNumeric, PropertyObject::ClassTypeId::NumericProperty),
    &NV_ns::property_object,
    &CF_ns::property_object,
    new UnsupportedPropertyObject(cjkCompatibilityVariant, PropertyObject::ClassTypeId::StringProperty),
    &DM_ns::property_object,
    &FC_NFKC_ns::property_object,
    &LC_ns::property_object,
    &NFKC_CF_ns::property_object,
    &SCF_ns::property_object,
    &SLC_ns::property_object,
    &STC_ns::property_object,
    &SUC_ns::property_object,
    &TC_ns::property_object,
    &UC_ns::property_object,
    &BMG_ns::property_object,
    &BPB_ns::property_object,
    new UnsupportedPropertyObject(cjkIICore, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(cjkIRG_GSource, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(cjkIRG_HSource, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(cjkIRG_JSource, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(cjkIRG_KPSource, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(cjkIRG_KSource, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(cjkIRG_MSource, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(cjkIRG_TSource, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(cjkIRG_USource, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(cjkIRG_VSource, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(cjkRSUnicode, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(EqUIdeo, PropertyObject::ClassTypeId::StringProperty),
    &ISC_ns::property_object,
    &JSN_ns::property_object,
    &NA_ns::property_object,
    &NA1_ns::property_object,
    &NAME_ALIAS_ns::property_object,
    &SCX_ns::property_object,
    &AGE_ns::property_object,
    &BLK_ns::property_object,
    &SC_ns::property_object,
    &BC_ns::property_object,
    &BPT_ns::property_object,
    &CCC_ns::property_object,
    &DT_ns::property_object,
    &EA_ns::property_object,
    &GC_ns::property_object,
    &GCB_ns::property_object,
    &HST_ns::property_object,
    &INPC_ns::property_object,
    &INSC_ns::property_object,
    &JG_ns::property_object,
    &JT_ns::property_object,
    &LB_ns::property_object,
    &NFC_QC_ns::property_object,
    &NFD_QC_ns::property_object,
    &NFKC_QC_ns::property_object,
    &NFKD_QC_ns::property_object,
    &NT_ns::property_object,
    &SB_ns::property_object,
    &VO_ns::property_object,
    &WB_ns::property_object,
    &AHEX_ns::property_object,
    &ALPHA_ns::property_object,
    &BIDI_C_ns::property_object,
    &BIDI_M_ns::property_object,
    &CASED_ns::property_object,
    &CE_ns::property_object,
    &CI_ns::property_object,
    &COMP_EX_ns::property_object,
    &CWCF_ns::property_object,
    &CWCM_ns::property_object,
    &CWKCF_ns::property_object,
    &CWL_ns::property_object,
    &CWT_ns::property_object,
    &CWU_ns::property_object,
    &DASH_ns::property_object,
    &DEP_ns::property_object,
    &DI_ns::property_object,
    &DIA_ns::property_object,
    &EXT_ns::property_object,
    &GR_BASE_ns::property_object,
    &GR_EXT_ns::property_object,
    &GR_LINK_ns::property_object,
    &HEX_ns::property_object,
    &HYPHEN_ns::property_object,
    &IDC_ns::property_object,
    &IDEO_ns::property_object,
    &IDS_ns::property_object,
    &IDSB_ns::property_object,
    &IDST_ns::property_object,
    &JOIN_C_ns::property_object,
    &LOE_ns::property_object,
    &LOWER_ns::property_object,
    &MATH_ns::property_object,
    &NCHAR_ns::property_object,
    &OALPHA_ns::property_object,
    &ODI_ns::property_object,
    &OGR_EXT_ns::property_object,
    &OIDC_ns::property_object,
    &OIDS_ns::property_object,
    &OLOWER_ns::property_object,
    &OMATH_ns::property_object,
    &OUPPER_ns::property_object,
    &PAT_SYN_ns::property_object,
    &PAT_WS_ns::property_object,
    &PCM_ns::property_object,
    &QMARK_ns::property_object,
    &RADICAL_ns::property_object,
    &RI_ns::property_object,
    &SD_ns::property_object,
    &STERM_ns::property_object,
    &TERM_ns::property_object,
    &UIDEO_ns::property_object,
    &UPPER_ns::property_object,
    &VS_ns::property_object,
    &WSPACE_ns::property_object,
    &XIDC_ns::property_object,
    &XIDS_ns::property_object,
    &XO_NFC_ns::property_object,
    &XO_NFD_ns::property_object,
    &XO_NFKC_ns::property_object,
    &XO_NFKD_ns::property_object,
    &EMOJI_ns::property_object,
    &EMOJIPRESENTATION_ns::property_object,
    &EMOJIMODIFIER_ns::property_object,
    &EMOJIMODIFIERBASE_ns::property_object,
    &EMOJICOMPONENT_ns::property_object,
    &EXTENDEDPICTOGRAPHIC_ns::property_object  }};
}

#endif