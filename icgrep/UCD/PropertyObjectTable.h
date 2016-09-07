#ifndef PROPERTYOBJECTTABLE_H
#define PROPERTYOBJECTTABLE_H
/*
 *  Copyright (c) 2016 International Characters, Inc.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters, Inc.
 *
 *  This file is generated by UCD_properties.py - manual edits may be lost.
 */

#include "PropertyAliases.h"
#include "PropertyObjects.h"
#include <array>
#include "Blocks.h"
#include "DerivedAge.h"
#include "DerivedBidiClass.h"
#include "DerivedBinaryProperties.h"
#include "DerivedCombiningClass.h"
#include "DerivedCoreProperties.h"
#include "DerivedDecompositionType.h"
#include "DerivedGeneralCategory.h"
#include "DerivedJoiningGroup.h"
#include "DerivedJoiningType.h"
#include "DerivedNumericType.h"
#include "EastAsianWidth.h"
#include "GraphemeBreakProperty.h"
#include "HangulSyllableType.h"
#include "LineBreak.h"
#include "PropList.h"
#include "ScriptExtensions.h"
#include "Scripts.h"
#include "SentenceBreakProperty.h"
#include "WordBreakProperty.h"

namespace UCD {

  const std::array<PropertyObject *, 118> property_object_table = {{
    new UnsupportedPropertyObject(cjkAccountingNumeric, PropertyObject::ClassTypeId::NumericProperty),
    new UnsupportedPropertyObject(cjkOtherNumeric, PropertyObject::ClassTypeId::NumericProperty),
    new UnsupportedPropertyObject(cjkPrimaryNumeric, PropertyObject::ClassTypeId::NumericProperty),
    new UnsupportedPropertyObject(nv, PropertyObject::ClassTypeId::NumericProperty),
    new UnsupportedPropertyObject(cf, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(cjkCompatibilityVariant, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(dm, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(FC_NFKC, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(lc, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(NFKC_CF, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(scf, PropertyObject::ClassTypeId::CodepointProperty),
    new UnsupportedPropertyObject(slc, PropertyObject::ClassTypeId::CodepointProperty),
    new UnsupportedPropertyObject(stc, PropertyObject::ClassTypeId::CodepointProperty),
    new UnsupportedPropertyObject(suc, PropertyObject::ClassTypeId::CodepointProperty),
    new UnsupportedPropertyObject(tc, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(uc, PropertyObject::ClassTypeId::StringProperty),
    new UnsupportedPropertyObject(bmg, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(bpb, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(cjkIICore, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(cjkIRG_GSource, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(cjkIRG_HSource, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(cjkIRG_JSource, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(cjkIRG_KPSource, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(cjkIRG_KSource, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(cjkIRG_MSource, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(cjkIRG_TSource, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(cjkIRG_USource, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(cjkIRG_VSource, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(cjkRSUnicode, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(isc, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(JSN, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(na, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(na1, PropertyObject::ClassTypeId::MiscellaneousProperty),
    new UnsupportedPropertyObject(Name_Alias, PropertyObject::ClassTypeId::MiscellaneousProperty),
    &SCX_ns::property_object,
    &AGE_ns::property_object,
    &BLK_ns::property_object,
    &SC_ns::property_object,
    &BC_ns::property_object,
    new UnsupportedPropertyObject(bpt, PropertyObject::ClassTypeId::EnumeratedProperty),
    &CCC_ns::property_object,
    &DT_ns::property_object,
    &EA_ns::property_object,
    &GC_ns::property_object,
    &GCB_ns::property_object,
    &HST_ns::property_object,
    new UnsupportedPropertyObject(InPC, PropertyObject::ClassTypeId::EnumeratedProperty),
    new UnsupportedPropertyObject(InSC, PropertyObject::ClassTypeId::EnumeratedProperty),
    &JG_ns::property_object,
    &JT_ns::property_object,
    &LB_ns::property_object,
    new UnsupportedPropertyObject(NFC_QC, PropertyObject::ClassTypeId::EnumeratedProperty),
    new UnsupportedPropertyObject(NFD_QC, PropertyObject::ClassTypeId::EnumeratedProperty),
    new UnsupportedPropertyObject(NFKC_QC, PropertyObject::ClassTypeId::EnumeratedProperty),
    new UnsupportedPropertyObject(NFKD_QC, PropertyObject::ClassTypeId::EnumeratedProperty),
    &NT_ns::property_object,
    &SB_ns::property_object,
    &WB_ns::property_object,
    &AHEX_ns::property_object,
    &ALPHA_ns::property_object,
    &BIDI_C_ns::property_object,
    &BIDI_M_ns::property_object,
    &CASED_ns::property_object,
    new UnsupportedPropertyObject(CE, PropertyObject::ClassTypeId::BinaryProperty),
    &CI_ns::property_object,
    new UnsupportedPropertyObject(Comp_Ex, PropertyObject::ClassTypeId::BinaryProperty),
    &CWCF_ns::property_object,
    &CWCM_ns::property_object,
    new UnsupportedPropertyObject(CWKCF, PropertyObject::ClassTypeId::BinaryProperty),
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
    &SD_ns::property_object,
    &STERM_ns::property_object,
    &TERM_ns::property_object,
    &UIDEO_ns::property_object,
    &UPPER_ns::property_object,
    &VS_ns::property_object,
    &WSPACE_ns::property_object,
    &XIDC_ns::property_object,
    &XIDS_ns::property_object,
    new UnsupportedPropertyObject(XO_NFC, PropertyObject::ClassTypeId::BinaryProperty),
    new UnsupportedPropertyObject(XO_NFD, PropertyObject::ClassTypeId::BinaryProperty),
    new UnsupportedPropertyObject(XO_NFKC, PropertyObject::ClassTypeId::BinaryProperty),
    new UnsupportedPropertyObject(XO_NFKD, PropertyObject::ClassTypeId::BinaryProperty)  }};
}

#endif
