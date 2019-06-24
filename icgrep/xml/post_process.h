/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <cinttypes>
#include <cstdlib>

extern "C" {

void postproc_validateNameStart(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum);

void postproc_validateName(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum);

void postproc_validatePIName(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum);

void postproc_validateCDATA(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum);

void postproc_validateGenRef(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum);

void postproc_validateHexRef(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum);

void postproc_validateDecRef(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum);

void postproc_validateAttRef(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum);

} // extern "C"
