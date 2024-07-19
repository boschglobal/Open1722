/*
 * Copyright (c) 2024, COVESA
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of COVESA nor the names of its contributors may be
 *      used to endorse or promote products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file
 * This file contains the fields descriptions of the IEEE 1722 AAF common stream PDUs and
 * functions to invoke corresponding parser and deparser.
 */

#pragma once

#include <stdint.h>

#include "avtp/pdu/Defines.h"

#define AVTP_AAF_COMMON_STREAM_HEADER_LEN              (6 * AVTP_QUADLET_SIZE)

typedef struct {
    uint8_t header[AVTP_AAF_COMMON_STREAM_HEADER_LEN];
    uint8_t payload[0];
} Avtp_AafCommonStream_t;

typedef enum {
    AVTP_AAF_COMMON_STREAM_FIELD_SUBTYPE,
    AVTP_AAF_COMMON_STREAM_FIELD_SV,
    AVTP_AAF_COMMON_STREAM_FIELD_VERSION,
    AVTP_AAF_COMMON_STREAM_FIELD_MR,
    AVTP_AAF_COMMON_STREAM_FIELD_TV,
    AVTP_AAF_COMMON_STREAM_FIELD_SEQUENCE_NUM,
    AVTP_AAF_COMMON_STREAM_FIELD_TU,
    AVTP_AAF_COMMON_STREAM_FIELD_STREAM_ID,
    AVTP_AAF_COMMON_STREAM_FIELD_AVTP_TIMESTAMP,
    AVTP_AAF_COMMON_STREAM_FIELD_FORMAT,
    AVTP_AAF_COMMON_STREAM_FIELD_AAF_FORMAT_SPECIFIC_DATA_1,
    AVTP_AAF_COMMON_STREAM_FIELD_STREAM_DATA_LENGTH,
    AVTP_AAF_COMMON_STREAM_FIELD_AFSD,
    AVTP_AAF_COMMON_STREAM_FIELD_SP,
    AVTP_AAF_COMMON_STREAM_FIELD_EVT,
    AVTP_AAF_COMMON_STREAM_FIELD_AAF_FORMAT_SPECIFIC_DATA_2,
    AVTP_AAF_COMMON_STREAM_FIELD_MAX
} Avtp_AafCommonStreamFields_t;

/**
 * Returns the value of an an AVTP AAF common stream field as specified in the IEEE 1722 Specification.
 *
 * @param pdu Pointer to the first bit of an 1722 AVTP PDU.
 * @param field Specifies the position of the data field to be read
 * @param value Pointer to location to store the value.
 * @returns This function returns 0 if the data field was successfully read from
 * the 1722 AVTP PDU.
 */
int Avtp_AafCommonStream_GetField(Avtp_AafCommonStream_t* pdu, Avtp_AafCommonStreamFields_t field, uint64_t* value);

/**
 * Sets the value of an an AVTP AAF common stream field as specified in the IEEE 1722 Specification.
 *
 * @param pdu Pointer to the first bit of an 1722 AVTP PDU.
 * @param field Specifies the position of the data field to be read
 * @param value Pointer to location to store the value.
 * @returns This function returns 0 if the data field was successfully set in
 * the 1722 AVTP PDU.
 */
int Avtp_AafCommonStream_SetField(Avtp_AafCommonStream_t* pdu, Avtp_AafCommonStreamFields_t field, uint64_t value);