/*
 * Copyright (c) 2024, COVESA
 * Copyright (c) 2019, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of COVESA, Intel Corporation nor the names of its
 *      contributors  may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
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

#pragma once

#include <stdint.h>

#include "avtp/pdu/Defines.h"

#define AVTP_AAF_PCM_STREAM_HEADER_LEN              (6 * AVTP_QUADLET_SIZE)

typedef struct {
    uint8_t header[AVTP_AAF_PCM_STREAM_HEADER_LEN];
    uint8_t payload[0];
} Avtp_AafPcmStream_t;

typedef enum {
    AVTP_AAF_PCM_STREAM_FIELD_SUBTYPE,
    AVTP_AAF_PCM_STREAM_FIELD_SV,
    AVTP_AAF_PCM_STREAM_FIELD_VERSION,
    AVTP_AAF_PCM_STREAM_FIELD_MR,
    AVTP_AAF_PCM_STREAM_FIELD_TV,
    AVTP_AAF_PCM_STREAM_FIELD_SEQUENCE_NUM,
    AVTP_AAF_PCM_STREAM_FIELD_TU,
    AVTP_AAF_PCM_STREAM_FIELD_STREAM_ID,
    AVTP_AAF_PCM_STREAM_FIELD_AVTP_TIMESTAMP,
    AVTP_AAF_PCM_STREAM_FIELD_FORMAT,
    AVTP_AAF_PCM_STREAM_FIELD_NSR,
    AVTP_AAF_PCM_STREAM_FIELD_CHANNELS_PER_FRAME,
    AVTP_AAF_PCM_STREAM_FIELD_BIT_DEPTH,
    AVTP_AAF_PCM_STREAM_FIELD_STREAM_DATA_LENGTH,
    AVTP_AAF_PCM_STREAM_FIELD_SP,
    AVTP_AAF_PCM_STREAM_FIELD_EVT,
    AVTP_AAF_PCM_STREAM_FIELD_MAX
} Avtp_AafPcmStreamFields_t;

// AAF 'format' field values
typedef enum {
    AVTP_AAF_FORMAT_USER = 0,
    AVTP_AAF_FORMAT_FLOAT_32BIT,
    AVTP_AAF_FORMAT_INT_32BIT,
    AVTP_AAF_FORMAT_INT_24BIT,
    AVTP_AAF_FORMAT_INT_16BIT,
    AVTP_AAF_FORMAT_AES3_32BIT,
} Avtp_AafFormat_t;

typedef enum {
    AVTP_AAF_PCM_NSR_USER = 0,
    AVTP_AAF_PCM_NSR_8KHZ,
    AVTP_AAF_PCM_NSR_16KHZ,
    AVTP_AAF_PCM_NSR_32KHZ,
    AVTP_AAF_PCM_NSR_44_1KHZ,
    AVTP_AAF_PCM_NSR_48KHZ,
    AVTP_AAF_PCM_NSR_88_2KHZ,
    AVTP_AAF_PCM_NSR_96KHZ,
    AVTP_AAF_PCM_NSR_176_4KHZ,
    AVTP_AAF_PCM_NSR_192KHZ,
    AVTP_AAF_PCM_NSR_24KHZ,
} Avtp_AafNsr_t;

// AAF 'sp' (sparse timestamp) field value
typedef enum {
    AVTP_AAF_PCM_SP_NORMAL = 0,
    AVTP_AAF_PCM_SP_SPARSE,
} Avtp_AafSp_t;

/**
 * Initializes a AAF PCM stream PDU as specified in the IEEE 1722-2016 Specification.
 *
 * @param pdu Pointer to the first bit of a 1722 PDU.
 */
int Avtp_AafPcmStream_Init(Avtp_AafPcmStream_t* pdu);

/**
 * Returns the value of an an AVTP AAF PCM stream field as specified in the IEEE 1722 Specification.
 *
 * @param pdu Pointer to the first bit of an 1722 AVTP PDU.
 * @param field Specifies the position of the data field to be read
 * @param value Pointer to location to store the value.
 * @returns This function returns 0 if the data field was successfully read from
 * the 1722 AVTP PDU.
 */
int Avtp_AafPcmStream_GetField(Avtp_AafPcmStream_t* pdu, Avtp_AafPcmStreamFields_t field, uint64_t* value);

/**
 * Sets the value of an an AVTP AAF PCM stream field as specified in the IEEE 1722 Specification.
 *
 * @param pdu Pointer to the first bit of an 1722 AVTP PDU.
 * @param field Specifies the position of the data field to be read
 * @param value Pointer to location to store the value.
 * @returns This function returns 0 if the data field was successfully set in
 * the 1722 AVTP PDU.
 */
int Avtp_AafPcmStream_SetField(Avtp_AafPcmStream_t* pdu, Avtp_AafPcmStreamFields_t field, uint64_t value);

/******************************************************************************
 * Legacy API (deprecated)
 *****************************************************************************/

#define AVTP_AAF_FIELD_SV               AVTP_AAF_PCM_STREAM_FIELD_SV
#define AVTP_AAF_FIELD_MR               AVTP_AAF_PCM_STREAM_FIELD_MR
#define AVTP_AAF_FIELD_TV               AVTP_AAF_PCM_STREAM_FIELD_TV
#define AVTP_AAF_FIELD_SEQ_NUM          AVTP_AAF_PCM_STREAM_FIELD_SEQUENCE_NUM
#define AVTP_AAF_FIELD_TU               AVTP_AAF_PCM_STREAM_FIELD_TU
#define AVTP_AAF_FIELD_STREAM_ID        AVTP_AAF_PCM_STREAM_FIELD_STREAM_ID
#define AVTP_AAF_FIELD_TIMESTAMP        AVTP_AAF_PCM_STREAM_FIELD_AVTP_TIMESTAMP
#define AVTP_AAF_FIELD_STREAM_DATA_LEN  AVTP_AAF_PCM_STREAM_FIELD_STREAM_DATA_LENGTH
#define AVTP_AAF_FIELD_FORMAT           AVTP_AAF_PCM_STREAM_FIELD_FORMAT
#define AVTP_AAF_FIELD_NSR              AVTP_AAF_PCM_STREAM_FIELD_NSR
#define AVTP_AAF_FIELD_CHAN_PER_FRAME   AVTP_AAF_PCM_STREAM_FIELD_CHANNELS_PER_FRAME
#define AVTP_AAF_FIELD_BIT_DEPTH        AVTP_AAF_PCM_STREAM_FIELD_BIT_DEPTH
#define AVTP_AAF_FIELD_SP               AVTP_AAF_PCM_STREAM_FIELD_SP
#define AVTP_AAF_FIELD_EVT              AVTP_AAF_PCM_STREAM_FIELD_EVT
#define AVTP_AAF_FIELD_MAX              AVTP_AAF_PCM_STREAM_FIELD_MAX

/**
 * @deprecated
 * Get value from AAF AVTPDU field.
 * @pdu: Pointer to PDU struct.
 * @field: PDU field to be retrieved.
 * @val: Pointer to variable which the retrieved value should be saved.
 *
 * Returns:
 *    0: Success.
 *    -EINVAL: If any argument is invalid.
 */
int avtp_aaf_pdu_get(void *pdu,
                Avtp_AafPcmStreamFields_t field, uint64_t *val);

/**
 * @deprecated
 * Set value from AAF AVTPDU field.
 * @pdu: Pointer to PDU struct.
 * @field: PDU field to be set.
 * @val: Value to be set.
 *
 * Returns:
 *    0: Success.
 *    -EINVAL: If any argument is invalid.
 */
int avtp_aaf_pdu_set(void *pdu, Avtp_AafPcmStreamFields_t field,
                                uint64_t val);

/**
 * @deprecated
 * Initialize AAF AVTPDU. All AVTPDU fields are initialized with zero except
 * 'subtype' (which is set to AVTP_SUBTYPE_AAF) and 'sv' (which is set to 1).
 * @pdu: Pointer to PDU struct.
 *
 * Return values:
 *    0: Success.
 *    -EINVAL: If any argument is invalid.
 */
int avtp_aaf_pdu_init(void *pdu);