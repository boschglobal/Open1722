/*
 * Copyright (c) 2024, COVESA
 * Copyright (c) 2018, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of Intel Corporation, COVESA nor the names of their
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

#include <string.h>
#include <errno.h>

#include "avtp/pdu/Crf.h"
#include "avtp/pdu/Utils.h"
#include "avtp/pdu/CommonHeader.h"

/**
 * This table maps all IEEE 1722 Clock Reference Format (CRF) specific header fields
 * to a descriptor.
 */
static const Avtp_FieldDescriptor_t Avtp_CrfFieldDescriptors[AVTP_CRF_FIELD_MAX] =
{
    [AVTP_CRF_FIELD_SUBTYPE]                = { .quadlet = 0, .offset = 0, .bits = 8 },
    [AVTP_CRF_FIELD_SV]                     = { .quadlet = 0, .offset = 8, .bits = 1 },
    [AVTP_CRF_FIELD_VERSION]                = { .quadlet = 0, .offset = 9, .bits = 3 },
    [AVTP_CRF_FIELD_MR]                     = { .quadlet = 0, .offset = 12, .bits = 1 },
    [AVTP_CRF_FIELD_RESERVED]               = { .quadlet = 0, .offset = 13, .bits = 1 },
    [AVTP_CRF_FIELD_FS]                     = { .quadlet = 0, .offset = 14, .bits = 1 },
    [AVTP_CRF_FIELD_TU]                     = { .quadlet = 0, .offset = 15, .bits = 1 },
    [AVTP_CRF_FIELD_SEQUENCE_NUM]           = { .quadlet = 0, .offset = 16, .bits = 8 },
    [AVTP_CRF_FIELD_TYPE]                   = { .quadlet = 0, .offset = 24, .bits = 8 },
    [AVTP_CRF_FIELD_STREAM_ID]              = { .quadlet = 1, .offset = 0, .bits = 64 },
    [AVTP_CRF_FIELD_PULL]                   = { .quadlet = 3, .offset = 0, .bits = 3 },
    [AVTP_CRF_FIELD_BASE_FREQUENCY]         = { .quadlet = 3, .offset = 3, .bits = 29 },
    [AVTP_CRF_FIELD_CRF_DATA_LENGTH]        = { .quadlet = 4, .offset = 0, .bits = 16 },
    [AVTP_CRF_FIELD_TIMESTAMP_INTERVAL]     = { .quadlet = 4, .offset = 16, .bits = 16 },
};

int Avtp_Crf_Init(Avtp_Crf_t* pdu) {

    if (pdu == NULL) {
        return -EINVAL;
    }

    memset(pdu, 0, sizeof(Avtp_Crf_t));
    Avtp_Crf_SetField(pdu, AVTP_CRF_FIELD_SUBTYPE, AVTP_SUBTYPE_CRF);
    Avtp_Crf_SetField(pdu, AVTP_CRF_FIELD_SV, 1);

    return 0;

}

int Avtp_Crf_GetField(Avtp_Crf_t* pdu, Avtp_CrfField_t field, uint64_t* value) { 
    return Avtp_GetField(Avtp_CrfFieldDescriptors, AVTP_CRF_FIELD_MAX, (uint8_t*)pdu, field, value);
}

int Avtp_Crf_SetField(Avtp_Crf_t* pdu, Avtp_CrfField_t field, uint64_t value) {
    return Avtp_SetField(Avtp_CrfFieldDescriptors, AVTP_CRF_FIELD_MAX, (uint8_t*)pdu, field, value);
}

/******************************************************************************
 * Legacy API
 *****************************************************************************/

int avtp_crf_pdu_get(const void *pdu, Avtp_CrfField_t field, uint64_t *val)
{
    return Avtp_Crf_GetField((Avtp_Crf_t*)pdu, field, val);
}

int avtp_crf_pdu_set(void *pdu, Avtp_CrfField_t field, uint64_t val)
{
    return Avtp_Crf_SetField((Avtp_Crf_t*)pdu, field, val);
}

int avtp_crf_pdu_init(void *pdu)
{
    return Avtp_Crf_Init((Avtp_Crf_t*)pdu);
}
