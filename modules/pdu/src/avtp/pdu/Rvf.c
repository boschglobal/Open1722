 /*
 * Copyright (c) 2024, COVESA
 * Copyright (c) 2021, Fastree3D
 * Adrian Fiergolski <Adrian.Fiergolski@fastree3d.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of Fastree3D, COVESA nor the names of their
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

#include "avtp/pdu/Rvf.h"
#include "avtp/pdu/Utils.h"
#include "avtp/pdu/CommonHeader.h"

/**
 * This table maps all IEEE 1722 Raw Video Format (RVF) specific header fields
 * to a descriptor.
 */
static const Avtp_FieldDescriptor_t Avtp_RvfFieldDescriptors[AVTP_RVF_FIELD_MAX] =
{
    [AVTP_RVF_FIELD_SUBTYPE]               = { .quadlet = 0, .offset = 0, .bits = 8 },
    [AVTP_RVF_FIELD_SV]                    = { .quadlet = 0, .offset = 8, .bits = 1 },
    [AVTP_RVF_FIELD_VERSION]               = { .quadlet = 0, .offset = 9, .bits = 3 },
    [AVTP_RVF_FIELD_MR]                    = { .quadlet = 0, .offset = 12, .bits = 1 },
    [AVTP_RVF_FIELD_RESERVED]              = { .quadlet = 0, .offset = 13, .bits = 2 },
    [AVTP_RVF_FIELD_TV]                    = { .quadlet = 0, .offset = 15, .bits = 1 },
    [AVTP_RVF_FIELD_SEQUENCE_NUM]          = { .quadlet = 0, .offset = 16, .bits = 8 },
    [AVTP_RVF_FIELD_RESERVED_2]            = { .quadlet = 0, .offset = 24, .bits = 7 },
    [AVTP_RVF_FIELD_TU]                    = { .quadlet = 0, .offset = 31, .bits = 1 },
    [AVTP_RVF_FIELD_STREAM_ID]             = { .quadlet = 1, .offset = 0, .bits = 64 },
    [AVTP_RVF_FIELD_AVTP_TIMESTAMP]        = { .quadlet = 3, .offset = 0, .bits = 32 },
    [AVTP_RVF_FIELD_ACTIVE_PIXELS]         = { .quadlet = 4, .offset = 0, .bits = 16 },
    [AVTP_RVF_FIELD_TOTAL_LINES]           = { .quadlet = 4, .offset = 16, .bits = 16 },
    [AVTP_RVF_FIELD_STREAM_DATA_LENGTH]    = { .quadlet = 5, .offset = 0, .bits = 16 },
    [AVTP_RVF_FIELD_AP]                    = { .quadlet = 5, .offset = 16, .bits = 1 },
    [AVTP_RVF_FIELD_RESERVED_3]            = { .quadlet = 5, .offset = 17, .bits = 1 },
    [AVTP_RVF_FIELD_F]                     = { .quadlet = 5, .offset = 18, .bits = 1 },
    [AVTP_RVF_FIELD_EF]                    = { .quadlet = 5, .offset = 19, .bits = 1 },
    [AVTP_RVF_FIELD_EVT]                   = { .quadlet = 5, .offset = 20, .bits = 4 },
    [AVTP_RVF_FIELD_PD]                    = { .quadlet = 5, .offset = 24, .bits = 1 },
    [AVTP_RVF_FIELD_I]                     = { .quadlet = 5, .offset = 25, .bits = 1 },
    [AVTP_RVF_FIELD_RESERVED_4]            = { .quadlet = 5, .offset = 26, .bits = 6 },
    [AVTP_RVF_FIELD_RESERVED_5]            = { .quadlet = 6, .offset = 0, .bits = 8 },
    [AVTP_RVF_FIELD_PIXEL_DEPTH]           = { .quadlet = 6, .offset = 8, .bits = 4 },
    [AVTP_RVF_FIELD_PIXEL_FORMAT]          = { .quadlet = 6, .offset = 12, .bits = 4 },
    [AVTP_RVF_FIELD_FRAME_RATE]            = { .quadlet = 6, .offset = 16, .bits = 8 },
    [AVTP_RVF_FIELD_COLORSPACE]            = { .quadlet = 6, .offset = 24, .bits = 4 },
    [AVTP_RVF_FIELD_NUM_LINES]             = { .quadlet = 6, .offset = 28, .bits = 4 },
    [AVTP_RVF_FIELD_RESERVED_6]            = { .quadlet = 7, .offset = 0, .bits = 8 },
    [AVTP_RVF_FIELD_I_SEQ_NUM]             = { .quadlet = 7, .offset = 8, .bits = 8 },
    [AVTP_RVF_FIELD_LINE_NUMBER]           = { .quadlet = 7, .offset = 16, .bits = 16 },
};

int Avtp_Rvf_Init(Avtp_Rvf_t* pdu)
{
    if (pdu == NULL) {
        return -EINVAL;
    }

    memset(pdu, 0, sizeof(Avtp_Rvf_t));
    Avtp_Rvf_SetField(pdu, AVTP_RVF_FIELD_SUBTYPE, AVTP_SUBTYPE_RVF);
    Avtp_Rvf_SetField(pdu, AVTP_RVF_FIELD_SV, 1);

    return 0;
}

int Avtp_Rvf_GetField(Avtp_Rvf_t* pdu, Avtp_RvfField_t field, uint64_t* value)
{
    return Avtp_GetField(Avtp_RvfFieldDescriptors, AVTP_RVF_FIELD_MAX, (uint8_t*)pdu, field, value);
}

int Avtp_Rvf_SetField(Avtp_Rvf_t* pdu, Avtp_RvfField_t field, uint64_t value)
{
    return Avtp_SetField(Avtp_RvfFieldDescriptors, AVTP_RVF_FIELD_MAX, (uint8_t*)pdu, field, value);
}

/******************************************************************************
 * Legacy API
 *****************************************************************************/

int avtp_rvf_pdu_get(const void* pdu, Avtp_RvfField_t field, uint64_t* val)
{
    return Avtp_Rvf_GetField((Avtp_Rvf_t*)pdu, field, val);
}

int avtp_rvf_pdu_set(void* pdu, Avtp_RvfField_t field, uint64_t val)
{
    return Avtp_Rvf_SetField((Avtp_Rvf_t*)pdu, field, val);
}

int avtp_rvf_pdu_init(void* pdu)
{
    return Avtp_Rvf_Init((Avtp_Rvf_t*)pdu);
}
