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

#include <errno.h>
#include <string.h>

#include "avtp/pdu/acf/Tscf.h"
#include "avtp/pdu/Utils.h" 
#include "avtp/pdu/CommonHeader.h"

/**
 * This table maps all IEEE 1722 TSCF-specific header fields to a descriptor.
 */
static const Avtp_FieldDescriptor_t Avtp_TscfFieldDesc[AVTP_TSCF_FIELD_MAX] =
{
    /* Common AVTP header */
    [AVTP_TSCF_FIELD_SUBTYPE]                  = { .quadlet = 0, .offset = 0, .bits = 8 },
    [AVTP_TSCF_FIELD_SV]                       = { .quadlet = 0, .offset = 8, .bits = 1 },
    [AVTP_TSCF_FIELD_VERSION]                  = { .quadlet = 0, .offset = 9, .bits = 3 },
    /* TSCF header*/    
    [AVTP_TSCF_FIELD_MR]                       = { .quadlet = 0, .offset = 12, .bits = 1 },
    [AVTP_TSCF_FIELD_TV]                       = { .quadlet = 0, .offset = 15, .bits = 1 },
    [AVTP_TSCF_FIELD_SEQUENCE_NUM]             = { .quadlet = 0, .offset = 16, .bits = 8 },
    [AVTP_TSCF_FIELD_TU]                       = { .quadlet = 0, .offset = 31, .bits = 1 },
    [AVTP_TSCF_FIELD_STREAM_ID]                = { .quadlet = 1, .offset = 0, .bits = 64 },
    [AVTP_TSCF_FIELD_AVTP_TIMESTAMP]           = { .quadlet = 3, .offset = 0, .bits = 32 },
    [AVTP_TSCF_FIELD_STREAM_DATA_LENGTH]       = { .quadlet = 5, .offset = 0, .bits = 16 },
};

int Avtp_Tscf_Init(Avtp_Tscf_t* pdu)
{
    if (!pdu) {
        return -EINVAL;
    }

    memset(pdu, 0, sizeof(Avtp_Tscf_t));
    Avtp_Tscf_SetField(pdu, AVTP_TSCF_FIELD_SUBTYPE, AVTP_SUBTYPE_TSCF);
    Avtp_Tscf_SetField(pdu, AVTP_TSCF_FIELD_SV, 1);

    return 0;
}

int Avtp_Tscf_GetField(Avtp_Tscf_t* pdu, Avtp_TscfFields_t field, uint64_t* value)
{
    return Avtp_GetField(Avtp_TscfFieldDesc, AVTP_TSCF_FIELD_MAX, (uint8_t*) pdu, (uint8_t) field, value);
}

int Avtp_Tscf_SetField(Avtp_Tscf_t* pdu, Avtp_TscfFields_t field, uint64_t value)
{
    return Avtp_SetField(Avtp_TscfFieldDesc, AVTP_TSCF_FIELD_MAX, (uint8_t*) pdu, (uint8_t) field, value); 
}
