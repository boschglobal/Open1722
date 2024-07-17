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
#include <string.h>
#include <errno.h>

#include "avtp/pdu/CommonHeader.h"
#include "avtp/pdu/aaf/CommonStream.h"
#include "avtp/pdu/Utils.h"

static const Avtp_FieldDescriptor_t Avtp_AafCommonStreamFieldDesc[AVTP_AAF_COMMON_STREAM_FIELD_MAX] =
{
    [AVTP_AAF_COMMON_STREAM_FIELD_SUBTYPE]                      = { .quadlet = 0, .offset =  0, .bits =  8 },
    [AVTP_AAF_COMMON_STREAM_FIELD_SV]                           = { .quadlet = 0, .offset =  8, .bits =  1 }, 
    [AVTP_AAF_COMMON_STREAM_FIELD_VERSION]                      = { .quadlet = 0, .offset =  9, .bits =  3 }, 
    [AVTP_AAF_COMMON_STREAM_FIELD_MR]                           = { .quadlet = 0, .offset = 12, .bits =  1 },
    [AVTP_AAF_COMMON_STREAM_FIELD_TV]                           = { .quadlet = 0, .offset = 15, .bits =  1 }, 
    [AVTP_AAF_COMMON_STREAM_FIELD_SEQUENCE_NUM]                 = { .quadlet = 0, .offset = 16, .bits =  8 },
    [AVTP_AAF_COMMON_STREAM_FIELD_TU]                           = { .quadlet = 0, .offset = 31, .bits =  1 },
    [AVTP_AAF_COMMON_STREAM_FIELD_STREAM_ID]                    = { .quadlet = 1, .offset =  0, .bits = 64 },
    [AVTP_AAF_COMMON_STREAM_FIELD_AVTP_TIMESTAMP]               = { .quadlet = 3, .offset =  0, .bits = 32 },
    [AVTP_AAF_COMMON_STREAM_FIELD_FORMAT]                       = { .quadlet = 4, .offset =  0, .bits =  8 },
    [AVTP_AAF_COMMON_STREAM_FIELD_AAF_FORMAT_SPECIFIC_DATA_1]   = { .quadlet = 4, .offset =  8, .bits = 24 },
    [AVTP_AAF_COMMON_STREAM_FIELD_STREAM_DATA_LENGTH]           = { .quadlet = 5, .offset =  0, .bits = 16 },
    [AVTP_AAF_COMMON_STREAM_FIELD_AFSD]                         = { .quadlet = 5, .offset = 16, .bits =  3 },
    [AVTP_AAF_COMMON_STREAM_FIELD_SP]                           = { .quadlet = 5, .offset = 19, .bits =  1 }, 
    [AVTP_AAF_COMMON_STREAM_FIELD_EVT]                          = { .quadlet = 5, .offset = 20, .bits =  4 }, 
    [AVTP_AAF_COMMON_STREAM_FIELD_AAF_FORMAT_SPECIFIC_DATA_2]   = { .quadlet = 5, .offset = 24, .bits =  8 },    
};

int Avtp_AafCommonStream_GetField(Avtp_AafCommonStream_t* pdu, Avtp_AafCommonStreamFields_t field, uint64_t* value)
{
    return Avtp_GetField(Avtp_AafCommonStreamFieldDesc, AVTP_AAF_COMMON_STREAM_FIELD_MAX, (uint8_t*)pdu, (uint8_t) field, value);
}

int Avtp_AafCommonStream_SetField(Avtp_AafCommonStream_t* pdu, Avtp_AafCommonStreamFields_t field, uint64_t value)
{
    return Avtp_SetField(Avtp_AafCommonStreamFieldDesc, AVTP_AAF_COMMON_STREAM_FIELD_MAX, (uint8_t*)pdu, (uint8_t) field, value); 
}