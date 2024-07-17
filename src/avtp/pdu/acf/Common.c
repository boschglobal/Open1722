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

#include "avtp/pdu/acf/Common.h"
#include "avtp/pdu/Utils.h" 
#include "avtp/pdu/Defines.h"

/**
 * This table maps all IEEE 1722 ACF common header fields to a descriptor.
 */
static const Avtp_FieldDescriptor_t Avtp_AcfCommonFieldDesc[AVTP_ACF_COMMON_FIELD_MAX] =
{
    /* ACF common header */
    [AVTP_ACF_FIELD_ACF_MSG_TYPE]            = { .quadlet = 0, .offset = 0, .bits = 7 },
    [AVTP_ACF_FIELD_ACF_MSG_LENGTH]          = { .quadlet = 0, .offset = 7, .bits = 9 },
};

int Avtp_AcfCommon_GetField(Avtp_AcfCommon_t* acf_pdu, Avtp_AcfCommonFields_t field, uint64_t* value)
{    
    return Avtp_GetField(Avtp_AcfCommonFieldDesc, AVTP_ACF_COMMON_FIELD_MAX, (uint8_t*)acf_pdu, (uint8_t)field, value);        
}

int Avtp_AcfCommon_SetField(Avtp_AcfCommon_t* acf_pdu, Avtp_AcfCommonFields_t field, uint64_t value)
{
    return Avtp_SetField(Avtp_AcfCommonFieldDesc, AVTP_ACF_COMMON_FIELD_MAX, (uint8_t*)acf_pdu, (uint8_t)field, value);        
}
