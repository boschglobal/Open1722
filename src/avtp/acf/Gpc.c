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

#include "avtp/acf/Common.h"
#include "avtp/acf/Gpc.h"
#include "avtp/Utils.h" 
#include "avtp/Defines.h"

/**
 * This table maps all IEEE 1722 ACF GPC header fields to a descriptor.
 */
static const Avtp_FieldDescriptor_t Avtp_GpcFieldDesc[AVTP_GPC_FIELD_MAX] =
{
    /* ACF common header fields */
    [AVTP_GPC_FIELD_ACF_MSG_TYPE]       = { .quadlet = 0, .offset =  0, .bits = 7 },
    [AVTP_GPC_FIELD_ACF_MSG_LENGTH]     = { .quadlet = 0, .offset =  7, .bits = 9 },  
    /* ACF GPC header fields */
    [AVTP_GPC_FIELD_GPC_MSG_ID]         = { .quadlet = 0, .offset =  16, .bits = 48 },
};

int Avtp_Gpc_Init(Avtp_Gpc_t* gpc_pdu)
{
    if(!gpc_pdu) {
        return -EINVAL;
    }

    memset(gpc_pdu, 0, sizeof(Avtp_Gpc_t));  
    Avtp_Gpc_SetField(gpc_pdu, AVTP_GPC_FIELD_ACF_MSG_TYPE, AVTP_ACF_TYPE_GPC);

    return 0;
}

int Avtp_Gpc_GetField(Avtp_Gpc_t* gpc_pdu, 
                            Avtp_GpcFields_t field, uint64_t* value)
{    
    return Avtp_GetField(Avtp_GpcFieldDesc, AVTP_GPC_FIELD_MAX, (uint8_t *) gpc_pdu, (uint8_t) field, value);
}

int Avtp_Gpc_SetField(Avtp_Gpc_t* gpc_pdu, 
                            Avtp_GpcFields_t field, uint64_t value)
{    
    return Avtp_SetField(Avtp_GpcFieldDesc, AVTP_GPC_FIELD_MAX, (uint8_t *) gpc_pdu, (uint8_t) field, value);
}