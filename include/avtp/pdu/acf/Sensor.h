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
 * This file contains the fields descriptions of the IEEE 1722 ACF Sensor PDUs and
 * functions to invoke corresponding parser and deparser.
 */

#pragma once

#include <stdint.h>

#include "avtp/pdu/Defines.h"
#include "avtp/pdu/acf/Common.h"

#define AVTP_SENSOR_HEADER_LEN         (3 * AVTP_QUADLET_SIZE)

typedef struct {
    uint8_t header[AVTP_SENSOR_HEADER_LEN];
    uint8_t payload[0];
} Avtp_Sensor_t;

typedef enum {

    /* ACF common header fields */
    AVTP_SENSOR_FIELD_ACF_MSG_TYPE = 0,
    AVTP_SENSOR_FIELD_ACF_MSG_LENGTH,

    /* ACF Sensor header fields */    
    AVTP_SENSOR_FIELD_MTV,
    AVTP_SENSOR_FIELD_NUM_SENSOR,
    AVTP_SENSOR_FIELD_SZ,
    AVTP_SENSOR_FIELD_SENSOR_GROUP,
    AVTP_SENSOR_FIELD_MESSAGE_TIMESTAMP,        

    /* Count number of fields for bound checks */
    AVTP_SENSOR_FIELD_MAX
} Avtp_SensorFields_t;

/**
 * Initializes an ACF Sensor PDU header as specified in the IEEE 1722 Specification.
 *
 * @param pdu Pointer to the first bit of a 1722 ACF Sensor PDU.
 */
int Avtp_Sensor_Init(Avtp_Sensor_t* sensor_pdu);

/**
 * Returns the value of an an ACF Sensor PDU field as specified in the IEEE 1722 Specification.
 *
 * @param pdu Pointer to the first bit of an 1722 ACF Sensor PDU.
 * @param field Specifies the position of the data field to be read
 * @param value Pointer to location to store the value.
 * @returns This function returns 0 if the data field was successfully read from
 * the 1722 ACF Sensor PDU.
 */
int Avtp_Sensor_GetField(Avtp_Sensor_t* sensor_pdu, Avtp_SensorFields_t field, uint64_t* value);

/**
 * Sets the value of an an ACF Sensor PDU field as specified in the IEEE 1722 Specification.
 *
 * @param pdu Pointer to the first bit of an 1722 ACF Sensor PDU.
 * @param field Specifies the position of the data field to be read
 * @param value Pointer to location to store the value.
 * @returns This function returns 0 if the data field was successfully set in
 * the 1722 ACF Sensor PDU.
 */
int Avtp_Sensor_SetField(Avtp_Sensor_t* sensor_pdu, Avtp_SensorFields_t field, uint64_t value);