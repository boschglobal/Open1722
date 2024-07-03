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
 * @file TODO
 */

#pragma once

#include <stdint.h>

/**
 * Identifiers for commands use within the middleware.
 */
typedef enum {
    /* Register Endpoint */
    AVTP_CMD_REGISTER                   = 0x0,
    AVTP_CMD_REGISTER_RESPONSE          = 0x1,
    /* Unregister Endpoint */
    AVTP_CMD_UNREGISTER                 = 0x2,
    AVTP_CMD_UNREGISTER_RESPONSE        = 0x3,
    /* Transfer data from/to Endpoint */
    AVTP_CMD_DATA                       = 0x4,
    /* Ping an Endpoint (e.g. to check if endpoint in separate process is still alive) */
    AVTP_CMD_PING                       = 0x5,
    AVTP_CMD_PING_RESPONSE              = 0x6,
    /* Query the capabilities and status of an Endpoint */
    AVTP_CMD_QUERY                      = 0x7,
    AVTP_CMD_QUERY_RESPONSE             = 0x8,
    /* Change the configuration of an Endpoint */
    AVTP_CMD_CONFIG                     = 0x9,
    AVTP_CMD_CONFIG_RESPONSE            = 0x10,
} Avtp_CmdType_t;

/**
 * <p>Common header shared by all commands exchanged within the middleware.</p>
 */
typedef struct {
    uint16_t cmdType;
    uint8_t payload[0];
} Avtp_Cmd_t;

/**
 * <p>Base header used by most commands exchanged within the middleware except
 * e.g. the register/unregister commands.</p>
 */
typedef struct {
    uint16_t cmdType;
    uint16_t srcEndpointId;
    uint16_t dstEndpointId;
    uint8_t payload[0];
} Avtp_BaseCmd_t;

typedef struct {
    uint16_t cmdType;
} Avtp_RegisterCmd_t;

typedef struct {
    uint16_t cmdType;
    bool result;
} Avtp_RegisterResponseCmd_t;

typedef struct {
    uint16_t cmdType;
} Avtp_UnregisterCmd_t;

typedef struct {
    uint16_t cmdType;
    bool result;
} Avtp_UnregisterResponseCmd_t;

typedef struct {
    Avtp_BaseCmd_t baseCmd;
    uint8_t payload[0];
} Avtp_DataCmd_t;

typedef struct {
    Avtp_BaseCmd_t baseCmd;
} Avtp_PingCmd_t;

typedef struct {
    Avtp_BaseCmd_t baseCmd;
} Avtp_PingResponseCmd_t;

typedef struct {
    Avtp_BaseCmd_t baseCmd;
} Avtp_QueryCmd_t;

typedef struct {
    Avtp_BaseCmd_t baseCmd;
    uint8_t payload[0];
} Avtp_QueryResponseCmd_t;

typedef struct {
    Avtp_BaseCmd_t baseCmd;
    uint8_t payload[0];
} Avtp_ConfigCmd_t;

typedef struct {
    Avtp_BaseCmd_t baseCmd;
} Avtp_ConfigResponseCmd_t;
