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

#include "avtp/mw/Cmd.h"

typedef struct {
    // TODO
} Avtp_CoreCfg_t;

typedef struct {
    // TODO
} Avtp_Core_t;

/**
 * <p>Initializes an instance of a middleware core.</p>
 * 
 * <p>This function might contain hardware dependent code and might be modified
 * for compatibility with different architectures or operating systems.</p>
 */
void Avtp_Core_Init(Avtp_Core_t* core, Avtp_CoreCfg_t* cfg);

/**
 * <p>This callback function is called whenever the core receives a frame from
 * one the network interfaces.</p>
 * 
 * <p>This function is expected to be called from a hardware specific interrupt
 * handler.</p>
 */
void Avtp_Core_OnRecvFromNetwork(Avtp_Core_t* core, uint8_t* frame, uint16_t len, uint16_t interfaceIdx);

/**
 * <p>This function is called by the Core to deliver data to the network</p>
 * 
 * <p>This function might contain hardware dependent code and might be modified
 * for compatibility with different architectures or operating systems.</p>
 */
void Avtp_Core_TransmitToNetwork(Avtp_Core_t* core, uint8_t* frame, uint16_t len, uint16_t interfaceIdx);

/**
 * <p>Registers an Endpoint at the Core and assigns it a new Endpoint ID.</p>
 */
int32_t Avtp_Core_RegisterEndpoint(Avtp_Core_t* core, Avtp_RegisterCmd_t* registerCmd);

/**
 * <p></p>
 */
void Avtp_Core_OnRecvFromEndpoint(Avtp_Core_t* core, Avtp_Cmd_t* msg, int32_t endpointId);

void Avtp_Core_TransmitToEndpoint(Avtp_Core_t* core, Avtp_Cmd_t* msg, uint16_t endpointId);

// void Avtp_Core_HandleCmd(Avtp_Core_t* core, Avtp_Cmd_t* msg);

void Avtp_Core_Run(Avtp_Core_t* core);
