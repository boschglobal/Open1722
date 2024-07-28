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

#include <alloca.h>
#include <argp.h>
#include <arpa/inet.h>
#include <assert.h>
#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "avtp/CommonHeader.h"
#include "avtp/cvf/Cvf.h"
#include "avtp/cvf/H264.h"
#include "avtp/cvf/Mjpeg.h"
#include "avtp/cvf/Jpeg2000.h"
#include "common/common.h"

/******************************************************************************
 * Defines
 *****************************************************************************/

#define DEFAULT_MAX_TRANSIT_TIME_NS     0
#define DEFAULT_STREAM_ID               0xAABBCCDDEEFF0001
#define MTU_SIZE                        1500
#define BUFFER_SIZE                     (MTU_SIZE * 2)
#define ARGUMENT_CODEC_SHORT            0x80

/******************************************************************************
 * Types
 *****************************************************************************/

/**
 * This struct holds all the client state for the CVF talker application
 */
typedef struct {
    char ifname[IFNAMSIZ];
    int priority;
    uint64_t max_transit_time_ns;
    uint8_t buffer[BUFFER_SIZE];
    size_t buffer_level;

    /** End of frame relative to buffer or -1 if not EOF found. */
    ssize_t eof;

    /** Sequence counter */
    uint8_t seq_num;

    /** Fragment counter */
    uint8_t fragment_num;

    /** If set to '1' use UDP otherwise raw Ethernet as transport protocol. */
    uint8_t use_udp;

    /** Video codec used. */
    Avtp_CvfFormatSubtype_t codec;

    /** Stream ID to set in generated packets. */
    uint64_t stream_id;

    /** Socket file descriptor for sending packets. */
    int socket_fd;

    /** Socket address for sending packets. */
    struct sockaddr_ll socket_addr;

    /** Destination MAC address to use when use_udp is False. */
    uint8_t macaddr[ETH_ALEN];

    /** File descriptor from whom the input data is red. */
    int input_fd;
} cvf_talker_state_t;

/******************************************************************************
 * Variables
 *****************************************************************************/

/**
 * Options used for parsing command line arguments.
 */
static struct argp_option options[] = {
    {"dst-addr",            'd',                    "MACADDR",  0,  "Stream Destination MAC address" },
    {"ifname",              'i',                    "IFNAME",   0,  "Network Interface" },
    {"udp",                 'u',                    0,          0,  "Network Interface" },
    {"codec",               ARGUMENT_CODEC_SHORT,   "CODEC",    0,  "Codec to be used. Supported codecs are 'h264' and 'mjpeg'. Default is 'h264'." },
    { 0 }
};

/**
 * Application state.
 */
cvf_talker_state_t cvf_talker_state;

/******************************************************************************
 * Function definitions
 *****************************************************************************/

/**
 * Initialize application state
 */
static void init(int argc, char** argv);

/**
 * Parse command line arguments
 */
static error_t parse_arguments(int key, char *arg, struct argp_state *state);

/**
 * Initialize CVF headers
 *
 * @returns 0 if successful
 */
static int init_pdu(void);

/**
 * Returns pointer to start of payload within cvf_talker_state.buffer. The
 * exact start of the payload is not static and depends on the codec used.
 *
 * @return uint8_t* Pointer to payload part within cvf_talker_state.buffer
 */
static uint8_t* payload_ptr(void);

/**
 * Copy video data from input file descriptor cvf_talker_state.input_fd to the
 * buffer cvf_stalker_state.buffer.
 *
 * @return ssize_t Number of bytes red.
 */
static ssize_t fill_buffer(void);

static int is_buffer_full(void);

static ssize_t search_eof(void);

static void transmit_fragment(void);

/******************************************************************************
 * Implementations
 *****************************************************************************/

static void init(int argc, char** argv)
{
    // Set default arguments
    memset(&cvf_talker_state, 0, sizeof(cvf_talker_state_t));
    cvf_talker_state.priority = -1;
    cvf_talker_state.max_transit_time_ns = DEFAULT_MAX_TRANSIT_TIME_NS;
    cvf_talker_state.buffer_level = 0;
    cvf_talker_state.seq_num = 0;
    cvf_talker_state.fragment_num = 0;
    cvf_talker_state.use_udp = 0;
    cvf_talker_state.codec = AVTP_CVF_FORMAT_SUBTYPE_H264;
    cvf_talker_state.stream_id = DEFAULT_STREAM_ID;
    cvf_talker_state.input_fd = STDIN_FILENO;

    // Parse arguments
    struct argp argp = { options, parse_arguments };
    argp_parse(&argp, argc, argv, 0, NULL, NULL);

    // Check if all required arguments have been parsed
    if (strcmp(cvf_talker_state.ifname, "") == 0) {
        fprintf(stderr, "No ifname argument was provided (-i, --ifname)\n");
        exit(EXIT_FAILURE);
    }

    // Open socket
    cvf_talker_state.socket_fd = create_talker_socket(cvf_talker_state.priority);
    if (cvf_talker_state.socket_fd < 0) {
        // fprintf(stderr, "Failed to open socket!\n");
        exit(EXIT_FAILURE);
    }

    // Set socket address
    if (cvf_talker_state.use_udp) {
        fprintf(stderr, "UDP not supported yet!\n");
        exit(EXIT_FAILURE);
    } else {
        int res = setup_socket_address(
                cvf_talker_state.socket_fd,
                cvf_talker_state.ifname,
                cvf_talker_state.macaddr,
                ETH_P_TSN,
                &cvf_talker_state.socket_addr);
        if (res < 0) {
            fprintf(stderr, "Failed to set socket address!\n");
            exit(EXIT_FAILURE);
        }
    }
}

static error_t parse_arguments(int key, char *arg, struct argp_state *state)
{
    int res;
    switch (key) {
    case 'd':
        res = sscanf(arg,
                "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                &cvf_talker_state.macaddr[0],
                &cvf_talker_state.macaddr[1],
                &cvf_talker_state.macaddr[2],
                &cvf_talker_state.macaddr[3],
                &cvf_talker_state.macaddr[4],
                &cvf_talker_state.macaddr[5]);
        if (res != 6) {
            fprintf(stderr, "Invalid address\n");
            exit(EXIT_FAILURE);
        }
        break;
    case 'i':
        strncpy(cvf_talker_state.ifname, arg, sizeof(cvf_talker_state.ifname) - 1);
        break;
    case 'm':
        cvf_talker_state.max_transit_time_ns = atoi(arg);
        break;
    case 'p':
        cvf_talker_state.priority = atoi(arg);
        break;
    case 'u':
        cvf_talker_state.use_udp = 1;
        break;
    case ARGUMENT_CODEC_SHORT:
        if (strcmp(arg, "h264") == 0) {
            cvf_talker_state.codec = AVTP_CVF_FORMAT_SUBTYPE_H264;
        // } else if (strcmp(arg, "mjpeg") == 0) {
        //     cvf_talker_state.codec = AVTP_CVF_FORMAT_SUBTYPE_MJPEG;
        // } else if (strcmp(arg, "jpeg2000") == 0) {
        //     cvf_talker_state.codec = AVTP_CVF_FORMAT_SUBTYPE_JPEG2000;
        } else {
            fprintf(stderr, "Codec not supported!\n");
            exit(EXIT_FAILURE);
        }
        break;
    }
    return 0;
}

static int init_pdu(void)
{
    Avtp_Cvf_t* cvf = (Avtp_Cvf_t*)cvf_talker_state.buffer;

    Avtp_Cvf_Init(cvf);
    Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_FORMAT_SUBTYPE, AVTP_CVF_FORMAT_SUBTYPE_H264);
    Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_FORMAT, AVTP_CVF_FORMAT_RFC);
    Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_TV, 1);
    Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_STREAM_ID, cvf_talker_state.stream_id);
    Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_M, 1);
    Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_PTV, 0);

    switch (cvf_talker_state.codec) {
    case AVTP_CVF_FORMAT_SUBTYPE_H264:
        Avtp_H264_t* h264 = (Avtp_H264_t*)(&cvf->payload);
        Avtp_H264_Init(h264);
        Avtp_H264_SetField(h264, AVTP_H264_FIELD_TIMESTAMP, 0);
        break;
    default:
        fprintf(stderr, "Codec not supported!\n");
        exit(EXIT_FAILURE);
        break;
    }

    cvf_talker_state.buffer_level = (size_t)(payload_ptr() - cvf_talker_state.buffer);

    return 0;
}

static uint8_t* payload_ptr(void)
{
    uint8_t* result = cvf_talker_state.buffer + sizeof(Avtp_Cvf_t);
    switch (cvf_talker_state.codec) {
    case AVTP_CVF_FORMAT_SUBTYPE_H264:
        result += sizeof(Avtp_H264_t);
        break;
    case AVTP_CVF_FORMAT_SUBTYPE_MJPEG:
        result += sizeof(Avtp_Mjpeg_t);
        break;
    case AVTP_CVF_FORMAT_SUBTYPE_JPEG2000:
        result += sizeof(Avtp_Jpeg2000_t);
        break;
    default:
        fprintf(stderr, "Codec not supported!\n");
        exit(EXIT_FAILURE);
        break;
    }
    return result;
}

static ssize_t fill_buffer(void)
{
    // TODO
    return 0;
}

static int is_buffer_full(void)
{
    // TODO
    return 1;
}

static ssize_t search_eof(void)
{
    // TODO
    return 0;
}

int main(int argc, char** argv)
{
    init(argc, argv);
    while (1) {
        fill_buffer();
        if (is_buffer_full() || )
    }
    return EXIT_SUCCESS;
}
