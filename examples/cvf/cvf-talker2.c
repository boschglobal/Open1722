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

#include "avtp/Udp.h"
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
#define DEFAULT_PORT                    17220
#define DEFAULT_STREAM_ID               0xAABBCCDDEEFF0001
#define MTU_SIZE                        1500
#define BUFFER_SIZE                     (MTU_SIZE * 2)
// #define H264_NAL_PREFIX                 ((uint8_t[]){0x0, 0x0, 0x0, 0x1})
#define CMD_ARG_CODEC_SHORT             0x80

/******************************************************************************
 * Types
 *****************************************************************************/

/**
 * This struct holds all the client state for the CVF talker application
 */
typedef struct {
    /** Interface name (used in combination with raw Ethernet encapsulation). */
    char ifname[IFNAMSIZ];

    /** Socket priority. */
    int priority;

    /** Maximum transmission time in network in nanoseconds. */
    uint64_t max_transit_time_ns;

    /** Buffer for constructing frames. */
    uint8_t buffer[BUFFER_SIZE];

    /** Number of bytes in buffer that are in use. */
    size_t buffer_level;

    /** End of frame relative to buffer or -1 if not EOF found. */
    ssize_t eof;

    /** Sequence counter. */
    uint8_t seq_num;

    /** Fragment counter. */
    uint8_t fragment_num;

    /** If set to '1' use UDP otherwise raw Ethernet as transport protocol. */
    uint8_t use_udp;

    /** Video codec used. */
    Avtp_CvfFormatSubtype_t codec;

    /** Stream ID to set in generated packets. */
    uint64_t stream_id;

    /** Socket file descriptor for sending packets. */
    int socket_fd;

    /** Socket address for sending UDP packets. */
    struct sockaddr_in sk_udp_addr;

    /** Socket address for sending link layer packets. */
    struct sockaddr_ll sk_ll_addr;

    /** Destination MAC address to use when use_udp is False. */
    uint8_t mac_addr[ETH_ALEN];

    /** Destination IPv4 address to use when use_udp is True. */
    struct in_addr ip_addr;

    /** Destination port if \ref cvf_talker_state_t.use_udp is True */
    uint16_t port;

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
    
    {"udp",                 'u',                    0,          0,  "Network Interface" },
    {"dst-addr",            'd',                    "mac_addr",  0,  "Stream Destination MAC address" },
    {"ifname",              'i',                    "IFNAME",   0,  "Network Interface" },
    {"codec",               CMD_ARG_CODEC_SHORT,    "CODEC",    0,  "Codec to be used. Supported codecs are 'h264' and 'mjpeg'. Default is 'h264'." },
    { 0 }
};

/******************************************************************************
 * Function definitions
 *****************************************************************************/

/**
 * Initialize application state
 */
static void init(cvf_talker_state_t* app_state, int argc, char** argv);

/**
 * Parse command line arguments
 */
static error_t parse_arguments(int key, char *arg, struct argp_state *argp_state);

/**
 * Initialize CVF headers
 *
 * @returns 0 if successful
 */
int init_pdu(cvf_talker_state_t* app_state);

static size_t header_size(cvf_talker_state_t* app_state);

/**
 * Copy video data from input file descriptor state->input_fd to the
 * buffer cvf_stalker_state.buffer.
 *
 * @return ssize_t Number of bytes red.
 */
static ssize_t fill_buffer(cvf_talker_state_t* app_state);

static int is_buffer_full(cvf_talker_state_t* app_state);

ssize_t search_eof(cvf_talker_state_t* app_state, size_t sof);

static ssize_t search_sof_h264(cvf_talker_state_t* app_state, size_t offset);

// static void transmit_fragment(cvf_talker_state_t* app_state);

void prepare_pdu(cvf_talker_state_t* app_state, size_t eof);

void transmit_pdu(cvf_talker_state_t* app_state, size_t eof);

void shift_buffer(cvf_talker_state_t* app_state, size_t eof);

/******************************************************************************
 * Implementations
 *****************************************************************************/

static void init(cvf_talker_state_t* app_state, int argc, char** argv)
{
    // Set default arguments
    memset(app_state, 0, sizeof(cvf_talker_state_t));
    app_state->priority = -1;
    app_state->max_transit_time_ns = DEFAULT_MAX_TRANSIT_TIME_NS;
    app_state->buffer_level = 0;
    app_state->seq_num = 0;
    app_state->fragment_num = 0;
    app_state->use_udp = 0;
    app_state->codec = AVTP_CVF_FORMAT_SUBTYPE_H264;
    app_state->stream_id = DEFAULT_STREAM_ID;
    app_state->port = DEFAULT_PORT;
    app_state->input_fd = STDIN_FILENO;

    // Parse arguments
    struct argp argp = { options, parse_arguments };
    argp_parse(&argp, argc, argv, 0, NULL, app_state);

    // Check if all required arguments have been parsed
    if (strcmp(app_state->ifname, "") == 0) {
        fprintf(stderr, "No ifname argument was provided (-i, --ifname)\n");
        exit(EXIT_FAILURE);
    }

    // Create socket
    if (app_state->use_udp) {
        app_state->socket_fd = create_talker_socket_udp(app_state->priority);
    } else {
        app_state->socket_fd = create_talker_socket(app_state->priority);
    }
    if (app_state->socket_fd < 0) {
        fprintf(stderr, "Failed to setup socket!\n");
        exit(EXIT_FAILURE);
    }

    // Setup socket target address
    int rc;
    if (app_state->use_udp) {
        rc = setup_udp_socket_address(
                &app_state->ip_addr,
                app_state->port,
                &app_state->sk_udp_addr);
    } else {
        rc = setup_socket_address(
                app_state->socket_fd,
                app_state->ifname,
                app_state->mac_addr,
                ETH_P_TSN,
                &app_state->sk_ll_addr);
    }
    if (rc < 0) {
        fprintf(stderr, "Failed to setup socket address!\n");
        exit(EXIT_FAILURE);
    }

    init_pdu(app_state);
}

static error_t parse_arguments(int key, char *arg, struct argp_state *argp_state)
{
    cvf_talker_state_t* app_state = argp_state->input;

    int res;
    switch (key) {
    case 'd':
        res = sscanf(arg,
                "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                &app_state->mac_addr[0],
                &app_state->mac_addr[1],
                &app_state->mac_addr[2],
                &app_state->mac_addr[3],
                &app_state->mac_addr[4],
                &app_state->mac_addr[5]);
        if (res != 6) {
            fprintf(stderr, "Invalid address\n");
            exit(EXIT_FAILURE);
        }
        break;
    case 'i':
        strncpy(app_state->ifname, arg, IFNAMSIZ);
        break;
    case 'm':
        app_state->max_transit_time_ns = atoi(arg);
        break;
    case 'p':
        app_state->priority = atoi(arg);
        break;
    case 'u':
        app_state->use_udp = 1;
        break;
    case CMD_ARG_CODEC_SHORT:
        if (strcmp(arg, "h264") == 0) {
            app_state->codec = AVTP_CVF_FORMAT_SUBTYPE_H264;
        // } else if (strcmp(arg, "mjpeg") == 0) {
        //     app_state->codec = AVTP_CVF_FORMAT_SUBTYPE_MJPEG;
        // } else if (strcmp(arg, "jpeg2000") == 0) {
        //     app_state->codec = AVTP_CVF_FORMAT_SUBTYPE_JPEG2000;
        } else {
            fprintf(stderr, "Codec not supported!\n");
            exit(EXIT_FAILURE);
        }
        break;
    }
    return 0;
}

int init_pdu(cvf_talker_state_t* app_state)
{
    Avtp_Cvf_t* cvf;
    if (app_state->use_udp) {
        cvf = (Avtp_Cvf_t*)(app_state->buffer + AVTP_UDP_HEADER_LEN);
    } else {
        cvf = (Avtp_Cvf_t*)app_state->buffer;
    }

    Avtp_Cvf_Init(cvf);
    Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_FORMAT, AVTP_CVF_FORMAT_RFC);
    Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_TV, 1);
    Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_STREAM_ID, app_state->stream_id);
    Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_M, 1);
    Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_PTV, 0);

    switch (app_state->codec) {
    case AVTP_CVF_FORMAT_SUBTYPE_H264:
        Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_FORMAT_SUBTYPE, AVTP_CVF_FORMAT_SUBTYPE_H264);
        Avtp_H264_t* h264 = (Avtp_H264_t*)(&cvf->payload);
        Avtp_H264_Init(h264);
        Avtp_H264_SetField(h264, AVTP_H264_FIELD_TIMESTAMP, 0);
        break;
    default:
        fprintf(stderr, "Codec not supported!\n");
        exit(EXIT_FAILURE);
        break;
    }

    app_state->buffer_level = header_size(app_state);

    return 0;
}

static size_t header_size(cvf_talker_state_t* app_state)
{
    size_t result = sizeof(Avtp_Cvf_t);
    switch (app_state->codec) {
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

static ssize_t fill_buffer(cvf_talker_state_t* app_state)
{
    ssize_t n;
    n = read(app_state->input_fd,
            app_state->buffer + app_state->buffer_level,
            BUFFER_SIZE - app_state->buffer_level);
    if (n < 0) {
        fprintf(stderr, "Could not read from input file descriptor!\n");
        exit(EXIT_FAILURE);
    }
    app_state->buffer_level += n;
    return n;
}

static int is_buffer_full(cvf_talker_state_t* app_state)
{
    return app_state->buffer_level >= BUFFER_SIZE;
}

ssize_t search_eof(cvf_talker_state_t* app_state, size_t sof)
{
    if (app_state->codec == AVTP_CVF_FORMAT_SUBTYPE_H264) {
        return search_sof_h264(app_state, sof + 1);
    } else {
        fprintf(stderr, "Codec not supported!\n");
        exit(EXIT_FAILURE);
    }
}

static ssize_t search_sof_h264(cvf_talker_state_t* app_state, size_t offset)
{
    uint8_t* buf = app_state->buffer;
    while (offset < app_state->buffer_level - 2) {
        if (buf[offset + 2] == 0x1) {
            if (buf[offset + 1] == 0x0 && buf[offset] == 0x0) {
                return offset;
            } else {
                offset += 3;
            }
        } else if (buf[offset + 2] == 0x0) {
            offset++;
        } else {
            offset += 3;
        }
    }
    return -1;
}

void prepare_pdu(cvf_talker_state_t* app_state, size_t eof)
{
    int rc = 0;

    Avtp_Cvf_t* cvf;
    if (app_state->use_udp) {
        cvf = (Avtp_Cvf_t*)(app_state->buffer + AVTP_UDP_HEADER_LEN);
    } else {
        cvf = (Avtp_Cvf_t*)app_state->buffer;
    }

    // Sequence number
    uint64_t seq_num;
    Avtp_Cvf_GetField(cvf, AVTP_CVF_FIELD_SEQUENCE_NUM, &seq_num);
    seq_num = (uint8_t)seq_num + 1;
    Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_SEQUENCE_NUM, seq_num);

    // Stream data length
    uint16_t stream_data_length = eof - header_size(app_state);
    Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_STREAM_DATA_LENGTH, stream_data_length);

    // AVTP timestamp
    uint32_t avtp_time = 0;
    rc = calculate_avtp_time(&avtp_time, app_state->max_transit_time_ns);
    if (rc == 0) {
        Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_AVTP_TIMESTAMP, avtp_time);
        Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_TV, 1);
    } else {
        Avtp_Cvf_SetField(cvf, AVTP_CVF_FIELD_TV, 0);
    }
}

void transmit_pdu(cvf_talker_state_t* app_state, size_t eof)
{
    int rc = 0;
    if (app_state->use_udp) {
        rc = sendto(
                app_state->socket_fd,
                app_state->buffer,
                eof,
                0,
                (struct sockaddr*)&app_state->sk_udp_addr,
                sizeof(app_state->sk_udp_addr));
    } else {
        rc = sendto(
                app_state->socket_fd,
                app_state->buffer,
                eof,
                0,
                (struct sockaddr *)&app_state->sk_ll_addr,
                sizeof(app_state->sk_ll_addr));
    }
    if (rc < 0) {
        fprintf(stderr, "Failed to send data!\n");
        exit(EXIT_FAILURE);
    }
}

void shift_buffer(cvf_talker_state_t* app_state, size_t eof)
{
    const size_t hsize = header_size(app_state);
    assert(eof >= hsize);
    memmove(
            app_state->buffer + hsize,
            app_state->buffer + eof,
            app_state->buffer_level - eof);
    app_state->buffer_level = app_state->buffer_level - eof + hsize;
}

int main(int argc, char** argv)
{
    cvf_talker_state_t app_state;
    init(&app_state, argc, argv);

    ssize_t sof, eof;
    const size_t hsize = header_size(&app_state);
    int i = 0;
    while (1) {
        fill_buffer(&app_state);
        sof = hsize;
        eof = search_eof(&app_state, sof);

        int num_nal = 1;
        // while (1) {
        //     ssize_t newEof = search_eof(&app_state, eof);
        //     if (newEof <= MTU_SIZE && eof > 0) {
        //         sof = eof;
        //         eof = newEof;
        //         num_nal++;
        //     } else {
        //         break;
        //     }
        // }

        if (eof >= 0) {
            // printf("Pre buffer level %ld\n", app_state.buffer_level);
            printf("Send frame #%d numNal=%d eof=%ld payloadLen=%ld\n", i++, num_nal, eof, eof-hsize);
            // if (i > 996 && i <= 1000) {
            //     dump_mem(app_state.buffer + header_size(&app_state), eof-hsize);
            // }
            prepare_pdu(&app_state, eof);
            transmit_pdu(&app_state, eof);
            shift_buffer(&app_state, eof);
            // printf("Post buffer level %ld\n", app_state.buffer_level);
        }
        if (i == 10) {
            break;
        }
        
        // while (eof != -1 && eof < BUFFER_SIZE - 1) {
        //     // TODO send fragment
        //     printf("Send frame sof=%ld eof=%ld\n", sof, eof);

        //     // TODO do fragmentation
        //     sof = eof + 1;
        //     eof = search_eof(&app_state, eof);
        // }
        // if (eof == -1) {
        //     fprintf(stderr, "Not enough buffer space!\n");
        //     exit(EXIT_FAILURE);
        // }
    }

    return EXIT_SUCCESS;
}
