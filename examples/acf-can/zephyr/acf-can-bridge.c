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

#include <zephyr/net/ethernet.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/socketcan.h>
#include <zephyr/net/socketcan_utils.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_l2.h>

LOG_MODULE_REGISTER(acf_can_bridge, LOG_LEVEL_DBG);

#include "avtp/Udp.h"
#include "avtp/acf/Ntscf.h"
#include "avtp/acf/Tscf.h"
#include "avtp/acf/AcfCommon.h"
#include "avtp/acf/Can.h"
#include "avtp/CommonHeader.h"
#include "acf-can-common.h"

#define THREAD_STACK_SIZE 4096
#define THREAD_PRIORITY_CAN_TO_AVTP -1
#define THREAD_PRIORITY_AVTP_TO_CAN -2

static uint8_t peer_mac[NET_ETH_ADDR_LEN];
static uint8_t dynamic_peer_mac[NET_ETH_ADDR_LEN];
static struct k_mutex dynamic_peer_mac_mutex;
static struct in_addr ip_addr;
static uint8_t use_tscf = CONFIG_ACF_CAN_BRIDGE_USE_TSCF;
static uint8_t use_udp = CONFIG_ACF_CAN_BRIDGE_USE_UDP;
static uint8_t use_dynamic_peer_mac = CONFIG_ACF_CAN_BRIDGE_USE_DYNAMIC_PEER_MAC;
static uint32_t udp_listen_port = CONFIG_ACF_CAN_BRIDGE_RECV_UDP_PORT;
static uint32_t udp_send_port = CONFIG_ACF_CAN_BRIDGE_SEND_UDP_PORT;
static uint8_t num_acf_msgs = CONFIG_ACF_CAN_BRIDGE_NUM_ACF_MSGS;
static uint64_t listener_stream_id;
static uint64_t talker_stream_id;
static Avtp_CanVariant_t can_variant = AVTP_CAN_CLASSIC;

int eth_socket = 0;
struct sockaddr* dest_addr;
struct sockaddr_ll sk_ll_addr;
struct sockaddr_in sk_udp_addr;
const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

struct sockaddr_ll src_addr = {0};
socklen_t addr_len = sizeof(src_addr);

static K_SEM_DEFINE(iface_up, 0, 1);
CAN_MSGQ_DEFINE(rx_msgq, 2);

struct k_thread can_to_avtp_thread, avtp_to_can_thread;
K_THREAD_STACK_DEFINE(can_to_avtp_stack, THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(avtp_to_can_stack, THREAD_STACK_SIZE);

static int init_can_dev()
{
    int ret;
    if  (!device_is_ready(can_dev)) {
        LOG_ERR("CAN: Device %s not ready.", can_dev->name);
        return -1;
    }
    ret = can_start (can_dev);
    if (ret != 0) {
        LOG_ERR("Error starting CAN controller [%d]", ret);
    }
    else {
        LOG_INF("Starting CAN controller [%d]", ret);
    }

    /* Let the device start before doing anything */
    k_sleep(K_SECONDS(2));
    return ret;
}

static int init_can_rx()
{
    // This is a generic receive filter that will accept all frames
    struct can_filter rx_filter={.id=1,.mask=0,.flags=0};
    int filter_id;
    filter_id = can_add_rx_filter_msgq(can_dev, &rx_msgq, &rx_filter);
    if(filter_id == -ENOSPC) {
        LOG_ERR("ENOSPC: there are no free filters");
    }
        if(filter_id == -ENOTSUP) {
        LOG_ERR("ENOTSUP: the requested filter type is not supported");
    }
    return filter_id;
}

static void iface_up_handler(struct net_mgmt_event_callback *cb,
                 uint32_t mgmt_event, struct net_if *iface)
{
    if (mgmt_event == NET_EVENT_IF_UP) {
        k_sem_give(&iface_up);
    }
}

static void wait_for_interface(void)
{
    struct net_if *iface = net_if_get_default();
    struct net_mgmt_event_callback iface_up_cb;
    if (net_if_is_up(iface)) {
        return;
    }

    net_mgmt_init_event_callback(&iface_up_cb, iface_up_handler,
                     NET_EVENT_IF_UP);
    net_mgmt_add_event_callback(&iface_up_cb);

    // Wait for the interface to come up.
    k_sem_take(&iface_up, K_FOREVER);

    net_mgmt_del_event_callback(&iface_up_cb);
}

static int create_listener_socket_udp(uint32_t udp_port) {

    int fd, res;
    struct sockaddr_in sk_addr;

    //create a UDP socket
    fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        LOG_ERR("Failed to open socket");
        return -1;
    }

    // Initialize the socket
    memset((char *) &sk_addr, 0, sizeof(sk_addr));
    sk_addr.sin_family = AF_INET;
    sk_addr.sin_port = htons(udp_port);
    sk_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    res = bind(fd, (struct sockaddr *) &sk_addr, sizeof(sk_addr));
    if (res < 0) {
        LOG_ERR("Couldn't bind() to port");
        close(fd);
        return -1;
    }

    return fd;
}

static int create_listener_socket(uint8_t* peer_mac, int protocol)
{
    int fd, res;
    struct sockaddr_ll sk_addr = {0};

    fd = socket(AF_PACKET, SOCK_DGRAM, htons(protocol));
    if (fd < 0) {
        LOG_ERR("Failed to open socket");
        return -1;
    }

    sk_addr.sll_family = AF_PACKET;
    sk_addr.sll_ifindex = net_if_get_by_iface(net_if_get_default());
    memcpy(&sk_addr.sll_addr, peer_mac, NET_ETH_ADDR_LEN);

    res = bind(fd, (struct sockaddr *) &sk_addr, sizeof(sk_addr));
    if (res < 0) {
        LOG_ERR("Couldn't bind() to interface");
        goto err;
    }

    return fd;

err:
    close(fd);
    return -1;
}

void can_to_avtp_runnable(void* p1, void* p2, void* p3) {

    uint8_t cf_seq_num = 0;
    uint32_t udp_seq_num = 0;

    uint8_t pdu[MAX_ETH_PDU_SIZE];
    uint16_t pdu_length = 0;
    frame_t can_frames[num_acf_msgs];
    int res;
    LOG_INF("Starting CAN-to-AVTP thread.");
    // Setup a socket address for sending to the peer
    if (use_udp) {
        sk_udp_addr.sin_family = AF_INET;
        res = inet_pton(AF_INET, CONFIG_ACF_CAN_BRIDGE_SEND_IP_ADDR, &ip_addr);
        if (!res) {
            LOG_ERR("Invalid IP address");
        }
        sk_udp_addr.sin_addr = ip_addr;
        sk_udp_addr.sin_port = htons(udp_send_port);
        dest_addr = (struct sockaddr*) &sk_udp_addr;
    } else {
        sk_ll_addr.sll_family = AF_PACKET;
        sk_ll_addr.sll_protocol = htons(ETH_P_TSN);
        sk_ll_addr.sll_halen = NET_ETH_ADDR_LEN;
        sk_ll_addr.sll_ifindex = net_if_get_by_iface(net_if_get_default());
        memcpy(sk_ll_addr.sll_addr, peer_mac, NET_ETH_ADDR_LEN);  // use dynamic MAC here


        dest_addr = (struct sockaddr*) &sk_ll_addr;
    }

    if (!eth_socket) {
        LOG_ERR("Ethernet socket failed. Stopping CAN-to-AVTP thread");
        return;
    }

    // Start an infinite loop to keep converting CAN frames to AVTP frames
    for(;;) {

        // Read acf_num_msgs number of CAN frames from the CAN socket
        int i = 0;
        while (i < num_acf_msgs) {
            // Get payload -- will 'spin' here until we get the requested number
            //                of CAN frames.
            res = k_msgq_get(&rx_msgq, &(can_frames[i].cc), K_FOREVER);
            if (res < 0) {
                LOG_ERR("Error reading CAN frames: %d", res);
                continue;
            }
            i++;
        }

        // Pack all the read frames into an AVTP frame
        pdu_length = can_to_avtp(can_frames, can_variant, pdu, use_udp, use_tscf,
                                    talker_stream_id, num_acf_msgs, cf_seq_num++, udp_seq_num++);

        if (use_dynamic_peer_mac) {
            k_mutex_lock(&dynamic_peer_mac_mutex, K_FOREVER);
            memcpy(sk_ll_addr.sll_addr, dynamic_peer_mac, NET_ETH_ADDR_LEN);
            k_mutex_unlock(&dynamic_peer_mac_mutex);
            dest_addr = (struct sockaddr*) &sk_ll_addr;
        }

        // Send the packed frame out over Ethernet
        if (use_udp) {
            res = sendto(eth_socket, pdu, pdu_length, 0,
                    (struct sockaddr *) dest_addr, sizeof(struct sockaddr_in));
        } else {
            res = sendto(eth_socket, pdu, pdu_length, 0,
                         (struct sockaddr *) dest_addr, sizeof(struct sockaddr_ll));
        }
        if (res < 0) {
            LOG_ERR("Failed to send data");
        }
    }

    return;
}

void avtp_to_can_runnable(void* p1, void* p2, void* p3) {

    uint16_t pdu_length = 0;
    int8_t num_can_msgs = 0;
    uint8_t exp_cf_seqnum = 0;
    uint32_t exp_udp_seqnum = 0;
    uint8_t pdu[MAX_ETH_PDU_SIZE];
    static frame_t can_frames[MAX_CAN_FRAMES_IN_ACF];
    struct zsock_pollfd fds[1];
    int ret;


    if (!eth_socket) {
        LOG_ERR("Ethernet socket failed. Stopping AVTP-to-CAN thread");
        return;
    }

    // Configure the poll fd structure
    fds[0].fd = eth_socket;
    fds[0].events = ZSOCK_POLLIN;

    // Start an infinite loop to keep converting AVTP frames to CAN frames
    for(;;) {
        LOG_INF("Waiting for data...");

        // Wait for data with a timeout (e.g., 500ms)
        ret = zsock_poll(fds, 1, 500);

        LOG_INF("Returned from poll with %d events", ret);

        if (ret < 0) {
            LOG_ERR("Poll failed: %d", errno);
            k_yield();
            continue;
        }
        else if (ret == 0) {
            // Poll timeout - no data available
            k_yield();
            continue;
        }

        // Data is available to read
        if (fds[0].revents & ZSOCK_POLLIN) {

            pdu_length = recvfrom(eth_socket, pdu, MAX_ETH_PDU_SIZE, 0,
                (struct sockaddr *)&src_addr, &addr_len);

            LOG_INF("Received %d bytes from %02x:%02x:%02x:%02x:%02x:%02x",
                pdu_length, src_addr.sll_addr[0], src_addr.sll_addr[1],
                src_addr.sll_addr[2], src_addr.sll_addr[3],
                src_addr.sll_addr[4], src_addr.sll_addr[5]);

            if (pdu_length < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    LOG_DBG("Would block, trying again later");
                    continue;
                } else {
                    LOG_ERR("Failed to receive data: %d", errno);
                    k_sleep(K_MSEC(100)); // Small delay before retry
                    continue;
                }
            } else if (pdu_length > MAX_ETH_PDU_SIZE) {
                LOG_ERR("Received packet too large (%d bytes)", pdu_length);
                continue;
            } else if (pdu_length == 0) {
                // Connection closed (unlikely for UDP but possible for other sockets)
                LOG_WRN("Connection closed");
                continue;
            }


            if (!use_udp && use_dynamic_peer_mac) {
                // Save the source MAC to dynamic_peer_mac
                k_mutex_lock(&dynamic_peer_mac_mutex, K_FOREVER);
                memcpy(dynamic_peer_mac, src_addr.sll_addr, NET_ETH_ADDR_LEN);
                k_mutex_unlock(&dynamic_peer_mac_mutex);
            }

            // Process the received data
            num_can_msgs = avtp_to_can(pdu, can_frames, can_variant, use_udp,
                                listener_stream_id, &exp_cf_seqnum, &exp_udp_seqnum);
            if (num_can_msgs <= 0) {
                continue;
            }
            exp_cf_seqnum++;
            exp_udp_seqnum++;

            // Send all extracted CAN messages to the CAN bus
            for (int8_t i = 0; i < num_can_msgs; i++) {
                int res;
                res = can_send(can_dev, &(can_frames[i].cc), K_NO_WAIT, NULL, NULL);
                if(res < 0) {
                    LOG_ERR("Failed to write to CAN bus: %d", res);
                }
            }
        }
    }
}


int main(void)
{
    int res;

    // Parse the configuration
    // Starting with Stream IDs
    char *uint64_str = CONFIG_ACF_CAN_BRIDGE_TALKER_STREAM_ID;
    talker_stream_id = strtoull(uint64_str, NULL, 16);
    uint64_str = CONFIG_ACF_CAN_BRIDGE_LISTENER_STREAM_ID;
    listener_stream_id = strtoull(uint64_str, NULL, 16);

    k_mutex_init(&dynamic_peer_mac_mutex);

    // Print current configuration
    LOG_INF("acf-can-bridge configuration:");
    if(use_tscf)
        LOG_INF("\tUsing TSCF");
    else
        LOG_INF("\tUsing NTSCF");
    if(can_variant == AVTP_CAN_CLASSIC)
        LOG_INF("\tUsing Classic CAN");
    else if(can_variant == AVTP_CAN_FD)
        LOG_INF("\tUsing CAN FD");
    if(use_udp) {
        LOG_INF("\tUsing UDP");
        LOG_INF("\tDestination IP: %s, Send port: %d, listening port: %d",
                CONFIG_ACF_CAN_BRIDGE_SEND_IP_ADDR, udp_send_port, udp_listen_port);
    } else {
        if (use_dynamic_peer_mac){
            LOG_INF("\tUsing dynamic peer mac");
            dynamic_peer_mac[0] = 0xFF;
            dynamic_peer_mac[1] = 0xFF;
            dynamic_peer_mac[2] = 0xFF;
            dynamic_peer_mac[3] = 0xFF;
            dynamic_peer_mac[4] = 0xFF;
            dynamic_peer_mac[5] = 0xFF;
        }
        LOG_INF("\tUsing Ethernet");
        res = sscanf(CONFIG_ACF_CAN_BRIDGE_PEER_MAC_ADDR, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                &peer_mac[0], &peer_mac[1], &peer_mac[2],
                &peer_mac[3], &peer_mac[4], &peer_mac[5]);
        if (res != 6) {
            LOG_ERR("Invalid MAC address");
            exit(EXIT_FAILURE);
        }
        LOG_INF("\tPeer MAC Address: %02x:%02x:%02x:%02x:%02x:%02x", peer_mac[0], peer_mac[1], peer_mac[2],
                                                        peer_mac[3], peer_mac[4], peer_mac[5]);
    }
    LOG_INF("\tListener Stream ID: 0x%llx, Talker Stream ID: 0x%llx", listener_stream_id, talker_stream_id);
    LOG_INF("\tNumber of ACF messages per AVTP frame in talker stream: %d", num_acf_msgs);

    // Open a CAN socket for reading frames
    // init CAN Dev
    res = init_can_dev();
    if (res < 0){
        LOG_ERR("Failed to init CAN device");
        return -1;
    }
    // init CAN RX
    res = init_can_rx();
    if (res < 0){
        return -1;
    }

    // Wait for the network interface to come up
    wait_for_interface();

    // Create an appropriate sockets: UDP or Ethernet raw
    // Setup the socket for sending to the peer
    if (use_udp) {
        eth_socket = create_listener_socket_udp(udp_listen_port);
    } else {
        eth_socket = create_listener_socket(peer_mac, ETH_P_TSN);
    }
    if (eth_socket < 0) return -1;

    k_tid_t t_id;
    t_id = k_thread_create(&avtp_to_can_thread, avtp_to_can_stack,
                    K_THREAD_STACK_SIZEOF(avtp_to_can_stack),
                    avtp_to_can_runnable, NULL, NULL, NULL,
                    THREAD_PRIORITY_AVTP_TO_CAN, 0, K_NO_WAIT);
    k_thread_name_set(t_id, "avtp_to_can_thread");
    t_id = k_thread_create(&can_to_avtp_thread, can_to_avtp_stack,
                    K_THREAD_STACK_SIZEOF(can_to_avtp_stack),
                    can_to_avtp_runnable, NULL, NULL, NULL,
                    THREAD_PRIORITY_CAN_TO_AVTP, 0, K_NO_WAIT);
    k_thread_name_set(t_id, "can_to_avtp_thread");

    LOG_INF("Main thread going to sleep");
    k_sleep(K_FOREVER);
    return 1;
}