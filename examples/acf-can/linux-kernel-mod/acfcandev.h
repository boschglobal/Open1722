/* acfcan.c - Virtual IEEE 1722 acf-can CAN interface
 *
 * Copyright (c) 2024 COVESA Open1722
 * All rights reserved.
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

#pragma once

#include <linux/types.h>
#include <linux/can/can-ml.h>
#include <net/net_trackers.h>
#include <linux/list.h>
#include <linux/sysfs.h>

#define IEEE1722_PROTO 0x22f0

#define TX_ENABLE (1 << 7)
#define RX_ENABLE (1 << 6)

//this is more guesswork. We need some space and it seems 
//raw can is using one int.... Not sure what would happen with
//ISO-TP and such things....
#define SKB_CB_LOCATION 4
#define SKB_CB_MINE (1 << 7)

/* Private per-device configuration */
struct acfcan_cfg
{ 
    struct list_head list; //we need a list so we can map received ethernet packets
    __u8 dstmac[6]; //send acf-can frames to this mac
    __u64 rx_streamid; //listen to this acf-can stream-id
	__u64 tx_streamid; //send acf-can frames with this stream-id
	__u8 flags;
	__u8 sequenceNum;
	__u8 canbusId;
    char ethif[IFNAMSIZ];
    struct net_device *eth_netdev; //this is the eth if used for sending and receiving
	struct net_device *can_netdev; //this is the (virtual) can if 
    netdevice_tracker tracker;
};

// get the acfcan_cfg struct from the device
#define get_acfcan_cfg(dev) ((struct acfcan_cfg *)((char *)(can_get_ml_priv(dev)) + sizeof(struct can_ml_priv)))

/* Device attribute handling */



static ssize_t dstmac_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if (count == 1 && *buf == '\n') {
		return 1;
	}
	struct net_device *net_dev = to_net_dev(dev);

	// Check if the device is up
    if (netif_running(net_dev)) {
        printk(KERN_INFO "ACF-CAN: Cannot change dst mac while device %s is up\n", net_dev->name);
        return -EBUSY; // Return an appropriate error code
    }
	struct acfcan_cfg *cfg = get_acfcan_cfg(net_dev);

	// Handle the input value here
	__u8 newmac[6] = {0,1,2,3,4,5};
	int rc = sscanf(buf,"%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",&newmac[0],&newmac[1],&newmac[2],&newmac[3],&newmac[4],&newmac[5]);
	if (rc != 6) {
		printk(KERN_INFO "ACF-CAN Can not set destination MAC for %s. to %s\n", net_dev->name,buf);
		return count;
	}
	memcpy(cfg->dstmac, newmac, 6);
	printk(KERN_INFO "ACF-CAN: Setting destination MAC for %s to %02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx \n", net_dev->name, newmac[0],newmac[1],newmac[2],newmac[3],newmac[4],newmac[5]);
	return 17;
}

static ssize_t dstmac_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "ACF-CAN: Reading destination MAC\n");
	struct net_device *net_dev = to_net_dev(dev);
	struct acfcan_cfg *cfg = get_acfcan_cfg(net_dev);

	return sprintf(buf, "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx", cfg->dstmac[0],cfg->dstmac[1],cfg->dstmac[2],cfg->dstmac[3],cfg->dstmac[4],cfg->dstmac[5]);
}

// Only store the interface name. we will check if it is valid later when the interface is opened/upped
static ssize_t ethif_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if (count == 1 && *buf == '\n') {
		return 1;
	}

	struct net_device *net_dev = to_net_dev(dev);

	// Check if the device is up
    if (netif_running(net_dev)) {
        printk(KERN_INFO "ACF-CAN: Cannot change interface while device %s is up\n", net_dev->name);
        return -EBUSY; // Return an appropriate error code
    }


	struct acfcan_cfg *cfg = get_acfcan_cfg(net_dev);

    if (count >= (IFNAMSIZ-1)) {
        printk(KERN_INFO "ACF-CAN: Interface name too long\n");
        return -EINVAL;
    }

    memcpy(cfg->ethif, buf, count);
    cfg->ethif[count] = '\0';

	printk(KERN_INFO "ACF-CAN Storing interface %s for %s\n", buf,net_dev->name);
	return count;
}

static ssize_t ethif_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device *net_dev = to_net_dev(dev);
	struct acfcan_cfg *cfg = get_acfcan_cfg(net_dev);

	if (cfg->eth_netdev) {
		return sprintf(buf, "%s", cfg->eth_netdev->name);
	} else {
		buf[0] = '\0';
		return 0;
	}
}


static ssize_t tx_streamid_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if (count == 1 && *buf == '\n') {
		return 1;
	}

	struct net_device *net_dev = to_net_dev(dev);

	// Check if the device is up
    if (netif_running(net_dev)) {
        printk(KERN_INFO "ACF-CAN: Cannot change stream id while device %s is up\n", net_dev->name);
        return -EBUSY; // Return an appropriate error code
    }


	struct acfcan_cfg *cfg = get_acfcan_cfg(net_dev);

    __u64 tx_streamid;
    int rc = sscanf(buf, "%llx", &tx_streamid);
    if (!rc) {
        printk(KERN_INFO "ACF-CAN: Invalid stream id\n");
        return -EINVAL;
    }

    cfg->tx_streamid = tx_streamid;


	printk(KERN_INFO "ACF-CAN TX streamid 0x%016llx for %s\n", tx_streamid,net_dev->name);
	return count;
}



static ssize_t rx_streamid_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if (count == 1 && *buf == '\n') {
		return 1;
	}

	struct net_device *net_dev = to_net_dev(dev);

	// Check if the device is up
    if (netif_running(net_dev)) {
        printk(KERN_INFO "ACF-CAN: Cannot change stream id while device %s is up\n", net_dev->name);
        return -EBUSY; // Return an appropriate error code
    }


	struct acfcan_cfg *cfg = get_acfcan_cfg(net_dev);

    __u64 rx_streamid;
    int rc = sscanf(buf, "%llx", &rx_streamid);
    if (!rc) {
        printk(KERN_INFO "ACF-CAN: Invalid stream id\n");
        return -EINVAL;
    }

    cfg->rx_streamid = rx_streamid;


	printk(KERN_INFO "ACF-CAN RX streamid 0x%016llx for %s\n", rx_streamid,net_dev->name);
	return count;
}

static ssize_t busid_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	if (count == 1 && *buf == '\n') {
		return 1;
	}

	struct net_device *net_dev = to_net_dev(dev);

	// Check if the device is up
    if (netif_running(net_dev)) {
        printk(KERN_INFO "ACF-CAN: Cannot change bus id while device %s is up\n", net_dev->name);
        return -EBUSY; // Return an appropriate error code
    }


	struct acfcan_cfg *cfg = get_acfcan_cfg(net_dev);

    __u8 busid;
    int rc = sscanf(buf, "%hhu", &busid);
    if (!rc) {
        printk(KERN_INFO "ACF-CAN: Invalid bus id\n");
        return -EINVAL;
    }

    cfg->canbusId = busid;


	printk(KERN_INFO "ACF-CAN setting busid to %u for %s\n", busid,net_dev->name);
	return count;
}


static ssize_t rx_streamid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device *net_dev = to_net_dev(dev);
	struct acfcan_cfg *cfg = get_acfcan_cfg(net_dev);

	return sprintf(buf, "0x%016llx", cfg->rx_streamid);
}

static ssize_t tx_streamid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device *net_dev = to_net_dev(dev);
	struct acfcan_cfg *cfg = get_acfcan_cfg(net_dev);

	return sprintf(buf, "0x%016llx", cfg->tx_streamid);
}


static ssize_t busid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device *net_dev = to_net_dev(dev);
	struct acfcan_cfg *cfg = get_acfcan_cfg(net_dev);

	return sprintf(buf, "%i", cfg->canbusId);
}


static DEVICE_ATTR_RW(dstmac);
static DEVICE_ATTR_RW(ethif);
static DEVICE_ATTR_RW(rx_streamid);
static DEVICE_ATTR_RW(tx_streamid);
static DEVICE_ATTR_RW(busid);




static struct attribute *dev_attrs[] = {
    &dev_attr_dstmac.attr,
	&dev_attr_ethif.attr,
    &dev_attr_tx_streamid.attr,
	&dev_attr_rx_streamid.attr,
	&dev_attr_busid.attr,
    NULL, /* NULL-terminated list */
};
