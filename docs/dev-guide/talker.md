# Implementing a Talker Application

This chapter contains guidelines for implementing talker applications.

## Command Line Arguments for Talker Applications

To keep talker applications consistent, all talker application should aim to reuse the following command line arguments (this does not mean that a talker application must implement all of these arguments).

``` txt
-h, --help
-u, --udp
-d, --dst-address
-i, --ifname
-s, --stream-id
-b, --bus-id
-p, --prio
-m, --max-transit-time
-t, --timeout
-c, --count
--tscf
--ntscf
```

### -h, --help

``` txt
required: False
```

If this argument is set, the application shall provide an overview on all available command line arguments and immediately terminate. 

### -u, --udp

``` txt
required: False
```

If this flag is set, the application shall use IEEE1722's UDP encapsulation to transport the data over the network.

### -d, --dst-address

``` txt
type: String
required: True
```

If the `-u` flag is set, this argument is used to set the destination IP address and port number in the format `127.0.0.1:100`. If the `-u` flag is not set, this argument is used to set a MAC address with the format `00:11:22:33:44:55`.

### -i, --ifname

``` txt
type: String
required: <if -u flag is set>
```

If the `-u` flag is not used and the talker transmits data on raw ethernet, this argument is used to specify the name of the Ethernet interface to use.

### -s, --stream-id

``` txt
type: String
required: False
default: 0
```

This argument is used to configure the stream ID used for outgoing traffic and the stream ID to filter incoming traffic. Valid formats are either `<stream-id-out>` or `<stream-id-in>:<stream-id-out>`. If no stream ID for incoming traffic is provided the application should accept all stream IDs, if one is provided the application shall filter out any incoming traffic that does not match the `<stream-id-in>`.

Having the option to filter for a stream ID on ingress is necessary to run multiple listener applications in parallel.

### -b, --bus-id

### -p, --prio

``` txt
type: int
required: False
default: 0
```

Socket priority.

### -m, --max-transit-time

### -t, --timeout

### -c, --count

### -f, --format

