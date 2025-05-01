tcp_driver
===============

This is the design document for the `tcp_driver` package.

# Purpose / Use cases

<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

The primary objective of the `tcp_driver` package is to provide wrappers for the
Transmission Control Protocol (TCP) and the Hyper Text Transport Protocol (HTTP)
to allow reliable, ordered delivery of a stream of octets (bytes) between applications
running on hosts (computers) and other devices in a network such as sensors.

The TCP and HTTP protocols are commonly used by sensors to allow configuration and to obtain status reports.

# Design

<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

The TCP and HTTP classes allow the synchronous and asynchronous reception of packets.

Both wrappers are implemented using the Boost.Asio library.

## Assumptions / Known limits

<!-- Required -->

It is assumed that the source of TCP, and HTTP packets might be unreliable. For that reason, the
wrappers implement a timeout option to allow the reconnection and recovery of the connection.

It is also assumed that the output type of the driver is a ROS 2 message.

The HTTP client only implements the GET and POST commands.
PUT and DELETE commands are currently not implemented.

# Inputs / Outputs / API

<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

## TCP and HTTP drivers

1. Instantiate a TcpDriver
2. Initiate the Socket using the `init_socket` method, setting the Host and device IP/Port to bind.
3. Open the socket using the `open` method.
4. Verify the connection is alive the `stopped` method inside Boost's `GetIOContext()`.
5. Restart eh connection if neccesary using the `restart` method inside Boost's `GetIOContext()`.
6. Use the `send` or the `asyncSend` methods to transfer data.
7. Optionally use `asyncSendReceiveHeaderPayload` to achieve transmission/reception.
7. Receive data using the `getPayload` method.

# Error detection and handling

<!-- Required -->

Errors in each of the methods are handled using try/catch and returned as a Status object.

# Related projects

The [Nebula](https://github.com/tier4/nebula) project uses both TCP and HTTP drivers to allow diagnostics acquisition
and sensor configuration for multiple Lidar sensors.
