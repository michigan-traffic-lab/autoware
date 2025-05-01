* **TCP Driver**

A package that encapsulates basic receiving and sending of TCP data.

(Some functions are specialized for Hesai LiDAR specifications.)

Provided within this package also is the `tcp_driver` library without the ROS2 dependencies, which could be used
elsewhere.

An example of usage for Hesai is as follows.

1. Create `TcpDriver` with `boost::asio::io_context`.
1. Initialize by calling `init_socket` with each parameter.
1. Open the socket with `->open()`.
1. Call `asyncSendReceiveHeaderPayload` with callbacks for the header and payload data, and
   post process.
1. Call `run` of `boost::asio::io_context` given at initialization.

* **HTTP Client Driver**

A package that encapsulates basic receiving and sending of HTTP data.

Provided within this package also is the `http_client_driver` library without the ROS2 dependencies, which could be used
elsewhere.

An example of usage is as follows.

1. Create `HttpClientDriver` with `boost::asio::io_context`.
1. Initialize by calling `init_client` with each parameter.
1. Open the client with `->client()->open()`.
1. Call `asyncGet` or `asyncPost` with a callback for the received body.
1. Call `run` of `boost::asio::io_context` given at initialization.
