// Copyright 2022 MAP IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TCP_DRIVER__TCP_DRIVER_HPP_
#define TCP_DRIVER__TCP_DRIVER_HPP_

#include <iostream>
#include <memory>
#include <string>

#include "tcp_socket.hpp"

namespace drivers
{
namespace tcp_driver
{

class TcpDriver
{
  /**
           * @brief Functor for TCP message processing
           *
           */
  using Functor = std::function<void (const std::vector<uint8_t> &)>;

public:
  /**
           * @brief TcpDriver constructor
           *
           * @param ctx io_context used for each process
           */
  explicit TcpDriver(std::shared_ptr<boost::asio::io_context> ctx);

  /**
           * @brief TcpDriver destructor
           *
           */
  ~TcpDriver();

  /**
           * @brief TcpSocket initialization with target information
           *
           * @param ip Device IP address
           * @param port Device TCP port number
           */
  void init_socket(const std::string & ip, uint16_t port);

  /**
           * @brief TcpSocket initialization with all information
           *
           * @param remote_ip Device IP address
           * @param remote_port Device TCP port number
           * @param host_ip PC IP address
           * @param host_port PC TCP port number
           */
  void init_socket(
    const std::string & remote_ip, uint16_t remote_port,
    const std::string & host_ip, uint16_t host_port);

  /**
           * @brief Get TcpSocket
           *
           * @return std::shared_ptr<TcpSocket> shared_ptr of the socket
           */
  std::shared_ptr<TcpSocket> socket() const;

  /**
           * @brief Open the socket with set parameters
           *
           * @return true if the connect function was successful
           * @return false if the connect function was not successful
           */
  bool open();

  /**
           * @brief Close the socket (sync)
           *
           */
  void closeSync();

  /**
           * @brief Close the socket
           *
           */
  void close();

  /**
           * @brief Check the socket open/close
           *
           * @return true if the socket is open
           * @return false if the socket is closed
           */
  bool isOpen() const;

  /**
           * @brief syncSend data and syncReceive with callbacks
           *
           * @param buff Data vector
           * @param func_header Callback of syncSendHandler for header data
           * @param func_payload Callback of syncSendHandler for payload data
           * @param func_finally Callback to process after func_payload
           * @return true if the sync send function was successful
           * @return false if the sync send function was not successful
           */
  bool
  syncSendReceiveHeaderPayload(
    std::vector<unsigned char> & buff, Functor func_header, Functor func_payload,
    std::function<void()> func_finally);

  /**
           * @brief asyncSend data without a callback
           *
           * @param buff Data vector
           * @return true if the async send function was successful
           * @return false if the async send function was not successful
           */
  bool asyncSend(std::vector<unsigned char> & buff);

  /**
           * @brief asyncSend data with a callback
           *
           * @param buff Data vector
           * @param callback Callback for asyncSendHandler
           * @return true if the async send function was successful
           * @return false if the async send function was not successful
           */
  bool asyncSend(std::vector<unsigned char> & buff, std::function<void()> callback);

  /**
           * @brief asyncSend data and asyncReceive with callbacks
           *
           * @param buff Data vector
           * @param func_header Callback of asyncSendHandler for header data
           * @param func_payload Callback of asyncSendHandler for payload data
           * @param func_finally Callback to process after func_payload
           * @return true if the async send function was successful
           * @return false if the async send function was not successful
           */
  bool
  asyncSendReceiveHeaderPayload(
    std::vector<unsigned char> & buff, Functor func_header, Functor func_payload,
    std::function<void()> func_finally);

  /**
           * @brief Get the Header objectGet header data (specialized for Hesai)
           *
           * @return std::vector<uint8_t> Header part in received data
           */
  std::vector<uint8_t> getHeader();

  /**
           * @brief Get the Payload objectGet payload data (specialized for Hesai)
           *
           * @return std::vector<uint8_t> Payload part in received data
           */
  std::vector<uint8_t> getPayload();

  /**
           * @brief Execute io_context's run
           *
           * @return boost::system::error_code from run
           */
  boost::system::error_code run();

  /**
           * @brief Get io_context
           *
           * @return std::shared_ptr<boost::asio::io_context> of this driver
           */
  std::shared_ptr<boost::asio::io_context> GetIOContext();

private:
  /**
           * @brief io_context for TcpSocket
           *
           */
  std::shared_ptr<boost::asio::io_context> m_ctx;

  /**
           * @brief TcpSocket for this driver
           *
           */
  std::shared_ptr<TcpSocket> m_socket;
};

}      // namespace tcp_driver
}  // namespace drivers

#endif  // TCP_DRIVER__TCP_DRIVER_HPP_
