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

#ifndef TCP_DRIVER__TCP_SOCKET_HPP_
#define TCP_DRIVER__TCP_SOCKET_HPP_

#include <array>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/lambda/lambda.hpp>

namespace drivers
{
namespace tcp_driver
{

class TcpSocket
{
  /**
           * @brief Functor for TCP message processing
           *
           */
  using Functor = std::function<void (const std::vector<uint8_t> &)>;

public:
  /**
           * @brief TcpSocket constructor with all parameters
           *
           * @param ctx io_context for this socket
           * @param remote_ip Device IP address
           * @param remote_port Device TCP port number
           * @param host_ip PC IP address
           * @param host_port PC TCP port number
           */
  TcpSocket(
    std::shared_ptr<boost::asio::io_context> ctx,
    const std::string & remote_ip, uint16_t remote_port,
    const std::string & host_ip, uint16_t host_port);

  /**
           * @brief TcpSocket constructor with target parameters
           *
           * @param ctx io_context for this socket
           * @param ip Device IP address
           * @param port Device TCP port number
           */
  TcpSocket(
    std::shared_ptr<boost::asio::io_context> ctx,
    const std::string & ip, uint16_t port);

  /**
           * @brief TcpSocket destructor
           *
           */
  ~TcpSocket();

  /**
           * @brief Disable copy constructor
           *
           */
  TcpSocket(const TcpSocket &) = delete;

  /**
           * @brief Disable copy assignment operator
           *
           * @return TcpSocket& (disabled)
           */
  TcpSocket & operator=(const TcpSocket &) = delete;

  /**
           * @brief Get Device IP address
           *
           * @return std::string Device IP address
           */
  std::string remote_ip() const;

  /**
           * @brief Get Device TCP port number
           *
           * @return uint16_t Device TCP port number
           */
  uint16_t remote_port() const;

  /**
           * @brief Get PC IP address
           *
           * @return std::string PC IP address
           */
  std::string host_ip() const;

  /**
           * @brief Get PC TCP port number
           *
           * @return uint16_t PC TCP port number
           */
  uint16_t host_port() const;

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
           * @brief bint host endpoint(not used)
           *
           */
  void bind();


  /**
           * @brief sync Send data without a callback
           *
           * @param buff Data vector
           * @return std::size_t Whether the sync send function was successful (1 or -1)
           */
  std::size_t send(std::vector<unsigned char> & buff);


  /**
           * @brief Read received data
           *
           * @param buff Buffer for received data
           * @param trans_size Receiving size for transfer_exactly (default: transfer_all)
           * @return size_t Length of received data
           */
  size_t receive(boost::asio::streambuf & buff, int trans_size = -1);

  /**
           * @brief syncSend and syncReceive with callbacks
           *
           * @param buff Data vector
           * @param func_header Callback of syncSendHandler for header data
           * @param func_payload Callback of syncSendHandler for payload data
           * @param func_finally Callback to process after func_payload
           */
  void
  syncSendReceiveHeaderPayload(
    std::vector<unsigned char> & buff, Functor func_header, Functor func_payload,
    std::function<void()> func_finally);

  /**
           * @brief asyncSend data without a callback
           *
           * @param buff Data vector
           */
  void asyncSend(std::vector<unsigned char> & buff);

  /**
           * @brief asyncSend data with a callback
           *
           * @param buff Data vector
           * @param callback Callback for async_write
           */
  void asyncSend(std::vector<unsigned char> & buff, std::function<void()> callback);


  /**
           * @brief asyncReceive data with a callback
           *
           * @param func Callback for async_receive
           */
  void asyncReceive(Functor func);

  /**
           * @brief asyncReceive with callbacks
           *
           * @param func_header Callback of asyncSendHandler for header data
           * @param func_payload Callback of asyncSendHandler for payload data
           * @param func_finally Callback to process after func_payload
           */
  void
  asyncReceiveHeaderPayload(
    Functor func_header, Functor func_payload,
    std::function<void()> func_finally);

  /**
           * @brief
           *
           * @param buff asyncReceive with callbacks (with retrying)
           * @param func_header Callback of asyncSendHandler for header data
           * @param func_payload Callback of asyncSendHandler for payload data
           * @param func_finally Callback to process after func_payload
           */
  void
  asyncReceiveHeaderPayloadRetry(
    std::vector<unsigned char> & buff, Functor func_header, Functor func_payload,
    std::function<void()> func_finally);

  /**
           * @brief asyncSend and asyncReceive with a callback
           *
           * @param buff Data vector
           * @param func Callback for async_write
           */
  void asyncSendReceive(std::vector<unsigned char> & buff, Functor func);

  /**
           * @brief asyncSend and asyncReceive with callbacks
           *
           * @param buff Data vector
           * @param func_header Callback of asyncSendHandler for header data
           * @param func_payload Callback of asyncSendHandler for payload data
           * @param func_finally Callback to process after func_payload
           */
  void
  asyncSendReceiveHeaderPayload(
    std::vector<unsigned char> & buff, Functor func_header, Functor func_payload,
    std::function<void()> func_finally);


  /**
           * @brief Get header data (specialized for Hesai)
           *
           * @return std::vector<uint8_t> Header part in received data
           */
  std::vector<uint8_t> getHeader();

  /**
           * @brief Get payload data (specialized for Hesai)
           *
           * @return std::vector<uint8_t> Payload part in received data
           */
  std::vector<uint8_t> getPayload();

  bool needReset() const;

private:
  /**
           * @brief Handler for async_write (Set a reset flag if there is an error)
           *
           * @param error error_code
           * @param bytes_transferred # of transferred data
           */
  void asyncSendHandler(
    boost::system::error_code error,
    std::size_t bytes_transferred);

  /**
           * @brief Base handler for async_read
           *
           * @param error error_code
           * @param bytes_transferred # of transferred data
           */
  void asyncReceiveHandler(
    boost::system::error_code error,
    std::size_t bytes_transferred);

  /**
           * @brief Handler for async_read (Specialized for Hesai's TCP data)
           *
           * @param error error_code
           * @param bytes_transferred # of transferred data
           */
  void asyncReceiveHandlerHeaderPayload(
    boost::system::error_code error,
    std::size_t bytes_transferred);

  /**
           * @brief Handler for async_read (Specialized for Hesai's TCP data) (with retrying)
           *
           * @param error error_code
           * @param bytes_transferred # of transferred data
           */
  void asyncReceiveHandlerHeaderPayloadRetry(
    boost::system::error_code error,
    std::size_t bytes_transferred);

private:
  /**
           * @brief io_context for TcpSocket
           *
           */
  std::shared_ptr<boost::asio::io_context> m_ctx;

  /**
           * @brief boost::asio::ip::tcp::socket for this driver
           *
           */
  std::shared_ptr<boost::asio::ip::tcp::socket> m_socket;

  /// @brief boost::asio::deadline_timer for connection
  boost::asio::deadline_timer deadline_;

  /**
           * @brief Device IP address
           *
           */
  boost::asio::ip::tcp::endpoint m_remote_endpoint;

  /**
           * @brief PC IP address
           *
           */
  boost::asio::ip::tcp::endpoint m_host_endpoint;

  /**
           * @brief backup message
           *
           */
  std::vector<unsigned char> m_mes;

  /**
           * @brief Callback
           *
           */
  Functor m_func;

  /**
           * @brief Callback for header data
           *
           */
  Functor m_func_h;

  /**
           * @brief Callback for payload data
           *
           */
  Functor m_func_p;

  /**
           * @brief Callback to process after m_func_p
           *
           */
  std::function<void()> m_func_f;

  /**
           * @brief Message buffer (not used)
           *
           */
  static const size_t m_recv_buffer_size{2048};

  /**
           * @brief Message buffer
           *
           */
  boost::asio::streambuf m_recv_buffer;

  /**
           * @brief Message buffer for header
           *
           */
  std::vector<uint8_t> m_recv_header;

  /**
           * @brief Message buffer for payload
           *
           */
  std::vector<uint8_t> m_recv_payload;

  /**
           * @brief A flag indicating if the socket needs a reset
           *
           */
  bool reset_flg = false;
};

}      // namespace tcp_driver
}  // namespace drivers

#endif  // TCP_DRIVER__TCP_SOCKET_HPP_
