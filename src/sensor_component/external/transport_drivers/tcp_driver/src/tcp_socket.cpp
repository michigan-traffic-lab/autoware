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

#include "boost_tcp_driver/tcp_socket.hpp"

#include <iostream>
#include <utility>
#include <string>
#include <system_error>
#include <vector>

#include <boost/asio.hpp>
#include "rclcpp/logging.hpp"

//#define WITH_DEBUG_STDCOUT_TCP_SOCKET // Use std::cout messages for debugging

namespace drivers
{
namespace tcp_driver
{

TcpSocket::TcpSocket(
  std::shared_ptr<boost::asio::io_context> ctx,
  const std::string & remote_ip,
  const uint16_t remote_port,
  const std::string & host_ip,
  const uint16_t host_port)
: m_ctx(ctx),
  m_socket(new boost::asio::ip::tcp::socket(*m_ctx)),
  deadline_(*m_ctx),
  m_remote_endpoint(boost::asio::ip::address::from_string(remote_ip), remote_port),
  m_host_endpoint(boost::asio::ip::address::from_string(host_ip), host_port)
{
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
    std::cout << "remote_ip=" << remote_ip << std::endl;
    std::cout << "remote_port=" << remote_port << std::endl;
    std::cout << "remote_ip=" << remote_ip << std::endl;
    std::cout << "host_port=" << host_port << std::endl;
#endif
  m_remote_endpoint = remote_ip.empty() ?
    boost::asio::ip::tcp::endpoint{boost::asio::ip::tcp::v4(), remote_port} :
  boost::asio::ip::tcp::endpoint{boost::asio::ip::address::from_string(remote_ip), remote_port};
  m_host_endpoint = host_ip.empty() ?
    boost::asio::ip::tcp::endpoint{boost::asio::ip::tcp::v4(), host_port} :
  boost::asio::ip::tcp::endpoint{boost::asio::ip::address::from_string(host_ip), host_port};
}

TcpSocket::TcpSocket(
  std::shared_ptr<boost::asio::io_context> ctx,
  const std::string & ip,
  const uint16_t port)
: TcpSocket{ctx, ip, port, ip, port} {}

TcpSocket::~TcpSocket()
{
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
  std::cout << "TcpSocket::~TcpSocket()" << std::endl;
#endif
  close();
  m_socket.reset();
}

/// @brief Verify whether or not the socket needs to be reset
/// @return True if the socket state requires a reset.
bool TcpSocket::needReset() const
{
  return reset_flg;
}

std::size_t TcpSocket::send(std::vector<unsigned char> & buff)
{
  try {
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
    std::cout << "TcpSocket::send" << std::endl;
#endif
    boost::system::error_code error;
    boost::asio::write(*m_socket, boost::asio::buffer(buff, buff.size()), error);
    if (error) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("TcpSocket::send"), error.message());
      reset_flg = true;
      return -1;
    }
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("TcpSocket::send(catch)"), error.what());
    reset_flg = true;
    return -1;
  }
  return 1;
}

size_t TcpSocket::receive(boost::asio::streambuf & buff, int trans_size)
{
  boost::asio::ip::tcp::endpoint sender_endpoint;
  boost::system::error_code error;

  std::size_t len = -1;
  if(trans_size <= 0){
    len = boost::asio::read(*m_socket, buff, boost::asio::transfer_all(), error);
  } else {
    len = boost::asio::read(*m_socket, buff, boost::asio::transfer_exactly(trans_size), error);
  }

  if (error && error != boost::asio::error::eof) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("TcpSocket::receive"), error.message());
    reset_flg = true;
    return -1;
  }
  return len;
}


void TcpSocket::syncSendReceiveHeaderPayload(
  std::vector<unsigned char> & buff, Functor func_header,
  Functor func_payload, std::function<void()> func_finally)
{
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
  std::cout << "boost::asio::async_write" << std::endl;
#endif
  TcpSocket::send(buff);
  size_t bytes_transferred = TcpSocket::receive(m_recv_buffer, 8);

  if (bytes_transferred > 0) {
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
    std::cout << "std::vector<uint8_t> target(m_recv_buffer.size())" << std::endl;
#endif
    m_recv_header.resize(m_recv_buffer.size());
    buffer_copy(boost::asio::buffer(m_recv_header), m_recv_buffer.data());
    if (func_header) {
      func_header(m_recv_header);
    }

    // for Hesai's header
    int payload_num = static_cast<int>(static_cast<unsigned char>(m_recv_header[4]) << 24 |
      static_cast<unsigned char>(m_recv_header[5]) << 16 |
      static_cast<unsigned char>(m_recv_header[6]) << 8 |
      static_cast<unsigned char>(m_recv_header[7]));

#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
    std::cout << "payload_num: " << payload_num << std::endl;
#endif

    if (0 < payload_num) {
      bytes_transferred = TcpSocket::receive(m_recv_buffer, payload_num);
      m_recv_payload.resize(m_recv_buffer.size());
      buffer_copy(boost::asio::buffer(m_recv_payload), m_recv_buffer.data());
      if (func_payload) {
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
          std::cout << "run func_payload" << std::endl;
#endif
        func_payload(m_recv_payload);
      }
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
          std::cout << "TcpSocket::asyncReceiveHandlerHeaderPayload fin" << std::endl;
#endif
      m_recv_buffer.consume(m_recv_buffer.size());
    } else {
      m_recv_buffer.consume(m_recv_buffer.size());
    }
    if (func_finally) {
      func_finally();
    }
  }
}

void TcpSocket::asyncSend(std::vector<unsigned char> & buff)
{
  boost::asio::async_write(
    *m_socket,
    boost::asio::buffer(buff, buff.size()),
    [this](boost::system::error_code ec, std::size_t bytes_transferred) {
      TcpSocket::asyncSendHandler(ec, bytes_transferred);
    });
}

void TcpSocket::asyncSend(std::vector<unsigned char> & buff, std::function<void()> callback)
{
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
  std::cout <<
    "void TcpSocket::asyncSend(std::vector<unsigned char> & buff, std::function<void()> callback)"
            << std::endl;
#endif
  boost::asio::async_write(
    *m_socket,
    boost::asio::buffer(buff, buff.size()),
    [this, callback](boost::system::error_code ec, std::size_t bytes_transferred) {
      TcpSocket::asyncSendHandler(ec, bytes_transferred);
      callback();
    });
}

void TcpSocket::asyncReceive(Functor func)
{
  m_func = std::move(func);
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
  std::cout << "boost::asio::async_read" << std::endl;
#endif
  boost::asio::async_read(
    *m_socket,
    m_recv_buffer,
    boost::asio::transfer_at_least(4),
    std::bind(
      &TcpSocket::asyncReceiveHandler, this,
      std::placeholders::_1, std::placeholders::_2));
}

void TcpSocket::asyncReceiveHeaderPayload(
  Functor func_header, Functor func_payload,
  std::function<void()> func_finally)
{
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
  std::cout << "TcpSocket::asyncReceiveHeaderPayload" << std::endl;
#endif
  m_func_h = std::move(func_header);
  m_func_p = std::move(func_payload);
  m_func_f = std::move(func_finally);

#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
  std::cout << "boost::asio::async_read" << std::endl;
#endif
  boost::asio::async_read(
    *m_socket,
    m_recv_buffer,
    boost::asio::transfer_exactly(8),          //for Hesai
    std::bind(
      &TcpSocket::asyncReceiveHandlerHeaderPayload, this,
      std::placeholders::_1, std::placeholders::_2));
}

void TcpSocket::asyncReceiveHeaderPayloadRetry(
  std::vector<unsigned char> & buff, Functor func_header,
  Functor func_payload, std::function<void()> func_finally)
{
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
  std::cout << "TcpSocket::asyncReceiveHeaderPayloadRetry" << std::endl;
#endif
  std::copy(buff.begin(), buff.end(), std::back_inserter(m_mes));
  m_func_h = std::move(func_header);
  m_func_p = std::move(func_payload);
  m_func_f = std::move(func_finally);
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
  std::cout << "boost::asio::async_read" << std::endl;
#endif
  boost::asio::async_read(
    *m_socket,
    m_recv_buffer,
    boost::asio::transfer_exactly(8),          //for Hesai
    std::bind(
      &TcpSocket::asyncReceiveHandlerHeaderPayloadRetry, this,
      std::placeholders::_1, std::placeholders::_2));
}

void TcpSocket::asyncSendReceive(std::vector<unsigned char> & buff, Functor func)
{
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
  std::cout << "boost::asio::async_write" << std::endl;
#endif
  boost::asio::async_write(
    *m_socket,
    boost::asio::buffer(buff, buff.size()),
    [this, func](boost::system::error_code ec, std::size_t bytes_transferred) {
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
      std::cout << "async_write, func" << std::endl;
#endif
      TcpSocket::asyncSendHandler(ec, bytes_transferred);
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
      std::cout << "asyncSendHandler, fin" << std::endl;
#endif
      TcpSocket::asyncReceive(func);
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
      std::cout << "asyncReceive, fin" << std::endl;
#endif
    });
}

void TcpSocket::asyncSendReceiveHeaderPayload(
  std::vector<unsigned char> & buff, Functor func_header,
  Functor func_payload, std::function<void()> func_finally)
{
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
  std::cout << "boost::asio::async_write" << std::endl;
#endif
  boost::asio::async_write(
    *m_socket,
    boost::asio::buffer(buff, buff.size()),
    [this, func_header, func_payload, func_finally, buff](boost::system::error_code ec,
    std::size_t bytes_transferred) {
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
      std::cout << "async_write, func" << std::endl;
#endif
      TcpSocket::asyncSendHandler(ec, bytes_transferred);
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
      std::cout << "asyncSendHandler, fin" << std::endl;
#endif

      if (needReset()) {
        close();
        if (!open()) {
          RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("TcpDriver::asyncSendReceiveHeaderPayload"),
            "socket open failed");
          return;
        }
        reset_flg = false;
        std::vector<unsigned char> buff_c;
        std::copy(buff.begin(), buff.end(), std::back_inserter(buff_c));
        asyncSendReceiveHeaderPayload(buff_c, func_header, func_payload, func_finally);
        return;
      }
      TcpSocket::asyncReceiveHeaderPayload(func_header, func_payload, func_finally);
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
      std::cout << "asyncReceive, fin" << std::endl;
#endif
    });
}

void TcpSocket::asyncSendHandler(
  boost::system::error_code error,
  std::size_t bytes_transferred)
{
  (void) bytes_transferred;
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("TcpSocket::asyncSendHandler"), error.message());
    reset_flg = true;
  }
}

void TcpSocket::asyncReceiveHandler(
  boost::system::error_code error,
  std::size_t bytes_transferred)
{
  (void) bytes_transferred;
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("TcpSocket::asyncReceiveHandler"), error.message());
    reset_flg = true;
    return;
  }
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
  std::cout << "TcpSocket::asyncReceiveHandler" << std::endl;
#endif

  if (bytes_transferred > 0 && m_func) {
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
    std::cout << "std::vector<uint8_t> target(m_recv_buffer.size())" << std::endl;
#endif
    std::vector<uint8_t> target(m_recv_buffer.size());
    buffer_copy(boost::asio::buffer(target), m_recv_buffer.data());
    m_func(target);

#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
    std::cout << "boost::asio::async_read" << std::endl;
#endif

    boost::asio::async_read(
      *m_socket,
      m_recv_buffer,
      boost::asio::transfer_at_least(4),
      std::bind(
        &TcpSocket::asyncReceiveHandler, this,
        std::placeholders::_1, std::placeholders::_2));
  }
}

void TcpSocket::asyncReceiveHandlerHeaderPayload(
  boost::system::error_code error,
  std::size_t bytes_transferred)
{
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
  std::cout << "TcpSocket::asyncReceiveHandlerHeaderPayload" << std::endl;
#endif
  (void) bytes_transferred;
  if (error && error != boost::asio::error::eof) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger(
        "TcpSocket::asyncReceiveHandlerHeaderPayload"), error.message());
    reset_flg = true;
    return;
  }

  if (bytes_transferred > 0) {
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
    std::cout << "bytes_transferred > 0" << std::endl;
    std::cout << "std::vector<uint8_t> target(m_recv_buffer.size())" << std::endl;
#endif
    m_recv_header.resize(m_recv_buffer.size());
    buffer_copy(boost::asio::buffer(m_recv_header), m_recv_buffer.data());
    if (m_func_h) {
      m_func_h(m_recv_header);
    }

    // for Hesai's header
    int payload_num = static_cast<int>(static_cast<unsigned char>(m_recv_header[4]) << 24 |
      static_cast<unsigned char>(m_recv_header[5]) << 16 |
      static_cast<unsigned char>(m_recv_header[6]) << 8 |
      static_cast<unsigned char>(m_recv_header[7]));

#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
    std::cout << "payload_num: " << payload_num << std::endl;
#endif

    if (0 < payload_num) {
      boost::asio::async_read(
        *m_socket,
        m_recv_buffer,
        boost::asio::transfer_exactly(payload_num),
        [this](boost::system::error_code error,
        std::size_t bytes_transferred) {
          (void) bytes_transferred;
          if (error) {
            RCLCPP_ERROR_STREAM(
              rclcpp::get_logger("TcpSocket::asyncReceiveHandlerHeaderPayload(async_read)"),
              error.message());
            reset_flg = true;
            return;
          }
          m_recv_payload.resize(m_recv_buffer.size());
          buffer_copy(boost::asio::buffer(m_recv_payload), m_recv_buffer.data());

          if (m_func_p) {
            m_func_p(m_recv_payload);
          }
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
          std::cout << "TcpSocket::asyncReceiveHandlerHeaderPayload fin" << std::endl;
#endif
          m_recv_buffer.consume(m_recv_buffer.size());
        });
    } else {
      m_recv_buffer.consume(m_recv_buffer.size());
    }
    if (m_func_f) {
      m_func_f();
    }
  }
  else
  {
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
    std::cout << "not (bytes_transferred > 0)" << std::endl;
#endif
  }
}

void TcpSocket::asyncReceiveHandlerHeaderPayloadRetry(
  boost::system::error_code error,
  std::size_t bytes_transferred)
{
#ifdef WITH_DEBUG_STDCOUT_TCP_SOCKET
  std::cout << "TcpSocket::asyncReceiveHandlerHeaderPayloadRetry" << std::endl;
#endif
  if (error) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("TcpSocket::asyncReceiveHandlerHeaderPayloadRetry"),
      error.message());
    reset_flg = true;
    if (needReset()) {
      close();
      if (!open()) {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("TcpDriver::asyncReceiveHandlerHeaderPayloadRetry"),
          "socket open failed");
        return;
      }
      reset_flg = false;
      asyncSendReceiveHeaderPayload(m_mes, m_func_h, m_func_p, m_func_f);
    }
    return;
  }

  asyncReceiveHandlerHeaderPayload(error, bytes_transferred);
}

std::string TcpSocket::remote_ip() const
{
  return m_remote_endpoint.address().to_string();
}

uint16_t TcpSocket::remote_port() const
{
  return m_remote_endpoint.port();
}

std::string TcpSocket::host_ip() const
{
  return m_host_endpoint.address().to_string();
}

uint16_t TcpSocket::host_port() const
{
  return m_host_endpoint.port();
}

// https://stackoverflow.com/questions/32692195/set-timeout-for-boost-socket-connect
bool TcpSocket::open()
{
  m_ctx->restart();
  m_socket->open(boost::asio::ip::tcp::v4());
  m_socket->set_option(boost::asio::ip::tcp::socket::reuse_address(true));

  std::cout << m_remote_endpoint << std::endl;

  boost::system::error_code ec = boost::asio::error::would_block;
  deadline_.expires_from_now(boost::posix_time::seconds(5));
  deadline_.async_wait([this](const boost::system::error_code& ec2) {
    if (!ec2) {
      std::cerr << "# Canceling socket operation due to timeout (5s).\n";
      m_socket->cancel();
      m_ctx->restart();
    }
  });
  m_socket->async_connect(m_remote_endpoint, boost::lambda::var(ec) = boost::lambda::_1);

  do m_ctx->run_one(); while (ec == boost::asio::error::would_block);

  if (ec || !m_socket->is_open()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("TcpSocket::open"), ec.message());
    m_socket->cancel();
    reset_flg = true;
    deadline_.cancel();
    m_ctx->restart();
    return false;
  } else {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("TcpSocket::open"), "connected");
  }
  reset_flg = false;
  deadline_.cancel();
  m_ctx->restart();
  return true;
}

void TcpSocket::closeSync()
{
  boost::system::error_code error;
  m_socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, error);
  if (error && error != boost::system::errc::not_connected) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("TcpSocket::shutdown"), error.message());
  }
  m_socket->close(error);
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("TcpSocket::close"), error.message());
  }
}

void TcpSocket::close()
{
  m_ctx->post([this]() {
    boost::system::error_code error;
    m_socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, error);
    if (error && error != boost::system::errc::not_connected) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("TcpSocket::shutdown"), error.message());
    }
  });
  m_ctx->post([this]() {
    boost::system::error_code error;
    m_socket->close(error);
    if (error) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("TcpSocket::close"), error.message());
    }
    m_socket.reset();
  });
  m_ctx->run();
}

bool TcpSocket::isOpen() const
{
  if(m_socket != NULL){
    return m_socket->is_open();
  }
  return false;
}

void TcpSocket::bind()
{
  m_socket->bind(m_host_endpoint);
}

std::vector<uint8_t> TcpSocket::getHeader()
{
  return m_recv_header;
}

std::vector<uint8_t> TcpSocket::getPayload()
{
  return m_recv_payload;
}

}      // namespace tcp_driver
}  // namespace drivers
