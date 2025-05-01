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

#include "boost_tcp_driver/tcp_driver.hpp"

#include <iostream>
#include <string>
#include <memory>

namespace drivers
{
namespace tcp_driver
{
TcpDriver::TcpDriver(std::shared_ptr<boost::asio::io_context> ctx)
: m_ctx(ctx)
{
}

TcpDriver::~TcpDriver()
{
  close();
  m_socket.reset();
}

void TcpDriver::init_socket(const std::string & ip, uint16_t port)
{
  /*
  if(m_socket){
    m_socket->closeSync();
  }
  */
  m_socket.reset(new TcpSocket(m_ctx, ip, port));
}

void TcpDriver::init_socket(
  const std::string & remote_ip, uint16_t remote_port,
  const std::string & host_ip, uint16_t host_port)
{
  /*
  if(m_socket){
    m_socket->closeSync();
  }
  */
  m_socket.reset(new TcpSocket(m_ctx, remote_ip, remote_port, host_ip, host_port));
}

std::shared_ptr<TcpSocket> TcpDriver::socket() const
{
  return m_socket;
}

bool TcpDriver::open()
{
  return m_socket->open();
}

void TcpDriver::closeSync()
{
  if(m_socket != NULL){
    m_socket->closeSync();
  }
}

void TcpDriver::close()
{
  if(m_socket != NULL){
    m_socket->close();
  }
}

bool TcpDriver::isOpen() const
{
  if(m_socket != NULL){
    return m_socket->isOpen();
  }
  return false;
}

bool TcpDriver::syncSendReceiveHeaderPayload(
  std::vector<unsigned char> & buff, Functor func_header,
  Functor func_payload, std::function<void()> func_finally)
{
  if (m_socket->needReset()) {
    close();
  }
  if (!isOpen()) {
    if (!open()) {
      std::cerr << "TcpDriver::asyncSendReceiveHeaderPayload" << ": " << "socket open failed" <<
        "\n";
      return false;
    }
  }
  m_socket->asyncSendReceiveHeaderPayload(buff, func_header, func_payload, func_finally);
  return true;
}

bool TcpDriver::asyncSend(std::vector<unsigned char> & buff)
{
  if (m_socket->needReset()) {
    close();
  }
  if (!isOpen()) {
    if (!open()) {
      std::cerr << "TcpDriver::asyncSend" << ": " << "socket open failed" << "\n";
      return false;
    }
  }
  m_socket->asyncSend(buff);
  return true;
}

bool TcpDriver::asyncSend(std::vector<unsigned char> & buff, std::function<void()> callback)
{
  if (m_socket->needReset()) {
    close();
  }
  if (!isOpen()) {
    if (!open()) {
      std::cerr << "TcpDriver::asyncSend" << ": " << "socket open failed" << "\n";
      return false;
    }
  }
  m_socket->asyncSend(buff, callback);
  return true;
}

bool TcpDriver::asyncSendReceiveHeaderPayload(
  std::vector<unsigned char> & buff, Functor func_header,
  Functor func_payload, std::function<void()> func_finally)
{
  if (m_socket->needReset()) {
    close();
  }
  if (!isOpen()) {
    if (!open()) {
      std::cerr << "TcpDriver::asyncSendReceiveHeaderPayload" << ": " << "socket open failed" <<
        "\n";
      return false;
    }
  }
  m_socket->asyncSendReceiveHeaderPayload(buff, func_header, func_payload, func_finally);
  return true;
}

std::vector<uint8_t> TcpDriver::getHeader()
{
  return m_socket->getHeader();
}

std::vector<uint8_t> TcpDriver::getPayload()
{
  return m_socket->getPayload();
}

boost::system::error_code TcpDriver::run()
{
  boost::system::error_code ec;
  m_ctx->run(ec);
  return ec;
}

std::shared_ptr<boost::asio::io_context> TcpDriver::GetIOContext()
{
  return m_ctx;
}


}      // namespace tcp_driver
}  // namespace drivers
