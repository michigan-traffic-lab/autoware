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

#ifndef TCP_DRIVER__HTTP_CLIENT_DRIVER_HPP_
#define TCP_DRIVER__HTTP_CLIENT_DRIVER_HPP_

#include <iostream>
#include <memory>
#include <string>

#include "http_client.hpp"

namespace drivers
{
namespace tcp_driver
{

class HttpClientDriver
{

  /**
           * @brief Functor for HTTP processing
           *
           */
  using Functor = std::function<void (const std::string &)>;

public:
  /**
           * @brief HttpClientDriver constructor
           *
           * @param ctx io_context for this driver
           */
  explicit HttpClientDriver(std::shared_ptr<boost::asio::io_context> ctx);

  /**
           * @brief HttpClientDriver destructor
           *
           */
  ~HttpClientDriver();

  /**
           * @brief HttpClientDriver initialization with target address and port
           *
           * @param ip Device IP address
           * @param port Device HTTP port number
           */
  void init_client(const std::string & ip, uint16_t port);

  /**
           * @brief HttpClientDriver initialization with target address, port and ver.
           *
           * @param ip Device IP address
           * @param port Device HTTP port number
           * @param http_ver HTTP version
           */
  void init_client(const std::string & ip, uint16_t port, uint16_t http_ver);

  /**
           * @brief HttpClient for this driver
           *
           * @return std::shared_ptr<HttpClient> for this driver
           */
  std::shared_ptr<HttpClient> client() const;

  /**
           * @brief Blocking Get Operation
           *
           * @param target Target of HTTP request
           * @return std::string http response data
           */
  std::string get(const std::string & target);

  /**
           * @brief Blocking Post Operation
           *
           * @param target Target of HTTP request
           * @param body Body of HTTP request
           * @return std::string http response data
           */
  std::string post(const std::string & target, const std::string & body);

  /**
           * @brief NonBlocking Get Operation
           *
           * @param func Callback after async_read
           * @param target Target of HTTP request
           */
  void asyncGet(Functor func, const std::string & target);

  /**
           * @brief NonBlocking Post Operation
           *
           * @param func Callback after async_read
           * @param target Target of HTTP request
           * @param body Body of HTTP request
           */
  void asyncPost(Functor func, const std::string & target, const std::string & body);

private:
  /**
           * @brief io_context for HttpClientDriver
           *
           */
  std::shared_ptr<boost::asio::io_context> m_ctx;

  /**
           * @brief HttpClient for HttpClientDriver
           *
           */
  std::shared_ptr<HttpClient> m_client;
};

}      // namespace tcp_driver
}  // namespace drivers

#endif  // TCP_DRIVER__HTTP_CLIENT_DRIVER_HPP_
