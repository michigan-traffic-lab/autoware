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

#include "boost_tcp_driver/http_client_driver.hpp"

#include <iostream>
#include <string>
#include <memory>

//#define WITH_DEBUG_STDCOUT_HTTP_CLIENT_DRIVER // Use std::cout messages for debugging

namespace drivers
{
namespace tcp_driver
{

HttpClientDriver::HttpClientDriver(std::shared_ptr<boost::asio::io_context> ctx)
: m_ctx(ctx)
{
}

HttpClientDriver::~HttpClientDriver()
{
  m_client.reset();
  m_ctx.reset();
}

void HttpClientDriver::init_client(const std::string & ip, uint16_t port)
{
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT_DRIVER
  std::cout << "HttpClientDriver::init_client: " << ip << " : " << port << std::endl;
#endif
  m_client.reset(new HttpClient(m_ctx, ip, port));
}

void HttpClientDriver::init_client(
  const std::string & ip, uint16_t port, uint16_t http_ver)
{
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT_DRIVER
  std::cout << "HttpClientDriver::init_client: " << ip << " : " << port << " : " << http_ver <<
    std::endl;
#endif
  m_client.reset(new HttpClient(m_ctx, ip, port, http_ver));
}


std::shared_ptr<HttpClient> HttpClientDriver::client() const
{
  return m_client;
}

std::string HttpClientDriver::get(const std::string & target)
{
  m_client->get(target);
  return m_client->read_response();
}

std::string HttpClientDriver::post(const std::string & target, const std::string & body)
{
  m_client->post(target, body);
  return m_client->read_result();
}

void HttpClientDriver::asyncGet(Functor func, const std::string & target)
{
  m_client->asyncGet(func, target);
}

void HttpClientDriver::asyncPost(Functor func, const std::string & target, const std::string & body)
{
  m_client->asyncPost(func, target, body);
}

}      // namespace tcp_driver
}  // namespace drivers
