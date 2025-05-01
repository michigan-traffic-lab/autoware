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


#include "boost_tcp_driver/http_client.hpp"

#include <cstdlib>

#include <iostream>
#include <utility>
#include <string>
#include <system_error>
#include <vector>

#include "boost/asio.hpp"

#include "rclcpp/logging.hpp"

//#define WITH_DEBUG_STDCOUT_HTTP_CLIENT // Use std::cout messages for debugging

namespace drivers
{
namespace tcp_driver
{
HttpClient::HttpClient(
  std::shared_ptr<boost::asio::io_context> ctx,
  const std::string & ip,
  const uint16_t port,
  const uint16_t http_ver)
: m_remote_host(ip),
  m_remote_port(port),
  m_http_ver(http_ver),
  m_ctx(ctx),
  m_resolver(*ctx),
  deadline_(*ctx)
{
  m_socket.reset(new boost::asio::ip::tcp::socket(*m_ctx));
  timeout_msec = 500;
}

HttpClient::HttpClient(
  std::shared_ptr<boost::asio::io_context> ctx,
  const std::string & ip,
  const uint16_t port)
: HttpClient{ctx, ip, port, 11}
{
}

HttpClient::~HttpClient()
{
  close();
}

std::size_t HttpClient::write(std::string target, boost::beast::http::verb method, std::string body)
{
  try {
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
    RCLCPP_INFO(
      rclcpp::get_logger("HttpClient::write"), "write: %s : %s",
      target.c_str(), body.c_str());
#endif
    boost::beast::http::request<boost::beast::http::string_body> req{method, target, m_http_ver};
    req.set(boost::beast::http::field::host, m_remote_host);
    req.set(boost::beast::http::field::user_agent, BOOST_BEAST_VERSION_STRING);
    if (method == boost::beast::http::verb::post) {
      req.set(boost::beast::http::field::content_type, "application/x-www-form-urlencoded");
      req.body() = body;
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
      RCLCPP_INFO(
        rclcpp::get_logger("HttpClient::write"), "body: %s : %s", body.c_str(),
        req.body().c_str());
#endif
    }

#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
    {
      std::ostringstream oss;
      oss << "Request message " << req;
      std::cout << (oss.str());
      m_recv_string = oss.str();
      oss.clear();
      return m_recv_string.length();
    }
#endif

#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
    {
      std::ostringstream oss;
      std::cout << "oss << Request message << req" << std::endl;
      oss << "Request message " << req;
      std::cout << (oss.str());
      RCLCPP_INFO(
        rclcpp::get_logger("HttpClient::write"), "Request message: %s",
        oss.str().c_str());
      oss.clear();
    }
#endif

    if (!isOpen()) {
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
      RCLCPP_INFO(rclcpp::get_logger("HttpClient::write"), "!isOpen()");
#endif
      open();
    }

#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
    RCLCPP_INFO(rclcpp::get_logger("HttpClient::write"), "http::write");
#endif
    boost::beast::http::write(*m_socket, req);
    boost::beast::flat_buffer buffer;
    boost::beast::http::response<boost::beast::http::dynamic_body> res;
    boost::beast::http::read(*m_socket, buffer, res);

    // need to handle the response

#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
    RCLCPP_INFO(rclcpp::get_logger("HttpClient::write"), "m_res.body().data()");
#endif
    m_res_string = boost::beast::buffers_to_string(res.body().data());
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
    RCLCPP_INFO(rclcpp::get_logger("HttpClient::write"), "m_res_string : %s", m_res_string.c_str());
    RCLCPP_INFO(rclcpp::get_logger("HttpClient::write"), "beast::buffers_to_string");
#endif
    m_recv_string = boost::beast::buffers_to_string(buffer.data());
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
    RCLCPP_INFO(
      rclcpp::get_logger("HttpClient::write"), "m_recv_string : %s",
      m_recv_string.c_str());
#endif
    return m_recv_string.length();
  } catch (const std::system_error & error) {
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("HttpClient::write"), error.what());
#endif
    return -1;
  }
}

std::size_t HttpClient::get(std::string target)
{
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
  RCLCPP_INFO(rclcpp::get_logger("HttpClient::get"), "write: %s", target.c_str());
#endif
  return HttpClient::write(target, boost::beast::http::verb::get);
}

std::size_t HttpClient::post(std::string target, std::string body)
{
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
  RCLCPP_INFO(
    rclcpp::get_logger("HttpClient::post"), "write: %s : %s", target.c_str(),
    body.c_str());
#endif
  return HttpClient::write(target, boost::beast::http::verb::post, body);
}

std::string HttpClient::read_result()
{
  return m_recv_string;
}

std::string HttpClient::read_response()
{
  return m_res_string;
}

/**
         * @brief Show error
         *
         * @param ec error_code
         * @param what Detail
         */
void fail(boost::system::error_code ec, char const * what)
{
  std::cerr << what << ": " << ec.message() << "\n";
}

void HttpClient::asyncWrite(
  Functor func, std::string target, boost::beast::http::verb method,
  std::string body)
{
  try {
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
    std::cout << m_remote_host << std::endl;
    RCLCPP_INFO(
      rclcpp::get_logger("HttpClient::asyncWrite"), "asyncWrite: %s : %s",
      target.c_str(), body.c_str());
    RCLCPP_INFO(
      rclcpp::get_logger(
        "HttpClient::asyncWrite"), "m_remote_host: %s", m_remote_host.c_str());
#endif
    m_func = func;
    m_req.version(m_http_ver);
    m_req.method(method);
    m_req.target(target);
    m_req.set(boost::beast::http::field::host, m_remote_host);
    m_req.set(boost::beast::http::field::user_agent, BOOST_BEAST_VERSION_STRING);
    if (method == boost::beast::http::verb::post) {
      m_req.set(boost::beast::http::field::content_type, "application/x-www-form-urlencoded");
      m_req.body() = body;
    }

#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
    {
      std::ostringstream oss;
      oss << "Request message " << m_req;
      std::cout << (oss.str());
      RCLCPP_INFO(
        rclcpp::get_logger("HttpClient::write"), "Request message: %s",
        oss.str().c_str());
      oss.clear();
    }
#endif

    // Not working well
    if (0 < timeout_msec) {
      deadline_.expires_after(std::chrono::milliseconds(timeout_msec));
    }
    boost::asio::io_context io;
    boost::asio::steady_timer connect_timer_(io);
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
    RCLCPP_INFO(rclcpp::get_logger("HttpClient::asyncWrite"), "async_resolve");
#endif
    m_resolver.async_resolve(
      m_remote_host,
      std::to_string(m_remote_port),
      boost::beast::bind_front_handler(
        &HttpClient::asyncOnResolve,
        shared_from_this()));

    // Not working well
    connect_timer_.expires_from_now(std::chrono::milliseconds(5000));
    connect_timer_.async_wait(
      [ = ](const boost::system::error_code & ec) {
        if (ec == boost::asio::error::operation_aborted) {
        } else {
          m_socket->cancel();
          std::cerr << "HttpClient::asyncWrite: " << ec.message() << std::endl;
        }
      });
    if (0 < timeout_msec) {
      deadline_.async_wait(std::bind(&HttpClient::check_deadline, this));
    }
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("HttpClient::asyncWrite"), error.what());
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("HttpClient::asyncWrite target: %s"), target.c_str());
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("HttpClient::asyncWrite body: %s"), body.c_str());
    return;
  }
}

void HttpClient::asyncGet(Functor func, std::string target)
{
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
  RCLCPP_INFO(rclcpp::get_logger("HttpClient::asyncGet"), "asyncGet: %s", target.c_str());
#endif
  return HttpClient::asyncWrite(func, target, boost::beast::http::verb::get);
}

void HttpClient::asyncPost(Functor func, std::string target, std::string body)
{
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
  RCLCPP_INFO(rclcpp::get_logger("HttpClient::asyncPost"), "asyncPost: %s", body.c_str());
#endif
  return HttpClient::asyncWrite(func, target, boost::beast::http::verb::post, body);
}

void HttpClient::asyncOnResolve(
  boost::beast::error_code ec,
  boost::asio::ip::tcp::resolver::results_type results)
{
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
  RCLCPP_INFO(rclcpp::get_logger("HttpClient::asyncOnResolve"), "asyncOnResolve");
#endif
  if (ec) {
    return fail(ec, "resolve");
  }

  if (0 < timeout_msec) {
    deadline_.expires_after(std::chrono::milliseconds(timeout_msec));
  }
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
  RCLCPP_INFO(rclcpp::get_logger("HttpClient::asyncOnResolve"), "async_connect");
#endif
  boost::asio::async_connect(
    *m_socket,
    results.begin(),
    results.end(),
    std::bind(
      &HttpClient::asyncOnConnect,
      shared_from_this(),
      std::placeholders::_1));

  if (0 < timeout_msec) {
    deadline_.async_wait(std::bind(&HttpClient::check_deadline, this));
  }
}

void HttpClient::asyncOnConnect(boost::system::error_code ec)
{
  RCLCPP_DEBUG(rclcpp::get_logger("HttpClient::asyncOnConnect"), "asyncOnConnect");
  if (ec) {
    return fail(ec, "asyncOnConnect");
  }

  if (0 < timeout_msec) {
    deadline_.expires_after(std::chrono::milliseconds(timeout_msec));
  }
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
  std::cout << "will http::async_write(*m_socket, m_req," << std::endl;
#endif
  RCLCPP_DEBUG(rclcpp::get_logger("HttpClient::asyncOnConnect"), "async_write");
  boost::beast::http::async_write(
    *m_socket, m_req,
    [this](boost::system::error_code ec, std::size_t bytes_transferred) {
      asyncOnWrite(ec, bytes_transferred);
    });
  if (0 < timeout_msec) {
    deadline_.async_wait(std::bind(&HttpClient::check_deadline, this));
  }
}

void HttpClient::asyncOnWrite(
  boost::system::error_code ec,
  std::size_t bytes_transferred)
{
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
  RCLCPP_INFO(rclcpp::get_logger("HttpClient::asyncOnWrite"), "asyncOnWrite");
#endif
  boost::ignore_unused(bytes_transferred);

  if (ec) {
    return fail(ec, "write");
  }

  if (0 < timeout_msec) {
    deadline_.expires_after(std::chrono::milliseconds(timeout_msec));
  }
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
  RCLCPP_INFO(rclcpp::get_logger("HttpClient::asyncOnWrite"), "async_read");
#endif
  // Receive the HTTP response
  boost::beast::http::async_read(
    *m_socket, m_recv_buffer, m_res,
    [this](boost::system::error_code ec, std::size_t bytes_transferred) {
      asyncOnRead(ec, bytes_transferred);
    });
}

void HttpClient::asyncOnRead(
  boost::system::error_code ec,
  std::size_t bytes_transferred)
{
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
  RCLCPP_INFO(rclcpp::get_logger("HttpClient::asyncOnRead"), "asyncOnRead");
#endif
  boost::ignore_unused(bytes_transferred);

  if (ec) {
    return fail(ec, "read");
  }

#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
  // Write the message to standard out
  RCLCPP_INFO(rclcpp::get_logger("HttpClient::asyncOnRead"), "m_res: %s", m_res);
  std::cout << m_res << std::endl;
#endif

  m_res_string = m_res.body().data();
  m_recv_string = boost::beast::buffers_to_string(m_recv_buffer.data());

  // Gracefully close the socket
  m_socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);

  // not_connected happens sometimes so don't bother reporting it.
  if (ec && ec != boost::system::errc::not_connected) {
    return fail(ec, "shutdown");
  }

  // If we get here then the connection is closed gracefully
  if (0 < m_recv_string.length()) {
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
    RCLCPP_INFO(
      rclcpp::get_logger(
        "HttpClient::asyncOnRead(m_recv_string)"), "m_func: %s", m_recv_string.c_str());
#endif
    m_func(m_recv_string);
  } else {
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
    RCLCPP_INFO(
      rclcpp::get_logger(
        "HttpClient::asyncOnRead(m_res_string)"), "m_func: %s", m_res_string.c_str());
#endif
    m_func(m_res_string);
  }
}

std::string HttpClient::remote_host() const
{
  return m_remote_host;
}

uint16_t HttpClient::remote_port() const
{
  return m_remote_port;
}

void HttpClient::open()
{
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
  RCLCPP_INFO(rclcpp::get_logger("HttpClient::open"), "open: st");
#endif
  // beast::tcp_stream didn't work...

  auto const rr = m_resolver.resolve(m_remote_host, std::to_string(m_remote_port));
  boost::asio::connect(*m_socket, rr.begin(), rr.end());
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
  RCLCPP_INFO(rclcpp::get_logger("HttpClient::open"), "open: ed");
#endif

}

void HttpClient::close()
{
#ifdef WITH_DEBUG_STDCOUT_HTTP_CLIENT
  RCLCPP_INFO(rclcpp::get_logger("HttpClient::close"), "close: st");
#endif
  stopped_ = true;
  deadline_.cancel();
  if (isOpen()) {
    boost::system::error_code ec;
    m_socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    if (ec && ec != boost::system::errc::not_connected) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("HttpClient::close"), ec.message());
    }
  }
  m_socket.reset(new boost::asio::ip::tcp::socket(*m_ctx));
}

bool HttpClient::isOpen() const
{
  return m_socket->is_open();
}


void HttpClient::check_deadline()
{
  if (!HttpClient::isOpen()) {
    return;
  }

  // Check whether the deadline has passed. We compare the deadline against
  // the current time since a new asynchronous operation may have moved the
  // deadline before this actor had a chance to run.
  if (deadline_.expiry() <= boost::asio::steady_timer::clock_type::now()) {
    std::cout << "expired..." << std::endl;
    // The deadline has passed. The socket is closed so that any outstanding
    // asynchronous operations are cancelled.
    HttpClient::close();

    // There is no longer an active deadline. The expiry is set to the
    // maximum time point so that the actor takes no action until a new
    // deadline is set.
    deadline_.expires_at(boost::asio::steady_timer::time_point::max());
  }

}

}     // namespace tcp_driver
} // namespace drivers
