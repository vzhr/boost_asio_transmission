
#ifndef ASIO_TRANS_TCP_SOCKET_CLIENT_SESSION_H
#define ASIO_TRANS_TCP_SOCKET_CLIENT_SESSION_H

#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include "session.h"
#include "udp_stream.h"

namespace asio_trans
{

using boost::asio::ip::tcp;

class TCPClientSession : public Session<tcp::socket>
{
public:
  TCPClientSession(boost::asio::io_service& io_service, tcp::endpoint server_endpoint, tcp::endpoint client_endpoint)
    : Session(io_service), timer_(io_service), server_endpoint_(server_endpoint), client_endpoint_(client_endpoint)
  {
    std::cout << "asio_trans TCP session created between " << server_endpoint << " and " << client_endpoint
              << std::endl;
    check_connection();
  }
  TCPClientSession(boost::asio::io_service& io_service, tcp::endpoint server_endpoint)
    : Session(io_service), timer_(io_service), server_endpoint_(server_endpoint)
  {
    check_connection_server();
  }

private:
  void check_connection_server()
  {
    if (!is_active())
    {
      //      socket().close();
      if (!socket().is_open())
        socket().open(tcp::v4());

      //      socket().bind(client_endpoint_);
      socket().async_connect(server_endpoint_,
                             boost::bind(&TCPClientSession::handle_connect, this, boost::asio::placeholders::error));
    }
    // Every second, check again if the connection should be reinitialized,
    // if the ROS node is still up.
    if (true)
    {
      timer_.cancel();
      timer_.expires_from_now(boost::posix_time::milliseconds(2000));
      timer_.async_wait(boost::bind(&TCPClientSession::check_connection_server, this));
    }
    else
    {
      shutdown();
    }
  }
  void check_connection()
  {
    if (!is_active())
    {
      socket().close();
      socket().open(tcp::v4());
      socket().bind(client_endpoint_);
      socket().async_connect(server_endpoint_,
                             boost::bind(&TCPClientSession::handle_connect, this, boost::asio::placeholders::error));
    }

    // Every second, check again if the connection should be reinitialized,
    // if the ROS node is still up.
    if (true)
    {
      timer_.cancel();
      timer_.expires_from_now(boost::posix_time::milliseconds(2000));
      timer_.async_wait(boost::bind(&TCPClientSession::check_connection, this));
    }
    else
    {
      shutdown();
    }
  }
  void handle_connect(const boost::system::error_code& error)
  {
    if (!error)
    {
      std::cout << "TCP connections on port " << socket().local_endpoint().port() << std::endl;
      std::cout << "TCP connections on server address: " << socket().remote_endpoint().address().to_string()
                << " port: " << socket().remote_endpoint().port() << std::endl;
      client_endpoint_ = socket().local_endpoint();
      start();
    }
    if (error)
    {
      if (error == boost::system::errc::io_error)
      {
        std::cerr << "Socket write operation returned IO error." << std::endl;
      }
      else if (error == boost::system::errc::no_such_device)
      {
        std::cerr << "Socket write operation returned no device." << std::endl;
      }
      else
      {
        std::cerr << "Unknown error returned during write operation: " << error << std::endl;
      }
    }
  }
  boost::asio::deadline_timer timer_;
  tcp::endpoint server_endpoint_;
  tcp::endpoint client_endpoint_;
};

}  // namespace asio_trans

#endif  // ASIO_TRANS_UDP_SOCKET_SESSION_H
