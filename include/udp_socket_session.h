
#ifndef ASIO_TRANS_UDP_SOCKET_SESSION_H
#define ASIO_TRANS_UDP_SOCKET_SESSION_H

#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include "session.h"
#include "udp_stream.h"


namespace asio_trans
{

using boost::asio::ip::udp;

class UdpSocketSession : public Session<UdpStream>
{
public:
  UdpSocketSession(boost::asio::io_service& io_service,
                   udp::endpoint server_endpoint,
                   udp::endpoint client_endpoint)
    : Session(io_service), timer_(io_service),
      server_endpoint_(server_endpoint), client_endpoint_(client_endpoint), specify_client_flag_(true)
  {
    std::cout << "asio_trans UDP session created between " << server_endpoint << " and " << client_endpoint << std::endl;
    check_connection();
  }

  UdpSocketSession(boost::asio::io_service& io_service,
                   udp::endpoint server_endpoint)
    : Session(io_service), timer_(io_service),
    server_endpoint_(server_endpoint), specify_client_flag_(false)
  {
    std::cout << "asio UDP session created " << server_endpoint  << std::endl;
    check_connection();
  }

private:
  void check_connection()
  {
    if (!is_active())
    {
      if (specify_client_flag_)
      {
        socket().open(server_endpoint_, client_endpoint_);
      }
      else {
        socket().open(server_endpoint_);
      }
      start();
    }

    // Every second, check again if the connection should be reinitialized,
    // if the ROS node is still up.
    if (true)
    {
      timer_.expires_from_now(boost::posix_time::milliseconds(2000));
      timer_.async_wait(boost::bind(&UdpSocketSession::check_connection, this));
    }
    else
    {
      shutdown();
    }
  }

  boost::asio::deadline_timer timer_;
  udp::endpoint server_endpoint_;
  udp::endpoint client_endpoint_;
  bool specify_client_flag_ = false;
};

}  // namespace

#endif  // ASIO_TRANS_UDP_SOCKET_SESSION_H
