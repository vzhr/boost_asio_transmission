#ifndef ASIO_TRANS_SERIAL_SESSION_H
#define ASIO_TRANS_SERIAL_SESSION_H

#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include "session.h"

namespace asio_trans
{

class SerialSession : public Session<boost::asio::serial_port>
{
public:
  SerialSession(boost::asio::io_service& io_service, std::string port, int baud)
    : Session(io_service), port_(port), baud_(baud), timer_(io_service)
  {
    std::cout << "rosserial_server session configured for " << port_ << " at " << baud << "bps." << std::endl;

    failed_connection_attempts_ = 0;
    check_connection();
  }

private:
  void check_connection()
  {
    if (!is_active())
    {
      attempt_connection();
    }

    // Every second, check again if the connection should be reinitialized,
    // if the ROS node is still up.
    if (true)
    {
      timer_.expires_from_now(boost::posix_time::milliseconds(2000));
      timer_.async_wait(boost::bind(&SerialSession::check_connection, this));
    }
    else
    {
      shutdown();
    }
  }

  void attempt_connection()
  {
    printf("Opening serial port.");

    boost::system::error_code ec;
    socket().open(port_, ec);
    if (ec) {
      failed_connection_attempts_++;
      if (failed_connection_attempts_ == 1) {
        std::cerr << "Unable to open port " << port_ << ": " << ec << std::endl;
      } else {
        std::cerr << "Unable to open port " << port_ << " (" << failed_connection_attempts_ << "): " << ec << std::endl;
      }
      return;
    }
    std::cout << "Opened " << port_ << std::endl;
    failed_connection_attempts_ = 0;

    typedef boost::asio::serial_port_base serial;
    socket().set_option(serial::baud_rate(baud_));
    socket().set_option(serial::character_size(8));
    socket().set_option(serial::stop_bits(serial::stop_bits::one));
    socket().set_option(serial::parity(serial::parity::none));
    socket().set_option(serial::flow_control(serial::flow_control::none));

    // Kick off the session.
    start();
  }

  std::string port_;
  int baud_;
  boost::asio::deadline_timer timer_;
  int failed_connection_attempts_;
};

}  // namespace

#endif  // ASIO_TRANS_SERIAL_SESSION_H
