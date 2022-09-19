

#ifndef ASIO_TRANS_UDP_STREAM_H
#define ASIO_TRANS_UDP_STREAM_H

#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include "session.h"


namespace asio_trans
{

using boost::asio::ip::udp;
#if BOOST_VERSION < 107000
using boost::asio::handler_type;
#endif


class UdpStream : public udp::socket
{
public:
  explicit UdpStream(boost::asio::io_service& io_service) : udp::socket(io_service)
  {
  }

  void open(udp::endpoint server_endpoint, udp::endpoint client_endpoint)
  {
    boost::system::error_code ec;
    const protocol_type protocol = server_endpoint.protocol();

    udp::socket::open(protocol, ec);
    boost::asio::detail::throw_error(ec, "open");

    udp::socket::bind(server_endpoint, ec);
    boost::asio::detail::throw_error(ec, "bind");
    client_endpoint_ = client_endpoint;
    specify_client_flag_ = true;
  }
  void open(udp::endpoint server_endpoint)
  {
    boost::system::error_code ec;
    const protocol_type protocol = server_endpoint.protocol();

    udp::socket::open(protocol, ec);
    boost::asio::detail::throw_error(ec, "open");

    udp::socket::bind(server_endpoint, ec);
    boost::asio::detail::throw_error(ec, "bind");
    specify_client_flag_ = false;
  }

  template <typename ConstBufferSequence, typename WriteHandler>
  BOOST_ASIO_INITFN_RESULT_TYPE(WriteHandler,
      void (boost::system::error_code, std::size_t))
  async_write_some(const ConstBufferSequence& buffers,
      BOOST_ASIO_MOVE_ARG(WriteHandler) handler)
  {
    // If you get an error on the following line it means that your handler does
    // not meet the documented type requirements for a WriteHandler.
    BOOST_ASIO_WRITE_HANDLER_CHECK(WriteHandler, handler) type_check;
#if (BOOST_VERSION >= 106600)
    // See: http://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio/net_ts.html
    boost::asio::async_completion<WriteHandler,
      void (boost::system::error_code, std::size_t)> init(handler);

    udp::socket::async_send_to(
        buffers, client_endpoint_, 0, init.completion_handler);

    return init.result.get();
#else
    return this->get_service().async_send_to(
        this->get_implementation(), buffers, client_endpoint_, 0,
        BOOST_ASIO_MOVE_CAST(WriteHandler)(handler));
#endif
  }

  template <typename MutableBufferSequence, typename ReadHandler>
  BOOST_ASIO_INITFN_RESULT_TYPE(ReadHandler,
      void (boost::system::error_code, std::size_t))
  async_read_some(const MutableBufferSequence& buffers,
      BOOST_ASIO_MOVE_ARG(ReadHandler) handler)
  {
    // If you get an error on the following line it means that your handler does
    // not meet the documented type requirements for a ReadHandler.
    BOOST_ASIO_READ_HANDLER_CHECK(ReadHandler, handler) type_check;
#if (BOOST_VERSION >= 106600)
    // See: http://www.boost.org/doc/libs/1_66_0/doc/html/boost_asio/net_ts.html
    boost::asio::async_completion<ReadHandler,
      void (boost::system::error_code, std::size_t)> init(handler);

    if (specify_client_flag_){
      udp::socket::async_receive(buffers, 0, init.completion_handler);
    }
    else{
      udp::socket::async_receive_from(
          buffers, client_endpoint_, init.completion_handler);
    }



    return init.result.get();
#else
    return this->get_service().async_receive_from(
        this->get_implementation(), buffers, client_endpoint_, 0,
        BOOST_ASIO_MOVE_CAST(ReadHandler)(handler));
#endif
  }

private:
  udp::endpoint client_endpoint_;
  bool specify_client_flag_ = false;
};

}  // namespace

#endif  // ASIO_TRANS_UDP_STREAM_H
