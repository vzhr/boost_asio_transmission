//
// Created by zhanghairong on 22-7-27.
//
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <tcp_socket_session.h>
#include <udp_socket_session.h>
using boost::asio::ip::udp;
using boost::asio::ip::tcp;
using boost::asio::ip::address;

void p(const uint8_t* data, size_t len){
  //  std::cout <<"callback: ";

  std::string dd((char *)data, len);
  std::cout << dd << std::endl;
}

int main()
{
  int tcp_port = 10086;

  // udp
  int udp_server_port = 8989;
  // whether specify client
  std::string client_addr{"127.0.0.1"};
  int client_port = 5555;

  boost::asio::io_service io_service;

  asio_trans::TcpServer<> tcp_server(io_service, tcp_port);
  std::cout << "Listening for asiotrans TCP connections on port " << tcp_port << std::endl;

  // 1. specify udp client address
  // 2. or decide by connection
//  asio_trans::UdpSocketSession udp_socket_session(
//      io_service,
//      udp::endpoint(udp::v4(), udp_server_port),
//      udp::endpoint(address::from_string(client_addr), client_port));

  asio_trans::UdpSocketSession udp_socket_session(
      io_service,
      udp::endpoint(udp::v4(), udp_server_port));

  tcp_server.addCallback(boost::bind(&asio_trans::UdpSocketSession::write_message,&udp_socket_session,_1,_2));
  udp_socket_session.addCallback(boost::bind(&asio_trans::TcpServer<>::write_message,&tcp_server,_1,_2));
  udp_socket_session.addCallback(boost::bind(p, _1, _2));
  io_service.run();
  return 0;
}