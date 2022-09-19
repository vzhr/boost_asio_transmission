#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <udp_socket_session.h>
using boost::asio::ip::udp;
using boost::asio::ip::address;

void p(const uint8_t* data, size_t len){
//  std::cout <<"callback: ";

  std::string dd((char *)data, len);
  std::cout << dd << std::endl;
}

int main()
{
  boost::asio::io_service io_service;
  int server_port = 8989;
  int server_port2 = 9898;

  // whether specify client
  std::string client_addr{"127.0.0.1"};
  int client_port = 5555;


  asio_trans::UdpSocketSession udp_socket_session(
      io_service,
      udp::endpoint(udp::v4(), server_port),
      udp::endpoint(address::from_string(client_addr), client_port));

  asio_trans::UdpSocketSession udp_socket_session2(
      io_service,
      udp::endpoint(udp::v4(), server_port2));

  udp_socket_session.addCallback([](auto && PH1, auto && PH2) { return p(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2)); });
  udp_socket_session2.addCallback([](auto && PH1, auto && PH2) { return p(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2)); });

  io_service.run();
  return 0;
}