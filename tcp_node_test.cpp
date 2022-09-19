#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <tcp_socket_session.h>
using boost::asio::ip::udp;
using boost::asio::ip::address;

void p(const uint8_t* data, size_t len){
//  std::cout <<"callback: ";

  std::string dd((char *)data, len);
  std::cout << dd << std::endl;
}

int main()
{
  int port = 10086;
  boost::asio::io_service io_service;
  asio_trans::TcpServer<> tcp_server(io_service, port);
  std::cout << "Listening for rosserial TCP connections on port " << port << std::endl;
  tcp_server.addCallback(boost::bind(&p,_1,_2));
  tcp_server.addCallback(boost::bind(&asio_trans::TcpServer<>::write_message,&tcp_server,_1,_2));
  io_service.run();
  return 0;
}