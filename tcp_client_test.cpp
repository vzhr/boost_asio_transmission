#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <tcp_socket_client_session.h>
using boost::asio::ip::tcp;
using boost::asio::ip::address;

void p(const uint8_t* data, size_t len){
//  std::cout <<"callback: ";

  std::string dd((char *)data, len);
  std::cout << dd;
}

int main()
{
  int client_port = 10088;
  int server_port = 9898;
  boost::asio::io_service io_service;
  tcp::endpoint server_endpoint(address::from_string("127.0.0.1"), server_port);
  tcp::endpoint client_endpoint(tcp::v4(), client_port);
//  asio_trans::TCPClientSession tcp_client(io_service, server_endpoint,client_endpoint);
  asio_trans::TCPClientSession tcp_client(io_service, server_endpoint);
  tcp_client.addCallback(boost::bind(&p,_1,_2));
  io_service.run();
  return 0;
}