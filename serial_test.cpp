//
// Created by zhanghairong on 22-7-28.
//
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <serial_session.h>

void p(const uint8_t* data, size_t len){
  //  std::cout <<"callback: ";
  for (int i = 0; i < len; ++i)
  {
    std::cout << std::hex << (data[i] & 0xff) << " ";
  }
  std::cout << std::endl;
}

int main()
{
  std::string port{"/dev/ttyUSB0"};
  int baud = 115200;

  boost::asio::io_service io_service;
  asio_trans::SerialSession serial_session(io_service, port, baud);
  serial_session.addCallback(p);

  io_service.run();

  return 0;
}