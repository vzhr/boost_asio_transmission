#ifndef ASIO_TRANS_TCP_SOCKET_SESSION_H
#define ASIO_TRANS_TCP_SOCKET_SESSION_H

#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include "session.h"
#include <set>
#include <mutex>
namespace asio_trans
{

using boost::asio::ip::tcp;

template <typename Session = asio_trans::Session<tcp::socket>>
class TcpServer
{
public:
  TcpServer(boost::asio::io_service& io_service, short port)
    : io_service_(io_service), acceptor_(io_service, tcp::endpoint(tcp::v4(), port)), clean_timer_(io_service)
  {
    start_accept();
    check_connection();
  }
  virtual ~TcpServer()
  {
    std::cout << "bye\n";
    std::for_each(session_set_.begin(), session_set_.end(), [](Session* it) {
      std::cout << "delete item\n";
      delete it;
    });
  }

  void addCallback(boost::function<void(const uint8_t* data, size_t len)> f)
  {
    callbacks_.push_back(f);
    for (auto& m : session_set_)
    {
      m->addCallback(f);
    }
  }

  void write_message(Buffer& message)
  {
    {
      std::lock_guard<std::mutex> lk(session_mutex_);
      for (auto it = session_set_.begin(); it != session_set_.end();)
      {
        if (!(*it)->is_active())
        {
          delete *it;
          it = session_set_.erase(it);
        }
        else
        {
          std::cout << "send to: " << (*it)->socket().remote_endpoint().address()
                    << ", port: " << (*it)->socket().remote_endpoint().port() << std::endl;

          (*it)->write_message(message);
          ++it;
        }
      }
    }
  }
  void write_message(const uint8_t* data, size_t length)
  {
    Buffer message(length);
    memcpy(&message[0], data, length);
    write_message(message);
  }


private:
  void check_connection()
  {
    clean_timer_.expires_from_now(boost::posix_time::milliseconds(2000));
    clean_timer_.async_wait(boost::bind(&TcpServer::check_connection, this));

    {
      std::lock_guard<std::mutex> lk(session_mutex_);
      for (auto it = session_set_.begin(); it != session_set_.end();)
      {
        if (!(*it)->is_active())
        {
          delete *it;
          it = session_set_.erase(it);
        }
        else
        {
          ++it;
        }
      }
    }

  }
  void start_accept()
  {
    Session* new_session = new Session(io_service_);
    {
      std::lock_guard<std::mutex> lk(session_mutex_);
      session_set_.insert(new_session);
    }
    acceptor_.async_accept(new_session->socket(),
                           boost::bind(&TcpServer::handle_accept, this, new_session, boost::asio::placeholders::error));
  }

  void handle_accept(Session* new_session, const boost::system::error_code& error)
  {
    if (!error)
    {
      std::cout << "connection: " << new_session->socket().remote_endpoint().address()
                << ", port: " << new_session->socket().remote_endpoint().port() << std::endl;
      new_session->start();
      for (auto& item : callbacks_)
      {
        new_session->addCallback(item);
      }
    }
    else
    {
      //      session_set_.erase(new_session);
      delete new_session;
    }

    start_accept();
  }
  std::set<Session*> session_set_;
  boost::asio::deadline_timer clean_timer_;
  std::mutex session_mutex_;
  boost::asio::io_service& io_service_;
  tcp::acceptor acceptor_;
  std::vector<boost::function<void(const uint8_t*, size_t)>> callbacks_;
};

}  // namespace asio_trans

#endif  // ASIO_TRANS_TCP_SOCKET_SESSION_H
