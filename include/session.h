
#ifndef ASIO_TRANS_H
#define ASIO_TRANS_H
#include <iostream>
#include <map>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/noncopyable.hpp>
#include "async_read_buffer.h"
namespace asio_trans
{

typedef std::vector<uint8_t> Buffer;
typedef boost::shared_ptr<Buffer> BufferPtr;

template<typename Socket>
class Session : boost::noncopyable
{
public:
  Session(boost::asio::io_service& io_service)
    : io_service_(io_service),
      socket_(io_service),
      sync_timer_(io_service),
      async_read_buffer_(socket_, buffer_max,
                         boost::bind(&Session::read_failed, this,
                                     boost::asio::placeholders::error))
  {
    active_ = false;
    attempt_interval_ = boost::posix_time::milliseconds(1000 * 1000);
    attempt_sync();
  }
  void start()
  {
    std::cout << "Starting session." << std::endl;
    active_ = true;
    attempt_sync();
    read_start();
  }

  Socket& socket()
  {
    return socket_;
  }
  void addCallback(boost::function<void(const uint8_t*, size_t)> f){
    callbacks_.push_back(f);
  }


  void stop()
  {
    sync_timer_.cancel();
//    callbacks_.clear();
    // Close the socket.
    socket_.close();
    active_ = false;
  }

  void shutdown()
  {
    if (is_active())
    {
      stop();
    }
    io_service_.stop();
  }

  bool is_active()
  {
    return active_;
  }

  //// SENDING MESSAGES ////

  void write_message(const uint8_t* data, size_t length)
  {
    Buffer message(length);
    memcpy(&message.at(0),data, length);
    write_message(message);
  }

  void write_message(Buffer& message)
  {
    auto length = message.size();
    BufferPtr buffer_ptr(new Buffer(length));
    memcpy(&buffer_ptr->at(0),&message[0],length);
    boost::asio::async_write(
        socket_, boost::asio::buffer(*buffer_ptr),
        boost::bind(&Session::write_completion_cb, this, boost::asio::placeholders::error, buffer_ptr));
  }

  void write_completion_cb(const boost::system::error_code& error, BufferPtr buffer_ptr)
  {
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
//      stop();
    }
//    printf("async_write: Sending buffer of %ld bytes to client.\n", buffer_ptr->size());
  }

private:

  //// RECEIVING MESSAGES ////
  // TODO: Total message timeout, implement primarily in ReadBuffer.

  void read_start()
  {
    attempt_sync();
    // read_sync_header();
    async_read_buffer_.read_all( boost::bind(&Session::read_callback, this, _1));
  }
  void read_callback(const Buffer& buffer)
  {
//    using namespace std::chrono_literals;
//    std::this_thread::sleep_for(100ms);

    for (auto& f : callbacks_)
    {
      f(buffer.data(), buffer.size());
    }
    read_start();
  }

  void read_sync_header() {
    attempt_sync();
    async_read_buffer_.read(1, boost::bind(&Session::read_sync_first, this, _1));
  }

  void read_sync_first(const Buffer& data) {
    uint8_t sync;
    sync = data[0];
    if (sync == 0x0f) {
      async_read_buffer_.read(34, boost::bind(&Session::read_sync_second, this, _1));
    } else {
      read_sync_header();
    }
  }

  void read_sync_second(const Buffer& data) {

    for (auto& f : callbacks_)
    {
      f(data.data(), data.size());
    }
    read_sync_header();
  }

  void read_body(const Buffer& data) {
    //process
    // callback

    // Kickoff next message read.
    read_sync_header();
  }


  void read_failed(const boost::system::error_code& error) {
    if (error == boost::system::errc::no_buffer_space) {
      // No worries. Begin syncing on a new message.
      std::cout << "Overrun on receive buffer. Attempting to regain rx sync." << std::endl;
      read_start();
    } else if (error) {
      // When some other read error has occurred, stop the session, which destroys
      // all known publishers and subscribers.
      std::cerr << "Socket asio error, closing socket: " << error << std::endl;
      stop();
    }
  }

  //// SYNC WATCHDOG ////
  void attempt_sync() {
    set_sync_timeout(attempt_interval_);
  }

  void set_sync_timeout(const boost::posix_time::time_duration& interval) {
    // change here
    if (true)
    {
      sync_timer_.cancel();
      sync_timer_.expires_from_now(interval);
      sync_timer_.async_wait(boost::bind(&Session::sync_timeout, this,
            boost::asio::placeholders::error));
    }
    else
    {
      shutdown();
    }
  }

  void sync_timeout(const boost::system::error_code& error) {
    if (error != boost::asio::error::operation_aborted) {
      std::cerr << "Sync with device lost." << std::endl;
      stop();
    }
  }

  boost::asio::io_service& io_service_;
  Socket socket_;
  AsyncReadBuffer<Socket> async_read_buffer_;
  enum { buffer_max = 1023 };
  bool active_;
  boost::posix_time::time_duration attempt_interval_;
  boost::asio::deadline_timer sync_timer_;
  std::vector<boost::function<void(const uint8_t*, size_t)>> callbacks_;

};

}  // namespace

#endif  // ASIO_TRANS_H
