/**
 *
 *  \file
 *  \brief      Helper object for successive reads from a ReadStream.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef ROSSERIAL_SERVER_ASYNC_READ_BUFFER_H
#define ROSSERIAL_SERVER_ASYNC_READ_BUFFER_H

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <iostream>

// ssize_t is POSIX-only type. Use make_signed for portable code.
#include <cstdint> // size_t
#include <type_traits> // std::make_signed
typedef std::make_signed<size_t>::type signed_size_t;

namespace asio_trans
{

template<typename AsyncReadStream>
class AsyncReadBuffer
{
public:
  AsyncReadBuffer(AsyncReadStream& s, size_t capacity,
                  boost::function<void(const boost::system::error_code&)> error_callback)
       : stream_(s), read_requested_bytes_(0), error_callback_(error_callback) {
    reset();
    mem_.resize(capacity);
  }

  void read_all(boost::function<void(const std::vector<uint8_t>&)> callback){
    read_success_callback_ = callback;
    if (bytesAvailable() > 0)
      memmove(&mem_[0], &mem_[read_index_], bytesAvailable());
    write_index_ = bytesAvailable();
    read_index_ = 0;
//    std::cout << "async_read: " << "Requesting transfer of all\n";
    boost::asio::async_read(stream_,
                            boost::asio::buffer(&mem_[write_index_], bytesHeadroom()),
                            boost::asio::transfer_at_least(1),
                            boost::bind(&AsyncReadBuffer::readAllCallback, this,
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));

  }

  void readAllCallback(const boost::system::error_code& error, size_t bytes_transferred)
  {
    if (error)
    {
      read_requested_bytes_ = 0;
      read_success_callback_.clear();
      if (error == boost::asio::error::operation_aborted)
      {
        // Special case for operation_aborted. The abort callback comes when the owning Session
        // is in the middle of teardown, which means the callback is no longer valid.
      }
      else
      {
        error_callback_(error);
      }
      return;
    }

    write_index_ += bytes_transferred;
    std::vector<uint8_t> buf(&mem_[read_index_], &mem_[read_index_] + bytesAvailable());
#if BOOST_VERSION >= 107000
    boost::asio::post(stream_.get_executor(), boost::bind(read_success_callback_, buf));
#else
    stream_.get_io_service().post(boost::bind(read_success_callback_, buf));
#endif
    reset();
  }



  /**
   * @brief Commands a fixed number of bytes from the buffer. This may be fulfilled from existing
   *        buffer content, or following a hardware read if required.
   */
  void read(size_t requested_bytes, boost::function<void(std::vector<uint8_t>)> callback) {
    read_success_callback_ = callback;
    read_requested_bytes_ = requested_bytes;

    if (read_requested_bytes_ > mem_.size())
    {
      error_callback_(boost::system::errc::make_error_code(boost::system::errc::no_buffer_space));
      return;
    }

    // Number of bytes which must be transferred to satisfy the request.
    signed_size_t transfer_bytes = read_requested_bytes_ - bytesAvailable();

    if (transfer_bytes > 0)
    {
      // If we don't have enough headroom in the buffer, we'll have to shift what's currently in there to make room.
      if (bytesHeadroom() < transfer_bytes)
      {
        memmove(&mem_[0], &mem_[read_index_], bytesAvailable());
        write_index_ = bytesAvailable();
        read_index_ = 0;
      }

      // Initiate a read from hardware so that we have enough bytes to fill the user request.
//      std::cout << "async_read: " << "Requesting transfer of at least " << transfer_bytes << " byte(s)." << std::endl;
      boost::asio::async_read(stream_,
          boost::asio::buffer(&mem_[write_index_], bytesHeadroom()),
          boost::asio::transfer_at_least(transfer_bytes),
          boost::bind(&AsyncReadBuffer::callback, this,
                      boost::asio::placeholders::error,
                      boost::asio::placeholders::bytes_transferred));
    }
    else
    {
      // We have enough in the buffer already, can fill the request without going to hardware.
      callSuccessCallback();
    }
  }

private:
  void reset()
  {
    read_index_ = 0;
    write_index_ = 0;
  }

  inline size_t bytesAvailable()
  {
    return write_index_ - read_index_;
  }

  inline size_t bytesHeadroom()
  {
    return mem_.size() - write_index_;
  }

  /**
   * @brief The internal callback which is called by the boost::asio::async_read invocation
   *        in the public read method above.
   */
  void callback(const boost::system::error_code& error, size_t bytes_transferred)
  {
    if (error)
    {
      read_requested_bytes_ = 0;
      read_success_callback_.clear();
//      ROS_DEBUG_STREAM_NAMED("async_read", "Read operation failed with: " << error);

      if (error == boost::asio::error::operation_aborted)
      {
        // Special case for operation_aborted. The abort callback comes when the owning Session
        // is in the middle of teardown, which means the callback is no longer valid.
      }
      else
      {
        error_callback_(error);
      }
      return;
    }

    write_index_ += bytes_transferred;
//    std::cout << bytes_transferred << std::endl;
//    ROS_DEBUG_STREAM_NAMED("async_read", "Successfully read " << bytes_transferred << " byte(s), now " << bytesAvailable() << " available.");
    callSuccessCallback();
  }

  /**
   * @brief Calls the user's callback. This is a separate function because it gets called from two
   *        places, depending whether or not an actual HW read is required to fill the request.
   */
  void callSuccessCallback()
  {

    // Post the callback rather than executing it here so, so that we have a chance to do the cleanup
    // below prior to it actually getting run, in the event that the callback queues up another read.

    std::vector<uint8_t> buf(&mem_[read_index_], &mem_[read_index_] + read_requested_bytes_);
#if BOOST_VERSION >= 107000

    boost::asio::post(stream_.get_executor(), boost::bind(read_success_callback_, buf));
#else
    stream_.get_io_service().post(boost::bind(read_success_callback_, buf));
#endif
    read_index_ += read_requested_bytes_;
    // Resetting these values clears our state so that we know there isn't a callback pending.
    read_requested_bytes_ = 0;
    read_success_callback_.clear();

    if (bytesAvailable() == 0)
    {
      reset();
    }
  }

  AsyncReadStream& stream_;
  std::vector<uint8_t> mem_;

  size_t write_index_;
  size_t read_index_;
  boost::function<void(const boost::system::error_code&)> error_callback_;

  boost::function<void(const std::vector<uint8_t>&)> read_success_callback_;
  size_t read_requested_bytes_;
};

}  // namespace

#endif  // ROSSERIAL_SERVER_ASYNC_READ_BUFFER_H
