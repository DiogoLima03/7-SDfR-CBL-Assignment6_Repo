#ifndef REMOTE_CAPTURE_HPP_
#define REMOTE_CAPTURE_HPP_

#include <vector>
#include <string>
#include <iostream>
#include <mutex>

#include "boost/bind/bind.hpp"
#include "boost/asio.hpp"
#include "boost/array.hpp"
#include "opencv2/core/mat.hpp"

#include "../include/visibility_control.h"

namespace remote
{

#define BUFFER_SIZE 65536
#define CHUNK_SIZE 9216

  class RemoteCapture
  {
  public:
    cam2image_vm2ros_PUBLIC
    RemoteCapture();
    cam2image_vm2ros_PUBLIC ~RemoteCapture();
    cam2image_vm2ros_PUBLIC
        cv::Mat &
        get_frame();

    cam2image_vm2ros_PUBLIC void set_ip(std::string ip_, int port_);
    cam2image_vm2ros_PUBLIC void initialize(size_t width, size_t height, size_t timeout);
    cam2image_vm2ros_PUBLIC void shutdown() { webcam_down = true; };
    cam2image_vm2ros_PUBLIC bool webcam_is_down() { return webcam_down; };

  private:
    using udp = boost::asio::ip::udp;
    using address = boost::asio::ip::address;

    cam2image_vm2ros_LOCAL void add_handler();
    cam2image_vm2ros_LOCAL void handle_receive(const boost::system::error_code &error, size_t bytes_transferred);
    cam2image_vm2ros_LOCAL void run_receiver();

    boost::asio::io_service io_service;
    udp::socket socket{io_service, udp::v4()};
    boost::array<char, BUFFER_SIZE> recv_buffer;
    std::string frame_buffer;
    udp::endpoint remote_endpoint;
    cv::Mat remote_buf;
    std::string ip;
    int port;
    int io_service_timeout;
    bool webcam_down;
    bool initialized;
  };

} // namespace remote

#endif // REMOTE_CAPTURE_HPP_