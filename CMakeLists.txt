cmake_minimum_required(VERSION 3.2)
project(boost_asio_trans)

set(CMAKE_CXX_STANDARD 14)
find_package(Boost COMPONENTS system thread)
add_executable(udp_node_test udp_node_test.cpp)
target_include_directories(udp_node_test PUBLIC include)
target_link_libraries(udp_node_test Boost::thread Boost::system)

add_executable(tcp_node_test tcp_node_test.cpp)
target_include_directories(tcp_node_test PUBLIC include)
target_link_libraries(tcp_node_test Boost::thread Boost::system)
add_executable(tcp_client_test tcp_client_test.cpp)
target_include_directories(tcp_client_test PUBLIC include)
target_link_libraries(tcp_client_test Boost::thread Boost::system)


add_executable(tcp_udp_test tcp_udp_test.cpp)
target_include_directories(tcp_udp_test PUBLIC include)
target_link_libraries(tcp_udp_test Boost::thread Boost::system)

add_executable(serial_test serial_test.cpp)
target_include_directories(serial_test PUBLIC include)
target_link_libraries(serial_test Boost::thread Boost::system)