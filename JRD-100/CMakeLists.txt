cmake_minimum_required(VERSION 3.10)
project(JRD100_RFID)

set(CMAKE_CXX_STANDARD 17)

add_library(jrd100 STATIC jrd100.cpp)
target_include_directories(jrd100 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(ReadMultipleTag examples/ReadMultipleTag.cpp)
target_link_libraries(ReadMultipleTag jrd100)

add_executable(WriteTag examples/WriteTag.cpp)
target_link_libraries(WriteTag jrd100)
