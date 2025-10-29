cmake_minimum_required(VERSION 3.10)
project(JRD100_Lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

include_directories(JRD-100)

add_library(JRD100 STATIC
    JRD-100/jrd100.cpp
)

add_executable(WriteTag examples/WriteTag.cpp)
target_link_libraries(WriteTag PRIVATE JRD100)

add_executable(ReadMultipleTag examples/ReadMultipleTag.cpp)
target_link_libraries(ReadMultipleTag PRIVATE JRD100)

add_executable(MultiplePollingIncTx examples/MultiplePollingIncTx.cpp)
target_link_libraries(MultiplePollingIncTx PRIVATE JRD100)


set_target_properties(WriteTag PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set_target_properties(ReadMultipleTag PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set_target_properties(MultiplePollingIncTx PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
