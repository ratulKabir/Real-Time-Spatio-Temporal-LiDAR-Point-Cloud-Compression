cmake_minimum_required(VERSION 3.16.3)
project(RTSTLPCC)

find_package(OpenCV REQUIRED
    Boost REQUIRED)

add_executable($(PROJECT_NAME) pcc_encoder.cpp 
    pcc_decoder.cpp 
    pcc_stream_encoder.cpp 
    pcc_stream_ecoder.cpp 
    pcc_test.cpp 
    pcc_stream_test.cpp)