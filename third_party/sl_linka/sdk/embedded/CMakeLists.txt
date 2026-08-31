cmake_minimum_required(VERSION 3.10)
project(sl_link C)

set(CMAKE_C_STANDARD 99)

# Options
option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_TESTS "Build test programs" OFF)

# Generated protobuf files directory (消息 payload)
set(GEN_C_DIR "${CMAKE_CURRENT_SOURCE_DIR}/message_gen")

find_package(Protobuf REQUIRED)
find_package(nanopb CONFIG REQUIRED)

set(PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../proto/sl_link.proto)
set(NANOPB_PROTO_DIR "/usr/lib/python3/dist-packages/proto")
set(NANOPB_GENERATOR "/usr/bin/nanopb_generator.py")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GEN_C_DIR}
)

add_custom_command(
    OUTPUT ${GEN_C_DIR}/sl_link.pb.c ${GEN_C_DIR}/sl_link.pb.h
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
            -I${CMAKE_CURRENT_SOURCE_DIR}/../../proto
            -I${NANOPB_PROTO_DIR}
            -o ${GEN_C_DIR}/sl_link.pb
            ${PROTO_FILE}
    COMMAND python3 ${NANOPB_GENERATOR}
            -D${GEN_C_DIR}
            ${GEN_C_DIR}/sl_link.pb
    DEPENDS ${PROTO_FILE}
    COMMENT "Generating NanoPb files to message_gen/"
)

# Source files
set(SL_LINK_SOURCES
    src/sl_crc16.c
    src/sl_packet.c
    src/sl_nanopb.c
    src/sl_link_monitor.c
    ${GEN_C_DIR}/sl_link.pb.c
)

# Static library
add_library(sllink STATIC ${SL_LINK_SOURCES})
target_link_libraries(sllink PUBLIC protobuf-nanopb)

# Example executable
if(BUILD_EXAMPLES)
    add_executable(sl_link_example examples/main.c)
    target_link_libraries(sl_link_example sllink)
endif()

# Tests
if(BUILD_TESTS)
    enable_testing()
    
    add_executable(test_crc16 tests/test_crc16.c)
    target_link_libraries(test_crc16 sllink)
    add_test(NAME crc16_test COMMAND test_crc16)
    
    add_executable(test_packet tests/test_packet.c)
    target_link_libraries(test_packet sllink)
    add_test(NAME packet_test COMMAND test_packet)
    
    add_executable(test_nanopb tests/test_nanopb.c)
    target_link_libraries(test_nanopb sllink)
    add_test(NAME nanopb_test COMMAND test_nanopb)
endif()

# Installation
install(TARGETS sllink
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include/sl_link
    FILES_MATCHING PATTERN "*.h"
)

install(FILES
    ${GEN_C_DIR}/sl_link.pb.h
    DESTINATION include/sl_link
)

# Print configuration
message(STATUS "SL-LinkA Configuration:")
message(STATUS "  Build examples: ${BUILD_EXAMPLES}")
message(STATUS "  Build tests: ${BUILD_TESTS}")
message(STATUS "  NanoPb proto dir: ${NANOPB_PROTO_DIR}")
