cmake_minimum_required(VERSION 3.20)

project(open1722 VERSION 0.1)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED TRUE)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib")

#### Libraries ################################################################

# PDU parsing library
add_library(open1722pdu SHARED 
    "src/avtp/pdu/CommonHeader.c"
    "src/avtp/pdu/Crf.c"
    "src/avtp/pdu/Rvf.c"
    "src/avtp/pdu/Udp.c"
    "src/avtp/pdu/Utils.c"
    "src/avtp/pdu/aaf/CommonStream.c"
    "src/avtp/pdu/aaf/PcmStream.c"
    "src/avtp/pdu/acf/Can.c"
    "src/avtp/pdu/acf/CanBrief.c"
    "src/avtp/pdu/acf/Common.c"
    "src/avtp/pdu/acf/Ntscf.c"
    "src/avtp/pdu/acf/Sensor.c"
    "src/avtp/pdu/acf/SensorBrief.c"
    "src/avtp/pdu/acf/Tscf.c"
    "src/avtp/pdu/cvf/Cvf.c"
    "src/avtp/pdu/cvf/H264.c"
    "src/avtp/pdu/cvf/Jpeg2000.c"
    "src/avtp/pdu/cvf/Mjpeg.c")
target_include_directories(open1722pdu PRIVATE "${PROJECT_BINARY_DIR}/include")

# link_directories(lib)

#### Examples #################################################################

# Common library accross all examples
add_library(open1722examples STATIC "examples/common/common.c")
target_include_directories(open1722examples PRIVATE
    "${PROJECT_BINARY_DIR}/examples"
    "${PROJECT_BINARY_DIR}/include")

# AAF listener app
add_executable(aaf-listener "examples/aaf/aaf-listener.c")
target_include_directories(aaf-listener PRIVATE
    "${PROJECT_BINARY_DIR}/examples"
    "${PROJECT_BINARY_DIR}/include")
target_link_libraries(aaf-listener open1722pdu open1722examples)

# AAF talker app
add_executable(aaf-talker "examples/aaf/aaf-talker.c")
target_include_directories(aaf-talker PRIVATE
    "${PROJECT_BINARY_DIR}/examples"
    "${PROJECT_BINARY_DIR}/include")
target_link_libraries(aaf-talker open1722pdu open1722examples)

# CAN talker app
add_executable(acf-can-talker "examples/acf-can/acf-can-talker.c")
target_include_directories(acf-can-talker PRIVATE
    "${PROJECT_BINARY_DIR}/examples"
    "${PROJECT_BINARY_DIR}/include")
target_link_libraries(acf-can-talker open1722pdu open1722examples)

# CAN listener app
add_executable(acf-can-listener "examples/acf-can/acf-can-listener.c")
target_include_directories(acf-can-listener PRIVATE
    "${PROJECT_BINARY_DIR}/examples"
    "${PROJECT_BINARY_DIR}/include")
target_link_libraries(acf-can-listener open1722pdu open1722examples)

# CRF talker app
add_executable(crf-talker "examples/crf/crf-talker.c")
target_include_directories(crf-talker PRIVATE
    "${PROJECT_BINARY_DIR}/examples"
    "${PROJECT_BINARY_DIR}/include")
target_link_libraries(crf-talker open1722pdu open1722examples m)

# CRF listener app
add_executable(crf-listener "examples/crf/crf-listener.c")
target_include_directories(crf-listener PRIVATE
    "${PROJECT_BINARY_DIR}/examples"
    "${PROJECT_BINARY_DIR}/include")
target_link_libraries(crf-listener open1722pdu open1722examples m)

# CVF talker app
add_executable(cvf-talker "examples/cvf/cvf-talker.c")
target_include_directories(cvf-talker PRIVATE
    "${PROJECT_BINARY_DIR}/examples"
    "${PROJECT_BINARY_DIR}/include")
target_link_libraries(cvf-talker open1722pdu open1722examples)

# CVF listener app
add_executable(cvf-listener "examples/cvf/cvf-listener.c")
target_include_directories(cvf-listener PRIVATE
    "${PROJECT_BINARY_DIR}/examples"
    "${PROJECT_BINARY_DIR}/include")
target_link_libraries(cvf-listener open1722pdu open1722examples)

#### Tests ####################################################################

enable_testing()

# find_package(cmocka 1.1.0 REQUIRED)

list(APPEND TEST_TARGETS test-aaf)
list(APPEND TEST_TARGETS test-avtp)
list(APPEND TEST_TARGETS test-can)
list(APPEND TEST_TARGETS test-crf)
list(APPEND TEST_TARGETS test-cvf)
list(APPEND TEST_TARGETS test-rvf)
# list(APPEND TEST_TARGETS test-stream)

foreach(TEST_TARGET IN LISTS TEST_TARGETS)
    add_executable(${TEST_TARGET} "unit/${TEST_TARGET}.c")
    target_include_directories(${TEST_TARGET} PRIVATE "${PROJECT_BINARY_DIR}/include")
    target_link_libraries(${TEST_TARGET} open1722pdu cmocka m)
    add_test(NAME ${TEST_TARGET} COMMAND "${PROJECT_BINARY_DIR}/bin/${TEST_TARGET}")
endforeach()

#### Install ##################################################################

install(TARGETS open1722pdu DESTINATION lib)
install(TARGETS
    aaf-listener
    aaf-talker
    acf-can-listener
    acf-can-talker
    crf-listener
    crf-talker
    cvf-listener
    cvf-talker
    DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

#### Packaging ################################################################

include(InstallRequiredSystemLibraries)
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_VERSION_MAJOR "${open1722_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${open1722_VERSION_MINOR}")
set(CPACK_PACKAGE_DIRECTORY "${PROJECT_BINARY_DIR}/release")
set(CPACK_GENERATOR "TGZ" "DEB")
set(CPACK_SOURCE_GENERATOR "TGZ" "DEB")

# Debian package
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Adriaan Niess [Robert Bosch GmbH]")

include(CPack)
