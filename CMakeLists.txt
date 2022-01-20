### ###
### @author   imindude@gmail.com
### ###

cmake_minimum_required(VERSION 3.0.0)
project(PiBalancer VERSION 0.0.1)
configure_file(version.h.in version.h)

# set(CMAKE_VERBOSE_MAKEFILE ON)

### ***

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
# set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(COMMON_CFLAGS "-pthread -Wall") # -Werror")

set(CMAKE_CXX_FLAGS "${COMMON_CFLAGS}")
set(CMAKE_C_FLAGS "${COMMON_CFLAGS}")

### ***

find_package(Boost REQUIRED COMPONENTS thread chrono)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/Libs
)

link_libraries(
  ${Boost_THREAD_LIBRARIES}
  ${Boost_CHRONO_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  i2c
)

### ***

add_subdirectory(Service)
add_subdirectory(Module)
add_subdirectory(Device)
add_subdirectory(Misc)
add_subdirectory(Libs/Fusion)

### ***

add_executable(${PROJECT_NAME}
  Main.cpp
)

target_link_libraries(${PROJECT_NAME}
  Service
  Module
  Device
  Misc
  Fusion
)

### ***

add_executable(${PROJECT_NAME}_bias
  MainBias.cpp
)

target_link_libraries(${PROJECT_NAME}_bias
  Device
  Misc
)

### ***

add_custom_target(
  build
  DEPENDS ${PROJECT_NAME} ${PROJECT_NAME}_bias
)

add_custom_target(
  balancer
  DEPENDS ${PROJECT_NAME}
)

add_custom_target(
  calibrator
  DEPENDS ${PROJECT_NAME}_bias
)

### ***

file(COPY ${CMAKE_SOURCE_DIR}/PiBalancerConfig.json DESTINATION ${CMAKE_BINARY_DIR}/)

### ###
