### ###
### @author   imindude@gmail.com
### ###

project(Service)

### ***

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

### ***

add_library(${PROJECT_NAME}
  PiBalancer_Impl.cpp
  PiBalancer.cpp
)

### ###
