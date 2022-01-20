### ###
### @author   imindude@gmail.com
### ###

project(Misc)

### ***

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_BINARY_DIR}
)

### ***

add_library(${PROJECT_NAME}
  AppConfig.cpp
)

### ###
