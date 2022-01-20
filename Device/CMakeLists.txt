### ###
### @author   imindude@gmail.com
### ###

project(Device)

### ***

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

### ***

add_library(${PROJECT_NAME}
  GamepadInput.cpp
  MpuSensor.cpp
  MotorDriver.cpp
)

### ###
