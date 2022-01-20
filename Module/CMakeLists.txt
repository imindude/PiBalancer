### ###
### @author   imindude@gmail.com
### ###

project(Module)

### ***

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

### ***

add_library(${PROJECT_NAME}
  InertialMotion_Impl.cpp
  InertialMotion.cpp
  ManualControl_Impl.cpp
  ManualControl.cpp
  MotorMixer_Impl.cpp
  MotorMixer.cpp
  PidControl.cpp
)

### ###
