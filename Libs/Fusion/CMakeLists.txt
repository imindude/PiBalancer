### ###
### @author   wuyong.yi@asoa.co.kr
### ###

project(Fusion)

### ***

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

### ***

add_library(${PROJECT_NAME}
  FusionAhrs.c
  FusionBias.c
  FusionCompass.c
)

### ###
