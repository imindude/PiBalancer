/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#pragma once

/// ********************************************************************************************************************

#include <string>
#include <fstream>

/// ********************************************************************************************************************

class GamepadInput
{
public:

  enum class Key
  {
    AXIS_UP,
    AXIS_DOWN,
    AXIS_LEFT,
    AXIS_RIGHT,
    BUTTON_X,
    BUTTON_Y,
    BUTTON_A,
    BUTTON_B,
    BUTTON_L,
    BUTTON_R,
    BUTTON_SELECT,
    BUTTON_START,
    NONE
  };

  explicit GamepadInput() = default;
  ~GamepadInput();

  bool  Init(const std::string &device);
  void  Term();
  std::pair<Key, bool>  Read(int32_t delay_ms = 100);

private:

  int   js_fd_;
};

/// EOF ****************************************************************************************************************
