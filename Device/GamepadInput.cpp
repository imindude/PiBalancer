/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include "GamepadInput.h"

/// ********************************************************************************************************************

GamepadInput::~GamepadInput()
{
  Term();
}

bool GamepadInput::Init(const std::string &device)
{
  do
  {
    if ((js_fd_ = open(device.c_str(), O_RDONLY)) < 0)
      break;

    return true;
  }
  while (false);

  js_fd_ = 0;

  return false;
}

void GamepadInput::Term()
{
  if (js_fd_ > 0)
    close(js_fd_);
  js_fd_ = 0;
}

std::pair<GamepadInput::Key, bool> GamepadInput::Read(int32_t delay_ms)
{
  do
  {
    struct timeval  tv;
    fd_set  fds;

    tv.tv_sec   = delay_ms / 1000;
    tv.tv_usec  = (delay_ms % 1000) * 1000;

    FD_ZERO(&fds);
    FD_SET(js_fd_, &fds);

    if (select(js_fd_ + 1, &fds, NULL, NULL, &tv) <= 0)
      break;

    struct js_event e;

    if (read(js_fd_, &e, sizeof(e)) != sizeof(e))
      break;

    GamepadInput::Key key = Key::NONE;

    if (e.type == JS_EVENT_BUTTON)
    {
      switch (e.number)
      {
      case 0:   key = Key::BUTTON_X;        break;
      case 1:   key = Key::BUTTON_A;        break;
      case 2:   key = Key::BUTTON_B;        break;
      case 3:   key = Key::BUTTON_Y;        break;
      case 4:   key = Key::BUTTON_L;        break;
      case 5:   key = Key::BUTTON_R;        break;
      case 8:   key = Key::BUTTON_SELECT;   break;
      case 9:   key = Key::BUTTON_START;    break;
      }
    }
    else if (e.type == JS_EVENT_AXIS)
    {
      if (e.number == 1)
      {
        if (e.value < 0)
          key = Key::AXIS_UP;
        else
          key = Key::AXIS_DOWN;
      }
      else if (e.number == 0)
      {
        if (e.value < 0)
          key = Key::AXIS_LEFT;
        else
          key = Key::AXIS_RIGHT;
      }
    }
    else
    {
      break;
    }

    return std::pair<Key, bool> { key, e.value };
  }
  while (false);

  return std::pair<Key, bool> { Key::NONE, false };
}

/// EOF ****************************************************************************************************************
