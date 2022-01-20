/// ********************************************************************************************************************
/// @author   imindude@gmail.com
/// ********************************************************************************************************************

#pragma once

/// ********************************************************************************************************************

#include <cmath>
#include <chrono>
#include <type_traits>

/// ********************************************************************************************************************

namespace std
{
  // under C++23
  template <typename E>
  constexpr auto to_underlying(E e) noexcept
  {
    return static_cast<std::underlying_type_t<E>>(e);
  }
}

class MiscTools
{
public:

  template<typename T>
  static T  ClockToDegree(int clock)
  {
    double  degree = std::fmod(static_cast<double>(clock) * 30.0, 360.0);

    if (degree == 360.0)
      degree = 0.0;

    return static_cast<T>(degree);
  }

  template<typename T>
  static int  DegreeToClock(T degree)
  {
    double  angle = static_cast<double>(degree);
    int     clock = std::lround(std::fmod(angle, 360.0) / 30.0);

    if (clock == 0)
      clock = 12;

    return clock;
  }

  template<typename T>
  static T  DegreeAdd(T deg1, T deg2)
  {
    double  angle1 = static_cast<double>(deg1);
    double  angle2 = static_cast<double>(deg2);
    double  degree = std::fmod(angle1 + angle2, 360.0);

    if (degree < 0.0)
      degree += 360.0;

    return static_cast<T>(degree);
  }

  template<typename T>
  static T  DegreeSub(T deg1, T deg2)
  {
    double  degree = static_cast<double>(deg2) - static_cast<double>(deg1);

    if (degree > 180.0)
      degree -= 360.0;
    else if (degree < -180.0)
      degree += 360.0;

    return static_cast<T>(degree);
  }

  template<typename T>
  static T  DegreeBetween(T deg1, T deg2)
  {
    double  change  = std::abs(static_cast<double>(deg2) - static_cast<double>(deg1));

    if (change > 180.0)
      change = 360.0 - change;

    return static_cast<T>(change);
  }

  template<typename T>
  static bool Tolerance(T in, T base, T tolerance)
  {
    T   min = base - tolerance;
    T   max = base + tolerance;

    return (min <= in) && (in <= max) ? true : false;
  }

  template<typename T>
  static bool ToleranceUnder(T in, T base, T tolerance)
  {
    T   min = base - tolerance;

    return min <= in ? true : false;
  }

  template<typename T>
  static bool ToleranceOver(T in, T base, T tolerance)
  {
    T   max = base + tolerance;

    return max >= in ? true : false;
  }

  template<typename T>
  static T  Constrain(T in, T min, T max)
  {
    return (in < min) ? min : (in > max) ? max : in;
  }

  template<typename T>
  static T  LimitMax(T in, T max)
  {
    return in > max ? max : in;
  }

  template<typename T>
  static T  DegreeToRadian(T deg)
  {
    // 1 deg = 0.01745329251994329576923690768489 rad
    // return static_cast<T>(static_cast<double>(deg) * 0.017453292519943295769)
    return static_cast<T>(static_cast<double>(deg) * M_PI / 180.0);
  }

  template<typename T>
  static T  RadianToDegree(T rad)
  {
    // 1 rad = 57.295779513082320876798154814105 deg
    // return static_cast<T>(static_cast<double>(rad) * 57.295779513082320876);
    return static_cast<T>(static_cast<double>(rad) * 180.0 / M_PI);
  }

  static int64_t  TimeSinceEpoch()
  {
    // return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  }
};

/// EOF ****************************************************************************************************************
