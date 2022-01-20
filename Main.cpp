/// ********************************************************************************************************************
/// @author   wuyong.yi@asoa.co.kr
/// ********************************************************************************************************************

#include <iostream>
#include <csignal>
#include "Service/PiBalancer.h"

/// ********************************************************************************************************************

#define RESET   "\e[0m"
#define RED     "\e[0;31m"
#define GREEN   "\e[0;32m"
#define CYAN    "\e[0;36m"

/// ********************************************************************************************************************

static void signal_handler(int signum)
{
  std::cerr << "Interrupt: " << signum << std::endl;
  exit(signum);
}

int main(int argc, char* argv[])
{
  std::signal(SIGINT, signal_handler);
  std::signal(SIGSEGV, signal_handler);
  std::signal(SIGTERM, signal_handler);

  PiBalancer  balancer_;

  balancer_.Init();
  balancer_.Term();

  return 0;
}

/// EOF ****************************************************************************************************************
