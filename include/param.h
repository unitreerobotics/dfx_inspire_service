#ifndef PARAM_H
#define PARAM_H

#include <stdint.h>
#include <iostream>
#include <chrono>
#include <spdlog/spdlog.h>
#include <boost/program_options.hpp>

#include "g1/Configuration.h"

namespace param
{

namespace po = boost::program_options;

inline std::string serial_port;
inline std::string network; 
inline std::string ns; 
inline float threhold;

po::variables_map helper(int argc, char** argv)
{
#ifndef NDEBUG
  spdlog::set_level(spdlog::level::debug);
#else
  spdlog::set_level(spdlog::level::info);
#endif


  po::options_description desc("Unitree H1 Inspire Hand Serial to DDS");
  desc.add_options()
    ("help,h", "produce help message")
    ("serial,s", po::value<std::string>(&serial_port)->default_value("/dev/ttyUSB0"), "serial port")
    ("network", po::value<std::string>(&network)->default_value(""), "DDS network interface")
    ("namespace", po::value<std::string>(&ns)->default_value("inspire"), "DDS topic namespace")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    exit(0);
  }

  if(ns.empty())
  {
    spdlog::error("Namespace cannot be empty");
    exit(1);
  }

  return vm;
}

// G1 has two independent serial buses. Keep its options separate so H1
// continues to use the original --serial interface and defaults unchanged.
po::variables_map g1_helper(int argc, char** argv, g1::Configuration& config)
{
#ifndef NDEBUG
  spdlog::set_level(spdlog::level::debug);
#else
  spdlog::set_level(spdlog::level::info);
#endif

  po::options_description desc("Unitree G1 Inspire Hand Serial to DDS");
  desc.add_options()
    ("help,h", "produce help message")
    ("right-port", po::value<std::string>(&config.right_port)->default_value("/dev/ttyUSB1"), "right hand serial port")
    ("left-port", po::value<std::string>(&config.left_port)->default_value("/dev/ttyUSB2"), "left hand serial port")
    ("left-only", po::bool_switch(&config.left_only)->default_value(false), "do not create, poll, or control a right hand")
    ("dry-run", po::bool_switch(&config.dry_run)->default_value(false), "use in-memory mock hands and never construct SerialPort")
    ("network", po::value<std::string>(&network)->default_value(""), "DDS network interface")
    ("namespace", po::value<std::string>(&ns)->default_value("inspire"), "DDS topic namespace")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    exit(0);
  }

  if(ns.empty())
  {
    spdlog::error("Namespace cannot be empty");
    exit(1);
  }

  return vm;
}

}

#endif // PARAM_H
