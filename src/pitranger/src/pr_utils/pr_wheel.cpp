#include "pr_utils/pr_wheel.h"
#include "pr_utils/pr_log.h"
#include "pr_utils/pr_time.h"
#include "pr_utils/pr_math.h"
#include <Roboteq.h>
#include <experimental/filesystem>
#include <fmt/format.h>
#include <unordered_map>
#include <unistd.h> 

// using namespace roboteq;

namespace pr {

namespace {

int check_port_for_roboteq(const std::string &port_name) {
  RoboteqDevice dev;
  int status = dev.Connect(port_name);
  if (status != RQ_SUCCESS) {
    return -1;
  }

  int cnod = 0;
  if (dev.GetConfig(_CNOD, cnod) != RQ_SUCCESS) {
    return -1;
  }
  dev.Disconnect();
  return cnod;
}

// Search /dev/tty* for roboteq devices.
// Return: A hashmap mapping Roboteq CAN IDS to port names.
std::unordered_map<int, std::string> find_roboteq_ports() {
  namespace fs = std::experimental::filesystem;

  std::unordered_map<int, std::string> roboteq_ports;

  // Iterate over all files in /dev/
  for (const auto &p : fs::directory_iterator("/dev/")) {
    const std::string path = p.path().string();

    // Search for paths matching /dev/ttyACM*.
    if (path.find("ttyACM") != std::string::npos) {

      // Check candidate paths for roboteq devices.
      int can_id = check_port_for_roboteq(path);

      // If a roboteq is found, add it to the hashmap.
      if (can_id >= 0) {
        // If this roboteq CAN ID is not unique, raise an exception.
        if (roboteq_ports.find(can_id) != roboteq_ports.end()) {
          throw std::runtime_error("ROBOTEQ Error: Your roboteq devices have not been "
                                   "assigned unique CAN IDs.");
        }
        roboteq_ports[can_id] = path;
      }
    }
  }
  return roboteq_ports;
}

} // anonymous namespace

WheelController::WheelController() {

  // Scan for Roboteqs and connect to them. Retry 10 times at 1 second intervals.
  {
    int num_retries = 10;
    bool found_left = false;
    bool found_right = false;
    std::unordered_map<int, std::string> port_map;

    for (int retry = 0; retry < num_retries; ++retry) {
      // Scan for roboteqs.
      port_map = find_roboteq_ports();

      found_left = port_map.find(WHEELS_LEFT_ROBOTEQ_CAN_ID) != port_map.end();
      found_right = port_map.find(WHEELS_RIGHT_ROBOTEQ_CAN_ID) != port_map.end();

      if (found_left && found_right) {
        break;
      }

      if (found_left && !found_right) {
        auto msg = fmt::format("Failed to find right Roboteq. Retrying... [{}/{}]\n", retry + 1,
                               num_retries);
        pr::log_warn(msg);
      } else if (!found_right && found_right) {
        auto msg = fmt::format("Failed to find left Roboteq. Retrying... [{}/{}]\n", retry + 1, num_retries);
        pr::log_warn(msg);
      } else if (!found_right && !found_right) {
        auto msg = fmt::format("Failed to find left and right Roboteqs. Retrying... [{}/{}]\n",
                               retry + 1, num_retries);
        pr::log_warn(msg);
      }
      pr::time::sleep(1);
    }

    if (!found_left) {
      auto msg =
          fmt::format("Failed to find left roboteq with CAN ID {}\n", WHEELS_LEFT_ROBOTEQ_CAN_ID);
      throw std::runtime_error(msg);
    }

    if (!found_right) {
      auto msg =
          fmt::format("Failed to find right roboteq with CAN ID {}\n", WHEELS_RIGHT_ROBOTEQ_CAN_ID);
      throw std::runtime_error(msg);
    }

    int status = left.Connect(port_map.at(WHEELS_LEFT_ROBOTEQ_CAN_ID));
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to connect to left Roboteq on port {}\n",
                             port_map.at(WHEELS_LEFT_ROBOTEQ_CAN_ID));
      throw std::runtime_error(msg);
    }

    status = right.Connect(port_map.at(WHEELS_RIGHT_ROBOTEQ_CAN_ID));
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to connect to left Roboteq on port {}\n",
                             port_map.at(WHEELS_RIGHT_ROBOTEQ_CAN_ID));
      throw std::runtime_error(msg);
    }
  }

  // Configure the Left Roboteq.
  {
    // Configure Roboteq
    disable_watchdog(left);
    use_velocity_mode(left);
    use_quadrature_encoder(left, WHEELS_ENCODER_PPR);
    set_max_vel(left, WHEELS_MAX_MOTOR_RPM);
    set_kp(left, 0.4);
    set_ki(left, 0.1);
    set_kd(left, 0.0);
    std::cout << "Set Left Motor Params" << std::endl;
  }

  // Configure the Right Roboteq.
  {
    // Configure Roboteq
    disable_watchdog(right);
    use_velocity_mode(right);
    use_quadrature_encoder(right, WHEELS_ENCODER_PPR);
    set_max_vel(right, WHEELS_MAX_MOTOR_RPM);
    set_kp(right, 0.4);
    set_ki(right, 0.1);
    set_kd(right, 0.0);
    std::cout << "Set Right Motor Params" << std::endl;
  }
}

WheelController::~WheelController() {
    set_left_rpm(0);
    set_right_rpm(0);
}

// Sends +ve value for forward driving
double WheelController::set_left_rpm(double rpm) {
    // Left wheels consistently runs slowly than right wheels ~ 1.00 (L) : 1.01 (R) @ 0.07
    int fl_cmd = pr::clamp<int>(rpm, -2000, 2000);
    int rl_cmd = pr::clamp<int>(rpm, -2000, 2000);

    int status;
    status = left.SetCommand(_MOTVEL, WHEELS_LEFT_FRONT_CHANNEL, fl_cmd);
    sleepms(20);
    status = left.SetCommand(_MOTVEL, WHEELS_LEFT_REAR_CHANNEL, rl_cmd);
    sleepms(20);
    std::cout << "SET LEFT: " << status << "," << fl_cmd << "," << rl_cmd << "\n";
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set left wheel RPM to {}.\n", rpm);
      std::cout << "Status: " << status << ", " << msg << std::endl;
      throw std::runtime_error(msg);
    }
}

// Sends -ve value for forward driving
// 1000 is about 5cm/s - 11.5s
double WheelController::set_right_rpm(double rpm) {
    int fr_cmd = pr::clamp<int>(rpm, -2000, 2000);
    int rr_cmd = pr::clamp<int>(rpm, -2000, 2000);

    int status;
    status =  right.SetCommand(_MOTVEL, WHEELS_RIGHT_FRONT_CHANNEL, fr_cmd);
    sleepms(20);
    status =  right.SetCommand(_MOTVEL, WHEELS_RIGHT_REAR_CHANNEL, rr_cmd);
    sleepms(20);   
    std::cout << "SET RIGHT: " << status << "," << fr_cmd << "," << rr_cmd << "\n";
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set right wheel RPM to {}.\n", rpm);
      std::cout << "Status: " << status << ", " << msg << std::endl;
      throw std::runtime_error(msg);
    }
}


std::vector<int> WheelController::get_rpm() {
  int status;

  int enc_fr = 0;
  status = right.GetValue(_ABSPEED, WHEELS_RIGHT_FRONT_CHANNEL, enc_fr);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get front right wheel encoder value.\n");
    std::cout << "Status: " << status << ", " << msg << std::endl;
    throw std::runtime_error(msg);
  }
  sleepms(10);

  int enc_rr;
  status = right.GetValue(_ABSPEED, WHEELS_RIGHT_REAR_CHANNEL, enc_rr);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get rear right wheel encoder value.\n");
    std::cout << "Status: " << status << ", " << msg << std::endl;
    throw std::runtime_error(msg);
  }
  sleepms(10);

  int enc_fl;
  status = left.GetValue(_ABSPEED, WHEELS_LEFT_FRONT_CHANNEL, enc_fl);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get front left wheel encoder value.\n");
    std::cout << "Status: " << status << ", " << msg << std::endl;
    throw std::runtime_error(msg);
  }
  sleepms(10);

  int enc_rl;
  status = left.GetValue(_ABSPEED, WHEELS_LEFT_REAR_CHANNEL, enc_rl);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get rear left wheel encoder value.\n");
    std::cout << "Status: " << status << ", " << msg << std::endl;
    throw std::runtime_error(msg);
  }
  sleepms(10);

  std::vector<int> encs;
  encs.push_back(enc_fl);
  encs.push_back(enc_fr);
  encs.push_back(enc_rl);
  encs.push_back(enc_rr);

  return encs;
}


std::vector<int> WheelController::get_abs_encs() {
  int status;

  int abs_enc_fr = 0;
  status = right.GetValue(_ABCNTR, WHEELS_RIGHT_FRONT_CHANNEL, abs_enc_fr);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get front right wheel encoder value.\n");
    std::cout << "Status: " << status << ", " << msg << std::endl;
    // throw std::runtime_error(msg);
    abs_enc_fr = _abs_enc_fr; // Take prev enc value
  }
  sleepms(10);
  _abs_enc_fr = abs_enc_fr;

  int abs_enc_rr;
  status = right.GetValue(_ABCNTR, WHEELS_RIGHT_REAR_CHANNEL, abs_enc_rr);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get rear right wheel encoder value.\n");
    std::cout << "Status: " << status << ", " << msg << std::endl;
    // throw std::runtime_error(msg);
    abs_enc_fr = _abs_enc_fr; // Take prev enc value
  }
  sleepms(10);
  _abs_enc_rr = abs_enc_rr;

  int abs_enc_fl;
  status = left.GetValue(_ABCNTR, WHEELS_LEFT_FRONT_CHANNEL, abs_enc_fl);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get front left wheel encoder value.\n");
    std::cout << "Status: " << status << ", " << msg << std::endl;
    // throw std::runtime_error(msg);
    abs_enc_fr = _abs_enc_fr; // Take prev enc value
  }
  sleepms(10);
  _abs_enc_fl = abs_enc_fl;

  int abs_enc_rl;
  status = left.GetValue(_ABCNTR, WHEELS_LEFT_REAR_CHANNEL, abs_enc_rl);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get rear left wheel encoder value.\n");
    std::cout << "Status: " << status << ", " << msg << std::endl;
    // throw std::runtime_error(msg);
    abs_enc_fr = _abs_enc_fr; // Take prev enc value
  }
  sleepms(10);
  _abs_enc_rl = abs_enc_rl;

  std::vector<int> encs;
  encs.push_back(abs_enc_fl);
  encs.push_back(abs_enc_fr);
  encs.push_back(abs_enc_rl);
  encs.push_back(abs_enc_rr);

  return encs;
}

std::vector<int> WheelController::get_rel_encs() {
  int status;

  int rel_enc_fr = 0;
  status = right.GetValue(_RELCNTR, WHEELS_RIGHT_FRONT_CHANNEL, rel_enc_fr);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get front right wheel encoder value.\n");
    std::cout << "Status: " << status << ", " << msg << std::endl;
    // throw std::runtime_error(msg);
    rel_enc_fr = _rel_enc_fr; // use prev value
  }
  sleepms(10);
  _rel_enc_fr = rel_enc_fr;

  int rel_enc_rr;
  status = right.GetValue(_RELCNTR, WHEELS_RIGHT_REAR_CHANNEL, rel_enc_rr);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get rear right wheel encoder value.\n");
    std::cout << "Status: " << status << ", " << msg << std::endl;
    // throw std::runtime_error(msg);
    rel_enc_rr = _rel_enc_rr; // use prev value
  }
  sleepms(10);
  _rel_enc_rr = rel_enc_rr;

  int rel_enc_fl;
  status = left.GetValue(_RELCNTR, WHEELS_LEFT_FRONT_CHANNEL, rel_enc_fl);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get front left wheel encoder value.\n");
    std::cout << "Status: " << status << ", " << msg << std::endl;
    // throw std::runtime_error(msg);
    rel_enc_fl = _rel_enc_fl; // use prev value
  }
  sleepms(10);
  _rel_enc_fl = rel_enc_fl;

  int rel_enc_rl;
  status = left.GetValue(_RELCNTR, WHEELS_LEFT_REAR_CHANNEL, rel_enc_rl);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to get rear left wheel encoder value.\n");
    std::cout << "Status: " << status << ", " << msg << std::endl;
    // throw std::runtime_error(msg);
    rel_enc_rl = _rel_enc_rl; // use prev value
  }
  sleepms(10);
  _rel_enc_rl = rel_enc_rl;

  std::vector<int> encs;
  encs.push_back(rel_enc_fl);
  encs.push_back(rel_enc_fr);
  encs.push_back(rel_enc_rl);
  encs.push_back(rel_enc_rr);

  return encs;
}

// double WheelController::get_front_right_rpm() {
//   int enc = get_front_right_encoder();
//   return -1 * enc / (double)WHEELS_MOTOR_GEAR_RATIO;
// }
// double WheelController::get_front_left_rpm() {
//   int enc = get_front_left_encoder();
//   return enc / (double)WHEELS_MOTOR_GEAR_RATIO;
// }
// double WheelController::get_rear_right_rpm() {
//   int enc = get_rear_right_encoder();
//   return -1 * enc / (double)WHEELS_MOTOR_GEAR_RATIO;
// }
// double WheelController::get_rear_left_rpm() {
//   int enc = get_rear_left_encoder();
//   return enc / (double)WHEELS_MOTOR_GEAR_RATIO;
// }

// int WheelController::get_front_right_encoder() {
//   int pos = 0;
//   int status = right.GetValue(_F, WHEELS_RIGHT_FRONT_CHANNEL, pos);
//   sleepms(20);
//   if (status != RQ_SUCCESS) {
//     auto msg = fmt::format("Failed to get front right wheel encoder value.\n");
//     std::cout << "Status: " << status << ", " << msg << std::endl;
//     throw std::runtime_error(msg);
//   }
//   return pos;
// }
// int WheelController::get_front_left_encoder() {
//   int pos;
//   int status = left.GetValue(_F, WHEELS_LEFT_FRONT_CHANNEL, pos);
//   sleepms(20);
//   if (status != RQ_SUCCESS) {
//     auto msg = fmt::format("Failed to get front left wheel encoder value.\n");
//     std::cout << "Status: " << status << ", " << msg << std::endl;
//     throw std::runtime_error(msg);
//   }
//   return pos;
// }
// int WheelController::get_rear_right_encoder() {
//   int pos;
//   int status = right.GetValue(_F, WHEELS_RIGHT_REAR_CHANNEL, pos);
//   sleepms(20);
//   if (status != RQ_SUCCESS) {
//     auto msg = fmt::format("Failed to get rear right wheel encoder value.\n");
//     std::cout << "Status: " << status << ", " << msg << std::endl;
//     throw std::runtime_error(msg);
//   }
//   return pos;
// }
// int WheelController::get_rear_left_encoder() {
//   int pos;
//   int status = left.GetValue(_F, WHEELS_LEFT_REAR_CHANNEL, pos);
//   sleepms(20);
//   if (status != RQ_SUCCESS) {
//     auto msg = fmt::format("Failed to get rear left wheel encoder value.\n");
//     std::cout << "Status: " << status << ", " << msg << std::endl;
//     throw std::runtime_error(msg);
//   }
//   return pos;
// }

// double WheelController::get_front_right_amps() {
//   int tenth_amps;
//   int status = right.GetValue(_MOTAMPS, WHEELS_RIGHT_FRONT_CHANNEL, tenth_amps);
//   if (status != RQ_SUCCESS) {
//     auto msg = fmt::format("Failed to get front right current draw.\n");
//     throw std::runtime_error(msg);
//   }
//   return tenth_amps/10.0;
// }

// double WheelController::get_front_left_amps() {
//   int tenth_amps;
//   int status = left.GetValue(_MOTAMPS, WHEELS_LEFT_FRONT_CHANNEL, tenth_amps);
//   if (status != RQ_SUCCESS) {
//     auto msg = fmt::format("Failed to get front left current draw.\n");
//     throw std::runtime_error(msg);
//   }
//   return tenth_amps/10.0;
// }

// double WheelController::get_rear_right_amps() {
//   int tenth_amps;
//   int status = right.GetValue(_MOTAMPS, WHEELS_RIGHT_REAR_CHANNEL, tenth_amps);
//   if (status != RQ_SUCCESS) {
//     auto msg = fmt::format("Failed to get rear right current draw.\n");
//     throw std::runtime_error(msg);
//   }
//   return tenth_amps/10.0;
// }

// double WheelController::get_rear_left_amps() {
//   int tenth_amps;
//   int status = left.GetValue(_MOTAMPS, WHEELS_LEFT_REAR_CHANNEL, tenth_amps);
//   if (status != RQ_SUCCESS) {
//     auto msg = fmt::format("Failed to get rear left current draw.\n");
//     throw std::runtime_error(msg);
//   }
//   return tenth_amps/10.0;
// }

void WheelController::disable_watchdog(RoboteqDevice &dev) {
  int status = dev.SetConfig(_RWD, 0);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to disable roboteq watchdog.\n");
    throw std::runtime_error(msg);
  }
}

void WheelController::disable_loop_error_detection(RoboteqDevice &dev) {
  int status = dev.SetConfig(_CLERD, 1, 0);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to disable loop error detection.\n");
    throw std::runtime_error(msg);
  }
  status = dev.SetConfig(_CLERD, 2, 0);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to disable loop error detection.\n");
    throw std::runtime_error(msg);
  }
}

void WheelController::use_velocity_mode(RoboteqDevice &dev) {
  int status = dev.SetConfig(_MMOD, 1, 1);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set operating mode to closed-loop speed.\n");
    throw std::runtime_error(msg);
  }
  status = dev.SetConfig(_MMOD, 2, 1);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set operating mode to closed-loop speed.\n");
    throw std::runtime_error(msg);
  }
}
void WheelController::use_quadrature_encoder(RoboteqDevice &dev, int encoder_ppr) {
  // Set Encoder Mode to Feedback
  {
    int status = dev.SetConfig(_EMOD, 1, 1 * 16 + 2);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set encoder mode to 'Feedback'.\n");
      throw std::runtime_error(msg);
    }
    status = dev.SetConfig(_EMOD, 2, 2 * 16 + 2);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set encoder mode to 'Feedback'.\n");
      throw std::runtime_error(msg);
    }
  }

  // Set Encoder Pulses Per Revolution
  {
    int status = dev.SetConfig(_EPPR, 1, encoder_ppr);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set encoder pulse per revolution.\n");
      throw std::runtime_error(msg);
    }
    status = dev.SetConfig(_EPPR, 2, encoder_ppr);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set encoder pulse per revolution.\n");
      throw std::runtime_error(msg);
    }
  }
}
void WheelController::set_max_vel(RoboteqDevice &dev, int motor_rpm) {
  // Set max motor RPM.
  {
    int status = dev.SetConfig(_MXRPM, 1, motor_rpm);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set max motor velocity.\n");
      throw std::runtime_error(msg);
    }
    status = dev.SetConfig(_MXRPM, 2, motor_rpm);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set max motor velocity.\n");
      throw std::runtime_error(msg);
    }
  }

  // Set max motor acceleration.
  {
    int status = dev.SetConfig(_MAC, 1, 3*motor_rpm * 10);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set max motor acceleration.\n");
      throw std::runtime_error(msg);
    }
    status = dev.SetConfig(_MAC, 2, 3*motor_rpm * 10);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set max motor acceleration.\n");
      throw std::runtime_error(msg);
    }
  }

  // Set max motor deceleration.
  {
    int status = dev.SetConfig(_MDEC, 1, 3*motor_rpm * 10);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set max motor deceleration.\n");
      throw std::runtime_error(msg);
    }
    status = dev.SetConfig(_MDEC, 2, 3*motor_rpm * 10);
    if (status != RQ_SUCCESS) {
      auto msg = fmt::format("Failed to set max motor deceleration.\n");
      throw std::runtime_error(msg);
    }
  }
}
void WheelController::set_kp(RoboteqDevice &dev, double kp) {
  int status = dev.SetConfig(_KP, 1, kp * 10);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set motor Kp.\n");
    throw std::runtime_error(msg);
  }

  status = dev.SetConfig(_KP, 2, kp * 10);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set motor Kp.\n");
    throw std::runtime_error(msg);
  }
}
void WheelController::set_ki(RoboteqDevice &dev, double ki) {
  int status = dev.SetConfig(_KI, 1, ki * 10);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set motor Ki.\n");
    throw std::runtime_error(msg);
  }

  status = dev.SetConfig(_KI, 2, ki * 10);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set motor Ki.\n");
    throw std::runtime_error(msg);
  }
}
void WheelController::set_kd(RoboteqDevice &dev, double kd) {
  int status = dev.SetConfig(_KD, 1, kd * 10);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set motor Kd.\n");
    throw std::runtime_error(msg);
  }

  status = dev.SetConfig(_KD, 2, kd * 10);
  if (status != RQ_SUCCESS) {
    auto msg = fmt::format("Failed to set motor Kd.\n");
    throw std::runtime_error(msg);
  }
}

} // namespace pr
