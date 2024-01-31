#pragma once

#include <units/length.h>
#include <units/angle.h>
#include <frc/DriverStation.h>
#include <iostream>
#include <vector>
#include <units/time.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <numbers>

// #define BETABOT
//  When using the second robot, uncomment the above line

// #define MOD_AMP
// When amp module is installed, uncomment the above line

// #define MOD_TRAP
//  When trap module is installed, uncomment the above line

// #define MOD_BUDDY
//  When buddy climber is installed, uncomment the above line

namespace CONSTANTS
{

  // An additive threshold (+/- value) that checks if 2 values (target & source) are within a range
  // A template so it can be used with units. Call it by:
  // CONSTANTS::IN_THRESHOLD<type>(x,y,z)
  template <typename T>
  static bool IN_THRESHOLD(T source, T target, T range)
  {
    return (source >= target - range && source <= target + range);
  }

  struct PidCoeff
  {
    const double
        p = .0,   /// Proportional. Based off distance from setpoint, most important but high values cause oscilation.
        i = .0,   /// Integral. Based off the error over time, try to avoid if possible.
        d = .0,   /// Derivative. Based off velocity from setpoint, can be used to smooth out oscilation caused by high P values.
        ff = .0,  /// Feed Forward. Provides a constant boost to the output. Used to fight gravity or similar things.
        iz = .0,  /// I Zone. A deadband (distance from zero) for when I takes effect. Try to avoid if possible.
        min = .0, /// Minimum output for control loop.
        max = .0; /// Maximum output for control loop.
  };

  namespace INTAKE
  {
    constexpr int BELT_ID = 20;
    constexpr int ANGLE_ID = 10;
    constexpr units::degree_t AUTO_PICKUP_THRESHOLD = 15_deg;
  } // namespace INTAKE
  namespace VISION
  {
    static const auto LEFT_CAMERA_A_TF = frc::Transform3d{0_m, 0_m, 0_m, frc::Rotation3d(0_rad, 0_rad, 0_rad)};
    static const auto LEFT_CAMERA_B_TF = frc::Transform3d{0_m, 0_m, 0_m, frc::Rotation3d(0_rad, 0_rad, 0_rad)};
    static const auto RIGHT_CAMERA_A_TF = frc::Transform3d{0_m, 0_m, 0_m, frc::Rotation3d(0_rad, 0_rad, 0_rad)};
    static const auto RIGHT_CAMERA_B_TF = frc::Transform3d{0_m, 0_m, 0_m, frc::Rotation3d(0_rad, 0_rad, 0_rad)};

  } // namespace VISION

  namespace CLIMBER
  {
    constexpr int left_climber = 4;  // CHANGEME
    constexpr int right_climber = 5; // CHANGEME
  }                                  // namespace CLIMBER

  namespace CANDLE
  {
    constexpr int CANDLE_ID = 5; // CHANGEME
  }

  namespace SHOOTER
  {
    constexpr int LEFT_ID = 1;      // CHANGEME
    constexpr int RIGHT_ID = 11;    // CHANGEME
    constexpr int ANGLE_ID = 2;     // CHANGME
    constexpr int CANCODER_ID = 13; // CHANGEME
    constexpr std::pair<units::turn_t, units::turn_t> FENDER_RANGE = {0_tr, 1_tr};
    constexpr double ANGLE_RATIO = 1;                    // CHANGEME
    constexpr units::degree_t FENDER_ANGLE = 15_deg;     // CHANGEME
    constexpr units::degree_t FENDER_TOLERANCE = 15_deg; // CHANGEME
    constexpr units::degree_t AMP_ANGLE = 30_deg;
    constexpr int BELT_ID = 4;
    constexpr units::turns_per_second_t LEFT_VELOCITY{10};  // CHANGEME;
    constexpr units::turns_per_second_t RIGHT_VELOCITY{10}; // CHANGEME;

  } // namespace SHOOTER

  namespace DRIVE
  {
    constexpr units::meters_per_second_t ROBOT_MAX_SPEED = 18.533_fps;
    constexpr units::radians_per_second_t ROBOT_MAX_ANGULAR_SPEED{std::numbers::pi * 1.25};
    constexpr units::meters_per_second_t TELEOP_MAX_SPEED = ROBOT_MAX_SPEED;
    constexpr units::radians_per_second_t TELEOP_MAX_ANGULAR_SPEED{std::numbers::pi * 2.25};
    constexpr units::meters_per_second_t TRAJ_MAX_SPEED = ROBOT_MAX_SPEED;
    constexpr units::acceleration::meters_per_second_squared_t TRAJ_MAX_ACCELERATION = TRAJ_MAX_SPEED / 0.5_s;
    constexpr units::radians_per_second_t TRAJ_MAX_ANGULAR_SPEED = CONSTANTS::DRIVE::ROBOT_MAX_ANGULAR_SPEED;
    constexpr units::radians_per_second_squared_t TRAJ_MAX_ANGULAR_ACCELERATION{std::numbers::pi};

    namespace CONFIG
    {
      struct ModuleConfig
      {
        int driver;
        int azimuth;
        int cancoder;
        units::turn_t offset;
      };
#ifndef BETABOT
#pragma message("First Robot Config active")
      /* -------------------------------------------------------------------------- */
      /*                     BEGIN FIRST ROBOT CONFIGURATION                        */
      /* -------------------------------------------------------------------------- */

      constexpr ModuleConfig FL{60, 61, 14, 0.31_tr};
      constexpr ModuleConfig FR{50, 51, 13, -0.182_tr};
      constexpr ModuleConfig BL{30, 31, 11, -0.286_tr};
      constexpr ModuleConfig BR{40, 41, 12, 0.03_tr};

      /* -------------------------------------------------------------------------- */
      /*                        END FIRST ROBOT CONFIGURATION                       */
      /* -------------------------------------------------------------------------- */
#endif // BETABOT

#ifdef BETABOT
#pragma message("Second Robot Config active")
/* -------------------------------------------------------------------------- */
/*                       BEGIN SECOND ROBOT CONFIGUATION                      */
/* -------------------------------------------------------------------------- */
#pragma warning("Values extraordinary wrong")
      constxpr ModuleConfig FL{60, 61, 14, 0.22_tr};
      constexpr ModuleConfig FR{50, 51, 13, -0.304_tr};
      constexpr ModuleConfig BL{30, 31, 11, -0.335_tr};
      constexpr ModuleConfig BR{40, 41, 12, 0.052_tr};

/* -------------------------------------------------------------------------- */
/*                        END SECOND ROBOT CONFIGUATION                       */
/* -------------------------------------------------------------------------- */
#endif // BETABOT
    }
  }
}
