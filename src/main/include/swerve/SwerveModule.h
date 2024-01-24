#pragma once

/// Comment below out if you want to drive
// #define CFG_NO_DRIVEBASE

#ifndef CFG_NO_DRIVEBASE
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/geometry/Translation2d.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include "Constants.h"
#include <frc/DriverStation.h>

class SwerveModule
{
public:
    /******************************************************************/
    /*                  Public Function Declarations                  */
    /******************************************************************/

    SwerveModule(int const &driver_adr, int const &turner_adr, int const &cancoder_adr, units::angle::turn_t offset);

    [[nodiscard]] frc::SwerveModuleState getState();

    [[nodiscard]] frc::SwerveModulePosition getPosition();

    [[nodiscard]] units::degree_t getAngle();

    [[nodiscard]] double getDriverTemp();

    [[nodiscard]] double getTurnerTemp();

    double get_current();

    void setDesiredState(const frc::SwerveModuleState &state);

    void percentOutputControl(double const &percent_output);

    void manualVelocityContol(double const &velocity_ticks_per_100ms);

    units::angle::turn_t getEncoder()
    {
        return driver.GetPosition().GetValue();
    }

    units::angle::turn_t get_angle_turns();

    units::angle::degree_t get_angle_degrees();

    // No copies/moves should be occuring (Talons don't support this)
    SwerveModule(SwerveModule const &) = delete;
    SwerveModule(SwerveModule &&) = delete;

private:
    inline units::meters_per_second_t wheel_speed_to_bot_speed(units::turns_per_second_t wheel_speed);
    inline units::turns_per_second_t bot_speed_to_wheel_speed(units::meters_per_second_t bot_speed);

    static constexpr units::length::meter_t WHEEL_RADIUS = 2.42_in; // measured
    static constexpr units::meter_t WHEEL_CIRCUMFERENCE = 1_in;     // Should be 12 but 2 is better.

    static constexpr auto DRIVER_GEAR_RATIO = 6.75;
    static constexpr auto TURNER_GEAR_RATIO = 150 / 7;

    static constexpr auto HUNDREDMILLISECONDS_TO_1SECOND = 10; // Ticks / 100 milliseconds * 10 = Ticks / 1 second
    static constexpr auto ONESECOND_TO_100MILLISECONDS = .1;   // Ticks / second * .1 = Ticks / 100 milliseconds

    /******************************************************************/
    /*                        Private Variables                       */
    /******************************************************************/

    ctre::phoenix6::hardware::TalonFX driver, turner;
    ctre::phoenix6::hardware::CANcoder cancoder;
    // double const magnet_offset;
    int turner_addr;
};
#endif
