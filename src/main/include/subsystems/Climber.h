// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
// #include <ctre/phoenix6/controls/Follower.hpp>
#include <units/angle.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/RunCommand.h>
#include "frc/DigitalInput.h"
#include "Constants.h"
class Climber : public frc2::SubsystemBase
{
public:
  Climber();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  frc2::CommandPtr UpCommand();
  frc2::CommandPtr DownCommand();
  frc2::CommandPtr StopCommand();

private:

  frc::DigitalInput left_limit_switch{0};
  frc::DigitalInput right_limit_switch{1};

  ctre::phoenix6::hardware::TalonFX left_climber{CONSTANTS::CLIMBER::LEFT_ID};   // CHANGEME (Make sure you change constants as well)
  ctre::phoenix6::hardware::TalonFX right_climber{CONSTANTS::CLIMBER::RIGHT_ID}; // CHANGEME (Make sure you change constants as well)
};
