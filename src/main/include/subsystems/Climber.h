// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
//#include <ctre/phoenix6/controls/Follower.hpp>
#include <units/angle.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
class Climber : public frc2::SubsystemBase
{
public:
  Climber(
      frc::XboxController *stick);
  void climb(double pos);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc2::CommandPtr BrakeCommand();
  frc2::CommandPtr CoastCommand();

private:

  frc::XboxController* m_stick;

  ctre::phoenix6::hardware::TalonFX left_climber{5}; //CHANGEME (Make sure you change constants as well)
  ctre::phoenix6::hardware::TalonFX right_climber{4}; //CHANGEME (Make sure you change constants as well)
};
