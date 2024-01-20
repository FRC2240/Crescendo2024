// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <units/angle.h>
#include <units/time.h>
#include <units/angular_velocity.h>
#include <frc2/command/RunCommand.h>
#include <frc/Servo.h>

class BuddyClimber : public frc2::SubsystemBase
{
public:
  BuddyClimber();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  
  frc2::CommandPtr DeployCommand();
  frc2::CommandPtr StartRightCommand();
  frc2::CommandPtr StartLeftCommand();
  //frc2::CommandPtr ManualOverrideCommand();

private:
  ctre::phoenix6::hardware::TalonFX m_rightMotor{4};
  ctre::phoenix6::hardware::TalonFX m_leftMotor{5}; //change?
  frc::Servo m_deployServo{1}; //change
  const double DEPLOY_ANGLE = 1.0; //change
  const units::angular_velocity::turns_per_second_t ROTOR_SPEED{25}; //change
};
