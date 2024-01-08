// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc2/command/RunCommand.h>

class Shooter : public frc2::SubsystemBase
{
public:
  Shooter();

  frc2::CommandPtr fender_shot();
  void set_angle(units::degree_t angle);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  ctre::phoenix6::hardware::CANcoder m_cancoder{CONSTANTS::SHOOTER::CANCODER_ID};
  ctre::phoenix6::hardware::TalonFX m_left_motor{CONSTANTS::SHOOTER::LEFT_ID};
  ctre::phoenix6::hardware::TalonFX m_right_motor{CONSTANTS::SHOOTER::RIGHT_ID};
  ctre::phoenix6::hardware::TalonFX m_angle_motor{CONSTANTS::SHOOTER::AZIMUTH_ID};
};
