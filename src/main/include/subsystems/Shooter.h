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
#include "subsystems/Intake.h"

class Shooter : public frc2::SubsystemBase
{
public:
  Shooter(Intake *intake);

  frc2::CommandPtr fender_shot();

  // This is a wrapper for set angle that is a cmdptr so it can be used in the auto shot compisiton
  // Overloaded so it can be used with vision
  frc2::CommandPtr set_angle_cmd(units::degree_t angle);
  frc2::CommandPtr set_angle_cmd(std::optional<units::degree_t> angle);

  units::degree_t get_angle();

  void set_angle(units::degree_t angle);

  frc2::CommandPtr execute_auto_shot();

  frc2::CommandPtr amp_shot();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  ctre::phoenix6::hardware::CANcoder m_cancoder{CONSTANTS::CAN_IDS::SHOOTER_CANCODER_ID};
  ctre::phoenix6::hardware::TalonFX m_left_motor{CONSTANTS::CAN_IDS::SHOOTER_LEFT_ID};
  ctre::phoenix6::hardware::TalonFX m_right_motor{CONSTANTS::CAN_IDS::SHOOTER_RIGHT_ID};
  ctre::phoenix6::hardware::TalonFX m_angle_motor{CONSTANTS::CAN_IDS::SHOOTER_ANGLE_ID};
  Intake *m_intake;
};
