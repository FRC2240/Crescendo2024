// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <units/angle.h>
#include <units/time.h>
#include <units/angular_velocity.h>

class Intake : public frc2::SubsystemBase
{
public:
  Intake();

  // TODO: FIX
  bool is_loaded();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc2::CommandPtr ExtendCommand();
  frc2::CommandPtr RetractCommand();
  frc2::CommandPtr EnableCommand();
  frc2::CommandPtr DisableCommand();

 private:
  ctre::phoenix6::hardware::TalonFX m_angleMotor{};
  ctre::phoenix6::hardware::TalonFX m_flywheelMotor{};
  ctre::phoenix6::hardware::TalonFX m_beltMotor{};

  units::turn_t kStartRotations = 0; //change
  units::turn_t kEndRotations = 100; //change
  //turn per second kFlywheelSpeed = 50
  //turn per second kBeltSpeed = 50

};
