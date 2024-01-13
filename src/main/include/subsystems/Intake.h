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
  frc2::CommandPtr StartSpinCommand();
  frc2::CommandPtr StopSpinCommand();
  frc2::CommandPtr StartCommand();
  frc2::CommandPtr StopCommand();

 private:
  ctre::phoenix6::hardware::TalonFX m_angleMotor{0};
  ctre::phoenix6::hardware::TalonFX m_beltMotor{2};

  units::angle::turn_t START_ROTATIONS{0}; //change
  units::angle::turn_t END_ROTATIONS{100}; //change
  units::angular_velocity::turns_per_second_t BELT_SPEED{50}; //change

};
