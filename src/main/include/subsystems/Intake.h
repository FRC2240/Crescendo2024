// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"
#include "swerve/Odometry.h"
#include "units/current.h"
#include "units/voltage.h"
#include <TimeOfFlight.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DataLogManager.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <iostream>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>

class Intake : public frc2::SubsystemBase {
public:
  Intake();

  // TODO: FIX
  bool is_loaded();
  bool is_lower_tof_loaded();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc2::CommandPtr ExtendCommand();
  frc2::CommandPtr RetractCommand();
  frc2::CommandPtr BraceCommand();
  frc2::CommandPtr StartSpinCommand();
  frc2::CommandPtr StopSpinCommand();
  frc2::CommandPtr StartCommand(); // extends + starts spinning
  frc2::CommandPtr StopCommand();  // retracts + stops spinning
  frc2::CommandPtr ManualFeedCommand(bool back);
  frc2::CommandPtr zero();
  frc2::CommandPtr Wes();

  enum IntakeState {
    INTAKING,
    DEFAULT,
    SLOWFEED,
  };

  IntakeState intake_state = DEFAULT;

  bool is_intaking = false;

  // Move all CAN ids to constants as well as all constants in this file.
  ctre::phoenix6::hardware::TalonFX m_beltMotor{CONSTANTS::INTAKE::BELT_ID};

private:
  units::volt_t m_belt_velocity = 0_V;
  frc::Timer m_timer;
  frc::TimeOfFlight m_tof{CONSTANTS::INTAKE::TOF_ID};
  frc::TimeOfFlight m_lower_tof{CONSTANTS::INTAKE::LOWER_TOF_ID};
  ctre::phoenix6::hardware::TalonFX m_angleMotor{CONSTANTS::INTAKE::ANGLE_ID};

  const units::angle::turn_t START_ROTATIONS{
      0}; // intake retracted position         (CHANGEME)
  const units::angle::turn_t END_ROTATIONS{
      100}; // intake extended position          (CHANGEME)
  const units::voltage::volt_t BELT_SPEED{
      12}; // volts to drive belt motor at      (CHANGEME)
  const units::angle::turn_t BRACE_ROTATIONS{150}; // brace position (CHANGEME)
  const units::angle::turn_t ROTATION_THRESHOLD{
      1}; // number of rotations to stop motor (CHANGEME)
};
