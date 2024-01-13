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

class BuddyClimber : public frc2::SubsystemBase
{
public:
  BuddyClimber();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  
  frc2::CommandPtr ExtendCommand();
  frc2::CommandPtr RetractCommand();

private:
  ctre::phoenix6::hardware::TalonFX m_clawMotor{4};

  units::angle::turn_t START_ROTATIONS{0}; //change
  units::angle::turn_t END_ROTATIONS{100}; //change
};
