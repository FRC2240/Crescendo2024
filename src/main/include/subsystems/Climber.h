// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
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

private:
  // Components (e.g. motor controllers and sensors) should generally be
  //  declared private and exposed only through public methods.

  frc::XboxController *m_stick;
  // TODO:
  // 1. Fix typo
  // 2. Add second motor
  // 3. Put CAN ID in constants
  ctre::phoenix6::hardware::TalonFX hieght_climber{19};
};
