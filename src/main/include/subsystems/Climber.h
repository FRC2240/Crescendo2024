// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <units/angle.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/XboxController.h>

class Climber : public frc2::SubsystemBase {
 public:
   Climber();
    void climb(double pos);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  frc::XboxController m_stick_init{0};
  frc::XboxController *m_stick;
 
 private:

  //Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
   
    ctre::phoenix6::hardware::TalonFX hieght_climber {0};

};
