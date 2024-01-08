// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{

  // m_odometry.resetPosition(frc::Pose2d(1.6_m, 5_m, frc::Rotation2d(0_rad)), frc::Rotation2d(0_rad));
  // Initialize all of your commands and subsystems here
  coral_auto = PathPlannerAuto("coral auto").ToPtr().Unwrap();
  frc::SmartDashboard::PutData("Coral Auto", coral_auto.get());
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
  // Configure your trigger bindings here
  m_trajectory.SetDefaultCommand(m_trajectory.manual_drive());
  m_stick.RightStick().OnTrue(m_trajectory.manual_drive());
  m_stick.X().OnTrue(m_shooter.fender_shot().WithTimeout(0.5_s));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  // An example command will be run in autonomous
}
