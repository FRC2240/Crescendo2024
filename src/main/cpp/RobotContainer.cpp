// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{

  // m_odometry.resetPosition(frc::Pose2d(1.6_m, 5_m, frc::Rotation2d(0_rad)),
  // frc::Rotation2d(0_rad)); Initialize all of your commands and subsystems
  // here Configure the button bindings

  m_chooser.AddOption("Cross Auto Line", AUTOS::AUTOLINE);
  m_chooser.AddOption("Two Game Piece", AUTOS::TWO_GP);
  m_chooser.AddOption("Test", AUTOS::TEST);

  frc::SmartDashboard::PutData(&m_chooser);
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
  // Configure your trigger bindings here
  m_trajectory.SetDefaultCommand(m_trajectory.manual_drive());
  m_stick.RightStick().OnTrue(m_trajectory.manual_drive());
  // m_stick.X().OnTrue(m_shooter.fender_shot().WithTimeout(0.5_s));

  //   m_stick.Y().OnTrue(
  //       m_trajectory.auto_score_align()
  //           .AlongWith(m_shooter.set_angle_cmd(m_vision.get_shooter_angle()))
  //           .AndThen(m_shooter.execute_auto_shot().WithTimeout(0.5_s)));

  // Buddy Climber
  m_stick.LeftBumper().OnTrue(m_buddyClimber.StartLeftCommand());
  m_stick.RightBumper().OnTrue(m_buddyClimber.StartRightCommand());
  m_stick.Start().OnTrue(m_buddyClimber.DeployCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  frc::DataLogManager::Log("here");
  switch (m_chooser.GetSelected())
  {
  case AUTOS::AUTOLINE:
    return autos::autoline(&m_trajectory);
    break;
  case AUTOS::TWO_GP:
    return autos::two_gp(&m_trajectory);
    break;
  case AUTOS::TEST:
    return autos::test(&m_trajectory);
    break;
  default:
    frc::DataLogManager::Log("WARN: NO ERROR SELECTED");
    break;
  }
}
