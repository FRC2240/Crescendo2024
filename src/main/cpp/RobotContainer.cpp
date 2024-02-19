// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{

  // m_odometry.resetPosition(frc::Pose2d(1.6_m, 5_m, frc::Rotation2d(0_rad)),
  // frc::Rotation2d(0_rad)); Initialize all of your commands and subsystems
  // here Configure the button bindings

  m_chooser.AddOption("Position 1 autoline", AUTOS::POS_1_LINE);
  m_chooser.AddOption("Position 2 autoline", AUTOS::POS_2_LINE);
  m_chooser.AddOption("Position 3 autoline", AUTOS::POS_3_LINE);
  m_chooser.AddOption("Position 1 two game piece", AUTOS::POS_1_GP2);
  m_chooser.AddOption("Position 2 two game piece", AUTOS::POS_2_GP2);
  m_chooser.AddOption("Position 3 two game piece", AUTOS::POS_3_GP2);
  m_chooser.AddOption("Position 1 three game piece", AUTOS::POS_1_GP3);
  m_chooser.AddOption("Position 2 three game piece", AUTOS::POS_2_GP3);
  m_chooser.AddOption("Position 3 three game piece", AUTOS::POS_3_GP3);
  m_chooser.AddOption("Position 1 four game piece", AUTOS::POS_1_GP4);
  m_chooser.AddOption("Position 2 four game piece", AUTOS::POS_2_GP4);
  m_chooser.AddOption("Position 3 four game piece", AUTOS::POS_3_GP4);

  frc::SmartDashboard::PutData(&m_chooser);
  m_odometry.putField2d();
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
  // Configure your trigger bindings here
  m_trajectory.SetDefaultCommand(m_trajectory.manual_drive());
  m_shooter.SetDefaultCommand(m_shooter.default_cmd());
  m_intake.SetDefaultCommand(m_intake.StopCommand());
  m_stick1.RightStick().OnTrue(m_trajectory.manual_drive());

  // Shooter
  m_stick0.X().ToggleOnTrue(m_shooter.test_shot());
  m_stick0.RightBumper().ToggleOnTrue(m_shooter.fender_shot());
  m_stick0.A().ToggleOnTrue(m_shooter.amp_shot());
  m_stick0.LeftBumper().ToggleOnTrue(m_intake.StartCommand());
  m_stick0.LeftTrigger().ToggleOnTrue(m_trajectory.auto_pickup());
  m_stick0.LeftTrigger().ToggleOnTrue(m_intake.StartCommand());

  // m_stick0.RightTrigger().ToggleOnTrue(
  // frc2::PrintCommand("button pressed").ToPtr().AndThen(m_trajectory.auto_score_align().AlongWith(m_shooter.set_angle_cmd(m_odometry.get_shooter_angle())).AndThen(m_shooter.execute_auto_shot().WithTimeout(1.5_s))));

  m_stick0.RightTrigger().ToggleOnTrue(frc2::cmd::DeferredProxy([this]
                                                                { return m_shooter.set_angle_cmd(m_odometry.get_shooter_angle()); }));

  // Buddy Climber

  m_stick1.LeftBumper()
      .OnTrue(m_buddyClimber.StartLeftCommand());
  m_stick1.RightBumper().OnTrue(m_buddyClimber.StartRightCommand());
  m_stick1.Start().OnTrue(m_buddyClimber.DeployCommand());
  m_stick1.Back().OnTrue(m_buddyClimber.ResetCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  fmt::println("get auto cmd");
  switch (m_chooser.GetSelected())
  {
  case AUTOS::POS_1_LINE:
    return autos::pos_1_line(&m_trajectory);
    break;
  case AUTOS::POS_2_LINE:
    return autos::pos_2_line(&m_trajectory);
    break;
  case AUTOS::POS_3_LINE:
    return autos::pos_3_line(&m_trajectory);
    break;
  case AUTOS::POS_1_GP2:
    return autos::pos_1_gp2(&m_trajectory);
    break;
  case AUTOS::POS_2_GP2:
    return autos::pos_2_gp2(&m_trajectory);
    break;
  case AUTOS::POS_3_GP2:
    return autos::pos_3_gp2(&m_trajectory);
    break;
  case AUTOS::POS_1_GP3:
    return autos::pos_1_gp3(&m_trajectory);
    break;
  case AUTOS::POS_2_GP3:
    return autos::pos_2_gp3(&m_trajectory);
    break;
  case AUTOS::POS_3_GP3:
    return autos::pos_3_gp3(&m_trajectory);
    break;
  case AUTOS::POS_1_GP4:
    return autos::pos_1_gp4(&m_trajectory);
    break;
  case AUTOS::POS_2_GP4:
    return autos::pos_2_gp4(&m_trajectory);
    break;
  case AUTOS::POS_3_GP4:
    return autos::pos_3_gp4(&m_trajectory);
    break;
  default:
    frc::DataLogManager::Log("WARN: NO ERROR SELECTED");
    break;
  }
}
