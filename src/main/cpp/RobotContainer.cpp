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
  m_stick1.RightStick().OnTrue(m_trajectory.manual_drive());
  m_stick0.X().ToggleOnTrue(m_shooter.fender_shot());

  //   m_stick.Y().OnTrue(
  //       m_trajectory.auto_score_align()
  //           .AlongWith(m_shooter.set_angle_cmd(m_vision.get_shooter_angle()))
  //           .AndThen(m_shooter.execute_auto_shot().WithTimeout(0.5_s)));

  // Buddy Climber
  m_stick1.LeftBumper().OnTrue(m_buddyClimber.StartLeftCommand());
  m_stick1.RightBumper().OnTrue(m_buddyClimber.StartRightCommand());
  m_stick1.Start().OnTrue(m_buddyClimber.DeployCommand());

  // Intake
  frc2::Trigger{[this] -> bool
                {
                  int pov = m_stick1.GetPOV();
                  return pov == 0;
                }}
      .OnTrue(m_intake.StartCommand()); // change to extend?

  frc2::Trigger{[this] -> bool
                {
                  frc::SmartDashboard::PutNumber("pov", m_stick1.GetPOV());
                  frc::SmartDashboard::PutBoolean("Threshold", CONSTANTS::IN_THRESHOLD<int>(m_stick1.GetPOV(), 180, 30));
                  return CONSTANTS::IN_THRESHOLD<int>(m_stick1.GetPOV(), 180, 30);
                }}
      .OnTrue(m_intake.StopCommand()); // change to retract?
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
