// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{

  // m_odometry.resetPosition(frc::Pose2d(1.6_m, 5_m, frc::Rotation2d(0_rad)),
  // frc::Rotation2d(0_rad)); Initialize all of your commands and subsystems
  // here Configure the button bindings

  m_chooser.AddOption("just shoot", AUTOS::SHOOT);
  m_chooser.AddOption("DO NOT USE IN COMP (3gp)", AUTOS::POS_2_GP3);
  // m_chooser.AddOption("Feederside 1gp", AUTOS::POS_3_GP1);
  // m_chooser.AddOption("Position 2 autoline", AUTOS::POS_2_LINE);
  // m_chooser.AddOption("Position 3 autoline", AUTOS::POS_3_LINE);
  // m_chooser.AddOption("Position 1 two game piece", AUTOS::POS_1_GP2);
  m_chooser.AddOption("2 GP", AUTOS::POS_2_GP2);
  // m_chooser.AddOption("Position 3 two game piece", AUTOS::POS_3_GP2);
  // m_chooser.AddOption("Position 1 three game piece", AUTOS::POS_1_GP3);
  // m_chooser.AddOption("Position 2 three game piece", AUTOS::POS_2_GP3);
  // m_chooser.AddOption("Position 3 three game piece", AUTOS::POS_3_GP3);
  // m_chooser.AddOption("Position 1 four game piece", AUTOS::POS_1_GP4);
  // m_chooser.AddOption("Position 2 four game piece", AUTOS::POS_2_GP4);
  // m_chooser.AddOption("Position 3 four game piece", AUTOS::POS_3_GP4);
  m_chooser.AddOption("Position 2 one game piece", AUTOS::POS_2_GP1);
  // m_chooser.AddOption("TEST", AUTOS::TEST);

  frc::SmartDashboard::PutData(&m_chooser);
  m_odometry.putField2d();
  ConfigureBindings();
  add_named_commands();
}

void RobotContainer::add_named_commands()
{
  using namespace pathplanner;

  NamedCommands::registerCommand("intake", std::move(m_intake.StartCommand()));
  NamedCommands::registerCommand("score", std::move(m_shooter.fender_shot()));
  NamedCommands::registerCommand("auto_score", std::move(m_shooter.set_angle_cmd(m_odometry.get_shooter_angle())));
}

void RobotContainer::ConfigureBindings()
{
  // Configure your trigger bindings here
  m_trajectory.SetDefaultCommand(m_trajectory.manual_drive());
  m_shooter.SetDefaultCommand(m_shooter.default_cmd());
  m_intake.SetDefaultCommand(m_intake.StopCommand());
  m_climber.SetDefaultCommand(m_climber.StopCommand());
  m_stick1.RightStick().ToggleOnTrue(m_trajectory.manual_drive());

  // Shooter
  m_stick1.X().ToggleOnTrue(m_shooter.test_shot()); // testing ONLY
  // m_stick0.RightBumper().ToggleOnTrue(m_shooter.execute_auto_shot());
  m_stick0.A().ToggleOnTrue(m_shooter.amp_shot());
  m_stick0.RightBumper().ToggleOnTrue(m_shooter.fender_shot());
  m_stick0.LeftBumper().ToggleOnTrue(m_intake.StartCommand());
  // m_stick0.LeftTrigger().ToggleOnTrue(m_trajectory.auto_pickup());
  m_stick1.RightTrigger().WhileTrue(m_shooter.ManualFeedCommand(true));
  m_stick1.RightTrigger().WhileTrue(m_intake.ManualFeedCommand(true));
  m_stick0.B().WhileTrue(m_shooter.spool_cmd());

  m_stick1.LeftTrigger().WhileTrue(m_shooter.ManualFeedCommand(false));
  m_stick1.LeftTrigger().WhileTrue(m_intake.ManualFeedCommand(false));
  // m_stick1.Start().OnTrue(m_intake.zero());
  // m_stick1.Start().OnTrue(m_shooter.zero());

  // m_stick0.RightTrigger().ToggleOnTrue(
  // frc2::PrintCommand("button pressed").ToPtr().AndThen(m_trajectory.auto_score_align().AlongWith(m_shooter.set_angle_cmd(m_odometry.get_shooter_angle())).AndThen(m_shooter.execute_auto_shot().WithTimeout(1.5_s))));

  // m_stick0.RightTrigger().ToggleOnTrue(frc2::cmd::DeferredProxy([this]
  // { return m_shooter.set_angle_cmd(m_odometry.get_shooter_angle()); }));

  // Buddy Climber
  // Climber
  frc2::Trigger{[this] -> bool
                {
                  int pov = m_stick1.GetPOV();
                  return pov == 0;
                }}
      .WhileTrue(m_climber.UpCommand());

  frc2::Trigger{[this] -> bool
                {
                  frc::SmartDashboard::PutNumber("pov", m_stick1.GetPOV());
                  frc::SmartDashboard::PutBoolean("Threshold", CONSTANTS::IN_THRESHOLD<int>(m_stick1.GetPOV(), 180, 30));
                  return CONSTANTS::IN_THRESHOLD<int>(m_stick1.GetPOV(), 180, 30);
                }}
      .WhileTrue(m_climber.DownCommand());
  // Candle
  // m_candle.SetDefaultCommand(m_candle.off());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  fmt::println("get auto cmd");
  switch (m_chooser.GetSelected())
  {
  case AUTOS::POS_2_GP2:
    return autos::pos_2_gp2(&m_trajectory);
    break;
  case AUTOS::POS_2_GP1:
    return autos::pos_2_gp1(&m_trajectory);
    break;
  case RobotContainer::AUTOS::SHOOT:
    return m_shooter.fender_shot();
    break;
  case RobotContainer::AUTOS::POS_2_GP3:
    return autos::pos_2_gp3(&m_trajectory);
    break;
  default:
    frc::DataLogManager::Log("WARN: NO AUTO SELECTED");
    // m_candle.auto_selected = false;
    break;
  }
}
