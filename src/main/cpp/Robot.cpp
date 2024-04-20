// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit()
{
  m_container.m_odometry.putField2d();
}

void Robot::RobotPeriodic()
{
  auto start_time = frc::Timer::GetFPGATimestamp();
  // auto candle_cmd = m_container.m_candle.get_command(&m_container.m_stick1);
  // candle_cmd.Schedule();

  // m_container.m_odometry.update_from_vision();
  m_container.m_odometry.update();
  frc2::CommandScheduler::GetInstance().Run();
  auto end_time = frc::Timer::GetFPGATimestamp() - start_time;
  frc::SmartDashboard::PutNumber("cycle time", end_time.value());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic()
{
  m_container.m_odometry.update_from_vision();
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit()
{
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic()
{
}

void Robot::AutonomousExit() {}

void Robot::TeleopInit()
{
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic()
{
  m_container.m_odometry.update_from_vision();
}

void Robot::TeleopExit() {}

void Robot::TestInit()
{
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {
  m_container.m_drivetrain.test_modules();
}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
