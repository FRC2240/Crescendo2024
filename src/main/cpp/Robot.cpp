// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {}

<<<<<<< HEAD
void Robot::RobotPeriodic()
{
  m_container.m_odometry.update();
||||||| parent of 216cacc (vision fixes)
void Robot::RobotPeriodic() {
=======
void Robot::RobotPeriodic()
{
  m_container.m_odometry.update_from_vision();
  m_container.m_odometry.update();
>>>>>>> 216cacc (vision fixes)
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit()
{
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }
}

<<<<<<< HEAD
void Robot::AutonomousPeriodic()
{
  m_container.m_odometry.update();
||||||| parent of 216cacc (vision fixes)
void Robot::AutonomousPeriodic() {
    m_container.m_odometry.update();

=======
void Robot::AutonomousPeriodic()
{
>>>>>>> 216cacc (vision fixes)
}

void Robot::AutonomousExit() {}

void Robot::TeleopInit()
{
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit()
{
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
