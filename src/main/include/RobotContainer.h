// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/Arm.h"
#include "commands/Intake.h"
#include "swerve/Drivetrain.h"
#include "swerve/Odometry.h"
#include "swerve/Trajectory.h"
#include "swerve/Vision.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer
{
public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  frc2::CommandXboxController m_driverController{
      CONSTANTS::XBOX_PORT};

  Drivetrain m_drivetrain;
  Odometry m_odometry{&m_drivetrain};
  Trajectory m_trajectory{&m_drivetrain, &m_odometry, &m_driverController};
  void ConfigureBindings();

private:
  // Replace with CommandPS4Controller or CommandJoystick if needed

  // The robot's subsystems are defined here...
  std::unique_ptr<frc2::Command> coral_auto;
  ExampleSubsystem m_subsystem;
  Arm m_arm;
  Intake m_intake;
};
