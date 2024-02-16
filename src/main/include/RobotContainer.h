// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "commands/Autos.h"
#include "subsystems/Climber.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/BuddyClimber.h"
#include "swerve/Drivetrain.h"
#include "swerve/Odometry.h"
#include "swerve/Trajectory.h"
#include "swerve/Vision.h"
#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/button/Trigger.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <subsystems/Shooter.h>
#include "subsystems/Candle.h"

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

  frc2::CommandXboxController m_stick0{0};
  frc2::CommandXboxController m_stick1{1};

  Drivetrain m_drivetrain;
  Vision m_vision{
      [this]() -> units::degree_t
      {
        return m_drivetrain.getAngle();
      }};
  Odometry m_odometry{&m_drivetrain, &m_vision};
  Trajectory m_trajectory{&m_drivetrain, &m_odometry, &m_stick0, &m_vision};
  void ConfigureBindings();

  enum AUTOS
  {
    POS_1_LINE,
    POS_2_LINE,
    POS_3_LINE,
    POS_1_GP2,
    POS_2_GP2,
    POS_3_GP2,
    POS_1_GP3,
    POS_2_GP3,
    POS_3_GP3,
    POS_1_GP4,
    POS_2_GP4,
    POS_3_GP4
  };
  Intake m_intake;
  Climber m_climber{&m_stick0};
  Shooter m_shooter{&m_intake};
  BuddyClimber m_buddyClimber;
  Candle m_candle;

  std::vector<std::optional<frc::Pose2d>> bot_pose = m_vision.get_bot_position();

private:
  // Replace with CommandPS4Controller or CommandJoystick if needed

  // The robot's subsystems are defined here...
  frc::SendableChooser<AUTOS> m_chooser;
};
