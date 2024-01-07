#pragma once

#include "swerve/Drivetrain.h"

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <pathplanner/lib/path/PathPlannerTrajectory.h>
#include <pathplanner/lib/path/PathPoint.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include "Constants.h"
#include <functional>
#include <cmath>
#include "swerve/ngr.h"
#include "swerve/Odometry.h"
#include <frc/DriverStation.h>

#include <frc/controller/HolonomicDriveController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/SubsystemBase.h>
#include <chrono>
#include <thread>
#include <frc/Timer.h>
#include <frc/XboxController.h>

#ifndef CFG_NO_DRIVEBASE
using namespace pathplanner; // PathPlanner keeps everything hidden behind 2 sets of namespaces so it's safe to remove the first layer

class Trajectory : public frc2::SubsystemBase
{
public:
    Trajectory(
        Drivetrain *drivetrain,
        Odometry *odometry,
        frc::XboxController *stick);

    // Note: a 2023 comment means it is Moonwalker Specific and can be safely removed.

    // 2023
    struct TrajDepends
    {
        units::meter_t current_x;
        units::meter_t current_y;
        units::degree_t current_head;
        units::degree_t current_rot;
        units::meter_t desired_x;
        units::meter_t desired_y;
        units::degree_t desired_head;
        units::degree_t desired_rot;
    };
    const PathConstraints DEFAULT_CONSTRAINTS = PathConstraints(14_fps, 7_fps_sq, 360_deg_per_s, 720_deg_per_s_sq);
    // constexpr PathConstraints DEFAULT_CONSTRAINTS = PathConstraints(CONSTANTS:)

    frc2::CommandPtr manual_drive(bool field_relative = true);

    frc2::CommandPtr make_absolute_line_path(frc::Pose2d target_pose);

    frc2::CommandPtr make_relative_line_path(units::meter_t x, units::meter_t y, frc::Rotation2d rot);

    frc2::CommandPtr extract(std::string auton);

private:
    Drivetrain *m_drivetrain;
    Odometry *m_odometry;
    frc::XboxController *m_stick;
};
#endif
