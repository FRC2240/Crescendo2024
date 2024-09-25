
#pragma once

#include <frc2/command/CommandPtr.h>
#include "swerve/Trajectory.h"
namespace autos
{
    /**
     * Example static factory for an autonomous command.
     */
    frc2::CommandPtr pos_1_line(Trajectory *traj);
    frc2::CommandPtr pos_2_line(Trajectory *traj);
    frc2::CommandPtr pos_3_line(Trajectory *traj);
    frc2::CommandPtr pos_2_gp1(Trajectory *traj);
    frc2::CommandPtr pos_1_gp2(Trajectory *traj);
    frc2::CommandPtr pos_2_gp2(Trajectory *traj);
    frc2::CommandPtr pos_3_gp2(Trajectory *traj);
    frc2::CommandPtr pos_1_gp3(Trajectory *traj);
    frc2::CommandPtr pos_2_gp3(Trajectory *traj);
    frc2::CommandPtr pos_3_gp3(Trajectory *traj);
    frc2::CommandPtr pos_1_gp4(Trajectory *traj);
    frc2::CommandPtr pos_2_gp4(Trajectory *traj);
    frc2::CommandPtr pos_3_gp4(Trajectory *traj);
    frc2::CommandPtr test(Trajectory *traj, Odometry *odom);
} // namespace autos