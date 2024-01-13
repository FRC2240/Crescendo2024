
#pragma once

#include <frc2/command/CommandPtr.h>
#include "swerve/Trajectory.h"
namespace autos
{
    /**
     * Example static factory for an autonomous command.
     */
    frc2::CommandPtr autoline(Trajectory *traj);
    frc2::CommandPtr two_gp(Trajectory *traj);
} // namespace autos