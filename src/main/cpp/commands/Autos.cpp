#include "commands/Autos.h"

#include <frc2/command/Commands.h>

frc2::CommandPtr autos::pos_1_gp2(Trajectory *traj)
{
  return frc2::PrintCommand("start two gamepiece pos_1").ToPtr().AndThen(traj->extract("A_Pos_1_GP2").AndThen(frc2::PrintCommand("two piece completed").ToPtr()));
}

frc2::CommandPtr autos::autoline(Trajectory *traj)
{
  return frc2::PrintCommand("start crossline").ToPtr().AndThen(traj->extract("autoline").AndThen(frc2::PrintCommand("line cross").ToPtr()));
}
frc2::CommandPtr autos::two_gp(Trajectory *traj)
{
  return frc2::PrintCommand("start two gamepiece").ToPtr().AndThen(traj->extract("GP2").AndThen(frc2::PrintCommand("two piece completed").ToPtr()));
}