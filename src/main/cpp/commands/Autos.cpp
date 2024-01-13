#include "commands/Autos.h"

#include <frc2/command/Commands.h>


frc2::CommandPtr autos::autoline(Trajectory *traj)
{
  return frc2::PrintCommand("start crossline").ToPtr().AndThen(traj->extract("crossline auto").AndThen(frc2::PrintCommand("line cross").ToPtr()));
}
frc2::CommandPtr autos::two_gp(Trajectory *traj){
    return frc2::PrintCommand("start two gamepiece").ToPtr().AndThen(traj->extract("two gp auto").AndThen(frc2::PrintCommand("two piece completed").ToPtr()));
}