#include "commands/Autos.h"

#include <frc2/command/Commands.h>

frc2::CommandPtr autos::pos_1_line(Trajectory *traj)
{
  return frc2::PrintCommand("start cross line pos_1").ToPtr().AndThen(traj->extract("pos_1_autoline").AndThen(frc2::PrintCommand("autoline completed").ToPtr()));
}

frc2::CommandPtr autos::pos_2_line(Trajectory *traj)
{
  return frc2::PrintCommand("start cross line pos_2").ToPtr().AndThen(traj->extract("pos_2_autoline").AndThen(frc2::PrintCommand("autoline completed").ToPtr()));
}

frc2::CommandPtr autos::pos_3_line(Trajectory *traj)
{
  return frc2::PrintCommand("start cross line pos_3").ToPtr().AndThen(traj->extract("a_pos_3").AndThen(frc2::PrintCommand("autoline completed").ToPtr()));
}

frc2::CommandPtr autos::pos_1_gp2(Trajectory *traj)
{
  return frc2::PrintCommand("start two gamepiece pos_1").ToPtr().AndThen(traj->extract("a_pos_1_gp2").AndThen(frc2::PrintCommand("two piece completed").ToPtr()));
}

frc2::CommandPtr autos::pos_2_gp2(Trajectory *traj)
{
  return frc2::PrintCommand("start two gamepiece pos_2").ToPtr().AndThen(traj->extract("a_pos_2_gp2").AndThen(frc2::PrintCommand("two piece completed").ToPtr()));
}

frc2::CommandPtr autos::pos_3_gp2(Trajectory *traj)
{
  return frc2::PrintCommand("start two gamepiece pos_3").ToPtr().AndThen(traj->extract("a_sourcide_gp2").AndThen(frc2::PrintCommand("two piece completed").ToPtr()));
}

frc2::CommandPtr autos::pos_1_gp3(Trajectory *traj)
{
  return frc2::PrintCommand("start three gamepiece pos_1").ToPtr().AndThen(traj->extract("a_pos_2_gp3v2").AndThen(frc2::PrintCommand("three piece completed").ToPtr()));
}

frc2::CommandPtr autos::pos_2_gp3(Trajectory *traj)
{
  return frc2::PrintCommand("start three gamepiece pos_2").ToPtr().AndThen(traj->extract("a_pos_2_gp3v2").AndThen(frc2::PrintCommand("three piece completed").ToPtr()));
}

frc2::CommandPtr autos::pos_3_gp3(Trajectory *traj)
{
  return frc2::PrintCommand("start three gamepiece pos_3").ToPtr().AndThen(traj->extract("a_pos_3_gp3").AndThen(frc2::PrintCommand("three piece completed").ToPtr()));
}

frc2::CommandPtr autos::pos_1_gp4(Trajectory *traj)
{
  return frc2::PrintCommand("start four gamepiece pos_1").ToPtr().AndThen(traj->extract("a_pos_1_gp4").AndThen(frc2::PrintCommand("four piece completed").ToPtr()));
}

frc2::CommandPtr autos::pos_2_gp4(Trajectory *traj)
{
  return frc2::PrintCommand("start four gamepiece pos_2").ToPtr().AndThen(traj->extract("a_pos_2_gp4v2").AndThen(frc2::PrintCommand("four piece completed").ToPtr()));
}

frc2::CommandPtr autos::pos_3_gp4(Trajectory *traj)
{
  return frc2::PrintCommand("start four gamepiece pos_1").ToPtr().AndThen(traj->extract("a_pos_3_gp4").AndThen(frc2::PrintCommand("four piece completed").ToPtr()));
}

frc2::CommandPtr autos::pos_2_gp1(Trajectory *traj)
{
  return frc2::PrintCommand("start four gamepiece pos_1").ToPtr().AndThen(traj->extract("a_pos_2_gp1").AndThen(frc2::PrintCommand("four piece completed").ToPtr()));
}
frc2::CommandPtr autos::test(Trajectory *traj)
{
  return frc2::PrintCommand("test").ToPtr().AndThen(traj->extract("a_test_all").AndThen(frc2::PrintCommand("four piece completed").ToPtr()));
}