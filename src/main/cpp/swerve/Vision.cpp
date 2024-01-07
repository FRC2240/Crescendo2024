#include "swerve/Vision.h"

Vision::Vision(Drivetrain *drivetrain, Odometry *odometry)
    : m_drivetrain{drivetrain}, m_odometry{odometry}
{
}
