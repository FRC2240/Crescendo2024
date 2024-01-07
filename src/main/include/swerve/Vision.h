#pragma once
#include <stdlib.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <networktables/NetworkTableInstance.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include "Odometry.h"
#include "SwerveModule.h"
#include "Drivetrain.h"

class Vision
{
public:
    Vision(Drivetrain *drivetrain, Odometry *odometry);

private:
    std::shared_ptr<nt::NetworkTable> m_right_table =
        nt::NetworkTableInstance::GetDefault().GetTable("limelight");

    ~Vision();
    Drivetrain *m_drivetrain;
    Odometry *m_odometry;

    // frc::SwerveDrivePoseEstimator<4> m_estimator {
    //     m_drivetrain->kinematics,
    //     m_drivetrain->getAngle(),

    // }
};
