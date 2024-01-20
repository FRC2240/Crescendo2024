#pragma once
#include "Constants.h"
#include <units/angle.h>
#include <stdlib.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <networktables/NetworkTableInstance.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Vision
{
public:
    /*
    At the time of writing, Vision must be capable of:
    1. Translation accuracy within 6in
    1.a. Doing so no slower than 0.9s
    2. determining the angle of a note to within 3 degrees
    3. Determing the distance to a note within 29 inches
    4. Determing the angle to an apriltag within 2 degrees
    */

    Vision();
    ~Vision();

    // Optionals are used liberally in this file due to the uncertain nature of vison.
    // See https://stackoverflow.com/questions/16860960/how-should-one-use-stdoptional for more info

    std::optional<units::degree_t> get_shooter_angle();

    // Returns a vector of optionals of camera outputs.
    // The caller is expected to handle absent data, not the function.
    std::vector<std::optional<frc::Pose2d>> get_bot_position();

    // Returns the angle to a gp
    std::optional<units::degree_t> get_coral_angle();

    // Returns angle to apriltag 4 or 7, depending on alliance color.
    // Could be modified to work with 3 and 8 as well
    std::optional<units::degree_t> get_apriltag_angle();

private:
    std::shared_ptr<nt::NetworkTable> m_limelight =
        nt::NetworkTableInstance::GetDefault().GetTable("limelight");

    // frc::SwerveDrivePoseEstimator<4> m_estimator {
    //     m_drivetrain->kinematics,
    //     m_drivetrain->getAngle(),

    // }
};
