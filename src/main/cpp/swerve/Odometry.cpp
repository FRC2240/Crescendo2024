#include "swerve/Odometry.h"

#ifndef CFG_NO_DRIVEBASE
// frc::TimeOfFlight tof_sensor{1};
/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

// extern frc::SwerveDriveKinematics<4> const kinematics;

// This is not how it should be but doing it "correctly" (++,+-,-+,--) causes
// the wheels to form an "X" instead of diamond while turning.
// It's wrong but it works, no touchy.

frc::Field2d field2d;

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/
Odometry::Odometry(Drivetrain *drivetrain, Vision *vision)
    : m_drivetrain{drivetrain}, m_vision{vision}
{
}

frc2::CommandPtr Odometry::set_pose_cmd(frc::Pose2d pose)
{
    return frc2::cmd::RunOnce([this, &pose]
                              { resetPosition(pose, frc::Rotation2d(0_rad)); },
                              {})
        .AndThen(frc2::PrintCommand("reset odometry").ToPtr());
}

void Odometry::putField2d()
{
    frc::SmartDashboard::PutData("Odometry Field", &field2d);
}

void Odometry::update()
{
    frc::Pose2d const pose = estimator.Update(m_drivetrain->getCCWHeading(),
                                              m_drivetrain->getModulePositions());
    // if constexpr (CONSTANTS::DEBUGGING)
    frc::SmartDashboard::PutNumber("odometry/X", pose.X().value());
    frc::SmartDashboard::PutNumber("odometry/Y", pose.Y().value());
    frc::SmartDashboard::PutString("Odometry: ", fmt::format("Pose X: {}, Y: {}, Z (Degrees): {}\n", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value()));
}

frc::Pose2d Odometry::getPose() { return estimator.Update(m_drivetrain->getCCWHeading(),
                                                          m_drivetrain->getModulePositions()); }

frc::ChassisSpeeds const Odometry::getFieldRelativeSpeeds()
{
    // Init for first time
    static frc::Timer speed_timer;
    speed_timer.Start();
    static frc::Pose2d previous_pose{};

    frc::Pose2d const current_pose = estimator.GetEstimatedPosition();

    frc::Pose2d const delta_pose = current_pose.RelativeTo(previous_pose);

    auto const time_elapsed = speed_timer.Get();
    units::meters_per_second_t const X = delta_pose.X() / time_elapsed;

    units::meters_per_second_t const Y = delta_pose.Y() / time_elapsed;

    units::degrees_per_second_t const rot{delta_pose.Rotation().Degrees() / time_elapsed};

    previous_pose = estimator.GetEstimatedPosition(); // Set the previous_pose for the next time this loop is run

    speed_timer.Reset(); // Time how long until next call

    return frc::ChassisSpeeds{X, Y, rot};
}

void Odometry::reset_position_from_vision(const frc::Pose2d &bot_pose)
{
    estimator.ResetPosition(m_drivetrain->getCCWHeading(),
                            m_drivetrain->getModulePositions(),
                            bot_pose);
}

// void Odometry::reset_from_distance()
// {
//     units::millimeter_t raw_dist {tof_sensor.GetRange()};
//     units::meter_t dist {raw_dist};
//     units::meter_t x;
//     units::meter_t y{Odometry::getPose().X()};
//     if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
//     {
//         x = 7.94_m - dist;
//     }
//     else
//     {
//         x = -7.94_m + dist;
//     }
//     frc::Pose2d pose{x, y, m_drivetrain->getCCWHeading()};

//     odometry.ResetPosition(m_drivetrain->getCCWHeading(),
//                            m_drivetrain->getModulePositions(),
//                            pose
//                            );
// }

void Odometry::resetPosition(const frc::Pose2d &bot_pose, const frc::Rotation2d &gyro_angle)
{
    estimator.ResetPosition(gyro_angle, m_drivetrain->getModulePositions(), bot_pose);
}

frc::FieldObject2d *Odometry::getField2dObject(std::string_view name)
{
    return field2d.GetObject(name);
}

void Odometry::add_vision_measurment(const frc::Pose2d &pose)
{
    estimator.AddVisionMeasurement(pose, frc::Timer::GetFPGATimestamp());
}

void Odometry::update_from_vision()
{
    auto pose = estimator.GetEstimatedPosition();
    frc::SmartDashboard::PutNumber("odometry/X", pose.X().value());
    frc::SmartDashboard::PutNumber("odometry/Y", pose.Y().value());

    for (std::optional<frc::Pose2d> i : m_vision->get_bot_position())
    {
        if (i)
        {
            frc::SmartDashboard::PutNumber("auto thing", i.value().X().value());
            estimator.AddVisionMeasurement(i.value(), frc::Timer::GetFPGATimestamp());
        }
    }
}

std::optional<units::degree_t> Odometry::get_coral()
{
}

std::optional<units::meter_t> Odometry::get_dist_to_tgt()
{
    frc::SmartDashboard::PutBoolean("tv", m_limelight->GetBoolean("tv", 0));
    if (m_limelight->GetNumber("tv", 0))
    {
        auto results = m_limelight->GetNumberArray("targetpose_robotspace", std::vector<double>(6));
        return units::meter_t{(std::sqrt(std::pow(results[0], 0) + std::pow(results[1], 2)))};
    }
    else
    {
        return std::nullopt;
    }
}
#endif
