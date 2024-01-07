#include "swerve/Trajectory.h"

#ifndef CFG_NO_DRIVEBASE
/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

// This is using lambdas in order to use setters at beginning of runtime & save performance later
/*static frc::HolonomicDriveController controller{
    frc2::PIDController{1, 0, 0},
    frc2::PIDController{1, 0, 0},*/
/*frc::ProfiledPIDController<units::radian>{
    //IMPORTANT: THIS DOES NOTHING.
    40, 0, 0,
    //0.8, 0.0, 0.0,
    frc::TrapezoidProfile<units::radian>::Constraints{

        CONSTANTS::DRIVE::TRAJ_MAX_ANGULAR_SPEED,
        CONSTANTS::DRIVE::TRAJ_MAX_ANGULAR_ACCELERATION}}};*/

frc::Timer m_trajTimer;

Trajectory::Trajectory(Drivetrain *drivetrain, Odometry *odometry, frc::XboxController *stick)
    : m_drivetrain{drivetrain}, m_odometry{odometry}, m_stick{stick}
{
    AutoBuilder::configureHolonomic(
        [this]() -> frc::Pose2d
        {
            return m_odometry->getPose();
        },
        [this](frc::Pose2d pose) -> void
        {
            m_odometry->resetPosition(pose, frc::Rotation2d(m_drivetrain->get_absolute_angle()));
        },
        [this]() -> frc::ChassisSpeeds
        {
            return m_drivetrain->getRobotRelativeSpeeds();
        },
        [this](frc::ChassisSpeeds speeds) -> void
        {
            return m_drivetrain->drive(speeds);
        },
        HolonomicPathFollowerConfig(
            PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5_mps,                     // Max module speed, in m/s
            10_in,                       // Drive base radius in meters. Distance from robot center to furthest module.
            ReplanningConfig()           // Default path replanning config. See the API for the options here),
            ),
        this);
}

frc2::CommandPtr Trajectory::manual_drive(bool field_relative)
{
    return frc2::cmd::Run(
        [this, field_relative]
        {
            if (m_stick->GetStartButtonReleased())
            {
                m_drivetrain->zero_yaw();
            }

            const units::meters_per_second_t left_right{-frc::ApplyDeadband(m_stick->GetLeftX(), 0.1) * CONSTANTS::DRIVE::TELEOP_MAX_SPEED};
            const units::meters_per_second_t front_back{frc::ApplyDeadband(m_stick->GetLeftY(), 0.1) * CONSTANTS::DRIVE::TELEOP_MAX_SPEED};
            auto const rot = frc::ApplyDeadband(m_stick->GetRightX(), .1) * m_drivetrain->TELEOP_MAX_ANGULAR_SPEED;
            m_drivetrain->drive(front_back, -left_right, -rot, field_relative);
        },
        {this});
}

frc2::CommandPtr Trajectory::make_relative_line_path(units::meter_t x, units::meter_t y, frc::Rotation2d rot)
{

    return frc2::cmd::DeferredProxy(
               [this, x, y, rot]
               {
                   auto pose = m_odometry->getPose();
                   fmt::println("Current Pose: {},{},{}", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value());
                   std::vector<frc::Pose2d> points{
                       pose, // First point is always where you are
                       frc::Pose2d(pose.X() + x, pose.Y() + y, rot)};
                   frc::SmartDashboard::PutString("here?", "here");

                   fmt::println("Target Pose: {},{},{}", (pose.X() + x).value(), (pose.Y() + y).value(), rot.Degrees().value());

                   std::vector<frc::Translation2d>
                       bezierPoints = PathPlannerPath::bezierFromPoses(points);
                   auto path = std::make_shared<PathPlannerPath>(bezierPoints, DEFAULT_CONSTRAINTS, GoalEndState(0.0_mps, rot));

                   return AutoBuilder::followPathWithEvents(path).AndThen(frc2::PrintCommand("here").ToPtr());
               })
        .AndThen(frc2::cmd::RunOnce(
            [=, this]
            {
            auto pose = m_odometry->getPose();
            fmt::println("Current Pose: {},{},{}", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value()); }));
}

frc2::CommandPtr Trajectory::make_absolute_line_path(frc::Pose2d target_pose)
{
    return frc2::PrintCommand("start").ToPtr().AndThen(AutoBuilder::pathfindToPose(target_pose, DEFAULT_CONSTRAINTS, 1_mps, 0.0_m)).AndThen(frc2::PrintCommand("end").ToPtr());
}

frc2::CommandPtr Trajectory::extract(std::string auton)
{
    return AutoBuilder::buildAuto(auton);
}
#endif
