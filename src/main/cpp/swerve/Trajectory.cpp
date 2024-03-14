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

Trajectory::Trajectory(Drivetrain *drivetrain, Odometry *odometry, frc::XboxController *stick, Vision *vision, Intake *intake)
    : m_drivetrain{drivetrain}, m_odometry{odometry}, m_stick{stick}, m_vision{vision}, m_intake{intake}
{
    AutoBuilder::configureHolonomic(
        [this]() -> frc::Pose2d
        {
            auto pose = m_odometry->getPose();
            frc::SmartDashboard::PutNumber("pp/X", pose.X().value());
            frc::SmartDashboard::PutNumber("pp/Y", pose.Y().value());

            return pose;
        },
        [this](frc::Pose2d pose) -> void
        {
            // fmt::println("WARN: RESET POSE");

            frc::SmartDashboard::PutNumber("pp/rp/X", pose.X().value());
            frc::SmartDashboard::PutNumber("pp/rp/Y", pose.Y().value());
            m_odometry->resetPosition(pose, frc::Rotation2d(m_drivetrain->get_absolute_angle()));
        },
        [this]() -> frc::ChassisSpeeds
        {
            return m_drivetrain->getRobotRelativeSpeeds();
        },
        [this](frc::ChassisSpeeds speeds) -> void
        {
            // Hey, bozos, we changed this on 2024-03-08 to make pathplanner work
            // Without this change pathplanner finds a barrier with remarkable efficency due to not correcting right
            // This is a result of inverting the get_distance function
            return m_drivetrain->drive(-speeds);
        },
        HolonomicPathFollowerConfig(PIDConstants(10, 0.0, 0), // Translation PID constants
                                    PIDConstants(5, 0.0, 0),  // Rotation PID constants
                                    4.5_mps,                  // Max module speed, in m/s
                                    17.324_in,                // Drive base radius in meters. Distance from robot center to furthest module.
                                    ReplanningConfig()        // Default path replanning config. See the API for the options here),
                                    ),
        [this]() -> bool
        {
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance)
            {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
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

            const units::meters_per_second_t left_right{frc::ApplyDeadband(m_stick->GetLeftX(), 0.1) * CONSTANTS::DRIVE::TELEOP_MAX_SPEED};
            const units::meters_per_second_t front_back{frc::ApplyDeadband(m_stick->GetLeftY(), 0.1) * CONSTANTS::DRIVE::TELEOP_MAX_SPEED};
            auto const rot = frc::ApplyDeadband(m_stick->GetRightX(), .1) * m_drivetrain->TELEOP_MAX_ANGULAR_SPEED;
            m_drivetrain->drive(front_back, left_right, rot, field_relative);
        },
        {this});
}

frc2::CommandPtr Trajectory::make_relative_line_path(units::meter_t x, units::meter_t y, frc::Rotation2d rot)
{

    return frc2::cmd::DeferredProxy(
               [this, x, y, rot]
               {
                   auto pose = m_odometry->getPose();
                   // fmt::println("Current Pose: {},{},{}", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value());
                   std::vector<frc::Pose2d> points{
                       pose, // First point is always where you are
                       frc::Pose2d(pose.X() + x, pose.Y() + y, rot)};
                   frc::SmartDashboard::PutString("here?", "here");

                   // fmt::println("Target Pose: {},{},{}", (pose.X() + x).value(), (pose.Y() + y).value(), rot.Degrees().value());

                   std::vector<frc::Translation2d>
                       bezierPoints = PathPlannerPath::bezierFromPoses(points);
                   auto path = std::make_shared<PathPlannerPath>(bezierPoints, DEFAULT_CONSTRAINTS, GoalEndState(0.0_mps, rot));

                   return AutoBuilder::followPathWithEvents(path).AndThen(frc2::PrintCommand("here").ToPtr());
               })
        .AndThen(frc2::cmd::RunOnce(
            [=, this]
            {
                auto pose = m_odometry->getPose();
                // fmt::println("Current Pose: {},{},{}", pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value());
            }));
}

frc2::CommandPtr Trajectory::make_absolute_line_path(frc::Pose2d target_pose)
{
    return frc2::PrintCommand("start").ToPtr().AndThen(AutoBuilder::pathfindToPose(target_pose, DEFAULT_CONSTRAINTS, 1_mps, 0.0_m)).AndThen(frc2::PrintCommand("end").ToPtr());
}

frc2::CommandPtr Trajectory::extract(std::string auton)
{
    // fmt::println("{}", auton);
    return PathPlannerAuto(auton).ToPtr();
}

frc2::CommandPtr Trajectory::auto_pickup()
{

    std::function<void()> init = [this]()
    {
        /*no data currently needed */
        frc::DataLogManager::Log("posapo"); };
    std::function<void()> periodic = [this]()
    {
        try
        {
            std::optional<units::degree_t> angle = m_vision->get_coral_angle();
            if (angle)
            {
                frc::SmartDashboard::PutString("coral intake state", "target locked");
                frc::SmartDashboard::PutNumber("ima sicko mode", angle.value().value());
                m_drivetrain->face_direction(0_deg, angle.value().value());
                frc::DataLogManager::Log("3");
                if (angle.value() < CONSTANTS::INTAKE::AUTO_PICKUP_THRESHOLD &&
                    angle.value() > -CONSTANTS::INTAKE::AUTO_PICKUP_THRESHOLD)
                {
                    frc::DataLogManager::Log("4");
                    m_drivetrain->drive(-10_fps, 0_mps, (0_deg / 1_s), false);
                }
            }
            else
            {
                frc::DataLogManager::Log("5");
                frc::SmartDashboard::PutString("coral intake state", "no target found");
            }
        }
        catch (const std::exception &e)
        {
            frc::DataLogManager::Log("6");
            //fmt::println("ERROR: coral optional exeption");
            std::cerr << e.what() << '\n';
        }
        frc::DataLogManager::Log("5.9"); };

    std::function<bool()> is_finished = [this]() -> bool
    {
        frc::DataLogManager::Log("fegij");
        return m_intake->is_lower_tof_loaded();
    };

    std::function<void(bool IsInterrupted)> end = [this](bool IsInterrupted)
    {
        frc::DataLogManager::Log("end");
    };
    return frc2::FunctionalCommand(init, periodic, end, is_finished, {this}).ToPtr();
}

frc2::CommandPtr Trajectory::auto_score_align()
{
    return frc2::cmd::Run([this]
                          { auto botpose = m_odometry->getPose();
                          frc::Pose2d speakerpose;
                          if (frc::DriverStation::GetAlliance().has_value() && 
                          frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue ){
                            speakerpose = frc::Pose2d(0_m, 5.5_m, frc::Rotation2d(0_rad)); //CHANGEME
                          }
                          else{
                            speakerpose = frc::Pose2d(8_m, 5.5_m, frc::Rotation2d(0_rad)); //CHANGEME
                          }
                          botpose = botpose.RelativeTo(speakerpose);
                          frc::SmartDashboard::PutNumber("as/rel x", botpose.X().value());
                          frc::SmartDashboard::PutNumber("as/rel y", botpose.Y().value());
                          frc::SmartDashboard::PutNumber("as/rel t", botpose.Rotation().Degrees().value());
                          m_drivetrain->face_direction(botpose.Rotation().Degrees()); },
                          {this})
        .Until([this] -> bool
               {
                   auto botpose = m_odometry->getPose();
                   frc::Pose2d speakerpose;
                   if (frc::DriverStation::GetAlliance().has_value() &&
                       frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
                   {
                       speakerpose = frc::Pose2d(0_m, 5.5_m, frc::Rotation2d(0_rad)); // CHANGEME
                   }
                   else
                   {
                       speakerpose = frc::Pose2d(8_m, 5.5_m, frc::Rotation2d(0_rad)); // CHANGEME
                   }
                   botpose = botpose.RelativeTo(speakerpose);
                   return CONSTANTS::IN_THRESHOLD<units::degree_t>(m_drivetrain->getAngle(), botpose.Rotation().Degrees(), 3_deg); });
}
#endif
