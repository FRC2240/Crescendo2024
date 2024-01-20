#include "swerve/Vision.h"

Vision::Vision() = default;

std::vector<std::optional<frc::Pose2d>> Vision::get_bot_position()
{
    // NOTE: THIS IS TO BE REWRITTEN TO A VECTOR OF ALL CAMERAS, NOT JUST 1
    auto ll_results = m_limelight->GetNumberArray("botpose_wpiblue", std::vector<double>(6));
    std::vector<std::optional<frc::Pose2d>> ret;
    frc::SmartDashboard::PutNumber("X", ll_results[0]);
    frc::SmartDashboard::PutNumber("Y", ll_results[1]);
    if (!(ll_results[0]==(int)0)){
    ret.push_back(frc::Pose2d(
        units::meter_t{ll_results[0]},
        units::meter_t{ll_results[1]},
        frc::Rotation2d(units::degree_t{ll_results[5]})));
    }
    else {
                ret.push_back(std::nullopt);
    }
    return ret;

}

std::optional<units::degree_t> Vision::get_coral_angle()
{
    if (m_limelight->GetString("tclass", "ERROR") == "note") // Assuming "note" is the correct key
    {
        units::degree_t tx{m_limelight->GetNumber("tx", 0.0)};
        // Target is valid, return info

        return std::optional<units::degree_t>{tx};
    }
    else
    {
        return std::nullopt;
    }
}
std::optional<units::degree_t> Vision::get_apriltag_angle()
{
    if (frc::DriverStation::GetAlliance())
    {
        if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kRed)
        {
            // Casted to int to strip off decimals and get rid of errors with equality
            if ((int)m_limelight->GetNumber("tid", 0.0) == 4)
            {
                return units::degree_t{m_limelight->GetNumber("tx", 0.0)};
            }
            else
            {
                fmt::println("DEBUG: apriltag 4 not found");
                return std::nullopt;
            }
        }
        else if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
        {
            // Casted to int to strip off decimals and get rid of errors with equality
            if ((int)m_limelight->GetNumber("tid", 0.0) == 7)
            {
                return units::degree_t{m_limelight->GetNumber("tx", 0.0)};
            }
            else
            {
                fmt::println("DEBUG: apriltag 7 not found");
                return std::nullopt;
            }
        }
    }
    else
    {
        fmt::println("WARN: Alliance color not set");
        return std::nullopt;
    }
}

std::optional<units::degree_t> Vision::get_shooter_angle()
{
}
Vision::~Vision() = default;