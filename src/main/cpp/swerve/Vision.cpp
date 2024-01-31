#include "swerve/Vision.h"
#include "swerve/Drivetrain.h"

Vision::Vision(std::function<units::degree_t()> get_angle_fn)
    : get_angle{get_angle_fn} {

      };

<<<<<<< HEAD
std::vector<std::optional<frc::Pose2d>> Vision::get_bot_position() {
  auto ll_results = m_fore_limelight->GetNumberArray("botpose_wpiblue",
                                                     std::vector<double>(6));
||||||| parent of b287fad (pv draft complete/added pvlib)
std::vector<std::optional<frc::Pose2d>> Vision::get_bot_position()
{
    // NOTE: THIS IS TO BE REWRITTEN TO A VECTOR OF ALL CAMERAS, NOT JUST 1
    auto ll_results = m_fore_limelight->GetNumberArray("botpose", std::vector<double>(6));
=======
std::vector<std::optional<frc::Pose2d>> Vision::get_bot_position()
{
    auto aft_results = m_aft_limelight->GetNumberArray("botpose_wpiblue", std::vector<double>(6));
>>>>>>> b287fad (pv draft complete/added pvlib)

  std::vector<std::optional<frc::Pose2d>> ret;

<<<<<<< HEAD
  for (auto &i : m_photoncam_vec) {
    auto result = i->GetLatestResult();
    if (result.MultiTagResult().result.isPresent) {
      auto pose = m_estimator.Update(result);
      if (pose) {
        if (CONSTANTS::IN_THRESHOLD<units::degree_t>(
                pose.value().estimatedPose.Rotation().ToRotation2d().Degrees(),
                Drivetrain::getAngle(), 3_deg)) {
||||||| parent of b287fad (pv draft complete/added pvlib)
    for (auto &i : m_photoncam_vec)
    {
        auto result = i->GetLatestResult();
        if (result.MultiTagResult().result.isPresent)
        {
            auto pose = m_estimator.Update(result);
            if (pose)
            {
                if (CONSTANTS::IN_THRESHOLD<units::degree_t>(pose.value().estimatedPose.Rotation().ToRotation2d().Degrees(), Drivetrain::getAngle(), 3_deg))
                {
                }
            }
=======
    for (auto &i : m_photoncam_vec)
    {
        // First is the camera, second is the estimator
        auto result = i.first->GetLatestResult();
        if (result.MultiTagResult().result.isPresent)
        {
            auto pose = i.second.Update(result);
            if (pose)
            {
                if (CONSTANTS::IN_THRESHOLD<units::degree_t>(pose.value().estimatedPose.Rotation().ToRotation2d().Degrees(), get_angle(), 3_deg))
                {
                    ret.push_back(pose.value().estimatedPose.ToPose2d());
                }
            }
>>>>>>> b287fad (pv draft complete/added pvlib)
        }
      }
    }
  }

<<<<<<< HEAD
  ret.push_back(frc::Pose2d(units::meter_t{ll_results[0]},
                            units::meter_t{ll_results[1]},
                            frc::Rotation2d(units::degree_t{ll_results[5]})));

  return ret;
||||||| parent of b287fad (pv draft complete/added pvlib)
    // m_estimator.Update(m_left_camera_a->GetLatestResult().)

    ret.push_back(frc::Pose2d(
        units::meter_t{ll_results[0]},
        units::meter_t{ll_results[1]},
        frc::Rotation2d(units::degree_t{ll_results[5]})));

    return ret;
=======
    ret.push_back(frc::Pose2d(
        units::meter_t{aft_results[0]},
        units::meter_t{aft_results[1]},
        frc::Rotation2d(units::degree_t{aft_results[5]})));

    return ret;
>>>>>>> b287fad (pv draft complete/added pvlib)
}

std::optional<units::degree_t> Vision::get_coral_angle() {
  if (m_fore_limelight->GetString("tclass", "ERROR") ==
      "note") // Assuming "note" is the correct key
  {
    units::degree_t tx{m_fore_limelight->GetNumber("tx", 0.0)};
    // Target is valid, return info

    return std::optional<units::degree_t>{tx};
  } else {
    return std::nullopt;
  }
}
std::optional<units::degree_t> Vision::get_apriltag_angle() {
  if (frc::DriverStation::GetAlliance()) {
    if (frc::DriverStation::GetAlliance().value() ==
        frc::DriverStation::Alliance::kRed) {
      // Casted to int to strip off decimals and get rid of errors with equality
      if ((int)m_aft_limelight->GetNumber("tid", 0.0) == 4) {
        return units::degree_t{m_aft_limelight->GetNumber("tx", 0.0)};
      } else {
        fmt::println("DEBUG: apriltag 4 not found");
        return std::nullopt;
      }
    } else if (frc::DriverStation::GetAlliance().value() ==
               frc::DriverStation::Alliance::kBlue) {
      // Casted to int to strip off decimals and get rid of errors with equality
      if ((int)m_aft_limelight->GetNumber("tid", 0.0) == 7) {
        return units::degree_t{m_aft_limelight->GetNumber("tx", 0.0)};
      } else {
        fmt::println("DEBUG: apriltag 7 not found");
        return std::nullopt;
      }
    }
  } else {
    fmt::println("WARN: Alliance color not set");
    return std::nullopt;
  }
}

std::optional<units::degree_t> Vision::get_shooter_angle() {}
Vision::~Vision() = default;
