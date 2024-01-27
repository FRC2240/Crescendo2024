#pragma once
#include "Constants.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc/DriverStation.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <stdlib.h>
#include <units/angle.h>
class Vision {
public:
  /*
  At the time of writing, Vision must be capable of:
  1. Translation accuracy within 6in
  1.a. Doing so no slower than 0.9s
  2. determining the angle of a note to within 3 degrees
  3. Determing the distance to a note within 29 inches
  4. Determing the angle to an apriltag within 2 degrees
  */

  Vision(std::function<units::degree_t()> get_angle_fn);
  ~Vision();

  // Optionals are used liberally in this file due to the uncertain nature of
  // vison. See
  // https://stackoverflow.com/questions/16860960/how-should-one-use-stdoptional
  // for more info

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
  std::function<units::degree_t()> get_angle;
  std::shared_ptr<photon::PhotonCamera> m_left_camera_a =
      std::make_shared<photon::PhotonCamera>("photon-left-a");
  std::shared_ptr<photon::PhotonCamera> m_left_camera_b =
      std::make_shared<photon::PhotonCamera>("photon-left-n");
  std::shared_ptr<photon::PhotonCamera> m_right_camera_a =
      std::make_shared<photon::PhotonCamera>("photon-right-a");
  std::shared_ptr<photon::PhotonCamera> m_right_camera_b =
      std::make_shared<photon::PhotonCamera>("photon-right-b");

  std::vector<std::shared_ptr<photon::PhotonCamera>> m_photoncam_vec = {
      m_left_camera_a, m_left_camera_b, m_right_camera_a, m_right_camera_b};

  frc::AprilTagFieldLayout layout =
      frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

  photon::PhotonPoseEstimator m_estimator{
      layout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
      frc::Transform3d(
          frc::Pose3d(0_m, 0_m, 0_m, frc::Rotation3d(0_rad, 0_rad, 0_rad)),
          frc::Pose3d(1_m, 1_m, 1_m, frc::Rotation3d(1_rad, 1_rad, 1_rad)))};

  std::shared_ptr<nt::NetworkTable> m_aft_limelight =
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-aft");

  std::shared_ptr<nt::NetworkTable> m_fore_limelight =
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-fore");
};
