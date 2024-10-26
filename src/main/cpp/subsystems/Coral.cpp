#include "subsystems/Coral.h"
#include <frc/smartdashboard/SmartDashboard.h>

Coral::Coral(Drivetrain *drivetrain, Odometry *odometry, Trajectory *trajectory) : m_drivetrain{drivetrain}, m_odometry{odometry}, m_trajectory{trajectory} {
    m_table = nt::NetworkTableInstance::GetDefault().GetTable(CONSTANTS::CORAL::LIMELIGHT_ID);
};

void Coral::SimulationPeriodic() {
    m_table->PutString("tclass", "note");
    m_table->PutNumber("ta", 0.0);
};

frc2::CommandPtr Coral::TrackCommand() {
    return frc2::RunCommand([this] -> void {
        
        float raw_ta = m_table->GetNumber("ta", 0.0);
        frc::SmartDashboard::PutNumber("raw_ta", raw_ta);

        if (m_table->GetString("tclass", "") == "note") {

            //get angle to note from limelight (relative to robot facing angle)
            units::degree_t rel_angle = units::degree_t{raw_ta};

            //compute absolute angle of note
            //maybe try get_absolute_angle()?
            units::degree_t abs_angle = m_drivetrain->get_absolute_angle() + rel_angle;
        
            //get x and y components of forward vector + multiply by coefficient
            // M_PI/180 is to convert to radians
            units::meters_per_second_t dx = cos((M_PI/180)* abs_angle.value()) * CONSTANTS::CORAL::APPROACH_SPEED;
            units::meters_per_second_t dy = sin((M_PI/180)* abs_angle.value()) * CONSTANTS::CORAL::APPROACH_SPEED;
            frc::SmartDashboard::PutNumber("coral/dy", dy.value());
            frc::SmartDashboard::PutNumber("coral/dx", dx.value());
            // frc::SmartDashboard::PutData("coral/pose2", frc::Field2d frc::Pose2d(
            //     (units::meter_t{this->m_odometry->getPose().X()+units::meter_t{dx.value()}}),
            //  (units::meter_t{this->m_odometry->getPose().Y()+units::meter_t{dy.value()}}),
            //   this->m_odometry->getPose().Rotation() ) )

            //move towards the note
            m_drivetrain->faceDirection(-dx, -dy, -abs_angle, true);
        } else {
            this->m_drivetrain->stop();
        }

    },{this, m_trajectory})
    .WithName("Track");
};