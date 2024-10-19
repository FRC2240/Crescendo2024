#include "subsystems/Coral.h"

Coral::Coral(Drivetrain *drivetrain) : m_drivetrain{drivetrain} {
    m_table = nt::NetworkTableInstance::GetDefault().GetTable(CONSTANTS::CORAL::LIMELIGHT_ID);
};

frc2::CommandPtr Coral::TrackCommand() {
    return frc2::RunCommand([this] -> void {
        
        float raw_ta = m_table->GetNumber("ta", 0.0);
        
        if (raw_ta != 0.0) { //TODO: find more robust way of detecting no note

            //get angle to note from limelight (relative to robot facing angle)
            units::degree_t rel_angle = units::degree_t{raw_ta};

            //compute absolute angle of note
            //maybe try get_absolute_angle()?
            units::degree_t abs_angle = m_drivetrain->getAngle() + rel_angle;
        
            //get x and y components of forward vector + multiply by coefficient
            units::meters_per_second_t dx = cos(abs_angle.value()) * CONSTANTS::CORAL::APPROACH_SPEED;
            units::meters_per_second_t dy = sin(abs_angle.value()) * CONSTANTS::CORAL::APPROACH_SPEED;

            //move towards the note
            m_drivetrain->faceDirection(dx, dy, abs_angle, true);
        }

    },{this})
    .WithName("Track");
};