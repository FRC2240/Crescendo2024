#include "subsystems/Coral.h"

Coral::Coral(Drivetrain *drivetrain) : m_drivetrain{drivetrain} {
    m_table = nt::NetworkTableInstance::GetDefault().GetTable(CONSTANTS::CORAL::LIMELIGHT_ID);
};

frc2::CommandPtr Coral::TrackCommand() {
    return frc2::RunCommand([this] -> void {
        
        //get angle to note from limelight (relative to robot facing angle)
        units::degree_t rel_x = units::degree_t{m_table->GetNumber("tx", 0.0)};
        units::degree_t rel_z = units::degree_t{m_table->GetNumber("ty", 0.0)};

        //compute absolute angle of note
        //maybe try get_absolute_angle()?
        units::degree_t abs_x = m_drivetrain->getAngle() + rel_x;
        
        //get x and y components of forward vector + multiply by coefficient
        units::meters_per_second_t dx = units::meters_per_second_t{cos(abs_x.value())} * CONSTANTS::CORAL::APPROACH_SPEED;
        units::meters_per_second_t dy = units::meters_per_second_t{sin(abs_x.value())} * CONSTANTS::CORAL::APPROACH_SPEED;

        //move towards the note
        m_drivetrain->faceDirection(dx, dy, abs_x, true);

    },{this})
    .WithName("Track");
};