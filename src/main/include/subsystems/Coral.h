#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/RunCommand.h>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <span>
#include "swerve/Drivetrain.h"
#include <math.h>
#include <iostream>

class Coral : public frc2::SubsystemBase {

    public:
        Coral(Drivetrain *drivetrain);
        void SimulationPeriodic();
        frc2::CommandPtr TrackCommand();

    private:
        std::shared_ptr<nt::NetworkTable> m_table;
        Drivetrain *m_drivetrain;

};
