// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include "ctre/phoenix/led/CANdle.h"
#include "ctre/phoenix/led/RainbowAnimation.h"
#include "ctre/phoenix/led/StrobeAnimation.h"
#include "ctre/phoenix/led/LarsonAnimation.h"
#include <frc/Timer.h>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/CommandXboxController.h>

class Candle : public frc2::SubsystemBase
{
public:
    Candle();
    void Periodic() override;
    frc2::CommandPtr get_command(frc2::CommandXboxController *m_stick);
    frc2::CommandPtr fast_yellow_blink();
    frc2::CommandPtr amp_blink();
    frc2::CommandPtr not_driver_controlled();
    frc2::CommandPtr off();
    frc2::CommandPtr run_disabled();
    bool auto_selected = false;
    bool has_vision = false;

private:
    ctre::phoenix::led::CANdle m_candle{CONSTANTS::CANDLE::CANDLE_ID};
    ctre::phoenix::led::RainbowAnimation rainbow_anim{0.5, 0.5, -1};
    ctre::phoenix::led::StrobeAnimation yellow_strobe_anim{255, 234, 0, 0, 1};
    ctre::phoenix::led::StrobeAnimation red_amp_anim{255, 0, 0, 0, 1};
    ctre::phoenix::led::StrobeAnimation blue_amp_anim{0, 0, 255, 0, 1};
    ctre::phoenix::led::LarsonAnimation red_no_control_anim{255, 0, 0};
    ctre::phoenix::led::LarsonAnimation blue_no_control_anim{0, 0, 255};

    frc::DriverStation::Alliance m_alliance;

    bool is_red();

    frc::Timer m_candle_timer;
};
