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
#include <cmath>

class Candle : public frc2::SubsystemBase
{
public:
    Candle();

    frc2::CommandPtr yellow_blink();
    frc2::CommandPtr red_blink();
    frc2::CommandPtr rainbow();
    frc2::CommandPtr off();
    frc2::CommandPtr ramp_up(double &current_rpm, double &desired_rpm);
    frc2::CommandPtr run_disabled();
    bool auto_selected = false;
    bool has_vision = false;

private:
    ctre::phoenix::led::CANdle m_candle{CONSTANTS::CANDLE::CANDLE_ID};
    ctre::phoenix::led::RainbowAnimation rainbow_anim{0.5, 0.5, -1};
    ctre::phoenix::led::StrobeAnimation red_strobe_anim{255, 0, 0};
    ctre::phoenix::led::StrobeAnimation yellow_strobe_anim{255, 234, 0};

    bool is_red();

    double p_rpm = 0;
    int p_led = 0;
};
