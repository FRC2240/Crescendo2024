// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Candle.h"

Candle::Candle()
{
    ctre::phoenix::led::CANdleConfiguration config;
    config.stripType = ctre::phoenix::led::LEDStripType::RGB;
    config.brightnessScalar = 0.5;
    m_candle.ConfigAllSettings(config);

    m_candle.SetLEDs(0, 0, 0);

    if (m_alliance == frc::DriverStation::Alliance::kRed)
    {
        is_red == true;
    } 
}

frc2::CommandPtr Candle::yellow_blink()
{
    return frc2::RunCommand([this]
                            { m_candle.Animate(yellow_strobe_anim); },
                            {this})
        .WithName("Yellow Blink");
};

frc2::CommandPtr Candle::red_blink()
{
    return frc2::RunCommand([this]
                            { m_candle.Animate(red_strobe_anim); },
                            {this})
        .WithName("Red Blink");
};

frc2::CommandPtr Candle::rainbow()
{
    return frc2::RunCommand([this]
                            { m_candle.Animate(rainbow_anim); },
                            {this})
        .WithName("Rainbow");
};

frc2::CommandPtr Candle::off()
{
    return frc2::RunCommand([this]
                            { m_candle.SetLEDs(0, 0, 0); },
                            {this})
        .WithName("Off");
};

frc2::CommandPtr Candle::ramp_up(double current_rpm, double desired_rpm)
{
    return frc2::RunCommand([this, current_rpm, desired_rpm]{ 
        p_rpm = current_rpm/desired_rpm;
        p_led = round(30*p_rpm);
        if (is_red){
            m_candle.SetLEDs(255, 0, 0, 0, 0, p_led);
        }
        else {
            m_candle.SetLEDs(0, 0, 255, 0, 0, p_led);
        }
    },
    {this})
    .WithName("Ramp Up");
};

frc2::CommandPtr Candle::run_disabled(bool fms, bool vision, bool auto_selected)
{
    return frc2::RunCommand([this, fms, vision, auto_selected]{  
        if (!fms) {
            Candle::red_blink();
        }
        else if (!vision || !auto_selected) {
            Candle::yellow_blink();
        }
        else if (fms && vision && auto_selected) {
            Candle::rainbow();
        }
    },
    {this})
    .WithName("Run Disabled");
}