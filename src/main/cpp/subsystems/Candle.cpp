// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
Default: Alliance


*/

#include "subsystems/Candle.h"

Candle::Candle(Intake *intake) : m_intake{intake}
{
    ctre::phoenix::led::CANdleConfiguration config;
    config.stripType = ctre::phoenix::led::LEDStripType::RGB; //GRB?
    config.brightnessScalar = 0.5;
    m_candle.ConfigAllSettings(config);
    m_candle.ClearAnimation(0);
    m_candle.SetLEDs(0, 0, 0);
}

bool Candle::is_red()
{
    if (frc::DriverStation::GetAlliance())
    {
        if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
        {
            return false;
        }
    }
    return true;
}

void Candle::Periodic(){}

frc2::CommandPtr Candle::fast_yellow_blink()
{
    return frc2::RunCommand([this]
                            { m_candle.Animate(yellow_strobe_anim); },
                            {this})
        .WithName("Fast Yellow Blink");
};

frc2::CommandPtr Candle::amp_blink()
{
    return frc2::RunCommand([this] { 
        if (is_red()) {
            m_candle.Animate(red_amp_anim);
        }
        else {
            m_candle.Animate(blue_amp_anim);
        }
    },
    {this})
    .WithName("Amp Blink");
};

frc2::CommandPtr Candle::not_driver_controlled()
{
    return frc2::RunCommand([this] { 
        if (is_red()) {
            m_candle.Animate(red_larson);
        }
        else {
            m_candle.Animate(blue_larson);
        }
    },
    {this})
    .WithName("No Control");
};

frc2::CommandPtr Candle::off()
{
    return frc2::RunCommand([this]
                            { m_candle.SetLEDs(0, 0, 0);
                              m_candle.ClearAnimation(0); },
                            {this})
        .WithName("Off");
};

frc2::CommandPtr Candle::default_command()
{
    return frc2::RunCommand([this] {       

        m_candle_timer.Start();
        if (!frc::DriverStation::IsFMSAttached()) {
        //if(false){
            m_candle.ClearAnimation(0);
            if (m_candle_timer.Get() < units::time::second_t(0.5)) {
                m_candle.SetLEDs(0, 0, 0);
            } else if (m_candle_timer.Get() < units::time::second_t(1.0)) {
                m_candle.SetLEDs(255, 0, 0);
            } else {
                m_candle_timer.Reset();
            }
        }

        else if (!has_vision || !auto_selected) {
        //else if(false){
            m_candle.ClearAnimation(0);
            if (m_candle_timer.Get() < units::time::second_t(0.5)) {
                m_candle.SetLEDs(0, 0, 0);
            } else if (m_candle_timer.Get() < units::time::second_t(1.0)) {
                m_candle.SetLEDs(255, 234, 0);
            } else {
                m_candle_timer.Reset();
                m_candle.SetLEDs(0, 0, 0);
            }
        }
        else if (frc::DriverStation::IsAutonomousEnabled()) {
        //else if (true) {
            if (is_red()) {
                m_candle.Animate(red_larson);
            }
            else {
                m_candle.Animate(blue_larson);
            }
        }
        else if (frc::DriverStation::IsTeleopEnabled()) {
            if (m_intake->is_loaded()) {
            //if(false){
                m_candle.Animate(green_blink);
            }
            else {
                m_candle.ClearAnimation(0);
                if (is_red()) {
                    m_candle.SetLEDs(255, 0, 0);
                }
                else {
                    m_candle.SetLEDs(0, 0, 255);
                }
            }
        }
        else {
            m_candle.Animate(rainbow_anim);
            m_candle_timer.Stop();
            m_candle_timer.Reset();
        } },
                            {this})
        .WithName("Default Command").IgnoringDisable(true);
}