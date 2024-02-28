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

void Candle::Periodic() {}

frc2::CommandPtr Candle::fast_yellow_blink()
{
    return frc2::RunCommand([this]
                            { m_candle.Animate(yellow_strobe_anim); },
                            {this})
        .WithName("Fast Yellow Blink");
};

frc2::CommandPtr Candle::amp_blink()
{
    return frc2::RunCommand([this]
                            { 
        if (m_alliance == frc::DriverStation::Alliance::kRed) {
            m_candle.Animate(red_amp_anim);
        }
        else {
            m_candle.Animate(blue_amp_anim);
        } },
                            {this})
        .WithName("Amp Blink");
};

frc2::CommandPtr Candle::not_driver_controlled()
{
    return frc2::RunCommand([this]
                            { 
        if (m_alliance == frc::DriverStation::Alliance::kRed) {
            m_candle.Animate(red_no_control_anim);
        }
        else {
            m_candle.Animate(blue_no_control_anim);
        } },
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

frc2::CommandPtr Candle::get_command()
{
    if (frc::DriverStation::IsEStopped())
    {
        return frc2::RunCommand(
                   [this]
                   {
                       m_candle.SetLEDs(255, 255, 255);
                   },
                   {this})
            .ToPtr();
    }
    if (frc::DriverStation::IsAutonomousEnabled())
    {
        if (is_red())
        {
            return frc2::RunCommand(
                       [this]
                       {
                           m_candle.Animate(red_no_control_anim);
                       })
                .ToPtr();
        }
        else
        {

            return frc2::RunCommand(
                       [this]
                       {
                           m_candle.Animate(blue_no_control_anim);
                       })
                .ToPtr();
        }
    }
    if (frc::DriverStation::IsTeleopEnabled())
    {
        if (!is_red())
        {
            return frc2::RunCommand(
                       [this]
                       {
                           m_candle.SetLEDs(0, 0, 255);
                       },
                       {this})
                .ToPtr();
        }
        else
        {
            return frc2::RunCommand(
                       [this]
                       {
                           m_candle.SetLEDs(255, 0, 0);
                       },
                       {this})
                .ToPtr()
        }
    }
    else
    {
        return run_disabled();
    }
}
frc2::CommandPtr Candle::run_disabled()
{
    return frc2::RunCommand([this]
                            {
        frc::SmartDashboard::PutBoolean("checklist/vision", has_vision);
        frc::SmartDashboard::PutBoolean("checklist/autos", auto_selected);
        frc::SmartDashboard::PutBoolean("checklist/js0", frc::DriverStation::IsJoystickConnected(0));
        frc::SmartDashboard::PutBoolean("checklist/js1", frc::DriverStation::IsJoystickConnected(1));
        m_candle_timer.Start();
        // if (!frc::DriverStation::IsFMSAttached()) {
        //     if (m_candle_timer.Get() < units::time::second_t(0.5)) {
        //         m_candle.SetLEDs(0, 0, 0);
        //     } else if (m_candle_timer.Get() < units::time::second_t(1.0)) {
        //         m_candle.SetLEDs(255, 0, 0);
        //     } else {
        //         m_candle_timer.Reset();
        //     }
        // }

        if (!has_vision || !auto_selected || !frc::DriverStation::IsJoystickConnected(0) || !frc::DriverStation::IsJoystickConnected(1)){
            if (m_candle_timer.Get() < units::time::second_t(0.5))
            {
                m_candle.SetLEDs(0, 0, 0);
            }
            else if (m_candle_timer.Get() < units::time::second_t(1.0))
            {
                m_candle.SetLEDs(255, 234, 0);
            }
            else
            {
                m_candle_timer.Reset();
                m_candle.SetLEDs(0, 0, 0);
            }
                            }
                            else if (frc::DriverStation::IsFMSAttached() has_vision && auto_selected && frc::DriverStation::IsJoystickConnected(0) && frc::DriverStation::IsJoystickConnected(1)){
        m_candle.Animate(rainbow_anim);
        m_candle_timer.Stop();
        m_candle_timer.Reset();
                            } },
                            {this})
        .WithName("Run Disabled")
        .IgnoringDisable(true);
}