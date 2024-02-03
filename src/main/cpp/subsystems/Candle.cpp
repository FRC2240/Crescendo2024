// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Candle.h"

Candle::Candle(){
    ctre::phoenix::led::CANdleConfiguration config;
    config.stripType = ctre::phoenix::led::LEDStripType::RGB; 
    config.brightnessScalar = 0.5; 
    m_candle.ConfigAllSettings(config);

    m_candle.SetLEDs(0, 0, 0);

}

frc2::CommandPtr Candle::Purple()
{
    return frc2::RunCommand([this]
                           {m_candle.SetLEDs(82, 28, 200); },
                          {this})
            .WithName("Purple").ToPtr();
};

frc2::CommandPtr Candle::Yellow()
{
    return frc2::RunCommand([this]
                           {m_candle.SetLEDs(254, 162, 1); },
                          {this})
            .WithName("Yellow").ToPtr();
};

frc2::CommandPtr Candle::Red()
{
    return frc2::RunCommand([this]
                           {m_candle.SetLEDs(255, 0, 0); },
                          {this})
            .WithName("Red").ToPtr();
};

frc2::CommandPtr Candle::Blue()
{
    return frc2::RunCommand([this]
                           {m_candle.SetLEDs(0, 0, 255); },
                          {this})
            .WithName("Blue").ToPtr();
};

frc2::CommandPtr Candle::Rainbow()
{
    return frc2::RunCommand([this]
                           {m_candle.Animate(Rainbow); },
                          {this})
            .WithName("Rainbow").ToPtr();
};

frc2::CommandPtr Candle::Off()
{
    return frc2::RunCommand([this]
                           {m_candle.SetLEDs(0, 0, 0); },
                          {this})
            .WithName("Off").ToPtr();
};