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

void Candle::Purple() {
    m_candle.SetLEDs(82, 28, 200);
}

frc2::CommandPtr Candle::Purple() 
{
    return frc2::RunCommand()
}

frc2::CommandPtr Intake::BraceCommand()
{
    return frc2::RunCommand([this]
                            { m_angleMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{BRACE_ROTATIONS}); },
                            {this})
        .WithName("Brace");
};

void Candle::Yellow() {
    m_candle.SetLEDs(254, 162, 1);
}

void Candle::Red() {
    m_candle.SetLEDs(255, 0, 0);
}

void Candle::Blue() {
    m_candle.SetLEDs(0, 0, 255);
}

void Candle::Rainbow() {
    m_candle.Animate(rainbow);
}

void Candle::Off() {
    m_candle.SetLEDs(0,0,0);
}