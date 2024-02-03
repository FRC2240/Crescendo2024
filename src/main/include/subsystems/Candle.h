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

class Candle : public frc2::SubsystemBase {
    public:

    Candle();

    frc2::CommandPtr Purple(); 
    frc2::CommandPtr Yellow(); 
    frc2::CommandPtr Red(); 
    frc2::CommandPtr Blue(); 
    frc2::CommandPtr Rainbow(); 
    frc2::CommandPtr Off(); 

    private:
    ctre::phoenix::led::CANdle m_candle {CONSTANTS::CANDLE::CANDLE_ID}; 
    ctre::phoenix::led::RainbowAnimation rainbow{0.5, 0.5, -1};
};
