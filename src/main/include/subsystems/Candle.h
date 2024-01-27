// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include "ctre/phoenix/led/CANdle.h"
#include "ctre/phoenix/led/RainbowAnimation.h"

class Candle : public frc2::SubsystemBase {
    public:

    Candle();

    void Purple();
    void Yellow();
    void Red();
    void Blue();
    void Rainbow();
    void Off();

    private:
    ctre::phoenix::led::CANdle m_candle {CONSTANTS::CANDLE::CANDLE_ID, ""};
    ctre::phoenix::led::RainbowAnimation rainbow{0.5, 0.5, -1};
};
