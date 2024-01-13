// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/BuddyClimber.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

BuddyClimber::BuddyClimber() {
    ctre::phoenix6::configs::TalonFXConfiguration claw_config{};
    claw_config.Audio.BeepOnBoot = true;
    claw_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    claw_config.CurrentLimits.SupplyCurrentLimit = 25; //change
    claw_config.Slot0.kP = 0.1;
    claw_config.Slot0.kD = 0.0;
    m_clawMotor.GetConfigurator().Apply(claw_config);
};


frc2::CommandPtr BuddyClimber::ExtendCommand() {
    return frc2::RunCommand([this] {
            m_clawMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{END_ROTATIONS});
    }, {this}).WithName("Extend");
};

frc2::CommandPtr BuddyClimber::RetractCommand() {
    return frc2::RunCommand([this] {
        m_clawMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{START_ROTATIONS});
    }, {this}).WithName("Retract");
};