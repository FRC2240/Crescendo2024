// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
#include <iostream>

Intake::Intake()
{
    ctre::phoenix6::configs::TalonFXConfiguration angle_config{};
    angle_config.Audio.BeepOnBoot = true;
    angle_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    angle_config.CurrentLimits.SupplyCurrentLimit = 25; // change
    angle_config.Slot0.kP = 0.5;
    angle_config.Slot0.kD = 0.0;
    m_angleMotor.GetConfigurator().Apply(angle_config);

    ctre::phoenix6::configs::TalonFXConfiguration belt_config{};
    belt_config.Audio.BeepOnBoot = true;
    belt_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    belt_config.CurrentLimits.SupplyCurrentLimit = 25; // change
    belt_config.Slot0.kP = 0.5;
    belt_config.Slot0.kD = 0.0;
    m_beltMotor.GetConfigurator().Apply(belt_config);
}

// This method will be called once per scheduler run
void Intake::Periodic(){};

#pragma warn("Intake::is_loaded() is UNIMPLEMENTED!")

bool Intake::is_loaded()
{
    return false;
};

frc2::CommandPtr Intake::ExtendCommand()
{
    return frc2::RunCommand([this] -> void
        {
            m_angleMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{END_ROTATIONS});
        }, {this})
        .Until([this] -> bool
            { return CONSTANTS::IN_THRESHOLD<units::turn_t>(m_angleMotor.GetPosition().GetValue(), END_ROTATIONS, ROTATION_THRESHOLD); })
        .WithName("Extend");
};

frc2::CommandPtr Intake::RetractCommand()
{
    return frc2::RunCommand([this] -> void {
            m_angleMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{START_ROTATIONS});
        }, {this})
        .Until([this] -> bool {
            return CONSTANTS::IN_THRESHOLD<units::turn_t>(m_angleMotor.GetPosition().GetValue(), START_ROTATIONS, ROTATION_THRESHOLD);
        })
        .WithName("Retract");
};


frc2::CommandPtr Intake::BraceCommand()
{
    return frc2::RunCommand([this]
                            { m_angleMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{BRACE_ROTATIONS}); },
                            {this})
        .WithName("Brace");
};

frc2::CommandPtr Intake::StartSpinCommand()
{
    return frc2::RunCommand([this]
                            {
                                m_beltMotor.SetControl(ctre::phoenix6::controls::VoltageOut(BELT_SPEED));
                            },
                            {this})
        .WithName("StartSpin");
};

frc2::CommandPtr Intake::StopSpinCommand()
{
    return frc2::RunCommand([this]
                            {
                                m_beltMotor.SetControl(ctre::phoenix6::controls::VoltageOut{units::voltage::volt_t{0}}); // wrong type?
                            },
                            {this})
        .WithName("StopSpin");
};

frc2::CommandPtr Intake::StartCommand()
{
    return frc2::PrintCommand("Start Intake").ToPtr().AndThen(ExtendCommand().AndThen(StartSpinCommand())).WithName("Start");
};

frc2::CommandPtr Intake::StopCommand()
{
    return frc2::PrintCommand("Stop Intake").ToPtr().AndThen(RetractCommand().AndThen(StopSpinCommand())).WithName("Stop");
};

/*
New position [DONE]
Belt combined
Motor in intake
Shooter gets pointer to class in constructor
How does Shooter access Belt?

*/

/*



*/