// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
#include <iostream>
#include "units/current.h"
#include "units/voltage.h"

Intake::Intake()
{
    // angle motor
    ctre::phoenix6::configs::TalonFXConfiguration angle_config{};
    angle_config.Audio.BeepOnBoot = true;
    angle_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    angle_config.CurrentLimits.SupplyCurrentLimit = 25; // CHANGEME
    angle_config.Slot0.kP = 0.25;
    angle_config.Slot0.kD = 0.01;
    angle_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    m_angleMotor.GetConfigurator().Apply(angle_config);

    // belt motor (pid stuff may be unnecessary)
    ctre::phoenix6::configs::TalonFXConfiguration belt_config{};
    belt_config.Audio.BeepOnBoot = true;
    belt_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    belt_config.CurrentLimits.SupplyCurrentLimit = 40; // CHANGEME
    belt_config.Slot0.kP = 0.5;
    belt_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    belt_config.Slot0.kD = 0.2;
    m_beltMotor.GetConfigurator().Apply(belt_config);
}

// This method will be called once per scheduler run
void Intake::Periodic(){
    // frc::SmartDashboard::PutNumber("tof", m_tof.GetRange());
    /*
    auto result = m_beltMotor.SetControl(ctre::phoenix6::controls::VoltageOut(units::volt_t{12}));

    if (result.IsError())
    {
        frc::SmartDashboard::PutString("result", "ERROR");
    }
    if (result.IsWarning())
    {
        frc::SmartDashboard::PutString("result", "WARN");
    }
    if (result.IsOK())
    {
        frc::SmartDashboard::PutString("result", "GOOD");
    }
    */
};

frc2::CommandPtr Intake::zero()
{
    return frc2::cmd::RunOnce(
        [this]
        {
            m_angleMotor.SetPosition(0_tr);
        },
        {this});
}

bool Intake::is_loaded()
{

    return m_tof.GetRange() < CONSTANTS::INTAKE::LOADED_DIST;
};

bool Intake::is_lower_tof_loaded()
{

    return m_lower_tof.GetRange() < CONSTANTS::INTAKE::LOWER_LOADED_DIST;
};

frc2::CommandPtr Intake::ManualFeedCommand(bool back)
{
    return frc2::cmd::Run([this, back]
                          { if (back) {m_beltMotor.Set(0.3);}
                          else { m_beltMotor.Set(-0.3);} },
                          {this});
}

frc2::CommandPtr Intake::ExtendCommand()
{
    return frc2::RunCommand([this] -> void
                            { 
                                is_intaking=true;
                                m_angleMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{CONSTANTS::INTAKE::DOWN_POSITION}); },
                            {this})
        .Until([this] -> bool // stop when done (to allow StartSpinCommand to run)
               { return CONSTANTS::IN_THRESHOLD<units::turn_t>(m_angleMotor.GetPosition().GetValue(), CONSTANTS::INTAKE::DOWN_POSITION, CONSTANTS::INTAKE::ROTATION_THRESHOLD); })
        .WithName("Extend");
};

frc2::CommandPtr Intake::RetractCommand()
{
    return frc2::RunCommand([this] -> void
                            { m_angleMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{CONSTANTS::INTAKE::UP_POSITION}); },
                            {this})
        .Until([this] -> bool { // stop when done (to allow StopSpinCommand to run)
            return CONSTANTS::IN_THRESHOLD<units::turn_t>(m_angleMotor.GetPosition().GetValue(), CONSTANTS::INTAKE::UP_POSITION, CONSTANTS::INTAKE::ROTATION_THRESHOLD);
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
    return frc2::InstantCommand([this]
                                {m_timer.Reset(); m_timer.Stop(); },
                                {this})
        .ToPtr()
        .AndThen(
            frc2::RunCommand([this]
                             {
                                is_intaking = true;
                                 m_beltMotor.SetControl(ctre::phoenix6::controls::VoltageOut(units::volt_t{CONSTANTS::INTAKE::INTAKE_VOLTAGE})); },
                             {this})
                .Until([this] -> bool
                       { 
                        if (is_loaded()) {
                        m_timer.Start();
                       }
                       return m_timer.Get() >= CONSTANTS::INTAKE::DELAY; }))

        .AndThen(StopSpinCommand())
        .WithName("StartSpin");
};

frc2::CommandPtr Intake::StopSpinCommand()
{
    return frc2::RunCommand([this]
                            {
                                is_intaking = false;
                                m_beltMotor.SetControl(ctre::phoenix6::controls::VoltageOut{units::voltage::volt_t{0}}); // wrong type?
                            },
                            {this})
        .Until([this] -> bool
               { return m_beltMotor.GetVelocity().GetValueAsDouble() < 1; })
        .WithName("StopSpin");
};

frc2::CommandPtr Intake::StartCommand()
{
    return ExtendCommand().AndThen(StartSpinCommand()).WithName("Start");
};

frc2::CommandPtr Intake::StopCommand()
{
    return StopSpinCommand().AndThen(RetractCommand()).WithName("Stop");
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