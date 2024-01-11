// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"


Intake::Intake() {
    configs::TalonFXConfiguration angle_config{};
    angle_config.Audio.BeepOnBoot = true;
    angle_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    angle_config.CurrentLimits.SupplyCurrentLimit = 25 //change
    //pid constants
    m_angleMotor.GetConfigurator().Apply(angle_config);
    
    configs::TalonFXConfiguration flywheel_config{};
    flywheel_config.Audio.BeepOnBoot = true;
    flywheel_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheel_config.CurrentLimits.SupplyCurrentLimit = 25 //change
    m_flywheelMotor.GetConfigurator().Apply(flywheel_config);
    
    configs::TalonFXConfiguration belt_config{};
    belt_config.Audio.BeepOnBoot = true;
    belt_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    belt_config.CurrentLimits.SupplyCurrentLimit = 25 //change
    m_beltMotor.GetConfigurator().Apply(belt_config);

}

// This method will be called once per scheduler run
void Intake::Periodic() {}

frc2::CommandPtr Intake::ExtendCommand() {
    return RunOnce([this] {
        m_angleMotor.SetControl(controls::PositionDutyCycle{kEndRotations});
    }).WithName("Extend");
}

frc2::CommandPtr Intake::RetractCommand() {
    return RunOnce([this] {
        m_angleMotor.SetControl(controls::PositionDutyCycle{kStartRotations});
    }).WithName("Retract");
}

frc2::CommandPtr Intake::EnableCommand() {
    return RunOnce([this] {
        m_flywheelMotor.SetControl(controls::VelocityDutyCycle{kFlywheelSpeed});
        m_beltMotor.SetControl(controls::VelocityDutyCycle{kBeltSpeed});
    }).WithName("Enable");
}

frc2::CommandPtr Intake::DisableCommand() {
    return RunOnce([this] {
        m_flywheelMotor.SetControl(controls::VelocityDutyCycle{0});
        m_beltMotor.SetControl(controls::VelocityDutyCycle{0}); //wrong type?
    }).WithName("Disable");
}