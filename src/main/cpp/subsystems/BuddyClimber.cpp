// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/BuddyClimber.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

BuddyClimber::BuddyClimber() {
    ctre::phoenix6::configs::TalonFXConfiguration right_config{};
    right_config.Audio.BeepOnBoot = true;
    right_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    right_config.CurrentLimits.SupplyCurrentLimit = 25; //change
    right_config.Slot0.kP = 1.0;
    right_config.Slot0.kD = 0.0;
    ctre::phoenix6::configs::TalonFXConfiguration left_config = right_config;
    left_config.MotorOutput.Inverted = left_config.MotorOutput.Inverted.CounterClockwise_Positive;

    m_rightMotor.GetConfigurator().Apply(right_config);
    m_leftMotor.GetConfigurator().Apply(left_config);
};


frc2::CommandPtr BuddyClimber::DeployCommand() {
    return frc2::RunCommand([this] {
        m_deployServo.Set(DEPLOY_ANGLE);
    }, {this}).WithName("Deploy");
};


frc2::CommandPtr BuddyClimber::StartRightCommand() {
    return frc2::RunCommand([this] {
            m_rightMotor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle{ROTOR_SPEED});
    }, {this}).WithName("Start Right");
};


frc2::CommandPtr BuddyClimber::StartLeftCommand() {
    return frc2::RunCommand([this] {
            m_leftMotor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle{ROTOR_SPEED});
    }, {this}).WithName("Start Left");
};