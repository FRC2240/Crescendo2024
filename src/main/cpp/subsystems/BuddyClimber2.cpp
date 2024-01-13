// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/BuddyClimber2.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

BuddyClimber2::BuddyClimber2() {
    ctre::phoenix6::configs::TalonFXConfiguration climb_config{};
    climb_config.Audio.BeepOnBoot = true;
    climb_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    climb_config.CurrentLimits.SupplyCurrentLimit = 25; //change
    climb_config.Slot0.kP = 0.1;
    climb_config.Slot0.kD = 0.0;
    m_climbMotor.GetConfigurator().Apply(climb_config);

    ctre::phoenix6::configs::TalonFXConfiguration grab_config{};
    grab_config.Audio.BeepOnBoot = true;
    grab_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    grab_config.CurrentLimits.SupplyCurrentLimit = 25; //change
    grab_config.Slot0.kP = 0.1;
    grab_config.Slot0.kD = 0.0;
    m_grabMotor.GetConfigurator().Apply(grab_config);
};


frc2::CommandPtr BuddyClimber2::ExtendCommand() {
    return frc2::RunCommand([this] {
            m_climbMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{END_ROTATIONS});
            m_grabMotor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle{GRAB_VELOCITY});
    }, {this}).WithName("Extend");
};

frc2::CommandPtr BuddyClimber2::RetractCommand() {
    return frc2::RunCommand([this] {
        m_climbMotor.SetControl(ctre::phoenix6::controls::PositionDutyCycle{START_ROTATIONS});
        m_grabMotor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle{units::angular_velocity::turns_per_second_t{0}});
    }, {this}).WithName("Retract");
};