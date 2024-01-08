// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter()
{
    ctre::phoenix6::configs::TalonFXConfiguration left_conf{};
    left_conf.Slot0.kP = 1;
    ctre::phoenix6::configs::TalonFXConfiguration right_conf = left_conf;
    right_conf.MotorOutput.Inverted = right_conf.MotorOutput.Inverted.CounterClockwise_Positive;

    ctre::phoenix6::configs::TalonFXConfiguration angle_conf{};
    angle_conf.Feedback.RotorToSensorRatio = CONSTANTS::SHOOTER::ANGLE_RATIO;
    angle_conf.Slot0.kP = 1;
    angle_conf.Feedback.WithRemoteCANcoder(m_cancoder);

    m_left_motor.GetConfigurator().Apply(left_conf);
    m_right_motor.GetConfigurator().Apply(right_conf);
    m_angle_motor.GetConfigurator().Apply(angle_conf);
}

// This method will be called once per scheduler run
void Shooter::Periodic() {}

void Shooter::set_angle(units::degree_t angle)
{
    ctre::phoenix6::controls::PositionVoltage req{angle};
    m_angle_motor.SetControl(req);
}

frc2::CommandPtr Shooter::fender_shot()
{
    return frc2::RunCommand(
               [this]
               {
                   set_angle(CONSTANTS::SHOOTER::FENDER_ANGLE);
                   if (m_cancoder.GetAbsolutePosition().GetValueAsDouble() * 0.95_tr >= CONSTANTS::SHOOTER::FENDER_RANGE.first &&
                       m_cancoder.GetAbsolutePosition().GetValueAsDouble() * 1.05_tr >= CONSTANTS::SHOOTER::FENDER_RANGE.second)
                   {
                       m_left_motor.SetControl(ctre::phoenix6::controls::DutyCycleOut(1));
                   }
               },
               {this})
        .ToPtr();
}
