// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter(Intake *intake) : m_intake{intake}
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
                   if (m_cancoder.GetAbsolutePosition().GetValue() + CONSTANTS::SHOOTER::FENDER_TOLERANCE < CONSTANTS::SHOOTER::FENDER_ANGLE &&
                       m_cancoder.GetAbsolutePosition().GetValue() - CONSTANTS::SHOOTER::FENDER_TOLERANCE > CONSTANTS::SHOOTER::FENDER_ANGLE)
                   {
                       m_left_motor.SetControl(ctre::phoenix6::controls::DutyCycleOut(1));
                   }
               },
               {this})
        .ToPtr();
}

units::degree_t Shooter::get_angle()
{
    return m_cancoder.GetAbsolutePosition().GetValue();
}

frc2::CommandPtr Shooter::set_angle_cmd(std::optional<units::degree_t> angle)
{
    std::function<void()> init = [this] {};
    std::function<void()> periodic = [this, &angle]()
    {
        try
        {
            if (angle)
            {
                set_angle(angle.value());
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    };

    std::function<bool()> is_finished = [this, &angle]() -> bool
    {
        try
        {
            if (angle)
            {
                return CONSTANTS::IN_THRESHOLD<units::turn_t>(get_angle(), angle.value(), 1_deg);
            }
            else
            {
                return false;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        return false;
    };
    std::function<void(bool IsInterrupted)> end = [this](bool IsInterrupted) {};

    return frc2::FunctionalCommand(
               init,
               periodic,
               end,
               is_finished,
               {this})
        .ToPtr();
}

frc2::CommandPtr Shooter::set_angle_cmd(units::degree_t angle)
{
    std::function<void()> init = [this] {};
    std::function<void()> periodic = [this, &angle]()
    {
        set_angle(angle);
    };

    std::function<bool()> is_finished = [this, &angle]() -> bool
    {
        return CONSTANTS::IN_THRESHOLD<units::degree_t>(get_angle(), angle, 1_deg);
    };
    std::function<void(bool IsInterrupted)> end = [this](bool IsInterrupted) {};

    return frc2::FunctionalCommand(
               init,
               periodic,
               end,
               is_finished,
               {this})
        .ToPtr();
}

frc2::CommandPtr Shooter::execute_auto_shot()
{
    return frc2::RunCommand([this]
                            { m_left_motor.SetControl(ctre::phoenix6::controls::DutyCycleOut(1)); })
        .ToPtr();
}

frc2::CommandPtr Shooter::amp_shot()
{
    return frc2::RunCommand(
               [this]
               {
                   set_angle(CONSTANTS::SHOOTER::AMP_ANGLE);
                   if (CONSTANTS::IN_THRESHOLD<units::degree_t>(get_angle(), CONSTANTS::SHOOTER::AMP_ANGLE, 1_deg))
                   {
                       m_intake->m_beltMotor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle{3_tr / 1_s});
                   }
               })
        .ToPtr();
}
