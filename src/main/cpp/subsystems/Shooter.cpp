// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter(Intake *intake)
 : m_intake{intake}
{

    ctre::phoenix6::configs::TalonFXConfiguration left_conf{};
    left_conf.Slot0.kP = 0.3;
    ctre::phoenix6::configs::TalonFXConfiguration right_conf = left_conf;
    right_conf.MotorOutput.Inverted = right_conf.MotorOutput.Inverted.CounterClockwise_Positive;
    // right_conf.Feedback.
    ctre::phoenix6::configs::TalonFXConfiguration angle_conf{};
    angle_conf.Feedback.RotorToSensorRatio = CONSTANTS::SHOOTER::ANGLE_RATIO;
    angle_conf.Slot0.kP = 0.3;
    angle_conf.Feedback.WithRemoteCANcoder(m_cancoder);
    m_left_motor.GetConfigurator().Apply(left_conf);
    m_right_motor.GetConfigurator().Apply(right_conf);
    m_angle_motor.GetConfigurator().Apply(angle_conf);
    m_angle_motor2.GetConfigurator().Apply(angle_conf);
    m_angle_motor2.SetControl(ctre::phoenix6::controls::Follower(m_angle_motor.GetDeviceID(), 1));
}

// This method will be called once per scheduler run
void Shooter::Periodic() {}

void Shooter::set_angle(units::degree_t angle)
{
    ctre::phoenix6::controls::PositionVoltage req{angle};
    m_angle_motor.SetControl(req);
    fmt::println("SetAngle");
}

frc2::CommandPtr Shooter::ResetEncodersCommand() {
    return RunOnce([this] {
        m_angle_motor.SetPosition(0_tr);
        m_angle_motor2.SetPosition(0_tr);
    });
}

frc2::CommandPtr Shooter::SetBrakeCommand(bool enabled) {
    return RunOnce([this, enabled] {
        ctre::phoenix6::configs::TalonFXConfiguration brake_config{};
        if(enabled) {
            brake_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        } else {
            brake_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        }
        m_left_motor.GetConfigurator().Apply(brake_config);
        m_right_motor.GetConfigurator().Apply(brake_config);
        m_angle_motor.GetConfigurator().Apply(brake_config);
        m_angle_motor2.GetConfigurator().Apply(brake_config);
    })
    .WithName("Brake");
}

frc2::CommandPtr Shooter::fender_shot()
{

    std::function<void()> init = [this] {};
    std::function<void()> periodic = [this]
    {
        set_angle(CONSTANTS::SHOOTER::FENDER_ANGLE);
        m_left_motor.SetControl(ctre::phoenix6::controls::VelocityVoltage(CONSTANTS::SHOOTER::LEFT_VELOCITY));
        m_right_motor.SetControl(ctre::phoenix6::controls::VelocityVoltage(-CONSTANTS::SHOOTER::RIGHT_VELOCITY));
    };
    std::function<bool()> is_finished = [this] -> bool
    {
        return CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(get_angle(), CONSTANTS::SHOOTER::FENDER_ANGLE, 1_deg)                                                                                                // change threshold?
               && CONSTANTS::IN_THRESHOLD<units::angular_velocity::turns_per_second_t>(m_left_motor.GetVelocity().GetValue(), CONSTANTS::SHOOTER::LEFT_VELOCITY, units::angular_velocity::turns_per_second_t{1})    // change threshold?
               && CONSTANTS::IN_THRESHOLD<units::angular_velocity::turns_per_second_t>(m_right_motor.GetVelocity().GetValue(), CONSTANTS::SHOOTER::RIGHT_VELOCITY, units::angular_velocity::turns_per_second_t{1}); // change threshold?
    };
    std::function<void(bool IsInterrupted)> end = [this](bool IsInterrupted) {};

    return frc2::FunctionalCommand(
               init,
               periodic,
               end,
               is_finished,
               {this})
        .ToPtr()
        .AndThen(frc2::RunCommand([this]
                                  {
                                      m_intake->m_beltMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{units::angular_velocity::turns_per_second_t{100}}); // changeme
                                  },
                                  {this})
                     .ToPtr())
        .WithTimeout(0.5_s);
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
