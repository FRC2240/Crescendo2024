// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter(Odometry *odometry, Intake *intake) : m_odometry{odometry}, m_intake{intake}
{
    frc::SmartDashboard::PutNumber("shooter/dangle", 0.0);
    ctre::phoenix6::configs::TalonFXConfiguration left_conf{};
    left_conf.Slot0.kP = 0.01;
    ctre::phoenix6::configs::TalonFXConfiguration right_conf = left_conf;
    right_conf.MotorOutput.Inverted = right_conf.MotorOutput.Inverted.CounterClockwise_Positive;
    // right_conf.Feedback.
    ctre::phoenix6::configs::TalonFXConfiguration angle_conf{};
    angle_conf.Feedback.RotorToSensorRatio = CONSTANTS::SHOOTER::ANGLE_RATIO;
    angle_conf.Slot0.kP = 1.4;
    angle_conf.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    // angle_conf.Feedback.WithRemoteCANcoder(m_cancoder);
    m_left_motor.GetConfigurator().Apply(left_conf);
    m_right_motor.GetConfigurator().Apply(right_conf);
    m_angle_motor.GetConfigurator().Apply(angle_conf);
    ctre::phoenix6::configs::TalonFXConfiguration belt_conf{};
    belt_conf.CurrentLimits.SupplyCurrentLimitEnable = true;
    belt_conf.CurrentLimits.SupplyCurrentLimit = 40;
    belt_conf.Slot0.kP = 0.1;
    belt_conf.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    m_belt_motor.GetConfigurator().Apply(belt_conf);
    frc::SmartDashboard::PutNumber("shooter/P", 0.0);
}

// This method will be called once per scheduler run
void Shooter::Periodic()
{
}

frc2::CommandPtr Shooter::default_cmd()
{
    return frc2::RunCommand(
               [this]
               {
                   //    ctre::phoenix6::controls::VelocityDutyCycle req = units::turns_per_second_t{0};
                   //    if (frc::DriverStation::GetAlliance() && frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
                   //    {
                   //        if (m_odometry->getPose().X() <= 5_m)
                   //        {
                   //         req=units::turns_per_second_t{CONSTANTS::SHOOTER::LEFT_VELOCITY};
                   //        }
                   //    }
                   //    if (frc::DriverStation::GetAlliance() && frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
                   //    {
                   //        if (m_odometry->getPose().X() <= 5_m)
                   //        {
                   //         req=units::turns_per_second_t{CONSTANTS::SHOOTER::LEFT_VELOCITY};
                   //        }
                   //    }
                   if (m_intake->is_intaking)
                   {
                       frc::SmartDashboard::PutBoolean("intaking", 1);
                       m_belt_motor.Set(.5);
                   }
                   else
                   {

                       m_belt_motor.SetControl(ctre::phoenix6::controls::VelocityVoltage(0_tr / 1_s));
                       frc::SmartDashboard::PutBoolean("intaking", 0);
                   }
                   m_angle_motor.Set(0);
                   m_left_motor.Set(0);
                   m_right_motor.Set(0);
               },
               {this})
        .ToPtr();
}

void Shooter::set_angle(units::degree_t angle)
{
    ctre::phoenix6::controls::PositionVoltage req{angle};
    m_angle_motor.SetControl(req);
    frc::SmartDashboard::PutNumber("shooter/angle", angle.value());
}
frc2::CommandPtr Shooter::test_shot()
{

    std::function<void()> init = [this]
    {
        frc::SmartDashboard::PutBoolean("shooter/fire", false);
    };
    std::function<void()> periodic = [this]
    {
        units::turn_t angle = units::turn_t{frc::SmartDashboard::GetNumber("shooter/dangle", 0.0)};
        fmt::println("{}", angle.value());
        set_angle(angle);
        // m_left_motor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle(CONSTANTS::SHOOTER::LEFT_VELOCITY));
        m_left_motor.SetControl(ctre::phoenix6::controls::VoltageOut(units::volt_t{12}));
        m_right_motor.SetControl(ctre::phoenix6::controls::VoltageOut(units::volt_t{12}));
        // m_right_motor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle(-CONSTANTS::SHOOTER::RIGHT_VELOCITY));
    };
    std::function<bool()> is_finished = [this] -> bool
    {
        frc::SmartDashboard::PutNumber("shooter/turns", get_angle().value() / 360);
        frc::SmartDashboard::PutBoolean("shooter/speed_threshold", CONSTANTS::IN_THRESHOLD<units::turns_per_second_t>(m_left_motor.GetVelocity().GetValue(), 80_tps, 5_tps));
        frc::SmartDashboard::PutBoolean("shooter/angle_threshold", CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(get_angle(), units::turn_t{frc::SmartDashboard::GetNumber("shooter/dangle", 0.0)}, 2_tr));

        return CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(get_angle(), units::turn_t{frc::SmartDashboard::GetNumber("shooter/dangle", 0.0)}, 2_tr) &&
               CONSTANTS::IN_THRESHOLD<units::turns_per_second_t>(m_left_motor.GetVelocity().GetValue(), 80_tps, 5_tps) &&
               frc::SmartDashboard::GetBoolean("shooter/fire", false);
        // change threshold?
        //    && CONSTANTS::IN_THRESHOLD<units::angular_velocity::turns_per_second_t>(m_left_motor.GetVelocity().GetValue(), CONSTANTS::SHOOTER::LEFT_VELOCITY, units::angular_velocity::turns_per_second_t{1})    // change threshold?
        //    && CONSTANTS::IN_THRESHOLD<units::angular_velocity::turns_per_second_t>(m_right_motor.GetVelocity().GetValue(), CONSTANTS::SHOOTER::RIGHT_VELOCITY, units::angular_velocity::turns_per_second_t{1}); // change threshold?
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
                                      m_belt_motor.SetControl(ctre::phoenix6::controls::VoltageOut{units::volt_t{12}}); // changeme
                                  },
                                  {this})
                     .ToPtr()
                     .WithTimeout(1.5_s));
}

frc2::CommandPtr Shooter::fender_shot()
{

    std::function<void()> init = [this] {};
    std::function<void()> periodic = [this]
    {
        set_angle(CONSTANTS::SHOOTER::FENDER_ANGLE);
        // m_left_motor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle(CONSTANTS::SHOOTER::LEFT_VELOCITY));
        m_left_motor.SetControl(ctre::phoenix6::controls::VoltageOut(units::volt_t{12}));
        m_right_motor.SetControl(ctre::phoenix6::controls::VoltageOut(units::volt_t{12}));
        // m_right_motor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle(-CONSTANTS::SHOOTER::RIGHT_VELOCITY));
    };
    std::function<bool()> is_finished = [this] -> bool
    {
        frc::SmartDashboard::PutNumber("shooter/turns", get_angle().value() / 360);
        return CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(get_angle(), CONSTANTS::SHOOTER::FENDER_ANGLE, 2_tr) && CONSTANTS::IN_THRESHOLD<units::turns_per_second_t>(m_left_motor.GetVelocity().GetValue(), 80_tps, 5_tps);
        // change threshold?
        //    && CONSTANTS::IN_THRESHOLD<units::angular_velocity::turns_per_second_t>(m_left_motor.GetVelocity().GetValue(), CONSTANTS::SHOOTER::LEFT_VELOCITY, units::angular_velocity::turns_per_second_t{1})    // change threshold?
        //    && CONSTANTS::IN_THRESHOLD<units::angular_velocity::turns_per_second_t>(m_right_motor.GetVelocity().GetValue(), CONSTANTS::SHOOTER::RIGHT_VELOCITY, units::angular_velocity::turns_per_second_t{1}); // change threshold?
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
                                      m_belt_motor.SetControl(ctre::phoenix6::controls::VoltageOut{units::volt_t{12}}); // changeme
                                  },
                                  {this})
                     .ToPtr()
                     .WithTimeout(1.5_s));
}

units::degree_t Shooter::get_angle()
{
    // return m_cancoder.GetAbsolutePosition().GetValue();
    return m_angle_motor.GetPosition().GetValue();
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
                       m_belt_motor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle{3_tr / 1_s});
                   }
               })
        .ToPtr();
}
