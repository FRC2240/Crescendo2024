// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter(Odometry *odometry, Intake *intake) : m_odometry{odometry}, m_intake{intake}
{
    frc::SmartDashboard::PutNumber("amp/dangle", 0.0);
    frc::SmartDashboard::PutNumber("amp/desired velocity", 0.0);

    frc::SmartDashboard::PutNumber("shooter/dangle", 0.0);
    ctre::phoenix6::configs::TalonFXConfiguration left_conf{};
    left_conf.Slot0.kP = 0.8;
    left_conf.Slot0.kP = 0;
    left_conf.Slot0.kV = 0.225;

    ctre::phoenix6::configs::TalonFXConfiguration right_conf{};
    right_conf.MotorOutput.Inverted = right_conf.MotorOutput.Inverted.Clockwise_Positive;
    right_conf.Slot0.kP = 0.8;
    right_conf.Slot0.kV = 0.225;
    ctre::phoenix6::configs::TalonFXConfiguration angle_conf{};
    angle_conf.Feedback.RotorToSensorRatio = CONSTANTS::SHOOTER::ANGLE_RATIO;
    angle_conf.Slot0.kP = 1.4;
    angle_conf.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
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
    if (m_intake->intake_state == Intake::IntakeState::INTAKING)
    {
        frc::DataLogManager::Log("here b");
        frc::SmartDashboard::PutBoolean("intaking", 1);
        m_belt_motor.Set(.22);
        set_angle(0_tr);
    }
    else if (m_intake->is_lower_tof_loaded())
    {
        if (!m_intake->is_loaded())
        {
            frc::DataLogManager::Log("here a");
            m_belt_motor.Set(0.12);
        }
        else
        {
            m_belt_motor.Set(0);
        }
    }
    else
    {
        if (frc::DriverStation::IsAutonomous())
        {
            m_left_motor.Set(0);
            m_right_motor.Set(0);
        }

        m_belt_motor.SetControl(ctre::phoenix6::controls::VelocityVoltage(0_tr / 1_s));
        frc::SmartDashboard::PutBoolean("intaking", 0);
    }
}

frc2::CommandPtr Shooter::spool_cmd()
{
    return frc2::cmd::Run(
        [this]
        {
            set_angle(m_odometry->get_shooter_angle());
            m_left_motor.Set(-0.8);
            m_right_motor.Set(-0.4);
        },
        {this});
}

frc2::CommandPtr Shooter::stop()
{
    return frc2::cmd::RunOnce(
        [this]
        {
            set_angle(CONSTANTS::SHOOTER::FENDER_ANGLE);
            m_left_motor.Set(1);
            m_right_motor.Set(1);
        },
        {this});
}

frc2::CommandPtr Shooter::default_cmd()
{
    return frc2::RunCommand(
               [this]
               {
                   //    ctre::phoenix6::controls::VoltageOut req = 0_V;
                   //    if (frc::DriverStation::GetAlliance() && frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
                   //    {
                   //        if (m_odometry->getPose().X() <=119 5_m)
                   //        {
                   //            req = 12_V;
                   //            set_angle(m_odometry->get_shooter_angle());
                   //        }
                   //    }
                   //    if (frc::DriverStation::GetAlliance() && frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
                   //    {
                   //        if (m_odometry->getPose().X() <= 5_m)
                   //        {
                   //            req = 12_V;
                   //            set_angle(m_odometry->get_shooter_angle());
                   //        }
                   //    }
                   //    if (m_intake->is_intaking)
                   //    {
                   //        frc::SmartDashboard::PutBoolean("intaking", 1);
                   //        m_belt_motor.Set(.25);
                   //        set_angle(0_tr);
                   //    }
                   //    else
                   //    {

                   //        m_belt_motor.SetControl(ctre::phoenix6::controls::VelocityVoltage(0_tr / 1_s));
                   //        frc::SmartDashboard::PutBoolean("intaking", 0);
                   //    }
                   //    //    m_left_motor.SetControl(req);
                   //    m_right_motor.SetControl(req);
                   set_angle(CONSTANTS::SHOOTER::REST_ANGLE);
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

void Shooter::set_angle(units::turn_t angle)
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
        // frc::SmartDashboard::PutNumber("shooter/desired velocity", 0.0);
    };
    std::function<void()> periodic = [this]
    {
        units::turn_t angle = units::turn_t{frc::SmartDashboard::GetNumber("shooter/dangle", 0.0)};
        // fmt::println("{}", angle.value());
        set_angle(angle);
        units::turns_per_second_t vel{frc::SmartDashboard::GetNumber("shooter/desired velocity", 0.0)};
        // m_left_motor.SetControl(ctre::phoenix6::controls::VelocityVoltage(vel));
        // m_right_motor.SetControl(ctre::phoenix6::controls::VelocityVoltage(vel * 0.8));
        m_left_motor.Set(-0.8);
        m_right_motor.Set(-0.4); };
    std::function<bool()> is_finished = [this] -> bool
    {
        frc::DataLogManager::Log("here");
        frc::SmartDashboard::PutNumber("shooter/turns", get_angle().value() / 360);
        frc::SmartDashboard::PutNumber("shooter/speed", m_left_motor.GetVelocity().GetValue().value());
        frc::SmartDashboard::PutBoolean("shooter/speed_threshold", m_left_motor.GetVelocity().GetValue() > 75_tps);
        frc::SmartDashboard::PutBoolean("shooter/angle_threshold", CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(get_angle(), units::turn_t{frc::SmartDashboard::GetNumber("shooter/dangle", 0.0)}, 0.25_tr));

        return CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(get_angle(), units::turn_t{frc::SmartDashboard::GetNumber("shooter/dangle", 0.0)}, 0.25_tr) &&
               m_left_motor.GetVelocity().GetValue() < -60_tps;
    };
    std::function<void(bool IsInterrupted)> end = [this](bool IsInterrupted) {};

    return frc2::FunctionalCommand(init, periodic, end, is_finished, {this}).ToPtr().AndThen(frc2::RunCommand([this]
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
        m_right_motor.SetControl(ctre::phoenix6::controls::VoltageOut(units::volt_t{6}));
        // m_right_motor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle(-CONSTANTS::SHOOTER::RIGHT_VELOCITY));
    };
    std::function<bool()> is_finished = [this] -> bool
    {
        if (frc::DriverStation::IsAutonomous())
        {
            frc::SmartDashboard::PutNumber("shooter/turns", get_angle().value() / 360);
            frc::SmartDashboard::PutNumber("shooter/velocity", m_left_motor.GetVelocity().GetValueAsDouble());
            return CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(get_angle(), CONSTANTS::SHOOTER::FENDER_ANGLE, 2_tr) &&
                   m_left_motor.GetVelocity().GetValue() > 80_tps;
            //    CONSTANTS::IN_THRESHOLD<units::turns_per_second_t>(m_left_motor.GetVelocity().GetValue(), CONSTANTS::SHOOTER::SHOOTER_VELOCITY, 5_tps);
        }
        else
        {
            frc::SmartDashboard::PutNumber("shooter/turns", get_angle().value() / 360);
            frc::SmartDashboard::PutNumber("shooter/velocity", m_left_motor.GetVelocity().GetValueAsDouble());
            return CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(get_angle(), CONSTANTS::SHOOTER::FENDER_ANGLE, 2_tr) &&
                   CONSTANTS::IN_THRESHOLD<units::turns_per_second_t>(m_left_motor.GetVelocity().GetValue(), CONSTANTS::SHOOTER::SHOOTER_VELOCITY, 5_tps);
        }
    };
    std::function<void(bool IsInterrupted)> end = [this](bool IsInterrupted) {};

    return frc2::FunctionalCommand(
               init,
               periodic,
               end,
               is_finished,
               {this})
        .ToPtr()
        .WithTimeout(0.5_s)
        .AndThen(frc2::RunCommand([this]
                                  {
                                      m_belt_motor.SetControl(ctre::phoenix6::controls::VoltageOut{units::volt_t{12}}); // changeme
                                  },
                                  {this})
                     .ToPtr()
                     .WithTimeout(0.75_s))
        .AndThen(frc2::cmd::RunOnce([this]
                                    {
                                    m_left_motor.Set(0); 
                                    m_right_motor.Set(0); 
                                    m_angle_motor.SetControl(ctre::phoenix6::controls::PositionVoltage{0_tr}); }));
}

frc2::CommandPtr Shooter::ManualFeedCommand(bool back)
{
    return frc2::RunCommand([this, back]
                            {
                                if (back)
                                {
                                m_belt_motor.SetControl(ctre::phoenix6::controls::VoltageOut{units::volt_t{-3}}); // changeme
                                m_left_motor.Set(0.5);
                                m_right_motor.Set(0.5);
                             }
                             else {
                                 m_belt_motor.SetControl(ctre::phoenix6::controls::VoltageOut{units::volt_t{3}}); // changeme
                                m_left_motor.Set(-0.5);
                                m_right_motor.Set(-0.5);
                             } },
                            {this})
        .ToPtr();
}

units::degree_t Shooter::get_angle()
{
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

frc2::CommandPtr Shooter::set_angle_cmd(units::turn_t angle)
{
    std::function<void()> init = [this] {};
    std::function<void()> periodic = [this, &angle]()
    {
        set_angle(angle);
    };

    std::function<bool()> is_finished = [this, &angle]() -> bool
    {
        auto ret = CONSTANTS::IN_THRESHOLD<units::degree_t>(get_angle(), angle, 1_deg);
        frc::SmartDashboard::PutBoolean("shooter/ret", ret);
        m_left_motor.Set(-0.8);
        m_right_motor.Set(-0.4);
        return ret;
    };
    std::function<void(bool IsInterrupted)> end = [this](bool IsInterrupted) {};

    return frc2::cmd::Run([this, angle]
                          { frc::SmartDashboard::PutNumber("shooter/ yet another stupid angle", angle.value());
                            set_angle(angle); },
                          {this})
        .WithTimeout(0.5_s)
        .AndThen(execute_auto_shot());
}

frc2::CommandPtr Shooter::amp_shot()
{

    std::function<void()> init = [this] {};
    std::function<void()> periodic = [this]
    {
        units::turn_t angle = units::turn_t{frc::SmartDashboard::GetNumber("amp/dangle", 0.0)};
        // units::turn_t angle = units::turn_t{11};
        // units::turn_t angle = units::turn_t{9};
        // units::volt_t vout = units::volt_t{-4};
        ctre::phoenix6::controls::VelocityVoltage req = units::turns_per_second_t{frc::SmartDashboard::GetNumber("amp/desired velocity", 0.0)};
        // ctre::phoenix6::controls::VelocityVoltage req = units::turns_per_second_t{-6.5};
        set_angle(angle);
        // set_angle(CONSTANTS::SHOOTER::AMP_ANGLE);
        // m_left_motor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle(CONSTANTS::SHOOTER::LEFT_VELOCITY));
        m_left_motor.SetControl(req);
        m_right_motor.SetControl(req);
        // m_right_motor.SetControl(ctre::phoenix6::controls::VelocityDutyCycle(-CONSTANTS::SHOOTER::RIGHT_VELOCITY));
    };
    std::function<bool()> is_finished = [this] -> bool
    {
        frc::SmartDashboard::PutNumber("shooter/turns", get_angle().value() / 360);
        frc::SmartDashboard::PutNumber("shooter/lvelocity", m_left_motor.GetVelocity().GetValueAsDouble());
        frc::SmartDashboard::PutNumber("shooter/rvelocity", m_right_motor.GetVelocity().GetValueAsDouble());
        return CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(get_angle(), CONSTANTS::SHOOTER::AMP_ANGLE, 0.25_tr) &&
               CONSTANTS::IN_THRESHOLD<units::turns_per_second_t>(m_left_motor.GetVelocity().GetValue(), units::turns_per_second_t{frc::SmartDashboard::GetNumber("amp/desired velocity", 0.0)}, 0.25_tps) &&
               CONSTANTS::IN_THRESHOLD<units::turns_per_second_t>(m_right_motor.GetVelocity().GetValue(), units::turns_per_second_t{frc::SmartDashboard::GetNumber("amp/desired velocity", 0.0)}, 0.25_tps);
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
        .WithTimeout(0.5_s)
        .AndThen(frc2::RunCommand([this]
                                  {
                                      m_belt_motor.Set(0.3); // changeme
                                  },
                                  {this})
                     .ToPtr()
                     .WithTimeout(1.5_s));
}

frc2::CommandPtr Shooter::zero()
{
    return frc2::cmd::RunOnce([this]
                              { m_angle_motor.SetPosition(0_tr); },
                              {this});
}

frc2::CommandPtr Shooter::intake_cmd()
{
    return frc2::cmd::Run([this]
                          { m_belt_motor.Set(.25); },
                          {this});
}

void Shooter::intake()
{
    m_belt_motor.Set(.25);
}

frc2::CommandPtr Shooter::execute_auto_shot()
{
    std::function<void()> init = [this]
    {
        frc::SmartDashboard::PutBoolean("shooter/fire", false);
        // frc::SmartDashboard::PutNumber("shooter/desired velocity", 0.0);
    };
    std::function<void()> periodic = [this]
    {
        units::turn_t angle = m_odometry->get_shooter_angle();
        set_angle(angle);
        frc::SmartDashboard::PutNumber("ANGLE THINGY", angle.value());
        m_left_motor.Set(-0.8);
        m_right_motor.Set(-0.4); };
    std::function<bool()> is_finished = [this] -> bool
    {
        return CONSTANTS::IN_THRESHOLD<units::angle::degree_t>(get_angle(), m_odometry->get_shooter_angle(), 0.25_tr) &&
               m_left_motor.GetVelocity().GetValue() < -60_tps;
    };
    std::function<void(bool IsInterrupted)> end = [this](bool IsInterrupted) {};

    return frc2::FunctionalCommand(init, periodic, end, is_finished, {this}).ToPtr().AndThen(frc2::RunCommand([this]
                                                                                                              {
                                                                                                                  m_belt_motor.SetControl(ctre::phoenix6::controls::VoltageOut{units::volt_t{12}}); // changeme
                                                                                                              },
                                                                                                              {this})
                                                                                                 .ToPtr()
                                                                                                 .WithTimeout(1.5_s));
}