// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber(frc::XboxController *stick)
    : m_stick{stick}
{
   ctre::phoenix6::configs::TalonFXConfiguration left_climber_config{};
   left_climber_config.Slot0.kP = 1;
   left_climber_config.Slot0.kI = 0.1;
   left_climber_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
   //left_climber_config.SoftwareLimitSwitch.WithForwardSoftLimitEnable(true);
   left_climber_config.SoftwareLimitSwitch.WithForwardSoftLimitThreshold(0);
   left_climber.GetConfigurator().Apply(left_climber_config);
   // ctre::phoenix6::controls::Follower req{4, true};

   ctre::phoenix6::configs::TalonFXConfiguration right_climber_config{};
   right_climber_config.Slot0.kP = 1;
   right_climber_config.Slot0.kI = 0.1;
   right_climber_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
   //right_climber_config.SoftwareLimitSwitch.WithForwardSoftLimitEnable(true);
   right_climber_config.SoftwareLimitSwitch.WithForwardSoftLimitThreshold(0);
   right_climber.GetConfigurator().Apply(right_climber_config);
   /*ctre::phoenix6::controls::Follower req{5, false};
   right_climber.SetControl(req);*/
}

frc2::CommandPtr Climber::UpCommand()
{
    return frc2::RunCommand([this]
                            { right_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{-0.25});
                              left_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{-0.25}); },
                            {this})
        .WithName("Up");
};

frc2::CommandPtr Climber::DownCommand()
{
    return frc2::RunCommand([this]
                            { if (left_limit_switch.Get()) {
                              left_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
                            } else {
                              left_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.25});
                            }
                            if (right_limit_switch.Get()) {
                              right_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.0});
                            } else {
                              right_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.25});
                            }},
                            {this})
        .WithName("Down");
};

frc2::CommandPtr Climber::StopCommand()
{
    return frc2::RunCommand([this]
                            { right_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0});
                              left_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0}); },
                            {this})
        .WithName("Stop");
};
