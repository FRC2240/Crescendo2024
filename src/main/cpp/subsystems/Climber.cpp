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
   left_climber.GetConfigurator().Apply(left_climber_config);
   //ctre::phoenix6::controls::Follower req{4, true};


   ctre::phoenix6::configs::TalonFXConfiguration right_climber_config{};
   right_climber_config.Slot0.kP = 1;
   right_climber_config.Slot0.kI = 0.1;
   right_climber_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
   right_climber.GetConfigurator().Apply(right_climber_config);
   /*ctre::phoenix6::controls::Follower req{5, false};
   right_climber.SetControl(req);*/
}

void Climber::Periodic()
{
   /*ctre::phoenix6::controls::Follower req{5, 1};
   right_climber.SetControl(req);*/
   double rightTrigger = m_stick->GetRightTriggerAxis();
   double leftTrigger = m_stick->GetLeftTriggerAxis();
   if (rightTrigger > 0.2)
   {
      frc::SmartDashboard::PutString("climbers", "up");
      right_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.5});
      left_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.5});
   }
   else if (leftTrigger > 0.2)
   {
      frc::SmartDashboard::PutString("climbers", "down");
      right_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{-0.5});
      left_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{-0.5});
   }
   else
   {
      right_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0});
      left_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0});
   }
  
   frc2::CommandPtr Climber::BrakeCommand() {
    return RunOnce([this] {
        ctre::phoenix6::configs::TalonFXConfiguration brake_config{};
        brake_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        
        right_climber.GetConfigurator().Apply(brake_config);
        left_climber.GetConfigurator().Apply(brake_config);
    })
    .WithName("Brake");
   }

   frc2::CommandPtr Climber::CoastCommand() {
      return RunOnce([this] {
         ctre::phoenix6::configs::TalonFXConfiguration coast_config{};
         coast_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        
         right_climber.GetConfigurator().Apply(coast_config);
         left_climber.GetConfigurator().Apply(coast_config);
      })
      .WithName("Brake");
   }
};
