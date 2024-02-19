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
   // ctre::phoenix6::controls::Follower req{4, true};

   ctre::phoenix6::configs::TalonFXConfiguration right_climber_config{};
   right_climber_config.Slot0.kP = 1;
   right_climber_config.Slot0.kI = 0.1;
   right_climber_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
   // right_climber_config.CurrentLimits.
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
   if (m_stick->GetPOV() == 0)
   {
      frc::SmartDashboard::PutString("climbers", "up");
      right_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{1});
      left_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{1});
   }
   else if (m_stick->GetPOV() == 180)
   {
      frc::SmartDashboard::PutString("climbers", "down");
      right_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{-1});
      left_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{-1});
   }
   else
   {
      right_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0});
      left_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0});
   }
};
