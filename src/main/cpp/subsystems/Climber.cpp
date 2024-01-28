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

   ctre::phoenix6::configs::TalonFXConfiguration right_climber_config{};
   right_climber_config.Slot0.kP = 1;
   right_climber_config.Slot0.kI = 0.1;
   right_climber_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
   right_climber.GetConfigurator().Apply(right_climber_config);
   ctre::phoenix6::controls::Follower req{41, true};
   right_climber.SetControl(req);
}

void Climber::Periodic()
{
   int yPos = m_stick->GetPOV();
   if (yPos == 0)
   {
      frc::SmartDashboard::PutString("climbers", "up");
      right_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.5});
   }
   else if (yPos == 180)
   {
      frc::SmartDashboard::PutString("climbers", "down");
      right_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{-0.5});
   }
   else
   {
      right_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0});
   }
};
