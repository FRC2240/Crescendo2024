// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber(frc::XboxController *stick)
    : m_stick{stick}
{
   ctre::phoenix6::configs::TalonFXConfiguration hieght_climber_config{};
   hieght_climber_config.Slot0.kP = 1;
   hieght_climber_config.Slot0.kI = 0.1;
   hieght_climber_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
   hieght_climber.GetConfigurator().Apply(hieght_climber_config);
}

void Climber::Periodic()
{
   int yPos = m_stick->GetPOV();
   if (yPos == 0)
   {
      frc::SmartDashboard::PutString("climbers", "up");
      hieght_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0.5});
   }
   else if (yPos == 180)
   {
      frc::SmartDashboard::PutString("climbers", "down");
      hieght_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{-0.5});
   }
   else
   {
      hieght_climber.SetControl(ctre::phoenix6::controls::DutyCycleOut{0});
   }
};
