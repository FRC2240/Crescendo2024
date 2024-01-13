// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

//Climber::Climber() = default;
   Climber::Climber() {
      ctre::phoenix6::configs::TalonFXConfiguration hieght_climber_config {};
      hieght_climber_config.Slot0.kP = 0.1;
      hieght_climber_config.Slot0.kI = 0.1;
      hieght_climber.GetConfigurator().Apply(hieght_climber_config);
      
      m_stick = &m_stick_init;
   }

void Climber::Periodic() {
   double yPos = m_stick->GetRightY();
   if (abs(yPos) > 0.05) {
      climb(yPos);
   }
};
   
   void Climber::climb (double pos) {

      if (pos > 0) {

         hieght_climber.SetControl(ctre::phoenix6::controls::PositionDutyCycle{units::angle::turn_t{25}});            
      
      } else if (pos < 0) {
         
         hieght_climber.SetControl(ctre::phoenix6::controls::PositionDutyCycle{units::angle::turn_t{25}}); 
      
      }
   }
  

  

    