// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

//Climber::Climber() = default;
 Climber::Climber() {
    ctre::phoenix6::configs::TalonFXConfiguration hieght_climber_config {};
    hieght_climber_config.Slot0.kP = 0.1;
    hieght_climber_config.Slot0.kI = 0.1
    hieght_climber.GetConfigurator().Apply(hieght_climber_config);
 }
   
   
   void Climber:: climb (double pos) 

    if m_stick.Y(Climber_up){

          hieght_climber.SetControl(ctre::phoenix6::controls::PostionDutyCycle{units::angle::turn_t{25}}) 

            
    }
   if m_stick.X(Climber_down){
      
        
         hieght_climber.SetControl(ctre::phoenix6::controls::PostionDutyCycle{units::angle::turn_t{-25}}) 
   

   }
   
    
// This method will be called once per scheduler run
void Climber::Periodic() {}
  

  

    