#ifndef BUTTONS_H_
#define BUTTONS_H_

#include <frc/XboxController.h>
#include <frc/MathUtil.h>
#include "Constants.h"

namespace BUTTON {
  
  inline frc::XboxController m_stick{0};

  inline frc::XboxController m_one_stick{1};

     inline bool ALLIANCE_BLINK() {return BUTTON::m_one_stick.GetAButton();}
     inline bool IDLE () {return BUTTON::m_one_stick.GetXButton();}
     inline bool FAKE_CHARGE_UP() {return BUTTON::m_one_stick.GetBButton();}
     inline bool LIGHTS () {return BUTTON::m_one_stick.GetYButton();}


}




namespace climber {

    inline climber_up(){return BUTTON::m_stick.GetRightTriggerAxis() > 0.05}
    inline climber_down(){return BUTTON::m_stick.GetLeftTriggerAxis() > -0.05}


    if (BUTTON::m_stick.GetRightTriggerAxis() > 0.05){

        return true;

    }
    else if (BUTTON::m_stick.GetLeftTriggerAxis() > -0.05){

       return true;

    }

}


namespace Buddy_Climber{
     inline bool Buddy_Climber_one(){return BUTTON::m_stick.GetRightBumper();}
     inline bool Buddy_Climber_two(){return BUTTON::m_stick.GetLeftBumper();}
     inline bool Buddy_Climber_rotate(){return BUTTON::m_stick}
}





namespace Intake{
    inline bool Intake_out(){return BUTTON::m_stick.GetPOV() == 0;}
    inline bool Intake_in(){return BUTTON::m_stick.GetPOV() == 180;}

   
}

namespace Shooter{
    inline bool Shooter_SPEAKER_ANGLE(){return BUTTON::m_stick.GetPOV() == 0;}
    inline bool Shooter_DOWN(){return BUTTON::m_stick.GetPOV() == 180;}
}

