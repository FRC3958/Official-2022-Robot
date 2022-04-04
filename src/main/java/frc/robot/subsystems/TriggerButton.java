package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class TriggerButton extends Button{
    XboxController m_xc;
    int m_TriggerID;  //trigger ID of 2 for the left trigger, 3 for the right trigger
    public TriggerButton(XboxController controller, int TriggerID) {
        m_xc = controller;
        m_TriggerID = TriggerID;
    }

    @Override
    public boolean get() {
        if(m_TriggerID == 2) {
            return m_xc.getLeftTriggerAxis() > 0.5;
            
        } else if (m_TriggerID == 3){
            return m_xc.getRightTriggerAxis() > .5;
        } else {
            return false;
        }
    }
}
