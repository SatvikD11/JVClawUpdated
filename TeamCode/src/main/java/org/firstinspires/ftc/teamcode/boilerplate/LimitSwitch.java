package org.firstinspires.ftc.teamcode.boilerplate;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class LimitSwitch {
    TouchSensor limit;
    public LimitSwitch(TouchSensor limitSwitch){
        limit = limitSwitch;
    }
    public boolean isPressed(){
        return !limit.isPressed();
    }
}
