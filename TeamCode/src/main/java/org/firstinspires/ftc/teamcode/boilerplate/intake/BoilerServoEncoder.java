package org.firstinspires.ftc.teamcode.boilerplate.intake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BoilerServoEncoder {
    double tolerance = 0.05;
    AnalogInput encoder;
    public BoilerServoEncoder(HardwareMap hardwareMap, String deviceName){
        encoder = hardwareMap.get(AnalogInput.class, deviceName);
    }
    public double getPosition(){
        return encoder.getVoltage() / 3.3;
    } //converts servo position to the 0 to 1 scale used in programing. servo position from the encoder is based on voltage on scale 0 to 3.3.
    public boolean checkPosition(double position){
        //is the calculated position vs. the actual position within the tolerance given
        double currentPos = getPosition();
        return Math.abs(currentPos - position) <= tolerance;
    }
}
