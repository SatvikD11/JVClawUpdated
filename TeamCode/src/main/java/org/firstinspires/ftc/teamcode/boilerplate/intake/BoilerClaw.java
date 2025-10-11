package org.firstinspires.ftc.teamcode.boilerplate.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//various functions for setting the claw to specific positions
//used in both auto and teleop
public class BoilerClaw {
    public ServoImplEx base, joint, wrist, claw;
    public BoilerServoEncoder jointPos, basePos;
    boolean aIsRunning = false;
    boolean bIsRunning = false;
    public double targetPositionA;
    public double targetPositionB;


    public BoilerClaw(HardwareMap hardwareMap){
        base = hardwareMap.get(ServoImplEx.class, "base");
        joint = hardwareMap.get(ServoImplEx.class, "joint");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        jointPos = new BoilerServoEncoder(hardwareMap, "jointPos");
        basePos = new BoilerServoEncoder(hardwareMap, "basePos");
    }
    void smoothlyMoveServoToTargetA(ServoImplEx servo, double targetPosition, double servoIncrement, double servoDampener) {
        if (aIsRunning) return; // Prevent multiple motions at once
        this.targetPositionA = Range.clip(targetPosition, 0.00, 1.00);
        Thread motionThread = new Thread(() -> {
            aIsRunning = true;
            double currentPosition = servo.getPosition();;
            while (Math.abs(currentPosition - targetPosition) > 0.01) {
                double step = (targetPosition > currentPosition) ? servoIncrement : -servoIncrement;
                if (Math.abs(targetPosition - currentPosition) < 0.2) {
                    step /= servoDampener;
                }

                currentPosition += step;
                currentPosition = Range.clip(currentPosition, 0.0, 1.0);
                servo.setPosition(currentPosition);
            }
            servo.setPosition(targetPosition); // snap to final target
            aIsRunning = false;
        });
        motionThread.start();
    }
    void smoothlyMoveServoToTargetB(ServoImplEx servo, double targetPosition, double servoIncrement, double servoDampener) {
        if (bIsRunning) return; // Prevent multiple motions at once
        this.targetPositionB = Range.clip(targetPosition, 0.00, 1.00);
        Thread motionThread = new Thread(() -> {
            aIsRunning = true;
            double currentPosition = servo.getPosition();;
            while (Math.abs(currentPosition - targetPosition) > 0.01) {
                double step = (targetPosition > currentPosition) ? servoIncrement : -servoIncrement;
                if (Math.abs(targetPosition - currentPosition) < 0.2) {
                    step /= servoDampener;
                }

                currentPosition += step;
                currentPosition = Range.clip(currentPosition, 0.0, 1.0);
                servo.setPosition(currentPosition);
            }
            servo.setPosition(targetPosition); // snap to final target
            bIsRunning = false;
        });
        motionThread.start();
    }


    public void setWristMiddle(){
        wrist.setPosition(0.73);//.73
    }
    public void setWristCurve(){
        wrist.setPosition(0.675);
    }
    public void setINTAKE_SCAN(){
         base.setPosition(0.62);
         joint.setPosition(1);
        setWristMiddle();
    }
    public void noWristScan(){
         base.setPosition(0.62);
         joint.setPosition(1);
        openClaw();
    }
    public void setINTAKE_MIDDLE_Scan(){
        base.setPosition(0.37);
        joint.setPosition(0.9);//.65
    }
    public void setINTAKE_TRANSFER(){
        setWristCurve();
        //base.setPosition((0.75);
        //joint.setPosition(0);
         base.setPosition(0.75);
        //changed to not run into submersible
        joint.setPosition(0.25);//0
    }
    public void setINTAKE_PICKUP(){
        openClaw();
        //joint.setPosition(0.83); changed to not slam the sample
        //base.setPosition((0.25);
         base.setPosition(0.25);
        //changed to not run into submersible
         joint.setPosition(0.9);//.65

    }
    public void setINTAKE_Middle(){
        setWristMiddle();
        //joint.setPosition(0.43);
        //base.setPosition((0.08);
         base.setPosition(0.08);
        //changed to not run into submersible
         joint.setPosition(0.43);

        openClaw();
    }
    public void setINTAKE_WALL(){
        setWristMiddle();
        //joint.setPosition(0.775);
        //base.setPosition((0.7);
         base.setPosition(0.7);
        //changed to not run into submersible
         joint.setPosition( 0.775);

        openClaw();
    }
    public void setINTAKE_WALL_UP(){
        closeClaw();
        sleep(200);
        setWristMiddle();
        //joint.setPosition(0.4);
        //base.setPosition((0.7);
         base.setPosition(0.7);
        //changed to not run into submersible
         joint.setPosition( 0.4);

    }
    public void setSUBMERSIBLEZONERETRACT(){
        setWristMiddle();
        //joint.setPosition(0.65);//
        //base.setPosition((1);//
         base.setPosition(0.99);
        //changed to not run into submersible
         joint.setPosition( 0.65);

    }
    public void noWristERETRACT(){
        closeClaw();
        sleep(200);
        //joint.setPosition(0.65);//
        //base.setPosition((1);//
         base.setPosition(0.99);
        //changed to not run into submersible
         joint.setPosition( 0.65);

    }
    public void setINTAKE_START(){
        setWristMiddle();
        joint.setPosition(0.8);
        base.setPosition(0.7);
         base.setPosition(0.7);
        //changed to not run into submersible
         joint.setPosition( 0.8);

        openClaw();
    }
    public void specimenFloor(){
        openClaw();
        //base.setPosition((0.15);//
        //joint.setPosition(0.67);//
         base.setPosition(0.15);
        //changed to not run into submersible
         joint.setPosition(0.75);

        sleep(200);
    }
    public void specimenFloorUp(){
        openClaw();
        //base.setPosition((0.15);//
        //joint.setPosition(0.67);//
         base.setPosition(0.15);
        //changed to not run into submersible
         joint.setPosition(0.75);

    }
    public void sleep(long mills){
        //localized sleep function to stop current thread only.
        ElapsedTime timer = new ElapsedTime(); //create timer
        timer.reset(); //set timer to zero
        while(timer.milliseconds() < mills){} //prevents moving on to the next line for mills milliseconds
    }
    public void closeClaw(){
        claw.setPosition(0.9);
    }
    public void openClaw(){
        claw.setPosition(0);
    }

    public void pickup(){
        sleep(300);
        setINTAKE_PICKUP();
        sleep(300);
        closeClaw();
        sleep(300);
        setSUBMERSIBLEZONERETRACT();
    }
    public void speciGroundPickup(){
        specimenFloorUp();
        sleep(1200);
        specimenFloor();
        sleep(200);
        closeClaw();
        sleep(200);
    }
}
