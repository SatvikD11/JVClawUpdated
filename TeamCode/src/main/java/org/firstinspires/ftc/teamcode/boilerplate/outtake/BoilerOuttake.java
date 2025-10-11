package org.firstinspires.ftc.teamcode.boilerplate.outtake;
/*
This file includes various functions for the outtake subassemblies.
It includes functions to:
    * reset the slides to set positions, reset the encoders
    * move to specific points while considering limits, objects held, and speed
    * adjust amp requirements for running the robot AND slides
    * check slide positions
*/



import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.boilerplate.LimitSwitch;
import org.firstinspires.ftc.teamcode.boilerplate.BoilerChassis;

import java.util.concurrent.atomic.AtomicBoolean;

public class BoilerOuttake {
    public DcMotorEx oSlideRight, oSlideLeft;
    public TouchSensor left, right;
    public LimitSwitch oLSLeft, oLSRight;
    public boolean hasObject = false;
    public boolean outtakeDocked = true;
    public double ampThreshold = 7000;
    public boolean busy = false;
    public AtomicBoolean extending = new AtomicBoolean(false);
    public ElapsedTime extentTimer = new ElapsedTime();
    ElapsedTime homeTimer = new ElapsedTime();
    BoilerOutArmAndBox delivery;
    BoilerChassis chassis;
    RevBlinkinLedDriver LED;
    public BoilerOuttake(HardwareMap hardwareMap, BoilerOutArmAndBox delivery, RevBlinkinLedDriver LED){
        oSlideRight = hardwareMap.get(DcMotorEx.class, "slidesRight" );
        oSlideLeft = hardwareMap.get(DcMotorEx.class, "slidesLeft");
        oSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        oSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        oSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        oSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        left = hardwareMap.get(TouchSensor.class, "outLeftLS");
        right = hardwareMap.get(TouchSensor.class, "outRightLS");
        oLSLeft = new LimitSwitch(left);
        oLSRight = new LimitSwitch(right);
        this.delivery = delivery;
        this.LED = LED;
    }
    public boolean slidesLimiterOuttake(double power, int upperBound, boolean override) {//function to safely move slides (not past limits), used in while loop
        double adjPower = 0;
        boolean obstructed = false;
        ampThresholdAdjuster();
        if(override){
            adjPower = power;
        }else if((oSlideRight.getCurrentPosition() > upperBound && power > 0)){ //if the position is past max, stop
            adjPower = 0;
        }else if(outtakeDocked() && power < 0){//if position is at min, stop
            adjPower = 0;
        }else if((oSlideRight.getCurrent(CurrentUnit.MILLIAMPS) > ampThreshold)){ //if slides are hitting something, stop
            adjPower = 0;
            obstructed = true;
        }else{ //continue if everything is working
            adjPower = power;
        }
        if(adjPower != 0){ //if power isn't 0 then the slides are not docked
            outtakeDocked = false;
        }
        oSlideLeft.setPower(-adjPower);
        oSlideRight.setPower(adjPower);
        return obstructed;
    }
    public boolean outtakeDocked(){ 
        if((oLSRight.isPressed() && oLSLeft.isPressed()) || oSlideRight.getCurrentPosition() <= 5){
            //if both limit switches are pressed or encoders and at the minimum, then the slides are docked
            outtakeDocked = true;
            return true;
        }else{
            return false;
        }
    }
    public boolean homeOuttake(){ //function used to reset the outtake. This would be in a while loop that would run the function until it returned true
        if(!busy){
            homeTimer.reset();
            busy = true;
            slidesLimiterOuttake(-1, 1000, false); //set the slides to go down
        }
        if(outtakeDocked){ //stops if docked
            busy = false;
            slidesLimiterOuttake(0, 1000, false);
            resetOuttake();
        }else if (homeTimer.seconds() > 1.3) { //stops if it takes to long
            busy = false;
            outtakeDocked = true;
            slidesLimiterOuttake(0, 1000, false);
            resetOuttake();

        }else{ //keep going if not docked
            slidesLimiterOuttake(-1, 1000, false);
            return false;
        }
        return true;
    }
    public boolean setSlides(int target){ //used in Boiler Basket Side to move slides to a specific point IF you have an object in the basket
        if(delivery.hasObject()){
            hasObject = true;
        }
        if(!hasObject){
            //STOP if there is no object in the box
            extending.set(false);
            slidesLimiterOuttake(0, target, false);
            return true;
        }
        if(!extending.get()){
            //update current status
            extentTimer.reset();
            extending.set(true);
        }
        if(oSlideRight.getCurrentPosition() <= target && extentTimer.seconds() < 2){
            //If not at the target, continue to move the slides
            slidesLimiterOuttake(1, target, false);
            return false;
        }
        slidesLimiterOuttake(0, target, false);
        //If at the target, stop the movement, update the status, and end the movement
        extending.set(false);
        return true;

    }
    public boolean setSlides(int target, boolean override){
        //used to move the slides to a specific point
        //doesn't check for object in the box

        if(!extending.get()){
            //checks to see if not already moving
            extentTimer.reset();
            extending.set(true);
        }
        if(oSlideRight.getCurrentPosition() <= target && extentTimer.seconds() < 5){
            slidesLimiterOuttake(2, target+50, false);
            return false;
        }
        slidesLimiterOuttake(0, target+50, false);
        extending.set(false);
        return true;

    }
    public boolean setSpeci(){
        if(!extending.get()){
            //check if moving
            extentTimer.reset();
            extending.set(true);
            //reset timer for total time moving
        }
        //need to make sure slides are moving before we check velo
        boolean velo_bool = !(oSlideRight.getCurrent(CurrentUnit.MILLIAMPS) > 0) || !(extentTimer.seconds() > 0.2) || Math.abs(oSlideRight.getVelocity()) > 1000;
        if(oSlideRight.getCurrentPosition() <= 500 &&  extentTimer.seconds() < 1 && velo_bool){
            //if the position is not at speci height, the total time moving is less than 1 sec, and the slides are moving fast, keep moving.
            slidesLimiterOuttake(1, 1000, false);
            return false;
        }
        //otherwise stop, and the slides will drift to the correct spot.
        slidesLimiterOuttake(0, 1000, false);
        extending.set(false);
        return true;

    }
    public boolean setSpeci(boolean override){
        //once again, similar to the other setSpeci function, does check for obejct in the box
        if(delivery.hasObject()){
            //stops if no object is in the box
            hasObject = true;
        }

        if(!hasObject){
            extending.set(false);
            slidesLimiterOuttake(0, 1000, false);
            return true;
        }

        if(!extending.get()){
            extentTimer.reset();
            extending.set(true);
        }
        //need to make sure slides are moving before we check velo
        boolean velo_bool = !(oSlideRight.getCurrent(CurrentUnit.MILLIAMPS) > 0) || !(extentTimer.seconds() > 0.2) || Math.abs(oSlideRight.getVelocity()) > 1000;
        if(oSlideRight.getCurrentPosition() <= 500 &&  extentTimer.seconds() < 1 && velo_bool){
            slidesLimiterOuttake(1, 1000, false);
            return false;
        }
        slidesLimiterOuttake(0, 1000, false);
        extending.set(false);
        return true;

    }

    public void resetOuttake(){
        //resets the encoders
        oSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        oSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        oSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void initializeAmpThreshold(BoilerChassis chassis){
        this.chassis = chassis;
    }
    public void ampThresholdAdjuster(){
        //lower amp threshold while driving
        if(Math.pow(chassis.otos.otos.getVelocity().x,2) + Math.pow(chassis.otos.otos.getVelocity().y, 2) > 2){ //if Vx^2+Vy^2 is greater than 2
            ampThreshold = 7000;
        }else{
            ampThreshold = 6500;
        }
    }
}
