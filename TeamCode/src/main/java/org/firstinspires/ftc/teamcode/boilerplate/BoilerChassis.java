package org.firstinspires.ftc.teamcode.boilerplate;
/*
This program deals with limelight checking closest objects and moving the robot
teleDrive calculates movement values based on the controller's inputs
drive sets the power values for the motors based on the variables given to it: magnitude, direction, rotation, and if it is in field centric mode
llcheck returns the color of the closest target
 */
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.boilerplate.helpers.Control;
import org.firstinspires.ftc.teamcode.boilerplate.outtake.BoilerOuttake;
import org.firstinspires.ftc.teamcode.boilerplate.sensorclasses.BoilerDeadwheel;
import org.firstinspires.ftc.teamcode.boilerplate.sensorclasses.SparkIMU;

import java.lang.annotation.Target;

public class BoilerChassis {
    public BoilerDeadwheel leftX, middle, rightX;
    public DcMotorEx fl,fr,br,bl, enc_left;
    public SparkIMU otos;
    public Limelight3A limelight = null;
    public BoilerChassis(HardwareMap hardwareMap, BoilerOuttake outtake, Telemetry telemetry){
        fl = hardwareMap.get(DcMotorEx.class, "FL");
        bl = hardwareMap.get(DcMotorEx.class, "BL");
        br = hardwareMap.get(DcMotorEx.class, "BR");
        fr = hardwareMap.get(DcMotorEx.class, "FR");
        otos = new SparkIMU(hardwareMap);


        try{ //stops program within {} if there is an error instead of stopping the entire program

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
            limelight.pipelineSwitch(2); // Switch to pipeline number 2 (what the limelight is detecting(yellow, blue, and red)
            limelight.start(); // This tells Limelight to start looking
            telemetry.addData("running: ", limelight.isRunning());

            LLResult results = limelight.getLatestResult();
            telemetry.addData("pl: ", results.getPipelineIndex());
            telemetry.update(); //feedback display to driver's hub

        }catch (Exception e){
            telemetry.addData("exception: ", e.toString());
            telemetry.update();
        }
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        enc_left = hardwareMap.get(DcMotorEx.class, "enc_left");
        leftX = new BoilerDeadwheel(enc_left); //set up deadwheels
        rightX = new BoilerDeadwheel(br);
        middle = new BoilerDeadwheel(outtake.oSlideLeft);
    }
    public void teleDrive(double x, double y, double r, double right_trigger, boolean kidMode, double rotationalMult, double latMult, boolean reversed,
                          boolean strafe_left, boolean strafe_right){
        //teleDrive finds the specifics of your goal position based on the inputs from the controller.
            //It finds the goal in terms of magnitude(length of the path), angle, rotation of the robot, and if you are in fieldcentric mode)
        //the boolean reversed is used to reverse the controls for the driver based on the starting orientation of the robot.
        //x and y and the joystick position
        double reverse = reversed ? -1 : 1; //if it is reverse, reverse=-1
        double speedmult = kidMode ? 0.45 : right_trigger == 0 ? 0.4 : 2; //push the right trigger to speed up the robot
        double rotational = r * rotationalMult * reverse; //how strong are the rotations (how far do you have to push the joystick to cause fast spins
        boolean fieldCentric = true;
        if(strafe_left){
            x = -0.5 * reverse;
            y = 0;
            rotational = 0;
            fieldCentric = false;
        }else if(strafe_right){
            x = 0.5 * reverse;
            y = 0;
            rotational = 0;
            fieldCentric = false;
        }else{
            x = -reverse * (x) * speedmult * latMult;
            y = -reverse * (y) * speedmult;
        }
        double mag = Math.pow(x, 2) + Math.pow(y, 2);
        mag = Math.sqrt(mag);
        double rads = Math.atan2(y, x);
        rads = (rads >= 0 && rads < Math.toRadians(270)) ? (-1 * rads) + Math.toRadians(90) : (-1 * rads) + Math.toRadians(450);
        rads = (rads < 0) ? Math.toRadians(360) + rads : rads;
        drive(mag,rads,rotational,fieldCentric);
    }
    public void drive(double mag, double direction,double rotational, boolean fieldCentric){
        if(fieldCentric){
            double rangle = otos.otos.getPosition().h;
            rangle = (rangle < 0) ? Math.toRadians(360) + rangle : rangle;

            double turn = (direction < rangle) ? (Math.toRadians(360) - rangle) + (Math.abs(0 - direction)) : direction - rangle; //figure out which way to turn and how much
            //double turn = rads;
            //turn effects the rotation of the robot. mag effects the speed of the robot.
            double equationone = (Math.sin(turn + (Math.PI / 4)) * mag);
            double equationtwo = -(Math.sin(turn - (Math.PI / 4)) * mag);
            fr.setPower((equationone + rotational));
            bl.setPower((equationone - rotational));
            br.setPower((equationtwo + rotational));
            fl.setPower((equationtwo - rotational));
        }else{
            //double turn = rads;
            //if not fieldcentric, don't need to worry about the turn compared to the field
            double equationone = (Math.sin(direction + (Math.PI / 4)) * mag);
            double equationtwo = -(Math.sin(direction - (Math.PI / 4)) * mag);
            fr.setPower((equationone + rotational));
            bl.setPower((equationone - rotational));
            br.setPower((equationtwo + rotational));
            fl.setPower((equationtwo - rotational));
        }
    }
    public void strafeTilStop(double power){
        //NOT USED
        ElapsedTime runTimer = new ElapsedTime();
        runTimer.reset();
        while (!middle.isStopped()){
            fr.setPower(-power);
            bl.setPower(-power);
            br.setPower(power);
            fl.setPower(power);
        }
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.setPower(0);
    }
    public void simpleDrive(double power){
        //pretty self explanatory
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
        fl.setPower(power);
    }
    public double[] llcheck(){
        try{
            LLResult result = limelight.getLatestResult();
            //target's x and y pos
            double tx = result.getTx();
            double ty = result.getTy();
            if(Math.abs(tx) < 6 && Math.abs(ty) < 6 && result.isValid()){
                String classname = result.getDetectorResults().get(0).getClassName();
                //looks at closet target and returns its color(in form of number)
                double colorIndex = 0;
                if(classname.equals("red")){
                    colorIndex = 1;
                }else if(classname.equals("blue")){
                    colorIndex = 2;
                }else if(classname.equals("yellow")){
                    colorIndex = 0;
                }
                double arr[] = {0.35, colorIndex};
                return arr;
            }else if(Math.abs(tx) < 20 && result.isValid()){
                //if too far away, return value for error
                double arr[] = {0.2, 3};
                return arr;
            }
        }catch (Exception e){
        }
        double arr[] = {0, -1};
        return arr;
    }


}
