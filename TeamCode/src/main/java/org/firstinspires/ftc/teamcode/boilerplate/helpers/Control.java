package org.firstinspires.ftc.teamcode.boilerplate.helpers;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.boilerplate.BoilerBot;
import org.firstinspires.ftc.teamcode.boilerplate.BoilerChassis;
import org.firstinspires.ftc.teamcode.boilerplate.sensorclasses.BoilerDeadwheel;

import java.util.function.BooleanSupplier;


/*
Quick crash course in Physics 1.
Velocity is how fast you are going. Velocity also considers direction. Velocity = Distance/Time
Acceleration is how fast you are changing velocity. Acceleration = Velocity/Time
If the robot is speeding up, you have positive acceleration. If the robot is slowing down, you have negative acceleration.

There is one main formula this program uses:
    x = x0 + v0t + 1/2*a*t^2
    Your current position = starting position + (starting velocity * time since start) + (0.5 * acceleration * (time since start ^2))
    This formula assumes constant acceleration. IE. you are changing your speed at a constant rate.
    You will see different versions of this formula throughout. Some may exclude the starting position and velocity. Some may exclude the acceleration if velocity is constant.
 */


public class Control {
    DcMotorEx fr,br,bl,fl;
    BoilerDeadwheel leftX,rightX,middle;
    double x=0, y=0; //calculated position
    private double x_pos; //current position based on deadwheels. Updated by arcLocalization
    private double y_pos;
    private double heading;
    public static double inchespertickX = 0.00047936899; //constants for the deadwheels
    public static double inchespertickY = 0.00047936899;

    //double max_velo = 92499.20743 * inchespertickX;
    double max_velo = 100000 * inchespertickX;

    double max_accel = max_velo / 0.5;
    RevBlinkinLedDriver leds;


    double lastError = 0;
    ElapsedTime cycleTimer = new ElapsedTime();
    ElapsedTime profileTimer = new ElapsedTime();
    double integralSum = 0;

    //SUBSTITUTE THIS VALUE FROM TRACK WIDTH TUNER
    public static double track_width = 10.7454843;
    double cycleTime = 0;
    public BoilerChassis chassis; //import the functions from BoilerChassis. IE> chassis.drive() runs the drive function from BoilerChassis
    public Control(BoilerChassis chassis){
        fr = chassis.fr;
        br = chassis.br;
        fl = chassis.fl;
        bl = chassis.bl;
        middle = chassis.middle;
        leftX = chassis.leftX;
        rightX = chassis.rightX;
        this.chassis =chassis;

    }
    public void pidmove(double x_coord, double y_coord, double fix_time){
        //NOT USED
        pidOnlyDrive(x_coord - x, y_coord - y, fix_time);
        x = x_coord;
        y = y_coord;
    }
    public void profilemove(double x_coord, double y_coord, double fix_time){
        //send desired coords to motionProfile to move. Then it updates the calculated position.
        motionProfile(x_coord - x, y_coord - y, fix_time);
        x = x_coord;
        y = y_coord;
    }
    public void resetCoords(){
        //NOT USED
        x = 0;
        y = 0;
    }
    public void motionProfile(double x, double y, double fix_time){
        //trapazoidal motion profile. This figures out how long to accelerate, cruise, deccelerate, and fix to get to the desired coords.
        resetOdo();
        double accel_time = max_velo / max_accel; //This is figuring out how long it will take to get to max velocity with max acceleration
        double accel_distance = 0.5*max_accel*(Math.pow(accel_time,2)); // x = 1/2*a*t^2, how far does the robot go while accelerating to max velocity?
        double total_dist = Math.sqrt((x*x)+(y*y)); // Find the magnitude, total distance, of the path you are taking. Pythogorean theorem. r = sqrt(x^2 + y^2)
        double cruise_time  = ((total_dist - (2*accel_distance))/max_velo); // how long will the robot cruise at max velocity? calculated based on the max velocity and total distance minus the distance to accelerate and deccelerate
        double total_time = cruise_time + (accel_time*2); // total time for the manuever to be executed.
        double halfway = total_dist / 2;
        //PID CONSTANTS
        //Need to tune these
        double kPl = 6;
        double kIl = 1;
        double errIntegralSum = 0;
        cycleTime = 0;
        double profile_dist;
        //TRAVERSAL ANGLE
        double theta = Math.atan2(y, x); // this find the angle between the horizantal x axis and the point (x,y), the angle the robot will need to go.
        rotationalPIDSetup();
        if(accel_distance > halfway){ //if we cant accelerate to max velocity by the distance halfway point. The distance you go will accelerating to max velocity is greater than the halfway point.
            accel_time = Math.sqrt((2*halfway) / max_accel); //rearrange the formula x = 1/2*a*t^2 to find the time it will take to accelerate to the halfway distance
            total_time = accel_time*2; //in this case the path is a triangle, the robot accelerates then deccelerates with no cruise time. This means the halfway point is when the robot changes from accelerating to deccelerating
            double adjusted_max_velo = max_accel * accel_time; //this is as fast as the robot will get, at the halfway point
            profileTimer.reset();
            while(profileTimer.seconds() < total_time + fix_time){
                //ACCELERATION
                arcLocalizationApprox();
                if(profileTimer.seconds() < accel_time){ //before the halfway point
                    if(leds!=null){
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    }
                    profile_dist = 0.5*max_accel*(Math.pow(profileTimer.seconds(), 2)); // x = 1/2*a*t^2, find how far the robot has gone
                    motionPid(profile_dist, errIntegralSum, kIl, kPl, theta); //move the robot
                    //DECELERATION
                }else if(profileTimer.seconds() < total_time){  // after the halfway point
                    if(leds!=null) {
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    }
                    profile_dist = halfway + (adjusted_max_velo*(profileTimer.seconds()-accel_time)) - (0.5*max_accel*(Math.pow((profileTimer.seconds()-accel_time), 2)));
                        // x = x0 + vt + 1/2*a*t^2, in this case you are deccelerating so a is negative. t is the time since you started deccelerating. x0 is the starting position. v is starting velocity.
                    motionPid(profile_dist, errIntegralSum,kIl, kPl, theta); //move the robot
                    //POSITION LOCK
                }else{
                    if(leds!=null) {
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    }
                    profile_dist = total_dist; //based on time, you should have reached the position
                    motionPid(profile_dist, errIntegralSum,kIl, kPl, theta); //move the robot to fix the position, fine tuning
                }

            }
            //once all completed, stop the robot
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);
        }
        if(accel_distance < halfway){ //if the distance is long enough that you will have some cruise time
            profileTimer.reset();
            while(profileTimer.seconds() < total_time + fix_time){
                //ACCEL
                arcLocalizationApprox();
                if(profileTimer.seconds() < accel_time){
                    if(leds!=null) {
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    }
                    profile_dist = 0.5*max_accel*(Math.pow(profileTimer.seconds(), 2)); // x = 1/2*a*t^2, find how far the robot has gone
                    motionPid(profile_dist, errIntegralSum,kIl, kPl,theta);
                    //CRUISE
                }else if(profileTimer.seconds() < accel_time + cruise_time){
                    if(leds!=null) {
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    }
                    profile_dist = accel_distance + (max_velo*(profileTimer.seconds()-accel_time)); // x = x0 + vt, the robot is now at a constant velocity (no longer accelerating). t is the time since you stopped accelerating
                    motionPid(profile_dist, errIntegralSum,kIl, kPl,theta);
                    //DECELERATE
                }else if(profileTimer.seconds() < total_time){
                    if(leds!=null) {
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    }
                    profile_dist = accel_distance + (max_velo*(profileTimer.seconds()-accel_time)) - (0.5*max_accel*(Math.pow((profileTimer.seconds()-(accel_time+cruise_time)), 2)));
                    // x = x0 + vt + 1/2*a*t^2, in this case you are deccelerating so a is negative. t is the time since you started deccelerating. x0 is the starting position. v is starting velocity.
                    motionPid(profile_dist, errIntegralSum,kIl, kPl,theta);
                }else{
                    if(leds!=null) {
                        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    }
                    //fine tune position
                    profile_dist = total_dist;
                    motionPid(profile_dist, errIntegralSum,kIl, kPl,theta);
                }
            }
            //stop robot
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);

        }

        resetOdo();


    }
    public void pidOnlyDrive(double x, double y, double fix_time){
        //NOT USED
        resetOdo();
        double total_dist = Math.sqrt((x*x)+(y*y));
        double total_time = total_dist / max_velo;
        //PID CONSTANTS
        //Need to tune these
        double kPl = 6;
        double kIl = 1;
        double errIntegralSum = 0;
        cycleTime = 0;
        double profile_dist;
        //TRAVERSAL ANGLE
        double theta = Math.atan2(y, x);
        //if we cant accelerate to max velocity by the distance halfway point
        rotationalPIDSetup();
        profileTimer.reset();
        while(profileTimer.seconds() < total_time + fix_time + 0.1){
            //ACCELERATION
            arcLocalizationApprox();
            profile_dist = total_dist;
            motionPid(profile_dist, errIntegralSum,kIl, kPl, theta);
        }
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        resetOdo();
    }

    public void motionPid(double profile_dist, double errIntegralSum, double kIl, double kPl, double travel_heading){
        //strafe err is the difference between the calculate position and the measured position along the y-axis
        double strafe_err = -(profile_dist*Math.cos(travel_heading)) - y_pos;
        //fwd err is the same thing along the x-axis
        double fwd_err = -(profile_dist*Math.sin(travel_heading)) - x_pos;

        double theta = joystickNormalize(Math.atan2(-fwd_err, strafe_err)) % (Math.PI*2); //sets the angle (theta) to account for the error in position
        //double equationOne = (Math.sin(theta + (Math.PI / 4))); //Front-Right and Back-Left
        //double equationTwo = (Math.cos(theta + (Math.PI / 4))); //Front-Left and Back-Right
        //PID calculations
        double headingPIDOutput = rotationalPID(0,1 , 0, 0);
        double posError = Math.sqrt((strafe_err*strafe_err)+(fwd_err*fwd_err));
        double delta_time = profileTimer.seconds() - cycleTime;
        errIntegralSum += (posError*delta_time);
        double integral = errIntegralSum * integralWindupClamp(errIntegralSum, posError) * kIl;
        double proportional = posError * kPl;
        double pidOutput = (integral + proportional) / 100;

        chassis.drive(pidOutput, theta, headingPIDOutput, true); //drives to position calculated
        //in order, magnitude of movement, angle, rotational heading, fieldcentric?
    /*
        fr.setPower((equationOne * pidOutput) + headingPIDOutput);
        bl.setPower((equationOne * pidOutput) - headingPIDOutput);
        br.setPower((equationTwo * pidOutput) + headingPIDOutput);
        fl.setPower((equationTwo * pidOutput) - headingPIDOutput);
     */


        cycleTime = profileTimer.seconds();
    }
    public void arcLocalizationApprox(){ //find current position based on deadwheels and updates x_pos, y_pos, and heading
        x_pos =inchespertickX*(leftX.getCurrentPosition()-rightX.getCurrentPosition())/2; //finds the position of the center of the robot (between the two x-axis deadwheels)
        y_pos = -middle.getCurrentPosition()*inchespertickY;
        heading = chassis.otos.otos.getPosition().h; //finds the angle the robot is at

    }
    public double joystickNormalize(double theta){ //converts the angle to some kind of standard. NEED TO ASK ABOUT ANGLE STUFF
        theta = (theta >= 0 && theta < Math.toRadians(270)) ? (-1 * theta) + Math.toRadians(90) : (-1 * theta) + Math.toRadians(450);
        theta = (theta < 0) ? Math.toRadians(360) + theta : theta;
        return theta;
    }
    public double rotationalPID(double deltaTheta, double kP, double kI, double kD){

        //run our PID until our output becomes super small
        arcLocalizationApprox();
        double error = deltaTheta - Math.toDegrees(heading);
        double deltaError = error - lastError; //change in error
        double proportional = kP*error;
        double derivative = (deltaError)/cycleTimer.time()*kD;
        integralSum += (error * cycleTimer.time());
        cycleTimer.reset();
        double integral = integralSum*kI;
        //become zero if integral and error have different signs (+-)
        integral *= integralWindupClamp(integral, error);

        double output = (proportional + integral + derivative) / 200;
        //need to abs this output because it could be negative
        if(Math.abs(output) < 0.01){
            return 0;
        }

        lastError = error; //update error value for future runs
        return output;
    }
    public void rotationalPIDSetup(){ //zero out timer and variables
        cycleTimer.reset();
        integralSum = 0;
        lastError = 0;
    }
    public int integralWindupClamp(double integral, double error){
        //if integral term is in opposite direction of error(accelerating the wrong way), zero the integral term
        if(Math.signum(integral) == Math.signum(error)){ //are the signs (+ or -) the same
            return 1;
        }else{
            return 0;
        }

    }
    public void rotateWithPID(double deltaTheta){
        rotationalPIDSetup();
        arcLocalizationApprox(); //where is the robot
        ElapsedTime rotateTimer = new ElapsedTime();
        rotateTimer.reset();
        while(rotateTimer.seconds() < 1.5){ //repeat steps for so long
            if(leds!=null) {
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }
            double power = rotationalPID(deltaTheta, 2, 1, 0); //find power needed to turn
            if(power == 0){
                break; //stop while loop if power = 0 (target reached)
            }
            turn(power); //turn with power calculated
        }
        turn(0); //stop turning
        resetOdo();
    }
    public void turn(double power){
        br.setPower(power);
        bl.setPower(-power);
        fl.setPower(-power);
        fr.setPower(power);
    }
    public void resetOdo(){
        leftX.reset();
        rightX.reset();
        middle.reset();

        x_pos = 0;
        y_pos = 0;
    }
    public void whileController(BooleanSupplier function){
        boolean conditional = function.getAsBoolean();
        while(!conditional){
            conditional = function.getAsBoolean();
        }
    }
    //delayed while controller runs in parallel
    public void whileController(BooleanSupplier function, double delay){
        new Thread(()-> {
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (timer.seconds() <= delay) {
                continue;
            }
            boolean conditional = function.getAsBoolean();
            while (!conditional) {
                conditional = function.getAsBoolean();
            }
        }).start();

    }
    public void whileController(BooleanSupplier function, double delay, Runnable exitFunction){
        new Thread(()-> {
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while (timer.seconds() <= delay) {
                continue;
            }
            boolean conditional = function.getAsBoolean();
            while (!conditional) {
                conditional = function.getAsBoolean();
            }
            exitFunction.run();

        }).start();
    }
    public static void sleep(double seconds){
        ElapsedTime sleepTimer = new ElapsedTime();
        while(sleepTimer.seconds() < seconds){

        }
    }
    public void llAdjustLeft(BooleanSupplier bool){

        double degreesAway = exceptionHandledLLResult();
        ElapsedTime runTimer = new ElapsedTime();
        boolean conditional;
        //if our limelight fails, just use SF
        if(degreesAway == 200) {
            conditional = bool.getAsBoolean();
        //if our ll works, use both
        }else{
            conditional = Math.abs(degreesAway) > 8 || bool.getAsBoolean();
        }
        while(conditional && runTimer.seconds() < 2){
            double power = 0.19;
            double headingPIDOutput = rotationalPID(0, 4, 0, 0);

            fr.setPower(power + headingPIDOutput);
            bl.setPower(power - headingPIDOutput);
            br.setPower(-power + headingPIDOutput);
            fl.setPower(-power - headingPIDOutput);
            degreesAway = exceptionHandledLLResult();
            if(degreesAway == 200) {
                conditional = bool.getAsBoolean();
                //if our ll works, use both
            }else{
                conditional = Math.abs(degreesAway) > 8 || bool.getAsBoolean();
            }
        }
        /*
        fr.setPower(-.25);
        bl.setPower(-.25);
        br.setPower(.25);
        fl.setPower(.25);
        sleep(0.2);
         */
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        x = chassis.otos.otos.getPosition().x;

    }
    public double exceptionHandledLLResult(){
        try {
            if(chassis.limelight.getLatestResult().isValid()){
                return chassis.limelight.getLatestResult().getTx();
            }
            return 200;
        }catch (Exception e){
            return 200;
        }
    }


}
