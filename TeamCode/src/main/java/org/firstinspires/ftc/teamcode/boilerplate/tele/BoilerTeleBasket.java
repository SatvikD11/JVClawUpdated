package org.firstinspires.ftc.teamcode.boilerplate.tele;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.boilerplate.BoilerBot;

@TeleOp
//This is a teleop file, meaning that this file is run repeatedly. It has all of the driving and state changes inside, and references other documents for functions.
public class BoilerTeleBasket extends LinearOpMode {
    BoilerBot robot;
    boolean chassisReversed = false;
    double lat_mult = 1;
    double slidesMult = 1;
    boolean tranferOkayed = false;
    boolean open = false;
    boolean homed_result = true;

    enum DrivingStates {
        FULL_SPEED_MANUAL,
        STRAFE,
        SLOW_SPEED_MANUAL
    }
    enum SubAssemblyStates {
        INTAKE,
        TRANSFER,
        OUTTAKE,
        OUTTAKE_AUTO,
        SPECI_AUTO,
        AUTO_DROPOFF_IDLE,
        SPECI_RETRACT
    }

    enum ClawStates {
        SUBRETRACT,
        SCAN,
        PICKUP
    }
    ClawStates clawState = ClawStates.SCAN;
    DrivingStates drivingState = DrivingStates.FULL_SPEED_MANUAL;
    SubAssemblyStates subState = SubAssemblyStates.INTAKE;
    public boolean outtake;
    public ElapsedTime boxTime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BoilerBot(hardwareMap, telemetry);
        waitForStart();
        robot.clawSub.wrist.scaleRange(0.05, 0.90);
        robot.clawSub.setINTAKE_SCAN();
        while(opModeIsActive()){
            slidesMult = robot.OutreachMode ? 0.5 : 1;
            robot.chassis.teleDrive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x,
                    gamepad1.right_trigger, robot.OutreachMode, robot.OutreachMode ? 0.5 : 0.8, lat_mult, chassisReversed, gamepad1.dpad_left, gamepad1.dpad_right);//drives the robot
            gp1();
            switch(subState){
                case INTAKE://if in inkate(tele-op) mode:
                    outtake = false;
                    if(!homed_result){//home the outtake
                        homed_result = robot.oSlides.homeOuttake();
                    }
                    robot.iSlides.slidesLimiterIntake(-gamepad2.left_stick_y, 1300, gamepad2.right_trigger > 0.2, slidesMult);//<- slide control
                    robot.deliverySub.setOUTTAKE_ARM_TRANSFER();//reset outtake(1/2)
                    robot.deliverySub.setOUTTAKE_TRANSFER_ADJUSTED();//reset outtake(2/2)
                    if(clawState == ClawStates.SCAN){//LED control
                        double check[] = robot.chassis.llcheck();
                        if(check[1] != -1){
                            if(check[1] == 0){
                                robot.leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                            }else if(check[1] == 1){
                                robot.leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                            }else if(check[1] == 2){
                                robot.leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                            }else if(check[1] == 3){
                                robot.leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                            }
                        } else{
                            robot.leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                        }
                        slidesMult = 1-check[0];
                    }
                    intakeSub();
                    break;
                case TRANSFER://if in transfer mode:
                    boxTime.reset();
                    transfer();//transfer
                    break;
                case OUTTAKE://if in outtake(tele-op) mode:
                    if(!outtake){boxTime.reset();}
                    outtake = true;
                    robot.iSlides.homeIntake();//home intake
                    robot.oSlides.slidesLimiterOuttake(-gamepad2.left_stick_y, 1500, gamepad2.right_trigger > 0.2);//<- slide control
                    robot.clawSub.setINTAKE_SCAN();//reset intake(1/2)
                    robot.clawSub.openClaw();//reset intake(2/2)
                    robot.deliverySub.setOUTTAKE_ARM_UP();//ready outtake
                    outtakeSub();
                    robot.outtakeLEDControl();
                    break;
                case OUTTAKE_AUTO://if in outtake(auto)
                    robot.leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
                    outtakeAuto();
                    break;
                case AUTO_DROPOFF_IDLE://if in intake(auto)
                    robot.leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    autoDropoffIdle();
                    break;
                case SPECI_AUTO://if in outtake(auto(specimen delivery))
                    robot.leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
                    autoSpeci();
                    break;
                case SPECI_RETRACT://if in outtake(auto(specimen retract)
                    robot.leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
                    speciRetract();
                    break;
            }

            if(!robot.deliverySub.colorSensorStaticy){
                telemetry.addData("RGB SUM: ", robot.deliverySub.color.getDistance(DistanceUnit.CM));
            }
            telemetry.addData("outtake: ", outtake);
            telemetry.addData("joint: ", robot.clawSub.jointPos.getPosition());
            telemetry.addData("intakeDocked: ", robot.iSlides.intakeDocked);
            telemetry.addData("outtakeDocked: ", robot.oSlides.outtakeDocked);
            telemetry.addData("oAmp: ", robot.oSlides.oSlideRight.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("iAmp: ", robot.iSlides.islide.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("iSlides: ", robot.iSlides.islide.getCurrentPosition());
            telemetry.addData("oSlides: ", robot.oSlides.oSlideRight.getCurrentPosition());
            telemetry.addData("iLS: ", robot.iSlides.iLSLeft.isPressed());
            telemetry.addData("iRS: ", robot.iSlides.iLSRight.isPressed());
            telemetry.addData("oRs: ", robot.oSlides.oLSRight.isPressed());
            telemetry.addData("iRs: ", robot.oSlides.oLSLeft.isPressed());
            telemetry.addData("strafe_velo: ", robot.chassis.middle.deadwheel.getVelocity());
            telemetry.addData("forward_velo: ", robot.chassis.leftX.deadwheel.getVelocity());
            telemetry.addData("outtake slide velo",robot.oSlides.oSlideRight.getVelocity());
            telemetry.update();
        }


    }
    public void gp1(){
        if(gamepad1.circle && getRuntime() > 0.2){//if circle is pressed reverse chassis
            chassisReversed = !chassisReversed;
            resetRuntime();
        }
        if(gamepad1.right_bumper && gamepad1.left_bumper){//if both left and right bumpers are pressed reset feild centric
            robot.chassis.otos.resetIMU();
        }
    }
    public void intakeSub(){
        if(gamepad2.cross && getRuntime() > 0.23){//toggling between ground pickup claw states
            resetRuntime();
            if(clawState == ClawStates.PICKUP) {
                robot.clawSub.noWristERETRACT();
                clawState = ClawStates.SUBRETRACT;
            }else if(clawState == ClawStates.SCAN){
                robot.clawSub.setINTAKE_PICKUP();
                clawState = ClawStates.PICKUP;
            }else{
                tranferOkayed = false;
                subState = SubAssemblyStates.TRANSFER;
                transfer();
            }
        }
        if(gamepad2.triangle && getRuntime() > 0.25){//toggling between wall pickup claw states
            resetRuntime();
            if(open){
                robot.clawSub.specimenFloorUp();
                open = false;
                clawState = ClawStates.SCAN;
            }else{
                robot.clawSub.specimenFloor();
                robot.clawSub.noWristERETRACT();
                open = true;
                clawState = ClawStates.SUBRETRACT;
            }
        }
        if(gamepad2.circle) {//reset to scan pos
            robot.clawSub.noWristScan();
            clawState = ClawStates.SCAN;
        }
            if (gamepad2.right_bumper && getRuntime() > 0.13) {//wrist twists
                robot.clawSub.setINTAKE_MIDDLE_Scan();
                resetRuntime();
                if (robot.clawSub.wrist.getPosition() < 0.56) {
                    robot.clawSub.wrist.setPosition(robot.clawSub.wrist.getPosition() + 0.13);
                } else {
                    robot.clawSub.wrist.setPosition(robot.clawSub.wrist.getPosition() + 0.10);
                }
            }
            if (gamepad2.left_bumper && getRuntime() > 0.13) {
                robot.clawSub.setINTAKE_MIDDLE_Scan();
                resetRuntime();
                if (robot.clawSub.wrist.getPosition() <= 0.56) {
                    robot.clawSub.wrist.setPosition(robot.clawSub.wrist.getPosition() - 0.13);
                } else {
                    robot.clawSub.wrist.setPosition(robot.clawSub.wrist.getPosition() - 0.10);
                }
            }

        if(gamepad2.square && getRuntime() > 0.3){//swap to outtake
            resetRuntime();
            swapToOuttake();
        }
    }
    public void outtakeSub(){
        if(gamepad2.cross){//drop or not
            robot.deliverySub.setOUTTAKE_DROP();
        }else if(boxTime.milliseconds() > 1500 && outtake){
            robot.deliverySub.setOUTTAKE_SPECIMEN_DROP();
        }else{
            robot.deliverySub.setOUTTAKE_SPECIMEN();
        }
        if(gamepad2.square && getRuntime() > 0.3){//swap to intake
            resetRuntime();
            swapToIntake();
        }
        if(gamepad2.dpad_up){
            subState = SubAssemblyStates.OUTTAKE_AUTO;
        }
    }
    public void outtakeAuto(){
        boolean result = robot.oSlides.setSlides(200, true);
        if(result){
            subState = SubAssemblyStates.AUTO_DROPOFF_IDLE;
        }
    }
    public void autoDropoffIdle(){
        if(gamepad2.dpad_up){
            subState = SubAssemblyStates.SPECI_AUTO;
        }
        if(gamepad2.dpad_down){
            subState = SubAssemblyStates.SPECI_RETRACT;
        }
    }
    public void autoSpeci(){
        boolean up = robot.oSlides.setSpeci(true);
        if(up){
            subState = SubAssemblyStates.SPECI_RETRACT;
        }
    }
    public void speciRetract(){
        boolean down = robot.oSlides.homeOuttake();
        if(down){
            subState = SubAssemblyStates.OUTTAKE;
        }
    }
    public void transfer(){
        boolean transferResult = robot.stateBasedTransfer(tranferOkayed, robot.deliverySub.colorSensorStaticy);
        if(transferResult){
            swapToOuttake();
        }
        if(gamepad2.touchpad){
            tranferOkayed = true;
        }
        if(gamepad2.triangle){
            robot.transferState = BoilerBot.TransferStates.ELSEWHERE;
            robot.transferStarted = false;
            swapToIntake();
        }

    }
    public void swapToIntake(){
        robot.iSlides.reset();
        subState = SubAssemblyStates.INTAKE;
        clawState = ClawStates.SCAN;
    }
    public void swapToOuttake(){
        robot.oSlides.resetOuttake();
        subState = SubAssemblyStates.OUTTAKE;
        homed_result = false;
    }

}
