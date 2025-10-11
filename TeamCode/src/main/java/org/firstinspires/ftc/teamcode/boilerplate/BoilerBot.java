package org.firstinspires.ftc.teamcode.boilerplate;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.boilerplate.helpers.Control;
import org.firstinspires.ftc.teamcode.boilerplate.intake.BoilerClaw;
import org.firstinspires.ftc.teamcode.boilerplate.intake.BoilerIntake;
import org.firstinspires.ftc.teamcode.boilerplate.outtake.BoilerOutArmAndBox;
import org.firstinspires.ftc.teamcode.boilerplate.outtake.BoilerOuttake;

import java.util.Collections;
import java.util.List;

//Can use this file for combined actions of sub-assemblies and chassis control
//this file has one main function, a control of the claw, slides, and LEDs based on the state of transfer the robot is in.
public class BoilerBot {
    HardwareMap hardwareMap;
    public BoilerChassis chassis;
    public BoilerOuttake oSlides;
    public BoilerClaw clawSub;
    public BoilerIntake iSlides;
    public BoilerOutArmAndBox deliverySub;
    public Control odo;
    public RevBlinkinLedDriver leds;
    public Boolean OutreachMode = false; //use this to switch to outreach and competition
    public enum TransferStates {
        ELSEWHERE,
        TRANSFER_READY,
        SAMPLE_DROPPED
    }
    public boolean transferStarted = false;
    public ElapsedTime transferTimer = new ElapsedTime();
    public TransferStates transferState = TransferStates.ELSEWHERE;
    public BoilerBot(HardwareMap hardwareMap, Telemetry telemetry){
        leds = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
        this.hardwareMap = hardwareMap;
        deliverySub = new BoilerOutArmAndBox(hardwareMap);
        oSlides = new BoilerOuttake(hardwareMap, deliverySub, leds);
        chassis = new BoilerChassis(hardwareMap, oSlides, telemetry);
        clawSub = new BoilerClaw(hardwareMap);
        iSlides = new BoilerIntake(hardwareMap);
        odo = new Control(chassis);
        oSlides.initializeAmpThreshold(chassis);

    }
    public boolean stateBasedTransfer(boolean transferOkayed, boolean timeOut){
        if(!transferStarted){
            transferTimer.reset();
            transferStarted = true;
        }else{
            if(transferTimer.seconds() < 2.2 || transferOkayed){ //timer < 2.2 sec OR transferOkayed is true
                if(iSlides.intakeDocked || transferOkayed){
                    if(oSlides.outtakeDocked || transferOkayed){
                        if(transferState == TransferStates.ELSEWHERE){ //if state ELSEWHERE
                            transferTimer.reset();
                            clawSub.setINTAKE_TRANSFER(); //set to specific pos
                            transferState = TransferStates.TRANSFER_READY; //set transfer state to READY
                        }
                        if(transferState == TransferStates.TRANSFER_READY){ //if state READY
                            clawSub.setINTAKE_TRANSFER();
                            if( transferTimer.seconds() > 0.4 && (clawSub.jointPos.checkPosition(.05)) ){
                                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                                clawSub.openClaw();
                                transferState = TransferStates.SAMPLE_DROPPED; //state is now SAMPLE_DROPPED
                                transferTimer.reset();
                            }
                        }
                        if(transferState == TransferStates.SAMPLE_DROPPED){ //if state SAMPLE_DROPPED
                            if(transferTimer.seconds() > 0.8){
                                deliverySub.setOUTTAKE_TRANSFER();
                            }
                        }
                        if(transferState == TransferStates.SAMPLE_DROPPED && (transferOkayed || deliverySub.hasObject() || (timeOut && transferTimer.seconds() > 1.3))){
                        //if state = SAMPLE_DROPPED AND (transferOkayed OR hasObject OR (time out AND timer>1.3sec))
                            if(deliverySub.hasObject() || deliverySub.colorSensorStaticy){
                                oSlides.hasObject = true;
                            }
                            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                            transferStarted = false;
                            transferState = TransferStates.ELSEWHERE; //state = ELSEWHERE now
                            return true;
                        }
                    }
                }else{
                    leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                    iSlides.homeIntake();
                }
            }
        }
        return false;
    }
    public void outtakeLEDControl(){
        if(deliverySub.hasObject()){
            /*
            List<Integer> vals = deliverySub.returnRGB();//blue,red,green(yellow)
            double max = vals.indexOf(Collections.max(vals));
            if(max == 0){
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }else if(max == 1){
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }else{
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            }
             */
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        }else{
            leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        }
    }


}
