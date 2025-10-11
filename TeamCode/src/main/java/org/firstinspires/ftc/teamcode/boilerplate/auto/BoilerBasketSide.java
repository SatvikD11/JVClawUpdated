package org.firstinspires.ftc.teamcode.boilerplate.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.boilerplate.BoilerBot;
//auto program for basket side
@Autonomous(name = "basket")
public class BoilerBasketSide extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BoilerBot robot = new BoilerBot(hardwareMap, telemetry);
        while(opModeInInit()){
            robot.deliverySub.setOUTTAKE_MECH_TRANSFER();
            robot.clawSub.setINTAKE_SCAN();
            telemetry.addData("heading: ", Math.toDegrees(robot.chassis.otos.otos.getPosition().h));
            telemetry.addData("rotPid: ", robot.odo.rotationalPID(0,1,0,0));
            telemetry.addData("l: ", robot.chassis.leftX.getCurrentPosition());
            telemetry.addData("m: ", robot.chassis.middle.getCurrentPosition());
            telemetry.addData("r: ", robot.chassis.rightX.getCurrentPosition());
            telemetry.update();
        }
        waitForStart();
        robot.chassis.otos.resetIMU();
        //preset intake slides and outtake slides
        robot.clawSub.setINTAKE_SCAN();
        robot.odo.whileController(()->robot.oSlides.setSlides(1450, false), 0);
        robot.clawSub.setINTAKE_SCAN();
        robot.odo.whileController(()->robot.iSlides.setSlides(390), 0);

        //dropoff preload
        robot.odo.profilemove(-25, 14.5/1.5, 0.5);
        while(robot.oSlides.oSlideRight.getCurrentPosition() < 1400 || robot.oSlides.extentTimer.seconds() < 1.3){}
        robot.deliverySub.autoBasketDropoff();
        robot.odo.whileController(()->robot.oSlides.homeOuttake(), 0);

        //pickup rightmost
        robot.odo.profilemove(-4, 14.5/1.5, 0.5);
        robot.odo.llAdjustLeft(()->(robot.chassis.otos.otos.getPosition().x > -6.9));
        robot.clawSub.pickup();
        robot.odo.whileController(()->robot.stateBasedTransfer(false, true), 0, ()->{
            robot.odo.whileController(()->robot.oSlides.setSlides(1450, false), 0);
            robot.clawSub.setINTAKE_SCAN();
        });
        //dropoff rightmost
        robot.odo.profilemove(-25,14.5/1.75,0.5);
        while((robot.oSlides.oSlideRight.getCurrentPosition() < 1400 || robot.oSlides.extentTimer.seconds() < 1.7)){}
        robot.deliverySub.autoBasketDropoff();
        robot.odo.whileController(()->robot.oSlides.homeOuttake(), 100);

        //pickup middle
        robot.clawSub.setINTAKE_SCAN();
        robot.odo.whileController(()->robot.iSlides.setSlides(390), 0);
        robot.odo.profilemove(-12, 14.5/1.75, 0.5);
        robot.odo.llAdjustLeft(()->(robot.chassis.otos.otos.getPosition().x > -16.6));
        robot.clawSub.pickup();
        //dropoff middle
        robot.odo.whileController(()->robot.stateBasedTransfer(false, false), 100, ()->{
            robot.odo.whileController(()->robot.oSlides.setSlides(1450, false), 0);
            robot.clawSub.setINTAKE_SCAN();
        });
        robot.odo.profilemove(-25,14.5/1.75,0.5);
        robot.oSlides.extentTimer.reset();
        while((robot.oSlides.oSlideRight.getCurrentPosition() < 1400 || robot.oSlides.extentTimer.seconds() < 1.7)){}
        robot.deliverySub.autoBasketDropoff();
        robot.odo.whileController(()->robot.oSlides.homeOuttake(), 100);
        sleep(10000);
        robot.clawSub.setINTAKE_SCAN();

        //pickup leftMost
        /*robot.odo.profilemove(-12,20,0.5);
        robot.odo.whileController(()->robot.iSlides.setSlides(390), 0);
        robot.clawSub.wrist.setPosition(0.7);
        robot.odo.rotateWithPID(42);
        robot.clawSub.pickup();
        robot.odo.whileController(()->robot.stateBasedTransfer(false, true), 0.3, ()->{
            robot.odo.whileController(()->robot.oSlides.setSlides(1470));
            robot.clawSub.setINTAKE_SCAN();
        });
        robot.odo.rotateWithPID(-42);
        //dropoff leftmost
        robot.odo.profilemove(-25,14.5/1.75,0.5);
        while((robot.oSlides.oSlideRight.getCurrentPosition() < 950 || robot.oSlides.extentTimer.seconds() < 1.3) && robot.oSlides.hasObject){}
        robot.deliverySub.autoBasketDropoff();
        robot.odo.whileController(()->robot.oSlides.homeOuttake());
        robot.odo.profilemove(-27,14.5/1.75,0.5);
        */
    }
}
