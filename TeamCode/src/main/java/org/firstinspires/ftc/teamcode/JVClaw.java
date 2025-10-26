/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;



import static org.firstinspires.ftc.teamcode.helpers.BasicTele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pathing.PedroPath;
/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: JV Claw", group="Robot")

public class JVClaw extends LinearOpMode {

    public DcMotor BL, BR, FL, FR, BaseClawMotor;
    public Servo ServoClaw, RightProng, LeftProng;

    public PedroPath PedroBody;

    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;
    double ClawPos = 0.1;
    double RightProngPos = 0.8;
    double LeftProngPos = 0.4;

    @Override
    public void runOpMode() {

        // Hardware setup
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

// Motor test block
        FL.setPower(0.5);
        sleep(1000);
        FL.setPower(0);
        FR.setPower(0.5);
        sleep(1000);
        FR.setPower(0);
        BL.setPower(0.5);
        sleep(1000);
        BL.setPower(0);
        BR.setPower(0.5);
        sleep(1000);
        BR.setPower(0);

// After confirming which wheel moves for each motor, adjust mapping:
        telemetry.addData(">", "Check motor directions. Then press Start.");
        telemetry.update();

        waitForStart();

        //BaseClawMotor = hardwareMap.get(DcMotor.class, "base_motor");

        //ServoClaw = hardwareMap.get(ServoImplEx.class, "servo_claw");
        //RightProng = hardwareMap.get(ServoImplEx.class, "right_prong");
        //LeftProng = hardwareMap.get(ServoImplEx.class, "left_prong");


        // Set motor directions
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.FORWARD);

        PedroBody = new PedroPath(this, hardwareMap,FL,FR,BL,BR);
        telemetry.addData(">", "Robot Ready. Press START.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            BasicTele(FL, FR, BL, BR, gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.right_bumper && gamepad1.left_bumper || gamepad1.left_stick_button);
            double SlideDownSpeed = gamepad1.left_trigger;
            double SlideSpeed = gamepad1.right_trigger;

            double HoldPower = 0.1;//adjust later

            if (gamepad1.right_stick_button){
                BL.setDirection(DcMotor.Direction.FORWARD);
                FL.setDirection(DcMotor.Direction.FORWARD);
                BR.setDirection(DcMotor.Direction.REVERSE);
                FR.setDirection(DcMotor.Direction.REVERSE);
            }
            else{
                BL.setDirection(DcMotor.Direction.REVERSE);
                FL.setDirection(DcMotor.Direction.REVERSE);
                BR.setDirection(DcMotor.Direction.FORWARD);
                FR.setDirection(DcMotor.Direction.FORWARD);
            }
            /*if (gamepad2.right_trigger > 0.1) {
                BaseClawMotor.setPower(SlideSpeed);
            }
            else if (gamepad2.left_trigger > 0.1) {
                BaseClawMotor.setPower(-SlideDownSpeed);
            }
            else{
                BaseClawMotor.setPower(HoldPower);
            }*/
            if (gamepad1.triangle && getRuntime() > 0.2){
                PedroBody.driveForward(11, 0.5);
            }
            else if (gamepad1.square && getRuntime() > 0.2){
                PedroBody.turnIMU(-90, 1);
            }
            else if (gamepad1.circle && getRuntime() > 0.2){
                PedroBody.driveForward(11, -0.5);
            }
            else if (gamepad1.square && getRuntime() > 0.2){
                PedroBody.turnIMU(90, 1);
            }

            /*if (gamepad1.x && getRuntime() > 0.2) {
                LeftProngPos -= 0.1;
                RightProngPos += 0.1;
                resetRuntime();
            }
            else if (gamepad1.b && getRuntime() > 0.2){
                LeftProngPos += 0.1;
                RightProngPos -= 0.1;
                resetRuntime();
            }
            if (gamepad1.y && getRuntime() > 0.2){
                ClawPos = 0.0;
                resetRuntime();
            }
            else if (gamepad1.a && getRuntime() > 0.2){
                ClawPos = 0.3;
                resetRuntime();
            }
            ClawPos = Range.clip(ClawPos, 0.0, 0.3);//clips after the inputs
            RightProngPos = Range.clip(RightProngPos, 0.4, 0.8);
            LeftProngPos = Range.clip(LeftProngPos, 0.0, 0.4);
            RightProng.setPosition(RightProngPos);//have to put it after the gamepad input so the position is proper
            LeftProng.setPosition(LeftProngPos);
            ServoClaw.setPosition(ClawPos);


            telemetry.addData("ClawPos", "%.2f", ClawPos);
            telemetry.addData("RightProngPos", "%.2f", RightProngPos);
            telemetry.addData("LeftProngPos", "%.2f", LeftProngPos);
            telemetry.update();
             */
            if (!opModeIsActive()) break;
        }
    }
}



