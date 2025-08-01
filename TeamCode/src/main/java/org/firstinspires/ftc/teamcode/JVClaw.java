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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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

    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;
    double ClawPos = 0.5;
    double ProngPos = 0.5;

    @Override
    public void runOpMode() {

        BL = hardwareMap.get(DcMotor.class, "leftRear");
        BR = hardwareMap.get(DcMotor.class, "rightRear");
        FL = hardwareMap.get(DcMotor.class, "leftFront");
        FR = hardwareMap.get(DcMotor.class, "rightFront");
        BaseClawMotor = hardwareMap.get(DcMotor.class, "base_motor");

        //ServoClaw = hardwareMap.get(Servo.class, "servo_claw");
        //RightProng = hardwareMap.get(Servo.class, "right_prong");
        //LeftProng = hardwareMap.get(Servo.class, "left_prong");


        // Set motor directions
        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData(">", "Robot Ready. Press START.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            BasicTele(FL, FR, BL, BR, gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.right_bumper && gamepad1.left_bumper || gamepad1.right_stick_button);
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
            }
            if (gamepad2.circle && getRuntime() > 0.2) {
                ProngPos += 0.2;
                resetRuntime();
            }
            else if (gamepad2.cross && getRuntime() > 0.2){
                ProngPos -= 0.2;
                resetRuntime();
            }
            if (gamepad2.left_bumper && getRuntime() > 0.2){
                ClawPos -= 0.2;
                resetRuntime();
            }
            else if (gamepad2.right_bumper && getRuntime() > 0.2){
                ClawPos += 0.2;
                resetRuntime();
            }
            ClawPos = Range.clip(ClawPos, 0.0, 1.0);//clips after the inputs
            ProngPos = Range.clip(ProngPos, 0.0, 1.0);
            RightProng.setPosition(ProngPos);//have to put it after the gamepad input so the position is proper
            LeftProng.setPosition(1.0-ProngPos);
            ServoClaw.setPosition(ClawPos);
*/

            telemetry.addData("ClawPos", "%.2f", ClawPos);
            telemetry.addData("RightProngPos", "%.2f", ProngPos);
            telemetry.update();

            sleep(50);
        }
    }
}

