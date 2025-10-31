package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pathing.PedroPath;
import org.firstinspires.ftc.teamcode.helpers;

@TeleOp(name = "Robot: JV Claw", group = "Robot")
public class JVClaw extends LinearOpMode {

    private DcMotor BL, BR, FL, FR;
    private PedroPath pedroBody;

    private double lastButtonTime = 0;

    @Override
    public void runOpMode() {
        // --- Hardware Map ---
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        // --- Set motor directions ---
        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);

        // --- Initialize PedroPath ---
        pedroBody = new PedroPath(this, hardwareMap);

        telemetry.addLine("Robot Ready. Press START.");
        telemetry.update();

        waitForStart();

        // --- Main TeleOp Loop ---
        while (opModeIsActive()) {

            // Basic drive (make sure the helper method name matches your helpers.java)
            helpers.BasicTele(
                    FL, FR, BL, BR,
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    gamepad1.dpad_right,
                    gamepad1.dpad_left,
                    gamepad1.dpad_up,
                    gamepad1.dpad_down,
                    gamepad1.right_bumper && gamepad1.left_bumper || gamepad1.left_stick_button
            );

            // --- Reverse orientation toggle ---
            if (gamepad1.right_stick_button) {
                BL.setDirection(DcMotor.Direction.FORWARD);
                FL.setDirection(DcMotor.Direction.FORWARD);
                BR.setDirection(DcMotor.Direction.REVERSE);
                FR.setDirection(DcMotor.Direction.REVERSE);
            }

            // --- Debounced Controls for Autonomous Snippets ---
            double now = getRuntime();

            // Forward
            if (gamepad1.y && now - lastButtonTime > 0.3) {
                pedroBody.driveForward(11, 0.5);
                lastButtonTime = now;
            }
            // Backward
            else if (gamepad1.b && now - lastButtonTime > 0.3) {
                pedroBody.driveForward(-11, 0.5);
                lastButtonTime = now;
            }
            // Turn Left 90°
            else if (gamepad1.x && now - lastButtonTime > 0.3) {
                pedroBody.turnIMU(-90, 0.5);
                lastButtonTime = now;
            }
            // Turn Right 90°
            else if (gamepad1.a && now - lastButtonTime > 0.3) {
                pedroBody.turnIMU(90, 0.5);
                lastButtonTime = now;
            }

            // --- Telemetry ---
            telemetry.addData("Heading", "%.2f°", pedroBody.getHeading());
            telemetry.addData("Encoder Avg", "%.1f",
                    (FL.getCurrentPosition() + FR.getCurrentPosition() + BL.getCurrentPosition() + BR.getCurrentPosition()) / 4.0);
            telemetry.update();

            idle();
        }
    }
}
