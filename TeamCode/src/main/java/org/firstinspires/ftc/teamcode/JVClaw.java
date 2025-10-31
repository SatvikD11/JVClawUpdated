package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pathing.PedroPath;
import org.firstinspires.ftc.teamcode.helpers.BasicTele;

@TeleOp(name="Robot: JV Claw", group="Robot")
public class JVClaw extends LinearOpMode {

    private DcMotor BL, BR, FL, FR;
    private PedroPath PedroBody;

    private double lastButtonTime = 0;

    @Override
    public void runOpMode() {
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        // --- Set motor directions ---
        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);

        // --- Initialize pathing ---
        PedroBody = new PedroPath(this, hardwareMap, FL, FR, BL, BR);

        telemetry.addData(">", "Robot Ready. Press START.");
        telemetry.update();

        waitForStart();

        // --- Main TeleOp loop ---
        while (opModeIsActive()) {

            // Basic drive control
            BasicTele.basicTele(
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

            // Reverse orientation toggle
            if (gamepad1.right_stick_button) {
                BL.setDirection(DcMotor.Direction.FORWARD);
                FL.setDirection(DcMotor.Direction.FORWARD);
                BR.setDirection(DcMotor.Direction.REVERSE);
                FR.setDirection(DcMotor.Direction.REVERSE);
            }

            // Debounced controls for autonomous snippets
            double now = getRuntime();

            if (gamepad1.triangle && now - lastButtonTime > 0.3) {
                PedroBody.driveForward(11, 0.5);
                lastButtonTime = now;
            }
            else if (gamepad1.circle && now - lastButtonTime > 0.3) {
                PedroBody.driveForward(11, -0.5);
                lastButtonTime = now;
            }
            else if (gamepad1.square && now - lastButtonTime > 0.3) {
                PedroBody.turnIMU(-90, 1);
                lastButtonTime = now;
            }
            else if (gamepad1.cross && now - lastButtonTime > 0.3) {
                PedroBody.turnIMU(90, 1);
                lastButtonTime = now;
            }

            telemetry.addData("Heading", "%.2f", PedroBody.getHeading());
            telemetry.update();

            idle();
        }
    }
}
