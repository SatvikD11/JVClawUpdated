package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name = "IMU Coordinate Test")
public class IMUCoordinateTest extends LinearOpMode {

    private IMU imu;


    private double x = 0.0;
    private double y = 0.0;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);

        telemetry.addLine("IMU initialized — waiting for start");
        telemetry.update();

        waitForStart();
        imu.resetYaw();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Get current orientation
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double heading = orientation.getYaw(AngleUnit.DEGREES);


            telemetry.addLine("---- Robot Pose ----");
            telemetry.addData("X", "%.2f", x);
            telemetry.addData("Y", "%.2f", y);
            telemetry.addData("Heading", "%.2f°", heading);

            telemetry.update();
        }
    }
}
