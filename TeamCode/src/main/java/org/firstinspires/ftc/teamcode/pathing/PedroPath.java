package org.firstinspires.ftc.teamcode.pathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PedroPath {
    private final DcMotor FL, FR, BL, BR;
    private final LinearOpMode opMode;
    private final BNO055IMU imu;
    private Pose currentPose = new Pose(0, 0, 0);
    private Orientation angles;
    private final ElapsedTime timer = new ElapsedTime();

    // PID coefficients
    private final double Kp = 0.01;
    private final double Ki = 0.0;
    private final double Kd = 0.002;

    private double lastError = 0.0;
    private double integral = 0.0;

    public PedroPath(LinearOpMode opMode, HardwareMap hardwareMap,
                     DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR) {
        this.opMode = opMode;
        this.FL = FL;
        this.FR = FR;
        this.BL = BL;
        this.BR = BR;

        // Initialize IMU
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        timer.reset();
    }

    /** Drive straight for a given distance in inches */
    public void driveForward(double targetDistanceInches, double power) {
        resetEncoders();
        double targetTicks = inchesToTicks(targetDistanceInches);

        while (opMode.opModeIsActive() &&
                Math.abs(avgEncoderPosition()) < Math.abs(targetTicks)) {

            double error = targetTicks - avgEncoderPosition();
            double derivative = error - lastError;
            integral += error;
            double correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

            double drivePower = RangeClip(power * Math.signum(targetDistanceInches) + correction, -1.0, 1.0);
            setDrivePower(drivePower);

            lastError = error;
            updatePose();
        }

        setDrivePower(0);
    }

    /** Turn a specific angle using IMU feedback */
    public void turnIMU(double targetAngle, double power) {
        double initialHeading = getHeading();
        double desiredHeading = initialHeading + targetAngle;
        double error;

        while (opMode.opModeIsActive() &&
                Math.abs(error = angleError(desiredHeading, getHeading())) > 1.0) {

            double derivative = error - lastError;
            integral += error;
            double correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

            double turnPower = RangeClip(power * Math.signum(error) + correction, -1.0, 1.0);
            setTurnPower(turnPower);

            lastError = error;
        }

        setDrivePower(0);
    }

    // --- Utility methods below ---

    private void resetEncoders() {
        DcMotor[] motors = {FL, FR, BL, BR};
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private double avgEncoderPosition() {
        return (FL.getCurrentPosition() + FR.getCurrentPosition() +
                BL.getCurrentPosition() + BR.getCurrentPosition()) / 4.0;
    }

    private void setDrivePower(double p) {
        FL.setPower(p);
        FR.setPower(p);
        BL.setPower(p);
        BR.setPower(p);
    }

    private void setTurnPower(double p) {
        FL.setPower(-p);
        FR.setPower(p);
        BL.setPower(-p);
        BR.setPower(p);
    }

    public double getHeading() {
        angles = imu.getAngularOrientation();
        return AngleUnit.DEGREES.normalize(angles.firstAngle);
    }

    private double angleError(double target, double current) {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff <= -180) diff += 360;
        return diff;
    }

    private double inchesToTicks(double inches) {
        double TICKS_PER_REV = 560; // Example: REV HD Hex motor
        double WHEEL_DIAMETER = 3.77;
        double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);
        return inches * TICKS_PER_INCH;
    }

    private void updatePose() {
        double x = currentPose.x;
        double y = currentPose.y;
        double heading = getHeading();
        currentPose = new Pose(x, y, heading);
    }

    public Pose getPose() {
        updatePose();
        return currentPose;
    }

    private double RangeClip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
