package org.firstinspires.ftc.teamcode.pathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PedroPath {
    private DcMotor FL, FR, BL, BR;
    private Pose currentPose = new Pose(0, 0, 0);
    private ElapsedTime timer = new ElapsedTime();

    private BNO055IMU imu;
    private Orientation angles;

    // PID coefficients
    private double Kp = 0.01;
    private double Ki = 0.0;
    private double Kd = 0.002;

    private double lastError = 0.0;
    private double integral = 0.0;

    public PedroPath(HardwareMap hardwareMap, DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR) {
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

    /** Simple forward drive path */
    public void driveForward(double targetDistanceInches, double power) {
        resetEncoders();
        double targetTicks = inchesToTicks(targetDistanceInches);

        while (Math.abs(avgEncoderPosition()) < Math.abs(targetTicks)) {
            double error = targetTicks - avgEncoderPosition();
            double derivative = error - lastError;
            integral += error;
            double correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

            setDrivePower(power * Math.signum(targetDistanceInches) + correction);
            lastError = error;
            updatePose();
        }

        setDrivePower(0);
    }

    /** Turn using IMU heading feedback */
    public void turnIMU(double targetAngle, double power) {
        double initialHeading = getHeading();
        double desiredHeading = initialHeading + targetAngle;
        double error, correction;

        while (Math.abs(error = angleError(desiredHeading, getHeading())) > 1.0) {
            double derivative = error - lastError;
            integral += error;
            correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
            setTurnPower(power * Math.signum(error) + correction);
            lastError = error;
        }

        setDrivePower(0);
    }

    /** Utility Methods */

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

    /** Get IMU heading in degrees (-180 to 180) */
    public double getHeading() {
        angles = imu.getAngularOrientation();
        return AngleUnit.DEGREES.normalize(angles.firstAngle);
    }

    /** Compute shortest angular difference */
    private double angleError(double target, double current) {
        double diff = target - current;
        while (diff > 180) diff -= 360;
        while (diff <= -180) diff += 360;
        return diff;
    }

    /** Convert inches to encoder ticks */
    private double inchesToTicks(double inches) {
        double TICKS_PER_REV = 560; // REV HD HEX 20:1
        double WHEEL_DIAMETER = 3.77; // inches
        double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);
        return inches * TICKS_PER_INCH;
    }

    /** Update currentPose (basic dead-reckoning) */
    public void updatePose() {
        // Simple placeholder pose update (encoder based)
        double x = currentPose.x;
        double y = currentPose.y;
        double heading = getHeading();
        currentPose = new Pose(x, y, heading);
    }

    public Pose getPose() {
        updatePose();
        return currentPose;
    }
}


