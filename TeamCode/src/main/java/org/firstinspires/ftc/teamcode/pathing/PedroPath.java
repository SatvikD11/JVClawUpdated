package org.firstinspires.ftc.teamcode.pathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PedroPath {
    private DcMotor FL, FR, BL, BR;
    private Pose currentPose = new Pose(0, 0, 0);
    private ElapsedTime timer = new ElapsedTime();

    private double Kp = 0.01;
    private double Ki = 0.0;
    private double Kd = 0.002;

    private double lastError = 0.0;
    private double integral = 0.0;

    public PedroPath(DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR) {
        this.FL = FL;
        this.FR = FR;
        this.BL = BL;
        this.BR = BR;
    }

    /** Simple forward drive path */
    public void driveForward(double targetDistanceInches, double power) {
        resetEncoders();

        double targetTicks = inchesToTicks(targetDistanceInches);
        while (Math.abs(avgEncoderPosition()) < Math.abs(targetTicks)) {
            double error = targetTicks - avgEncoderPosition();
            double derivative = (error - lastError);
            integral += error;
            double correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

            setDrivePower(power * Math.signum(targetDistanceInches) + correction);

            lastError = error;
        }

        setDrivePower(0);
    }

    public void turn(double degrees, double power) {
        resetEncoders();

        double target = degreesToTicks(degrees);
        while (Math.abs(avgTurnPosition()) < Math.abs(target)) {
            double error = target - avgTurnPosition();
            double derivative = error - lastError;
            integral += error;
            double correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

            setTurnPower(power * Math.signum(degrees) + correction);
            lastError = error;
        }

        setDrivePower(0);
    }

    /** Utility methods */
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

    private double avgTurnPosition() {
        return (FR.getCurrentPosition() + BR.getCurrentPosition()
                - FL.getCurrentPosition() - BL.getCurrentPosition()) / 4.0;
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

    private double inchesToTicks(double inches) {
        double TICKS_PER_REV = 537.7; // GoBilda 312 RPM
        double WHEEL_DIAMETER = 3.78; // in inches
        double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);
        return inches * TICKS_PER_INCH;
    }

    private double degreesToTicks(double degrees) {
        // tune for your robot’s turn geometry
        double ROBOT_CIRCUMFERENCE = 18.0 * Math.PI;
        double TICKS_PER_REV = 537.7;
        double WHEEL_DIAMETER = 3.78;
        double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);
        return (degrees / 360.0) * (ROBOT_CIRCUMFERENCE * TICKS_PER_INCH);
    }
}

