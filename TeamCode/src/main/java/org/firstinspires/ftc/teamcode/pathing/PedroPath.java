package org.firstinspires.ftc.teamcode.pathing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class PedroPath {
    private final DcMotor FL, FR, BL, BR;
    private final LinearOpMode opMode;
    private final BNO055IMU imu;
    private Orientation angles;
    private final ElapsedTime timer = new ElapsedTime();

    // --- PID coefficients (used for turning) ---
    private final double Kp = 0.01;
    private final double Ki = 0.0;
    private final double Kd = 0.002;

    private double lastError = 0.0;
    private double integral = 0.0;

    // --- Constants for encoder drive ---
    private static final double TICKS_PER_REV = 560; // GoBilda 312 RPM motor
    private static final double WHEEL_DIAMETER_INCHES = 3.77;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    public PedroPath(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;

        // Initialize drive motors
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        // Set directions — adjust if reversed
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to brake when zero power is applied
        DcMotor[] motors = {FL, FR, BL, BR};
        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Initialize IMU
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);

        timer.reset();
    }

    // --- Drive Forward using Encoders ---
    public void driveForward(double inches, double power) {
        int moveCounts = (int) (inches * TICKS_PER_INCH);

        FL.setTargetPosition(FL.getCurrentPosition() + moveCounts);
        FR.setTargetPosition(FR.getCurrentPosition() + moveCounts);
        BL.setTargetPosition(BL.getCurrentPosition() + moveCounts);
        BR.setTargetPosition(BR.getCurrentPosition() + moveCounts);

        DcMotor[] motors = {FL, FR, BL, BR};
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(Math.abs(power));
        }

        while (opMode.opModeIsActive() && (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) {
            opMode.idle();
        }

        // Stop and reset to normal
        setDrivePower(0);
        for (DcMotor m : motors)
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // --- Turn with IMU Feedback ---
    public void turnIMU(double targetAngle, double maxPower) {
        double startAngle = getHeading();
        double target = startAngle + targetAngle;
        double error;

        integral = 0;
        lastError = 0;

        while (opMode.opModeIsActive() && Math.abs(error = angleError(target, getHeading())) > 1) {
            double derivative = error - lastError;
            integral += error;
            double correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

            double turnPower = clip(correction, -maxPower, maxPower);
            setTurnPower(turnPower);

            lastError = error;
        }

        setDrivePower(0);
    }

    // --- Utility Functions ---
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

    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
