package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class helpers {

    public static void setDrivePowers(
            DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR,
            double fl, double fr, double bl, double br
    ) {
        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);
    }

    public static void BasicTele(
            DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR,
            double straight, double strafe, double turn,
            boolean strafe_right, boolean strafe_left,
            boolean up, boolean down
    ) {
        double y = -straight;
        double x = strafe;
        double rx = turn;

        double flPower = y + x + rx;
        double frPower = y - x - rx;
        double blPower = y - x + rx;
        double brPower = y + x - rx;

        double max = Math.max(Math.abs(blPower), Math.max(Math.abs(brPower), Math.max(Math.abs(flPower), Math.abs(frPower))));
        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        if (strafe_right) {
            setDrivePowers(FL, FR, BL, BR, -0.75, 0.75, 0.75, -0.75);
        } else if (strafe_left) {
            setDrivePowers(FL, FR, BL, BR, 0.75, -0.75, -0.75, 0.75);
        } else if (up) {
            setDrivePowers(FL, FR, BL, BR, 0.75, 0.75, 0.75, 0.75);
        } else if (down) {
            setDrivePowers(FL, FR, BL, BR, -0.75, -0.75, -0.75, -0.75);
        } else {
            setDrivePowers(
                    FL, FR, BL, BR,
                    Math.pow(flPower, 3),
                    Math.pow(frPower, 3),
                    Math.pow(blPower, 3),
                    Math.pow(brPower, 3)
            );
        }
    }
}
