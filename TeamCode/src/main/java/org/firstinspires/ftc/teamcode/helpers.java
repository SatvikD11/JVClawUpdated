package org.firstinspires.ftc.teamcode;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

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
            boolean up, boolean down, boolean speedController
    ) {
        double y = -straight;
        double x = strafe;
        double rx = turn;

        double power = Math.hypot(x, y);
        double theta = Math.atan2(y, x);

        double sin45 = Math.sin(theta - Math.PI / 4);
        double cos45 = Math.cos(theta - Math.PI / 4);

        double max = Math.max(Math.abs(sin45), Math.abs(cos45));

        double flPower = (power * cos45 / max) + rx;
        double frPower = (power * sin45 / max) - rx;
        double blPower = (power * sin45 / max) + rx;
        double brPower = (power * cos45 / max) - rx;

        double denom = power + Math.abs(rx);
        if (denom > 1.0) {
            flPower /= denom;
            frPower /= denom;
            blPower /= denom;
            brPower /= denom;
        }

        double Max = Math.max(Math.abs(blPower), Math.max(Math.abs(brPower), Math.max(Math.abs(flPower), Math.abs(frPower))));
        if (max > 1.0) {
            flPower /= Max;
            frPower /= Max;
            blPower /= Max;
            brPower /= Max;
        }
        if (speedController){
            flPower = flPower;
            frPower = frPower;
            blPower = blPower;
            brPower = brPower;
        } else{
            flPower = 0.5 * flPower;
            frPower = 0.5 * frPower;
            blPower = 0.5 * blPower;
            brPower = 0.5 * brPower;
        }


        if (strafe_right) {
            setDrivePowers(FL, FR, BL, BR, 0.75, -0.75, -0.75, 0.75);
        } else if (strafe_left) {
            setDrivePowers(FL, FR, BL, BR, -0.75, 0.75, 0.75, -0.75);
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
