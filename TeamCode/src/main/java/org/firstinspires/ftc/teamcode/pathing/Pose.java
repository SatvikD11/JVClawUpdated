package org.firstinspires.ftc.teamcode.pathing;

/**
 * Simple class to store robot position (x, y, heading)
 * Used by PedroPath and localization systems
 */
public class Pose {
    public double x;
    public double y;
    public double heading; // in radians

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose() {
        this(0, 0, 0);
    }

    public void set(Pose other) {
        this.x = other.x;
        this.y = other.y;
        this.heading = other.heading;
    }

    public Pose copy() {
        return new Pose(this.x, this.y, this.heading);
    }

    @Override
    public String toString() {
        return String.format("Pose(x=%.2f, y=%.2f, heading=%.2f°)", x, y, Math.toDegrees(heading));
    }
}
