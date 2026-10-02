package org.firstinspires.ftc.teamcode.util;

public class Pose2d {
    // Exact by definition: 1 inch = 2.54 cm
    public static final double METERS_PER_INCH = 0.0254;

    // Always stored in meters
    public double x, y;
    public Rotation2d heading;

    public Pose2d(double x, double y, Rotation2d heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d(double x, double y, double heading) {
        this(x, y, new Rotation2d(heading));
    }

    public Pose2d() {
        this(0, 0, new Rotation2d(0));
    }

    /** Creates a pose from meters (same as the constructor, but explicit about units). */
    public static Pose2d fromMeters(double xMeters, double yMeters, Rotation2d heading) {
        return new Pose2d(xMeters, yMeters, heading);
    }

    /** Creates a pose from inches. Converts to meters first, since Pose2d always stores meters. */
    public static Pose2d fromInches(double xInches, double yInches, Rotation2d heading) {
        return new Pose2d(xInches * METERS_PER_INCH, yInches * METERS_PER_INCH, heading);
    }

    public double getXInches() {
        return x / METERS_PER_INCH;
    }

    public double getYInches() {
        return y / METERS_PER_INCH;
    }

    @Override
    public String toString() {
        return String.format("Pose2d(x: %.2f, y: %.2f, heading: %.2f)", x, y, heading.getDegrees());
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Rotation2d getHeading() {
        return heading;
    }
}
