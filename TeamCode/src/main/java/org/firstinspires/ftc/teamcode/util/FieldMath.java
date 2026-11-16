package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/** Field geometry for aiming. Meters and radians inside, like Pose2d. */
public final class FieldMath {
    private FieldMath() {}

    /** A field point given in inches, stored in meters. */
    public static Translation2d inches(double xIn, double yIn) {
        return new Translation2d(xIn * Pose2d.METERS_PER_INCH, yIn * Pose2d.METERS_PER_INCH);
    }

    /**
     * A pose described from the start pose (x forward, y left, as the robot stood at the start)
     * turned into the field frame: rotate the offset by the start heading, then add the start position.
     */
    public static Pose2d fromStart(Pose2d start, Pose2d relative) {
        Translation2d offset = new Translation2d(relative.x, relative.y).rotateBy(start.heading);
        return new Pose2d(start.x + offset.getX(), start.y + offset.getY(),
                start.heading.plus(relative.heading));
    }

    /** Field position of the turret pivot. forward / left: pivot position on the robot, in meters. */
    public static Translation2d turretPivot(Pose2d robot, double forwardM, double leftM) {
        Translation2d offset = new Translation2d(forwardM, leftM).rotateBy(robot.heading);
        return new Translation2d(robot.x, robot.y).plus(offset);
    }

    /** Direction from one field point to another: degrees counterclockwise from field +X. */
    public static double fieldAngleDeg(Translation2d from, Translation2d to) {
        return Math.toDegrees(Math.atan2(to.getY() - from.getY(), to.getX() - from.getX()));
    }

    /** Turret angle that points at the goal: degrees from the robot's front, counterclockwise +. */
    public static double turretAngleToGoalDeg(Pose2d robot, Translation2d pivot, Translation2d goal) {
        return TurretSubsystem.normalizeDeg(fieldAngleDeg(pivot, goal) - robot.heading.getDegrees());
    }

    /**
     * How far off the opening's facing direction the shooter is, 0..180 degrees.
     * 0 = straight in front of the opening, 90 = beside it, 180 = behind it.
     */
    public static double approachAngleDeg(Translation2d opening, double facingDeg, Translation2d shooter) {
        return Math.abs(TurretSubsystem.normalizeDeg(fieldAngleDeg(opening, shooter) - facingDeg));
    }
}
