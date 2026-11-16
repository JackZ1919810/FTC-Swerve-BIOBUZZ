package org.firstinspires.ftc.teamcode.util;

/*
 * Carries the robot pose from Autonomous to TeleOp. Static fields live as long as the
 * Robot Controller app does, so they survive switching OpModes (not a restart or power cycle).
 */
public final class PoseStorage {
    private static Pose2d pose = null;
    private static boolean red = true;

    private PoseStorage() {}

    public static void save(Pose2d robotPose, boolean redAlliance) {
        pose = robotPose;
        red = redAlliance;
    }

    public static boolean hasPose() {
        return pose != null;
    }

    /** The saved pose, given out once: an old pose is never reused by mistake. */
    public static Pose2d take() {
        Pose2d saved = pose;
        pose = null;
        return saved;
    }

    /** Alliance of the last Autonomous (red if none ran). */
    public static boolean isRed() {
        return red;
    }
}
