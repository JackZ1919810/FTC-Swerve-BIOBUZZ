package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.util.DrivetrainPIDController;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Path;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Translation2d;

/**
 * Pure pursuit: drives through every waypoint without stopping, then settles on the last one.
 * While more than PATH_HANDOFF of the path is left, it aims at a point PATH_LOOKAHEAD ahead
 * along the path. Inside PATH_HANDOFF, the normal drivetrain PID takes over for the final stop.
 * Speed, lookahead and accel come from the PathConstraints passed in (e.g. DrivetrainConstants.PATH_FAST),
 * or from the global PATH_ values when none is given.
 */
public class FollowPath extends Command {
    // Below this commanded speed (m/s) there's no direction to split the accel limit along
    private static final double DIRECTION_MIN_SPEED = 0.05;

    private final SwerveDrivetrain drivetrain;
    private final DrivetrainConstants.PathConstraints constraints;   // null = the global PATH_ values
    private final Pose2d[] waypoints;
    private final DrivetrainPIDController finalPID =
            new DrivetrainPIDController(DrivetrainConstants.autoPIDConstants);
    private final PIDController headingPID;

    private Path path;
    private double s;              // how far along the path we are (m); never decreases
    private int segment;           // which segment s is on
    private boolean finalPhase;    // true once the final PID has taken over
    private double lastVx, lastVy; // last commanded field velocity, for the accel limit
    private long lastNanos;

    /** Uses the global PATH_ values. */
    public FollowPath(SwerveDrivetrain drivetrain, Pose2d... waypoints) {
        this(drivetrain, (DrivetrainConstants.PathConstraints) null, waypoints);
    }

    /** Uses this path's own limits, e.g. DrivetrainConstants.PATH_FAST. */
    public FollowPath(SwerveDrivetrain drivetrain, DrivetrainConstants.PathConstraints constraints,
                      Pose2d... waypoints) {
        this.drivetrain = drivetrain;
        this.constraints = constraints;
        this.waypoints = waypoints;
        DrivetrainConstants.PIDCoefficients rot = DrivetrainConstants.autoPIDConstants.rotationCoefficients;
        this.headingPID = new PIDController(rot.kP, rot.kI, rot.kD);
        this.headingPID.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);
    }

    @Override
    public void start() {
        // Waypoint 0 is where the robot really is, not where the last command aimed
        Pose2d[] all = new Pose2d[waypoints.length + 1];
        all[0] = drivetrain.getPose();
        System.arraycopy(waypoints, 0, all, 1, waypoints.length);
        path = new Path(all);

        s = 0;
        segment = 0;
        finalPhase = path.numSegments() == 0;   // already sitting on the only waypoint
        lastVx = 0;
        lastVy = 0;
        lastNanos = System.nanoTime();
        finalPID.reset();
        headingPID.reset();
    }

    @Override
    public void update() {
        long now = System.nanoTime();
        double dt = Math.min((now - lastNanos) / 1e9, 0.05);   // cap after a slow loop
        lastNanos = now;

        Pose2d pose = drivetrain.getPose();
        Translation2d robot = new Translation2d(pose.x, pose.y);

        // 1. Where am I on the path? s only moves forward.
        Path.Projection pr = path.projection(robot, segment);
        if (pr.s > s) {
            s = pr.s;
            segment = pr.segment;
        }
        double remaining = path.getTotalLength() - s;
        if (remaining <= DrivetrainConstants.PATH_HANDOFF) finalPhase = true;

        double vx, vy, omega;
        Translation2d target;
        if (!finalPhase) {
            // 2. Aim at the lookahead point
            target = path.pointAt(s + lookahead());
            Translation2d toTarget = target.minus(robot);
            double dist = toTarget.getNorm();

            // 3. Cruise speed, but never faster than we can still stop from
            double speed = Math.min(maxSpeed(), Math.sqrt(2 * maxAccel() * remaining));
            vx = dist > 1e-6 ? toTarget.getX() / dist * speed : 0;
            vy = dist > 1e-6 ? toTarget.getY() / dist * speed : 0;

            // 4. Heading follows the waypoint headings along the path
            headingPID.setSetpoint(path.headingAt(s));
            omega = headingPID.calculate(pose.heading.getRadians());
        } else {
            // 5. Final approach: the existing PID, capped so it never speeds up at the handoff
            Pose2d end = path.getEnd();
            target = new Translation2d(end.x, end.y);
            ChassisSpeeds pid = finalPID.calculate(pose, end);
            double cap = Math.sqrt(2 * maxAccel() * robot.getDistance(target));
            double mag = Math.hypot(pid.vxMetersPerSecond, pid.vyMetersPerSecond);
            double scale = mag > cap ? cap / mag : 1.0;
            vx = pid.vxMetersPerSecond * scale;
            vy = pid.vyMetersPerSecond * scale;
            omega = pid.omegaRadiansPerSecond;
        }

        // 6. Limit acceleration: smooth start, smooth corners, no jump at handoff
        double dvx = vx - lastVx;
        double dvy = vy - lastVy;
        double lastSpeed = Math.hypot(lastVx, lastVy);
        if (lastSpeed > DIRECTION_MIN_SPEED) {
            // Split the change into along the current velocity (speed) and sideways (direction).
            // Separate limits, so speeding up after a corner doesn't use up the budget for turning
            double ux = lastVx / lastSpeed;
            double uy = lastVy / lastSpeed;
            double maxAlong = maxAccel() * dt;
            double maxSide = turnAccel() * dt;
            double along = clip(dvx * ux + dvy * uy, maxAlong);
            double side = clip(-dvx * uy + dvy * ux, maxSide);
            dvx = along * ux - side * uy;
            dvy = along * uy + side * ux;
        } else {
            // Nearly stopped: no direction yet, so limit the whole change
            double dv = Math.hypot(dvx, dvy);
            double maxDv = maxAccel() * dt;
            if (dv > maxDv) {
                dvx *= maxDv / dv;
                dvy *= maxDv / dv;
            }
        }
        lastVx += dvx;
        lastVy += dvy;

        drivetrain.driveFieldCentric(new ChassisSpeeds(lastVx, lastVy, omega));
        drivetrain.setPathOverlay(path, target, pr.distance);   // draw on the Dashboard field
    }

    @Override
    public boolean isDone() {
        if (!finalPhase) return false;
        // finalPID.calculate() already ran this loop: NextFTC calls update() before isDone()
        ChassisSpeeds v = drivetrain.getFieldVelocity();
        return finalPID.atPosition()
                && Math.hypot(v.vxMetersPerSecond, v.vyMetersPerSecond) < DrivetrainConstants.SETTLE_SPEED
                && Math.abs(v.omegaRadiansPerSecond) < DrivetrainConstants.SETTLE_TURN_RATE;
    }

    @Override
    public void stop(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
        drivetrain.setPathOverlay(null, null, 0);   // clear the drawing
    }

    // This path's limits, read every loop so Dashboard changes apply right away

    private double maxSpeed() {
        return constraints != null ? constraints.maxSpeed : DrivetrainConstants.PATH_MAX_SPEED;
    }

    private double lookahead() {
        return constraints != null ? constraints.lookahead : DrivetrainConstants.PATH_LOOKAHEAD;
    }

    private double maxAccel() {
        return constraints != null ? constraints.maxAccel : DrivetrainConstants.PATH_MAX_ACCEL;
    }

    private double turnAccel() {
        return constraints != null ? constraints.turnAccel : DrivetrainConstants.PATH_TURN_ACCEL;
    }

    /** value limited to -max..max */
    private static double clip(double value, double max) {
        return Math.max(-max, Math.min(max, value));
    }
}
