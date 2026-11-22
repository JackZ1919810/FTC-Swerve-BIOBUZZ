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
 */
public class FollowPath extends Command {
    private final SwerveDrivetrain drivetrain;
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

    public FollowPath(SwerveDrivetrain drivetrain, Pose2d... waypoints) {
        this.drivetrain = drivetrain;
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
            target = path.pointAt(s + DrivetrainConstants.PATH_LOOKAHEAD);
            Translation2d toTarget = target.minus(robot);
            double dist = toTarget.getNorm();

            // 3. Cruise speed, but never faster than we can still stop from
            double speed = Math.min(DrivetrainConstants.PATH_MAX_SPEED,
                    Math.sqrt(2 * DrivetrainConstants.PATH_MAX_ACCEL * remaining));
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
            double cap = Math.sqrt(2 * DrivetrainConstants.PATH_MAX_ACCEL * robot.getDistance(target));
            double mag = Math.hypot(pid.vxMetersPerSecond, pid.vyMetersPerSecond);
            double scale = mag > cap ? cap / mag : 1.0;
            vx = pid.vxMetersPerSecond * scale;
            vy = pid.vyMetersPerSecond * scale;
            omega = pid.omegaRadiansPerSecond;
        }

        // 6. Limit acceleration as a vector: smooth start, smooth corners, no jump at handoff
        double dvx = vx - lastVx;
        double dvy = vy - lastVy;
        double dv = Math.hypot(dvx, dvy);
        double maxDv = DrivetrainConstants.PATH_MAX_ACCEL * dt;
        if (dv > maxDv) {
            dvx *= maxDv / dv;
            dvy *= maxDv / dv;
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
}
