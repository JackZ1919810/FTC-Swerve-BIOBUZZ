package org.firstinspires.ftc.teamcode.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.util.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.util.Path;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Rotation2d;
import org.firstinspires.ftc.teamcode.util.SwerveDriveKinematics;
import org.firstinspires.ftc.teamcode.util.SwerveModuleState;
import org.firstinspires.ftc.teamcode.util.Translation2d;

public class SwerveDrivetrain implements Subsystem {
    private final SwerveModule leftFront = new SwerveModule();
    private final SwerveModule rightFront = new SwerveModule();
    private final SwerveModule leftRear = new SwerveModule();
    private final SwerveModule rightRear = new SwerveModule();
    private final SparkFunOTOS.Pose2D otosPos = new SparkFunOTOS.Pose2D();
    private final SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
    private final SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
    private ChassisSpeeds fieldVelocity = new ChassisSpeeds();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            DrivetrainConstants.LEFT_FRONT_LOCATION,
            DrivetrainConstants.RIGHT_FRONT_LOCATION,
            DrivetrainConstants.LEFT_REAR_LOCATION,
            DrivetrainConstants.RIGHT_REAR_LOCATION
    );

    private final HardwareMap hardwareMap;
    private Command defaultCommand = new NullCommand();
    private SparkFunOTOS otos;
    private boolean isInitialized = false;
    private Pose2d pose = new Pose2d(0,0,0);
    private boolean poseKnown = false;                       // true once setPose put the robot in the field frame
    private Rotation2d driverForward = new Rotation2d(0);    // field heading the driver calls "forward"
    private Path overlayPath;
    private Translation2d overlayTarget;
    private double overlayCrossTrack;

    private SwerveModuleState[] lastDesiredStates = new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
    };

    public SwerveDrivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public Command getDefaultCommand() {
        return defaultCommand;
    }

    public void setDefaultCommand(Command command) {
        this.defaultCommand = command;
    }

    @Override
    public void initialize() {
        leftFront.init(hardwareMap, "motorLF", "servoLF", "encoderLF", DrivetrainConstants.LEFT_FRONT_OFFSET);
        leftFront.setPID(DrivetrainConstants.STEER_P, DrivetrainConstants.STEER_I, DrivetrainConstants.STEER_D);
        rightFront.init(hardwareMap, "motorRF", "servoRF", "encoderRF", DrivetrainConstants.RIGHT_FRONT_OFFSET);
        rightFront.setPID(DrivetrainConstants.STEER_P, DrivetrainConstants.STEER_I, DrivetrainConstants.STEER_D);
        leftRear.init(hardwareMap, "motorLR", "servoLR", "encoderLR", DrivetrainConstants.LEFT_REAR_OFFSET);
        leftRear.setPID(DrivetrainConstants.STEER_P, DrivetrainConstants.STEER_I, DrivetrainConstants.STEER_D);
        rightRear.init(hardwareMap, "motorRR", "servoRR", "encoderRR", DrivetrainConstants.RIGHT_REAR_OFFSET);
        rightRear.setPID(DrivetrainConstants.STEER_P, DrivetrainConstants.STEER_I, DrivetrainConstants.STEER_D);

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        configureOtos();
        isInitialized = true;
    }

    private void configureOtos() {
        otos.setLinearUnit(DistanceUnit.METER);
        otos.setAngularUnit(AngleUnit.RADIANS);

        // Sensor position from the robot center (x forward, y left), so the OTOS reports the robot center
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(
                FieldConstants.OtosForwardIn * Pose2d.METERS_PER_INCH,
                FieldConstants.OtosLeftIn * Pose2d.METERS_PER_INCH, 0);
        otos.setOffset(offset);

        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        otos.calibrateImu();
        otos.resetTracking();
    }

    @Override
    public void periodic() {
        if (!isInitialized) return;

        // One I2C read for position, velocity and acceleration
        otos.getPosVelAcc(otosPos, otosVel, otosAcc);
        pose = new Pose2d(otosPos.x, otosPos.y, new Rotation2d(otosPos.h));
        fieldVelocity = new ChassisSpeeds(otosVel.x, otosVel.y, otosVel.h);

        leftFront.update(DrivetrainConstants.LF_STEER_P,
                DrivetrainConstants.LF_STEER_I, DrivetrainConstants.LF_STEER_D);
        rightFront.update(DrivetrainConstants.RF_STEER_P,
                DrivetrainConstants.RF_STEER_I, DrivetrainConstants.RF_STEER_D);
        leftRear.update(DrivetrainConstants.LR_STEER_P,
                DrivetrainConstants.LR_STEER_I, DrivetrainConstants.LR_STEER_D);
        rightRear.update(DrivetrainConstants.RR_STEER_P,
                DrivetrainConstants.RR_STEER_I, DrivetrainConstants.RR_STEER_D);

        updateDashboard();
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        // Desaturate based on the physical max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.MAX_SPEED_MPS);

        leftFront.setDesiredState(states[0]);
        rightFront.setDesiredState(states[1]);
        leftRear.setDesiredState(states[2]);
        rightRear.setDesiredState(states[3]);

        lastDesiredStates = states;
    }

    /**
     * X-lock: every wheel points at the robot center (an X seen from above), drive speed 0.
     * A push from any direction makes at least two wheels scrub sideways, and turning makes all four scrub,
     * so the robot is hard to move. Call it every loop while the lock is wanted; drive() again to leave it.
     */
    public void lock() {
        SwerveModuleState[] states = new SwerveModuleState[] {
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),    // left front
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),   // right front
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),   // left rear
                new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))     // right rear
        };

        leftFront.setDesiredState(states[0]);
        rightFront.setDesiredState(states[1]);
        leftRear.setDesiredState(states[2]);
        rightRear.setDesiredState(states[3]);

        // With the sticks idle, the kinematics keeps the last headings it gave. Make those the X,
        // so letting go of the lock with the sticks idle leaves the wheels in the X until the driver moves
        kinematics.resetHeadings(states[0].angle, states[1].angle, states[2].angle, states[3].angle);

        lastDesiredStates = states;
    }

    public void driveFieldCentric(ChassisSpeeds fieldSpeeds) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldSpeeds.vxMetersPerSecond,
                fieldSpeeds.vyMetersPerSecond,
                fieldSpeeds.omegaRadiansPerSecond,
                getPose().getHeading()
        ));
    }

    public Pose2d getPose() {
        return pose;
    }
    public void setPose(Pose2d pose){
        otos.setPosition(new SparkFunOTOS.Pose2D(pose.x, pose.y, pose.heading.getRadians()));
        poseKnown = true;
        periodic();
    }

    public void resetPose() {
        if (isInitialized) otos.resetTracking();
        poseKnown = false;
    }

    /** Measured robot velocity from the OTOS (m/s, rad/s). Used for "has it stopped?" checks. */
    public ChassisSpeeds getFieldVelocity() {
        return fieldVelocity;
    }

    /** True once setPose has put the robot in the field frame (and resetPose hasn't undone it). */
    public boolean isPoseKnown() {
        return poseKnown;
    }

    /** Sets the field heading the driver calls "forward" for driver-centric drive. */
    public void setDriverForward(Rotation2d fieldHeading) {
        driverForward = fieldHeading;
    }

    /** "Forward" becomes the way the robot faces now. Replaces resetPose() for TeleOp. */
    public void resetDriverForward() {
        driverForward = getPose().getHeading();
    }

    /** Field-centric drive measured from the driver's forward instead of field +X. */
    public void driveDriverCentric(ChassisSpeeds driverSpeeds) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                driverSpeeds.vxMetersPerSecond,
                driverSpeeds.vyMetersPerSecond,
                driverSpeeds.omegaRadiansPerSecond,
                getPose().getHeading().minus(driverForward)   // robot heading seen from the driver
        ));
    }

    public void setPathOverlay(Path path, Translation2d target, double crossTrackMeters){
        overlayPath = path;
        overlayTarget = target;
        overlayCrossTrack = crossTrackMeters;
    }

    private void updateDashboard() {
        Pose2d pose = getPose();
        SwerveModuleState[] currentStates = new SwerveModuleState[] {
                leftFront.getState(),
                rightFront.getState(),
                leftRear.getState(),
                rightRear.getState()
        };

        TelemetryPacket packet = new TelemetryPacket();

        // AdvantageScope Odometry: [x, y, rotation] (units: meters, radians)
        packet.put("Odometry", String.format(java.util.Locale.US, "[%.3f, %.3f, %.3f]", 
                pose.x, pose.y, pose.heading.getRadians()));

        // AdvantageScope Swerve States: [angle1, velocity1, angle2, velocity2, ...]
        packet.put("RealStates", String.format(java.util.Locale.US, "[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                currentStates[0].angle.getRadians(), currentStates[0].speedMetersPerSecond,
                currentStates[1].angle.getRadians(), currentStates[1].speedMetersPerSecond,
                currentStates[2].angle.getRadians(), currentStates[2].speedMetersPerSecond,
                currentStates[3].angle.getRadians(), currentStates[3].speedMetersPerSecond
        ));

        packet.put("DesiredStates", String.format(java.util.Locale.US, "[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                lastDesiredStates[0].angle.getRadians(), lastDesiredStates[0].speedMetersPerSecond,
                lastDesiredStates[1].angle.getRadians(), lastDesiredStates[1].speedMetersPerSecond,
                lastDesiredStates[2].angle.getRadians(), lastDesiredStates[2].speedMetersPerSecond,
                lastDesiredStates[3].angle.getRadians(), lastDesiredStates[3].speedMetersPerSecond
        ));


        // Draw on Dashboard Field
        Canvas fieldOverlay = packet.fieldOverlay();
        double xInches = pose.x * 39.3701;
        double yInches = pose.y * 39.3701;
        fieldOverlay.setStroke("#3F51B5");
        fieldOverlay.strokeCircle(xInches, yInches, 9);
        fieldOverlay.setStroke("#FF5722");
        double lineX = xInches + Math.cos(pose.heading.getRadians()) * 12;
        double lineY = yInches + Math.sin(pose.heading.getRadians()) * 12;
        fieldOverlay.strokeLine(xInches, yInches, lineX, lineY);

        packet.put("Speed (m/s)", Math.hypot(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond));

        if (overlayPath != null) {
            int n = overlayPath.numSegments() + 1;
            double[] xs = new double[n];
            double[] ys = new double[n];
            for (int i = 0; i < n; i++) {
                xs[i] = overlayPath.xy(i).getX() / Pose2d.METERS_PER_INCH;
                ys[i] = overlayPath.xy(i).getY() / Pose2d.METERS_PER_INCH;
            }
            fieldOverlay.setStroke("#9E9E9E");
            fieldOverlay.strokePolyline(xs, ys);
            packet.put("Path cross-track (in)", overlayCrossTrack / Pose2d.METERS_PER_INCH);
        }
        if (overlayTarget != null) {
            fieldOverlay.setFill("#FF5722");
            fieldOverlay.fillCircle(overlayTarget.getX() / Pose2d.METERS_PER_INCH,
                    overlayTarget.getY() / Pose2d.METERS_PER_INCH, 2);
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
