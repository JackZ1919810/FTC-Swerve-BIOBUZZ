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
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.util.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Rotation2d;
import org.firstinspires.ftc.teamcode.util.SwerveDriveKinematics;
import org.firstinspires.ftc.teamcode.util.SwerveModuleState;

public class SwerveDrivetrain implements Subsystem {
    private final SwerveModule leftFront = new SwerveModule();
    private final SwerveModule rightFront = new SwerveModule();
    private final SwerveModule leftRear = new SwerveModule();
    private final SwerveModule rightRear = new SwerveModule();

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

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        otos.calibrateImu();
        otos.resetTracking();
    }

    @Override
    public void periodic() {
        if (!isInitialized) return;

        SparkFunOTOS.Pose2D otosPose = otos.getPosition();
        pose = new Pose2d(otosPose.x, otosPose.y, new Rotation2d(otosPose.h));

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
        periodic();
    }

    public void resetPose() {
        if (isInitialized) otos.resetTracking();
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

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
