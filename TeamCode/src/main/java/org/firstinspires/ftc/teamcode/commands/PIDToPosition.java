package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.util.DrivetrainPIDController;
import org.firstinspires.ftc.teamcode.util.Pose2d;

public class PIDToPosition extends Command {
    private final SwerveDrivetrain drivetrain;
    private final DrivetrainPIDController controller;
    private final Pose2d targetPose;

    public PIDToPosition(SwerveDrivetrain drivetrain, Pose2d targetPose, DrivetrainConstants.DrivetrainPIDConstants constants) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        this.controller = new DrivetrainPIDController(constants);
        addRequirements(drivetrain);
    }

    public PIDToPosition(SwerveDrivetrain drivetrain, Pose2d targetPose) {
        this(drivetrain, targetPose, DrivetrainConstants.autoPIDConstants);
    }

    @Override
    public void start() {
        controller.reset();
    }

    @Override
    public void update() {
        Pose2d currentPose = drivetrain.getPose();
        ChassisSpeeds speeds = controller.calculate(currentPose, targetPose);
        drivetrain.driveFieldCentric(speeds);
    }

    @Override
    public boolean isDone() {
        ChassisSpeeds v = drivetrain.getFieldVelocity();
        return controller.atPosition()
                && Math.hypot(v.vxMetersPerSecond, v.vyMetersPerSecond) < DrivetrainConstants.SETTLE_SPEED
                && Math.abs(v.omegaRadiansPerSecond) < DrivetrainConstants.SETTLE_TURN_RATE;
    }

    @Override
    public void stop(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }
}
