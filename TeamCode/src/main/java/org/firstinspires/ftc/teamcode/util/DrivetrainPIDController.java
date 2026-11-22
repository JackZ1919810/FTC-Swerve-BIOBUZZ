package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
public class DrivetrainPIDController {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    public DrivetrainPIDController(DrivetrainConstants.DrivetrainPIDConstants constants) {
        this.xController = new PIDController(
                constants.translationCoefficients.kP,
                constants.translationCoefficients.kI,
                constants.translationCoefficients.kD
        );
        this.yController = new PIDController(
                constants.translationCoefficients.kP,
                constants.translationCoefficients.kI,
                constants.translationCoefficients.kD
        );
        this.thetaController = new PIDController(
                constants.rotationCoefficients.kP,
                constants.rotationCoefficients.kI,
                constants.rotationCoefficients.kD
        );

        this.xController.setTolerance(constants.translationTolerances.position, constants.translationTolerances.velocity);
        this.yController.setTolerance(constants.translationTolerances.position, constants.translationTolerances.velocity);
        this.thetaController.setTolerance(constants.rotationTolerances.position, constants.rotationTolerances.velocity);

        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d targetPose) {
        xController.setSetpoint(targetPose.x);
        yController.setSetpoint(targetPose.y);
        thetaController.setSetpoint(targetPose.heading.getRadians());

        double xFeedback = xController.calculate(currentPose.x);
        double yFeedback = yController.calculate(currentPose.y);
        double thetaFeedback = thetaController.calculate(currentPose.heading.getRadians());

        return new ChassisSpeeds(xFeedback, yFeedback, thetaFeedback);
    }

    public boolean atSetpoint() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

    public boolean atPosition() {
        return xController.atPosition() && yController.atPosition() && thetaController.atPosition();
    }

    public void reset() {
        xController.reset();
        yController.reset();
        thetaController.reset();
    }
}
