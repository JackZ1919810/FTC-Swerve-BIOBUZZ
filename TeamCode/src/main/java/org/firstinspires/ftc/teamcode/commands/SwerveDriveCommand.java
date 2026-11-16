package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import java.util.function.BooleanSupplier;

public class SwerveDriveCommand extends Command {
    private final SwerveDrivetrain drivetrain;
    private final DoubleSupplier vxSupplier, vySupplier, omegaSupplier;
    private final BooleanSupplier fieldCentricSupplier;

    public SwerveDriveCommand(SwerveDrivetrain drivetrain, 
                              DoubleSupplier vxSupplier, 
                              DoubleSupplier vySupplier, 
                              DoubleSupplier omegaSupplier,
                              BooleanSupplier fieldCentricSupplier) {
        this.drivetrain = drivetrain;
        this.vxSupplier = vxSupplier;
        this.vySupplier = vySupplier;
        this.omegaSupplier = omegaSupplier;
        this.fieldCentricSupplier = fieldCentricSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void update() {
        double vx = vxSupplier.getAsDouble() * DrivetrainConstants.MAX_SPEED_MPS;
        double vy = vySupplier.getAsDouble() * DrivetrainConstants.MAX_SPEED_MPS;
        double omega = omegaSupplier.getAsDouble() * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RAD_PER_SEC;

        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
        if (fieldCentricSupplier.getAsBoolean()) {
            drivetrain.driveDriverCentric(speeds);   // forward = the driver's forward, not field +X
        } else {
            drivetrain.drive(speeds);
        }
    }

    @Override
    public boolean isDone() {
        return false;
    }
}
