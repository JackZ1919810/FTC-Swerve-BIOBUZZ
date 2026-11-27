package org.firstinspires.ftc.teamcode.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Every loop, sets the intake to whatever power the supplier gives. Never ends on its own.
 * Meant to be a default command (like SwerveDriveCommand), e.g. fed by a gamepad button.
 * leftOnly true = only the left motor runs (normal intake), false = both (shooter feed, outtake).
 */
public class IntakeFollowPower extends Command {
    private final IntakeSubsystem intake;
    private final DoubleSupplier power;
    private final BooleanSupplier leftOnly;

    /** Both motors. */
    public IntakeFollowPower(IntakeSubsystem intake, DoubleSupplier power) {
        this(intake, power, () -> false);
    }

    public IntakeFollowPower(IntakeSubsystem intake, DoubleSupplier power, BooleanSupplier leftOnly) {
        this.intake = intake;
        this.power = power;
        this.leftOnly = leftOnly;
        addRequirements(intake);
    }

    @Override
    public void update() {
        intake.setPower(power.getAsDouble(), leftOnly.getAsBoolean());
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public void stop(boolean interrupted) {
        intake.stop();
    }
}
