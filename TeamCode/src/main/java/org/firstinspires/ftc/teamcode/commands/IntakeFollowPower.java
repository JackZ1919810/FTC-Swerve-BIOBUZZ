package org.firstinspires.ftc.teamcode.commands;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Every loop, sets the intake to whatever power the supplier gives. Never ends on its own.
 * Meant to be a default command (like SwerveDriveCommand), e.g. fed by a gamepad button.
 */
public class IntakeFollowPower extends Command {
    private final IntakeSubsystem intake;
    private final DoubleSupplier power;

    public IntakeFollowPower(IntakeSubsystem intake, DoubleSupplier power) {
        this.intake = intake;
        this.power = power;
        addRequirements(intake);
    }

    @Override
    public void update() {
        intake.setPower(power.getAsDouble());
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
