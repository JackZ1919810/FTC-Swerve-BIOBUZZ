package org.firstinspires.ftc.teamcode.commands;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Every loop, sets the shooter's target speed to whatever RPM the supplier gives (0 = off).
 * Never ends on its own. Meant to be a default command, e.g. fed by a gamepad button.
 */
public class ShooterFollowRPM extends Command {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier rpm;

    public ShooterFollowRPM(ShooterSubsystem shooter, DoubleSupplier rpm) {
        this.shooter = shooter;
        this.rpm = rpm;
        addRequirements(shooter);
    }

    @Override
    public void update() {
        shooter.setTargetRPM(rpm.getAsDouble());
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public void stop(boolean interrupted) {
        shooter.stop();
    }
}
