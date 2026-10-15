package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Sets the shooter's target speed and finishes immediately.
 * The shooter keeps that speed until another command changes it, so there is no stop().
 */
public class ShooterSetRPM extends Command {
    private final ShooterSubsystem shooter;
    private final double rpm;

    public ShooterSetRPM(ShooterSubsystem shooter, double rpm) {
        this.shooter = shooter;
        this.rpm = rpm;
        addRequirements(shooter);
    }

    @Override
    public void start() {
        shooter.setTargetRPM(rpm);
    }

    @Override
    public boolean isDone() {
        return true;
    }
}
