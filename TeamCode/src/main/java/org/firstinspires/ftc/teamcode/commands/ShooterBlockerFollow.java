package org.firstinspires.ftc.teamcode.commands;

import java.util.function.BooleanSupplier;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Every loop, opens the shooter's blocker while the supplier says true and closes it otherwise.
 * Never ends on its own. Start it with schedule(), not as a default command.
 *
 * It does NOT require the shooter: ShooterFollowRPM (the shooter's default command) already does, and
 * NextFTC cancels a command when another one needing the same subsystem starts. This one only moves
 * the blocker, never the flywheel, so the two can run side by side.
 */
public class ShooterBlockerFollow extends Command {
    private final ShooterSubsystem shooter;
    private final BooleanSupplier open;

    public ShooterBlockerFollow(ShooterSubsystem shooter, BooleanSupplier open) {
        this.shooter = shooter;
        this.open = open;
    }

    @Override
    public void update() {
        shooter.setServoPos(open.getAsBoolean());
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public void stop(boolean interrupted) {
        shooter.setServoPos(false);   // closed when the command ends
    }
}
