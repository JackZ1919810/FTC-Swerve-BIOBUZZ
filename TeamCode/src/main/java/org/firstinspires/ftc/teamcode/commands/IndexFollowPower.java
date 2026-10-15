package org.firstinspires.ftc.teamcode.commands;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;

/**
 * Every loop, sets the indexer to whatever power the supplier gives. Never ends on its own.
 * Meant to be a default command (like SwerveDriveCommand), e.g. fed by gamepad buttons.
 */
public class IndexFollowPower extends Command {
    private final IndexerSubsystem indexer;
    private final DoubleSupplier power;

    public IndexFollowPower(IndexerSubsystem indexer, DoubleSupplier power) {
        this.indexer = indexer;
        this.power = power;
        addRequirements(indexer);
    }

    @Override
    public void update() {
        indexer.setPower(power.getAsDouble());
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public void stop(boolean interrupted) {
        indexer.stop();
    }
}
