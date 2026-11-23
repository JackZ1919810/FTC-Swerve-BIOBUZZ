package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.FlowerIntakeSubsystem;

/**
 * Opens (true) or closes (false) the flower intake and finishes immediately.
 * The servo stays there until the next command. Meant for Autonomous, where the state
 * should be said outright instead of depending on what it was before (like FlowerToggle does).
 */
public class FlowerSetOpen extends Command {
    private final FlowerIntakeSubsystem flower;
    private final boolean open;

    public FlowerSetOpen(FlowerIntakeSubsystem flower, boolean open) {
        this.flower = flower;
        this.open = open;
        addRequirements(flower);
    }

    @Override
    public void start() {
        flower.setOpen(open);
    }

    @Override
    public boolean isDone() {
        return true;
    }
}
