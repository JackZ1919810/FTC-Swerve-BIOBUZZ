package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.FlowerIntakeSubsystem;

/**
 * Opens the flower intake if it's closed, closes it if it's open, and finishes immediately.
 * The servo stays there until the next command. Meant for a button press in TeleOp.
 */
public class FlowerToggle extends Command {
    private final FlowerIntakeSubsystem flower;

    public FlowerToggle(FlowerIntakeSubsystem flower) {
        this.flower = flower;
        addRequirements(flower);
    }

    @Override
    public void start() {
        flower.toggle();
    }

    @Override
    public boolean isDone() {
        return true;
    }
}
