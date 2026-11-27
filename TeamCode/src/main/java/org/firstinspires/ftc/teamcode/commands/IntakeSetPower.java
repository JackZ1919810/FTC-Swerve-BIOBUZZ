package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
public class IntakeSetPower extends Command{
    private final IntakeSubsystem intake;
    private final double power;
    private final boolean leftOnly;

    /** Both motors. */
    public IntakeSetPower(IntakeSubsystem intake, double power){
        this(intake, power, false);
    }

    /** leftOnly true = only the left motor runs (normal intake), false = both (shooter feed, outtake). */
    public IntakeSetPower(IntakeSubsystem intake, double power, boolean leftOnly){
        this.intake = intake;
        this.power = power;
        this.leftOnly = leftOnly;
        addRequirements(intake);
    }

    @Override
    public void start() {
        intake.setPower(power, leftOnly);
    }

    @Override
    public boolean isDone() {
        return true;
    }

    /*
    no public void stop() here because you want the motor to keep at the changed value
    you want to manually change it back after you don't want the change.
     */

}
