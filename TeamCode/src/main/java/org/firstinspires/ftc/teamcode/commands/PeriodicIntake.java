package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class PeriodicIntake extends Command{
    private final IntakeSubsystem intake;
    private final double power;
    private final boolean leftOnly;

    //constructor: both motors
    public PeriodicIntake (IntakeSubsystem intake, double power){
        this(intake, power, false);
    }

    // leftOnly true = only the left motor runs (normal intake), false = both (shooter feed, outtake)
    public PeriodicIntake (IntakeSubsystem intake, double power, boolean leftOnly){
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
        return false;
    }

    @Override
    public void stop(boolean interrupted) {
        intake.stop();
    }
}
