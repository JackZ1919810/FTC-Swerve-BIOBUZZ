package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class PeriodicIntake extends Command{
    private final IntakeSubsystem intake;
    private final double power;

    //constructor
    public PeriodicIntake (IntakeSubsystem intake, double power){
        this.intake = intake;
        this.power = power;
        addRequirements(intake);
    }

    @Override
    public void start() {
        intake.setPower(power);
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
