package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class PeriodicIntake extends Command{
    public final IntakeSubsystem intake;
    public final double power;

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
