package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
public class IntakeSetPower extends Command{
    public final IntakeSubsystem intake;
    public final double power;

    public IntakeSetPower(IntakeSubsystem intake, double power){
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
        return true;
    }

    /*
    no public void stop() here because you want the motor to keep at the changed value
    you want to manually change it back after you don't want the change.
     */

}
