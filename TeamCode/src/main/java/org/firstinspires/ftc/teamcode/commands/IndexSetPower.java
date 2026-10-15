package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
public class IndexSetPower extends Command {
    private final IndexerSubsystem index;
    private final double power;

    public IndexSetPower(IndexerSubsystem index, double power){
        this.index = index;
        this.power = power;
        addRequirements(index);
    }

    @Override
    public void start(){
        index.setPower(power);
    }

    @Override
    public boolean isDone(){
        return true;
    }

}
