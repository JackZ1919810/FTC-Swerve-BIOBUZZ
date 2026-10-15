package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.commands.IndexSetPower;
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;

@Autonomous(name = "Index Test")
public class IndexTestAuto extends NextFTCOpMode{
    private IndexerSubsystem index;

    @Override
    public void onInit(){
        index = new IndexerSubsystem(hardwareMap);
        index.initialize();
        addComponents(new SubsystemComponent(index));
        telemetry.addData("Is Initialized:", true);
        telemetry.update();
    }

    @Override
    public void onUpdate(){
        telemetry.addData("Indexer target power", index.getPower());
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
                new IndexSetPower(index, SubsystemConstants.IndexPower),
                new Delay(10.0),
                new IndexSetPower(index, 0.0)
        ).schedule();
    }
}
