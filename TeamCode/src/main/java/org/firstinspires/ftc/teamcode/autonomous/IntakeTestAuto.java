package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.IntakeSetPower;
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;

@Autonomous(name = "Intake Test")
public class IntakeTestAuto extends NextFTCOpMode{
    private IntakeSubsystem intake;

    @Override
    public void onInit(){
        intake = new IntakeSubsystem(hardwareMap);
        intake.initialize();
        addComponents(new SubsystemComponent(intake));
        telemetry.addData("Is Initialized:", true);
        telemetry.update();
    }

    @Override
    public void onUpdate(){
        telemetry.addData("Intake target power", intake.getPower());
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
                new IntakeSetPower(intake, SubsystemConstants.IntakePower),
                new Delay(10.0),
                new IntakeSetPower(intake, 0)
        ).schedule();
    }
}
