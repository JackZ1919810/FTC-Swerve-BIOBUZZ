package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commands.PeriodicIntake;
import org.firstinspires.ftc.teamcode.commands.IntakeSetPower;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

@Autonomous(name = "Intake Test")
public class IntakeTestAuto extends NextFTCOpMode{
    private IntakeSubsystem intake;

    @Override
    public void onInit(){
        intake = new IntakeSubsystem(hardwareMap);
        intake.initialize();
        addComponents(new SubsystemComponent(intake));
        telemetry.addData("Is Initialized:", true);
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
                new PeriodicIntake(intake, IntakeConstants.IntakePower),
                new Delay(1.0),
                new IntakeSetPower(intake,0)
        );
    }
}
