package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.commands.ShooterSetRPM;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.commands.IntakeSetPower;
import org.firstinspires.ftc.teamcode.commands.IndexSetPower;
import org.firstinspires.ftc.teamcode.commands.ShooterSetRPM;
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;

@Autonomous (name = "Intake-Shooting Sequence Test")
public class IntakeShootingSequenceAutoTest extends NextFTCOpMode{
    private IndexerSubsystem index;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;

    @Override
    public void onInit(){
        index = new IndexerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        intake.initialize();
        index.initialize();
        shooter.initialize();

        addComponents(new SubsystemComponent(index), new SubsystemComponent(intake), new SubsystemComponent(shooter));

        telemetry.addData("Is Initialized:", true);
        telemetry.update();
    }

    @Override
    public void onUpdate(){
        telemetry.addData("Intake target power", intake.getPower());
        telemetry.addData("Indexer target power", index.getPower());
        telemetry.addData("Shooter target RPM", shooter.getRPM());
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
                new ParallelGroup(
                        new IndexSetPower(index, SubsystemConstants.IndexPower),
                        new IntakeSetPower(intake, SubsystemConstants.IntakePower),
                        new ShooterSetRPM(shooter, SubsystemConstants.defaultRPM)
                        ),
                new Delay(25.0),
                new ParallelGroup(
                        new IndexSetPower(index, 0.0),
                        new IntakeSetPower(intake, 0.0),
                        new ShooterSetRPM(shooter, 0.0)
                        )
        ).schedule();
    }
}
