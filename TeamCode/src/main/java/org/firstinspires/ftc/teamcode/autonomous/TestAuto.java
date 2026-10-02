package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.PIDToPosition;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Rotation2d;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous
public class TestAuto extends NextFTCOpMode {

    private SwerveDrivetrain drive;

    @Override
    public void onInit() {
        drive = new SwerveDrivetrain(hardwareMap);
        drive.initialize();

        addComponents(new SubsystemComponent(drive));
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
                new PIDToPosition(drive, new Pose2d(0.3, 0.0, 0)),
            new PIDToPosition(drive, new Pose2d(0.3, 0.3, new Rotation2d(3.14)))
        ).schedule();

    }
}
