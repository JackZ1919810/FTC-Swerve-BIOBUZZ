package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.PIDToPosition;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Rotation2d;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Red Shoot-First Auto")
public class RedAutoShootFirstV1 extends NextFTCOpMode {

    private SwerveDrivetrain drive;

    @Override
    public void onInit() {
        drive = new SwerveDrivetrain(hardwareMap);
        drive.initialize();

        addComponents(new SubsystemComponent(drive));
    }

    @Override
    public void onStartButtonPressed(){

        new SequentialGroup(
                //1. Shoot
                new Delay(1.0),

                //2. Turn 90 and drive to pick up 52in
                new PIDToPosition(drive, new Pose2d(0,0, Rotation2d.fromDegrees(90))).endAfter(5),
                new PIDToPosition(drive, Pose2d.fromInches(0, 52, Rotation2d.fromDegrees(90))).endAfter(5),

                //3. Move back to shooting pos. and turn to 0 deg.
                new PIDToPosition(drive, new Pose2d(0,0,new Rotation2d(0))).endAfter(5),

                //4. Shoot
                new Delay(1.0),

                //5. Go to parking pos. 96in x+, 52in y+
                new PIDToPosition(drive, Pose2d.fromInches(96, 52, new Rotation2d(0)))
        ).schedule();

    }

}
