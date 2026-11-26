package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.Pose2d;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Pure pursuit test")
public class PurePursuitTest extends NextFTCOpMode{
    private SwerveDrivetrain swerve;

    @Override
    public void onInit(){
        swerve = new SwerveDrivetrain(hardwareMap);
        swerve.initialize();
        addComponents(new SubsystemComponent(swerve), BulkReadComponent.INSTANCE);
    }

    @Override
    public void onStartButtonPressed(){
        new FollowPath(swerve, DrivetrainConstants.PATH_FAST,
                new Pose2d(1.0,0.0,0),
                new Pose2d(0.3,0.7, 0)
        ).endAfter(8).schedule();
    }
}
