package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.SubsystemComponent;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.commands.PIDToPosition;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Rotation2d;

@Autonomous(name = "Rotate 90 deg Auto")
public class RotateAuto extends NextFTCOpMode {
    
    private SwerveDrivetrain drivetrain;

    @Override
    public void onInit() {
        drivetrain = new SwerveDrivetrain(hardwareMap);
        drivetrain.initialize();
        
        // Register the drivetrain so it is initialized and updated by the scheduler
        addComponents(new SubsystemComponent(drivetrain));
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Pose2d targetPose = new Pose2d(12.0 * 0.0254, 0, new Rotation2d(Math.toRadians(90)));

        // Schedule the command to run until completion using default PID constants
        new PIDToPosition(drivetrain, targetPose).schedule();
    }
}
