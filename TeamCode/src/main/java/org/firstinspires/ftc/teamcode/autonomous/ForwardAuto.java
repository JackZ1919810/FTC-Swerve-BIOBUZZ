package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.SubsystemComponent;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.commands.PIDToPosition;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Rotation2d;

@Autonomous(name = "Forward 12 Inch Auto")
public class ForwardAuto extends NextFTCOpMode {
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
        // Target: 12 inches forward (positive X)
        // 12 inches = 0.3048 meters
        double targetX = 12.0 * 0.0254; 
        Pose2d targetPose = new Pose2d(targetX, 0, new Rotation2d(0));

        // Schedule the command to run until completion using default PID constants
        new PIDToPosition(drivetrain, targetPose).schedule();
    }

    @Override
    public void onUpdate() {
        telemetry.addData("x pos", drivetrain.getPose().x);
    }
}
