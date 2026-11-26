package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.PIDToPosition;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.FieldMath;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.Rotation2d;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Red Shoot-First Auto")
public class RedAutoShootFirstV1 extends NextFTCOpMode {

    private SwerveDrivetrain drive;
    private TurretSubsystem turret;
    private Pose2d start;

    @Override
    public void onInit() {
        drive = new SwerveDrivetrain(hardwareMap);
        drive.initialize();
        start = FieldConstants.startPose(true);
        drive.setPose(start);              // the robot is in the field frame from here on

        // The turret sends nothing during INIT: it keeps reading where it was pointed by hand
        turret = new TurretSubsystem(hardwareMap);
        turret.initialize();

        addComponents(new SubsystemComponent(drive, turret), BulkReadComponent.INSTANCE);
    }

    @Override
    public void onWaitForStart() {
        telemetry.addData("Turret zero", TurretSubsystem.getZeroStatus());
        telemetry.addData("Center / trim", "%.4f / %.4f", TurretSubsystem.centerPos(), TurretSubsystem.rightTrim());
        telemetry.addData("Reach", "%.0f to %.0f deg  %s", TurretSubsystem.minDeg(), TurretSubsystem.maxDeg(),
                TurretSubsystem.reachesEveryDirection() ? "OK" : "SHORT: some directions unreachable");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed(){
        turret.center();   // first turret command: freezes the hand-set zero and holds the turret forward

        new SequentialGroup(

        ).schedule();

    }

    @Override
    public void onStop() {
        PoseStorage.save(drive.getPose(), true);   // TeleOp continues from here
    }

    /** A target written the old way (inches from the start: x forward, y left), in field coordinates. */
    private Pose2d fromStart(double xIn, double yIn, double headingDeg) {
        return FieldMath.fromStart(start, Pose2d.fromInches(xIn, yIn, Rotation2d.fromDegrees(headingDeg)));
    }
}
