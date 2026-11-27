package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.FlowerSetOpen;
import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.commands.IntakeFollowPower;
import org.firstinspires.ftc.teamcode.commands.IntakeSetPower;
import org.firstinspires.ftc.teamcode.commands.PIDToPosition;
import org.firstinspires.ftc.teamcode.commands.PeriodicIntake;
import org.firstinspires.ftc.teamcode.commands.ShooterBlockerFollow;
import org.firstinspires.ftc.teamcode.commands.ShooterSetRPM;
import org.firstinspires.ftc.teamcode.commands.TurretSetPosition;
import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.FlowerIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.FieldMath;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.Rotation2d;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Red Shoot-First Auto-Without Safety")
public class RedAutoShootFirstConservative extends NextFTCOpMode {

    private SwerveDrivetrain drive;
    private TurretSubsystem turret;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private FlowerIntakeSubsystem flower;
    private Pose2d start;

    private double Pos1RPM = 3400;
    private double Pos2RPM = 3500;

    @Override
    public void onInit() {
        drive = new SwerveDrivetrain(hardwareMap);
        drive.initialize();
        start = FieldConstants.startPose(true);
        drive.setPose(start);              // the robot is in the field frame from here on

        // The turret sends nothing during INIT: it keeps reading where it was pointed by hand
        turret = new TurretSubsystem(hardwareMap);
        turret.initialize();

        // Nothing moves during INIT: the intake and shooter sit at 0, the blocker and flower get no signal
        intake = new IntakeSubsystem(hardwareMap);
        intake.initialize();
        shooter = new ShooterSubsystem(hardwareMap);
        shooter.initialize();
        flower = new FlowerIntakeSubsystem(hardwareMap);
        flower.initialize();

        addComponents(new SubsystemComponent(drive, turret, intake, shooter, flower), BulkReadComponent.INSTANCE);
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

        // Known state from START, same as TeleOp: blocker closed (balls can't reach the flywheel), flower closed
        shooter.setServoPos(false);
        flower.setOpen(false);

        new SequentialGroup(

                turretSetPos(-90),
                shoot(2.0, Pos1RPM),

                //turn on the intake at the start of this
                new IntakeSetPower(intake, SubsystemConstants.IntakePower, true),   // left motor only, like TeleOp R2
                new FollowPath(drive, DrivetrainConstants.PATH_FAST,
                        Pose2d.fromInches(65.75,-64.75, Rotation2d.fromDegrees(-90))
                ),

                //turn off intake after 0.5 second this ended
                // (the next path starts right away; the intake stops 0.5 s into it)
                new ParallelGroup(
                        new IntakeSetPower(intake, 0.0).afterTime(0.5),
                        new FollowPath(drive, DrivetrainConstants.PATH_FAST,
                                Pose2d.fromInches(24, -48, Rotation2d.fromDegrees(180)),
                                Pose2d.fromInches(-24, -48, Rotation2d.fromDegrees(180)),
                                Pose2d.fromInches(-53.5, -24.5, Rotation2d.fromDegrees(180))
                        )
                ),

                //flower intake down
                new FlowerSetOpen(flower, true),

                turretSetPos(-90),

                new FollowPath(drive, DrivetrainConstants.PATH_SLOW,
                        Pose2d.fromInches(-58.5, -24.5, Rotation2d.fromDegrees(180))
                ).endAfter(1),

                //shooter on, shoot for 2s, off shooter
                shoot(2.0, Pos2RPM),
                //intake on for 1s
                intakeFor(1.0),
                //shooter on and shoot for 2s, shooter off
                shoot(2.0, Pos2RPM),

                new PIDToPosition(drive,
                        Pose2d.fromInches(-53.5,-24.5, Rotation2d.fromDegrees(180))
                ).endAfter(1),

                new FollowPath(drive, DrivetrainConstants.PATH_FAST,
                        Pose2d.fromInches(-44, -60, Rotation2d.fromDegrees(180))
                )

        ).schedule();

    }

    @Override
    public void onStop() {
        PoseStorage.save(drive.getPose(), true);   // TeleOp continues from here
    }

    // ----- Actions used in the sequence above -----

    /**
     * Shooter on, shoot for `seconds`, shooter off. Waits for the flywheel to reach speed first
     * (at most AutoSpinUpTimeout, then feeds anyway). Feeding is what TeleOp's R1 + Triangle do:
     * blocker open and the intake at the slow feed power, scaled by battery voltage.
     */
    private Command shoot(double seconds, double RPM) {
        return new SequentialGroup(
                new ShooterSetRPM(shooter, RPM),
                new WaitUntil(() -> shooter.atSpeed()).endAfter(SubsystemConstants.AutoSpinUpTimeout),
                new ParallelGroup(
                        new ShooterBlockerFollow(shooter, () -> true),   // closes the blocker when it ends
                        new IntakeFollowPower(intake,
                                () -> SubsystemConstants.SlowIntakePower * shooter.getVoltageScale())   // stops the intake when it ends
                ).endAfter(seconds),
                new ShooterSetRPM(shooter, 0.0)
        );
    }

    /** Normal intake (left motor only, like TeleOp R2) at full power for `seconds`, then off. */
    private Command intakeFor(double seconds) {
        return new PeriodicIntake(intake, SubsystemConstants.IntakePower, true).endAfter(seconds);
    }

    /** Turret to `deg` (degrees from the robot's front, closest reachable way), finishes immediately. */
    private Command turretSetPos(double deg){
        return new TurretSetPosition(turret, deg);
    }

    /** A target written the old way (inches from the start: x forward, y left), in field coordinates. */
    private Pose2d fromStart(double xIn, double yIn, double headingDeg) {
        return FieldMath.fromStart(start, Pose2d.fromInches(xIn, yIn, Rotation2d.fromDegrees(headingDeg)));
    }
}
