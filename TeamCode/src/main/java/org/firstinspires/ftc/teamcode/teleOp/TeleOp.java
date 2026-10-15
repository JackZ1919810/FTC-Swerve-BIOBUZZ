package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.commands.IndexFollowPower;
import org.firstinspires.ftc.teamcode.commands.IntakeFollowPower;
import org.firstinspires.ftc.teamcode.commands.ShooterFollowRPM;
import org.firstinspires.ftc.teamcode.commands.SwerveDriveCommand;
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.Pose2d;

/*
 * Controls (all gamepad1):
 *   Left stick / right stick  drive / turn          Y     reset pose
 *   BACK                      toggle field centric
 *   Right trigger (hold)      intake  + indexer
 *   Left trigger  (hold)      indexer only
 *   Right bumper  (hold)      shooter + indexer (see SubsystemConstants.FeedOnlyAtSpeed)
 *   D-pad up / down           shooter RPM +/- one step
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends NextFTCOpMode {
    // How far a trigger must be pressed (0..1) to count as "held"
    private static final double TRIGGER_THRESHOLD = 0.5;

    private SwerveDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;
    private ShooterSubsystem shooter;

    private boolean fieldCentric = true;
    private double selectedRPM = SubsystemConstants.defaultRPM;   // changed with the d-pad

    @Override
    public void onInit() {
        // Redirect telemetry to FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drivetrain = new SwerveDrivetrain(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        indexer = new IndexerSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        // addComponents() is called inside onInit, which is too late for NextFTC
        // to call initialize() for us, so call it here
        drivetrain.initialize();
        intake.initialize();
        indexer.initialize();
        shooter.initialize();

        addComponents(
                new SubsystemComponent(drivetrain, intake, indexer, shooter),
                BindingsComponent.INSTANCE
        );

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Default commands and bindings are set at START, not INIT,
        // because NextFTC already runs them during INIT and the robot must not move then

        drivetrain.setDefaultCommand(new SwerveDriveCommand(
                drivetrain,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                () -> fieldCentric
        ));

        intake.setDefaultCommand(new IntakeFollowPower(intake,
                () -> intakeHeld() ? SubsystemConstants.IntakePower : 0.0));

        // The indexer runs whenever the intake, the indexer button, or shooter feeding is in use
        indexer.setDefaultCommand(new IndexFollowPower(indexer,
                () -> indexHeld()  ? SubsystemConstants.IndexPower : 0.0));

        shooter.setDefaultCommand(new ShooterFollowRPM(shooter,
                () -> shootHeld() ? selectedRPM : 0.0));

        // One action per press
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> changeRPM(+SubsystemConstants.RPMStep));
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> changeRPM(-SubsystemConstants.RPMStep));
        Gamepads.gamepad1().y().whenBecomesTrue(() -> drivetrain.resetPose());
        Gamepads.gamepad1().back().whenBecomesTrue(() -> fieldCentric = !fieldCentric);
    }

    @Override
    public void onUpdate() {
        Pose2d pose = drivetrain.getPose();
        telemetry.addData("X", "%.2f in", pose.getXInches());
        telemetry.addData("Y", "%.2f in", pose.getYInches());
        telemetry.addData("Heading", "%.2f deg", pose.getHeading().getDegrees());
        telemetry.addData("Drive Mode", fieldCentric ? "Field Centric" : "Robot Centric");

        telemetry.addLine();
        telemetry.addLine("--- Shooter ---");
        telemetry.addData("State", shootHeld() ? "SHOOTING" : "off");
        telemetry.addData("Selected RPM", "%.0f  (max %.0f)", selectedRPM, shooter.getMaxRPM());
        telemetry.addData("Target RPM", "%.0f", shooter.getTargetRPM());
        telemetry.addData("Actual RPM", "%.0f", shooter.getRPM());
        telemetry.addData("At speed", shooter.atSpeed());
        telemetry.addData("Feeding", feedShooter());

        telemetry.addLine();
        telemetry.addLine("--- Intake / Indexer ---");
        telemetry.addData("Intake power", "%.2f", intake.getPower());
        telemetry.addData("Indexer power", "%.2f", indexer.getPower());

        telemetry.update();
    }

    // ----- Button helpers: the one place that says which button does what -----

    private boolean intakeHeld() {
        return gamepad1.right_trigger > TRIGGER_THRESHOLD;
    }

    private boolean indexHeld() {
        return gamepad1.left_trigger > TRIGGER_THRESHOLD;
    }

    private boolean shootHeld() {
        return gamepad1.right_bumper;
    }

    // Should the indexer push balls into the shooter right now?
    private boolean feedShooter() {
        if (!shootHeld()) return false;
        return !SubsystemConstants.FeedOnlyAtSpeed || shooter.atSpeed();
    }

    private void changeRPM(double delta) {
        // Range.clip needs min <= max; if min were bigger, the result would jump between the two
        double min = SubsystemConstants.MinRPM;
        double max = Math.max(min, shooter.getMaxRPM());
        selectedRPM = Range.clip(selectedRPM + delta, min, max);
    }
}
