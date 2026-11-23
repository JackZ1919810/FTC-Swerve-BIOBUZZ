package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.commands.FlowerToggle;
import org.firstinspires.ftc.teamcode.commands.IntakeFollowPower;
import org.firstinspires.ftc.teamcode.commands.ShooterBlockerFollow;
import org.firstinspires.ftc.teamcode.commands.ShooterFollowRPM;
import org.firstinspires.ftc.teamcode.commands.SwerveDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TurretAimAtGoal;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.FlowerIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.Path;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.Translation2d;
import org.firstinspires.ftc.teamcode.util.UpCell;

/*
 * Controls (gamepad1, except the last line):
 *   Left stick / right stick  drive / turn          Y     "forward" = where the robot faces now
 *   BACK                      toggle field centric  B     turret aim on / off
 *   D-pad left / right        our upward CELL is on my left / right (pick it in INIT, again after every TIP)
 *   X                         relocalize: robot is on this alliance's start spot
 *   Right trigger (hold)      intake  + indexer
 *   Left trigger  (hold)      outtake: intake backward (wins over the other intake buttons)
 *   A             (hold)      slow intake
 *   Right bumper  (hold)      shooter + indexer (see SubsystemConstants.FeedOnlyAtSpeed)
 *   Left bumper   (hold)      open the blocker (let the ball reach the flywheel)
 *   D-pad up / down           shooter RPM +/- one step
 *   Gamepad 2 A               flower intake: press to open, press again to close
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends NextFTCOpMode {
    // How far a trigger must be pressed (0..1) to count as "held"
    private static final double TRIGGER_THRESHOLD = 0.5;

    private SwerveDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private FlowerIntakeSubsystem flower;
    private TurretAimAtGoal aimCommand;

    private boolean aimOn = false;        // B toggles
    private boolean redAlliance = true;   // from the last Autonomous
    private UpCell upCell = UpCell.UNKNOWN; // which of our two CELLS faces up: the driver sets it

    private boolean fieldCentric = true;
    private double selectedRPM = SubsystemConstants.defaultRPM;   // changed with the d-pad

    @Override
    public void onInit() {
        // Redirect telemetry to FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drivetrain = new SwerveDrivetrain(hardwareMap);

        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        /*
        addComponents() is called inside onInit, which is too late for NextFTC
        to call initialize() for us, so call it here
        */

        drivetrain.initialize();
        intake.initialize();
        shooter.initialize();

        turret = new TurretSubsystem(hardwareMap);
        turret.initialize();

        flower = new FlowerIntakeSubsystem(hardwareMap);
        flower.initialize();

        // Pick up where Autonomous left off. initialize() reset the OTOS, so put the pose back
        redAlliance = PoseStorage.isRed();
        if (PoseStorage.hasPose()) {
            drivetrain.setPose(PoseStorage.take());
            drivetrain.setDriverForward(FieldConstants.driverForward(redAlliance));
        }

        addComponents(
                new SubsystemComponent(drivetrain, intake, shooter, turret, flower),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE   // one bulk read per hub per loop (MANUAL mode, cleared every loop)
        );

        // D-pad works from INIT on (BindingsComponent updates during INIT too), so the driver can
        // pick the CELL during the AUTO-to-TELEOP transition. It only sets a variable: nothing moves.
        // Press the side where you see our upward CELL; press again after every TIP
        Gamepads.gamepad1().dpadRight().whenBecomesTrue(() -> upCell = UpCell.onDriversSide(redAlliance, true));
        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(() -> upCell = UpCell.onDriversSide(redAlliance, false));
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void onWaitForStart() {
        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Upward CELL", upCell == UpCell.UNKNOWN ? "NOT SET: D-pad toward it" : upCell.name());
        telemetry.addData("Turret zero", TurretSubsystem.getZeroStatus());
        telemetry.addData("Center / trim", "%.4f / %.4f", TurretSubsystem.centerPos(), TurretSubsystem.rightTrim());
        telemetry.addData("Reach", "%.0f to %.0f deg  %s", TurretSubsystem.minDeg(), TurretSubsystem.maxDeg(),
                TurretSubsystem.reachesEveryDirection() ? "OK" : "SHORT: some directions unreachable");
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

        intake.setDefaultCommand(new IntakeFollowPower(intake, () -> intakePower()));

        shooter.setDefaultCommand(new ShooterFollowRPM(shooter,
                () -> shootHeld() ? selectedRPM : 0.0));

        // Blocker: runs next to ShooterFollowRPM (it doesn't require the shooter, so it's scheduled
        // instead of being a default command). For now it opens while the button is held.
        // Later, open only at speed: () -> shootHeld() && shooter.atSpeed()
        new ShooterBlockerFollow(shooter, () -> blockerHeld()).schedule();

        aimCommand = new TurretAimAtGoal(turret, drivetrain, () -> aimOn, () -> redAlliance,
                () -> upCell);
        turret.setDefaultCommand(aimCommand);

        // Flower: closed and held from START, so it's in a known state (it got no signal during INIT)
        flower.setOpen(false);

        // One action per press
        Gamepads.gamepad1().a().whenBecomesTrue(new FlowerToggle(flower));   // open / close, stays until the next press
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> changeRPM(+SubsystemConstants.RPMStep));
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> changeRPM(-SubsystemConstants.RPMStep));
        Gamepads.gamepad1().y().whenBecomesTrue(() -> drivetrain.resetDriverForward());   // keeps the pose
        Gamepads.gamepad2().x().whenBecomesTrue(() -> aimOn = !aimOn);
        Gamepads.gamepad1().x().whenBecomesTrue(() -> {
            // Relocalize: the robot is sitting on this alliance's start spot
            drivetrain.setPose(FieldConstants.startPose(redAlliance));
            drivetrain.setDriverForward(FieldConstants.driverForward(redAlliance));   // old "forward" was in the old frame
        });
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
        telemetry.addLine("--- Turret aim ---");
        telemetry.addData("Aim", aimCommand == null ? "OFF" : aimCommand.getStatus());
        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Target", aimCommand == null ? "-" : aimCommand.getTarget());
        telemetry.addData("Approach", "%.0f deg (max %.0f)",
                aimCommand == null ? Double.NaN : aimCommand.getApproachDeg(), FieldConstants.MaxApproachDeg);
        telemetry.addData("Goal distance", "%.1f in", aimCommand == null ? Double.NaN : aimCommand.getDistanceIn());
        telemetry.addData("Aim angle", "%.1f deg", aimCommand == null ? Double.NaN : aimCommand.getAimDeg());
        telemetry.addData("Turret target", "%.1f deg", turret.getTargetDeg());

        telemetry.addLine();
        telemetry.addLine("--- Shooter ---");
        telemetry.addData("State", shootHeld() ? "SHOOTING" : "IDLE");
        telemetry.addData("Selected RPM", "%.0f  (max %.0f)", selectedRPM, shooter.getMaxRPM());
        telemetry.addData("Target RPM", "%.0f", shooter.getTargetRPM());
        telemetry.addData("Actual Left RPM", "%.0f", shooter.getLeftRPM());
        telemetry.addData("Actual Right RPM", "%.0f", shooter.getRightRPM());
        telemetry.addData("Battery (V)", "%.2f", shooter.getBatteryVoltage());
        telemetry.addData("At speed", shooter.atSpeed());
        telemetry.addData("Blocker", shooter.isBlockerOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Feeding", feedShooter());

        telemetry.addLine();
        telemetry.addLine("--- Intake / Indexer ---");
        telemetry.addData("Intake power", "%.2f", intake.getPower());
        telemetry.addData("Flower", flower.isOpen() ? "OPEN" : "CLOSED");

        telemetry.update();
    }

    // ----- Button helpers: the one place that says which button does what -----

    private boolean intakeHeld() {
        return gamepad2.right_trigger > TRIGGER_THRESHOLD;
    }

    private boolean outtakeHeld() {
        return gamepad2.left_bumper;
    }

    private boolean slowIntakeHeld() {
        return gamepad2.right_bumper;
    }

    /** Intake power from the buttons. Outtake wins, so a jam can always be cleared. */
    private double intakePower() {
        if (outtakeHeld()) return SubsystemConstants.OuttakePower;
        if (intakeHeld()) return SubsystemConstants.IntakePower;
        if (slowIntakeHeld()) return SubsystemConstants.SlowIntakePower;
        return 0.0;
    }

    private boolean shootHeld() {
        return gamepad2.left_trigger > TRIGGER_THRESHOLD;
    }

    private boolean blockerHeld() {
        return gamepad2.left_bumper;
    }

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
