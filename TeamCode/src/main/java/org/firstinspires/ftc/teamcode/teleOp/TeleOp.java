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
import org.firstinspires.ftc.teamcode.util.FieldMath;
import org.firstinspires.ftc.teamcode.util.InterpolationTable;
import org.firstinspires.ftc.teamcode.util.Path;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.Translation2d;
import org.firstinspires.ftc.teamcode.util.UpCell;

/*
 * Controls (PS controller names, SDK names in brackets)
 *
 * Gamepad 1, driver:
 *   Left stick                drive (translation)
 *   Right stick               turn (rotation)
 *   R2 (hold)                 X-lock: wheels in an X so the robot is hard to push. Release to drive again
 *   Square [X]                field reset: robot is on this alliance's start spot
 *   Cross [A]                 FLOWER intake: press to open, press again to close
 *   Triangle [Y]              "forward" = where the robot faces now (keeps the pose)
 *   Share [BACK]              toggle field centric
 *
 * Gamepad 2, operator:
 *   L2 (hold)                 shooter. Gamepad 2 rumbles once when it first reaches speed
 *                             (again only after L2 is released and pressed again)
 *   L1 (hold)                 outtake: intake backward (wins over the other intake buttons)
 *   R2 (hold)                 full-speed intake
 *   R1 (hold)                 shooter feed: slow-speed intake
 *   Triangle [Y] (hold)       blocker open (the driver decides when, no at-speed lock)
 *   Square [X]                turret: center & disable / press again to aim
 *   D-pad left / right        our upward CELL is on my left / right (pick it in INIT, again after every TIP)
 *   D-pad up / down           shooter RPM +/- one step (table on: a trim added to the table's RPM)
 *
 * Shooter RPM: SubsystemConstants.ShooterUseTable = true picks it from the distance to our CELL
 * (ShooterTableDistIn / ShooterTableRPM). With the table off, or no distance yet (no pose / CELL not picked),
 * it's the D-pad RPM.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends NextFTCOpMode {
    // How far a trigger must be pressed (0..1) to count as "held"
    private static final double TRIGGER_THRESHOLD = 0.5;
    // How long gamepad 2 rumbles when the shooter first reaches speed
    private static final int READY_RUMBLE_MS = 400;

    private SwerveDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private FlowerIntakeSubsystem flower;
    private TurretAimAtGoal aimCommand;

    private boolean aimOn = true;         // gamepad 2 Square toggles: false = turret centered and held there
    private boolean redAlliance = true;   // from the last Autonomous
    private UpCell upCell = UpCell.UNKNOWN; // which of our two CELLS faces up: the driver sets it

    private boolean fieldCentric = true;
    private double selectedRPM = SubsystemConstants.defaultRPM;   // changed with the d-pad (table off)
    private double tableTrimRPM = 0.0;                            // changed with the d-pad (table on)
    private boolean readyRumbleArmed = true;   // true = the next time the shooter reaches speed, rumble

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
        Gamepads.gamepad2().dpadRight().whenBecomesTrue(() -> upCell = UpCell.onDriversSide(redAlliance, true));
        Gamepads.gamepad2().dpadLeft().whenBecomesTrue(() -> upCell = UpCell.onDriversSide(redAlliance, false));
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
                () -> fieldCentric,
                () -> xLockHeld()
        ));

        intake.setDefaultCommand(new IntakeFollowPower(intake, () -> intakePower()));

        shooter.setDefaultCommand(new ShooterFollowRPM(shooter,
                () -> shootHeld() ? shooterRPM() : 0.0));

        // Blocker: runs next to ShooterFollowRPM (it doesn't require the shooter, so it's scheduled
        // instead of being a default command). Open while Triangle is held: the driver decides when,
        // helped by the ready rumble (see updateReadyRumble)
        new ShooterBlockerFollow(shooter, () -> gateHeld()).schedule();

        aimCommand = new TurretAimAtGoal(turret, drivetrain, () -> aimOn, () -> redAlliance,
                () -> upCell);
        turret.setDefaultCommand(aimCommand);

        // Flower: closed and held from START, so it's in a known state (it got no signal during INIT)
        flower.setOpen(false);

        // One action per press
        Gamepads.gamepad1().a().whenBecomesTrue(new FlowerToggle(flower));   // open / close, stays until the next press
        Gamepads.gamepad2().dpadUp().whenBecomesTrue(() -> changeRPM(+SubsystemConstants.RPMStep));
        Gamepads.gamepad2().dpadDown().whenBecomesTrue(() -> changeRPM(-SubsystemConstants.RPMStep));
        Gamepads.gamepad1().y().whenBecomesTrue(() -> drivetrain.resetDriverForward());   // keeps the pose
        // Turret: aiming starts on. Off = TurretAimAtGoal centers the turret and holds it there
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
        updateReadyRumble();

        Pose2d pose = drivetrain.getPose();
        telemetry.addData("X", "%.2f in", pose.getXInches());
        telemetry.addData("Y", "%.2f in", pose.getYInches());
        telemetry.addData("Heading", "%.2f deg", pose.getHeading().getDegrees());
        telemetry.addData("Drive Mode", (fieldCentric ? "Field Centric" : "Robot Centric")
                + (xLockHeld() ? "  [X-LOCK]" : ""));

        telemetry.addLine();
        telemetry.addLine("--- Turret aim ---");
        telemetry.addData("Turret", aimOn ? "AIM ENABLED" : "CENTERED + DISABLED (gamepad 2 Square to aim)");
        telemetry.addData("Aim", aimCommand == null ? "OFF" : aimCommand.getStatus());
        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Target", aimCommand == null ? "-" : aimCommand.getTarget());
        telemetry.addData("Approach", "%.0f deg (max %.0f)",
                aimCommand == null ? Double.NaN : aimCommand.getApproachDeg(), FieldConstants.MaxApproachDeg);
        telemetry.addData("Shot distance", "%.1f in  (for the RPM table)", shotDistanceIn());
        telemetry.addData("Aim angle", "%.1f deg", aimCommand == null ? Double.NaN : aimCommand.getAimDeg());
        telemetry.addData("Turret target", "%.1f deg", turret.getTargetDeg());

        telemetry.addLine();
        telemetry.addLine("--- Shooter ---");
        telemetry.addData("State", shootHeld() ? "SHOOTING" : "IDLE");
        telemetry.addData("RPM source", rpmSource());
        telemetry.addData("Selected RPM", "%.0f  (max %.0f)", selectedRPM, shooter.getMaxRPM());
        telemetry.addData("Target RPM", "%.0f", shooter.getTargetRPM());
        telemetry.addData("Actual Left RPM", "%.0f", shooter.getLeftRPM());
        telemetry.addData("Actual Right RPM", "%.0f", shooter.getRightRPM());
        telemetry.addData("Encoders", shooter.hasEncoderMismatch() ? "MISMATCH: check cables / directions" : "OK");
        telemetry.addData("Battery (V)", "%.2f", shooter.getBatteryVoltage());
        telemetry.addData("Voltage scale", "x%.3f  (tuned at %.1f V%s)", shooter.getVoltageScale(),
                SubsystemConstants.TunedBatteryVolts, SubsystemConstants.VoltageCompensation ? "" : ", OFF");
        telemetry.addData("At speed", shooter.atSpeed());
        telemetry.addData("Blocker", shooter.isBlockerOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Feeding", feedHeld());

        telemetry.addLine();
        telemetry.addLine("--- Intake / Indexer ---");
        telemetry.addData("Intake power", "%.2f", intake.getPower());
        telemetry.addData("Flower", flower.isOpen() ? "OPEN" : "CLOSED");

        telemetry.update();
    }

    // ----- Button helpers: the one place that says which button does what -----

    /** Driver R2 held: X-lock (wheels in an X, robot hard to push). Released: normal driving. */
    private boolean xLockHeld() {
        return gamepad1.right_trigger > TRIGGER_THRESHOLD;
    }

    private boolean intakeHeld() {
        return gamepad2.right_trigger > TRIGGER_THRESHOLD;
    }

    private boolean outtakeHeld() {
        return gamepad2.left_bumper;
    }

    /** Shooter feed (R1): slow-speed intake and the blocker open, together. */
    private boolean feedHeld() {
        return gamepad2.right_bumper;
    }

    /** Intake power from the buttons. Outtake wins, so a jam can always be cleared. */
    private double intakePower() {
        if (outtakeHeld()) return SubsystemConstants.OuttakePower;
        if (intakeHeld()) return SubsystemConstants.IntakePower;
        // Shooter feed: scaled by battery voltage, so balls reach the flywheel at the same speed on a drained battery
        if (feedHeld()) return SubsystemConstants.SlowIntakePower * shooter.getVoltageScale();
        return 0.0;
    }

    private boolean shootHeld() {
        return gamepad2.left_trigger > TRIGGER_THRESHOLD;
    }

    private boolean gateHeld(){
        return gamepad2.y;
    }

    /**
     * Rumbles gamepad 2 once per spin-up: the first time the shooter reaches its target RPM after starting.
     * Not again when the speed dips after a ball and comes back; only after the shooter stops (L2 released)
     * and spins up again. Works on controllers with rumble motors (PS4 / PS5), not on the Logitech F310.
     */
    private void updateReadyRumble() {
        if (shooter.getTargetRPM() == 0) {   // shooter stopped: the next spin-up may rumble again
            readyRumbleArmed = true;
            return;
        }
        if (readyRumbleArmed && shooter.atSpeed()) {
            gamepad2.rumble(1.0, 1.0, READY_RUMBLE_MS);   // both rumble motors, full strength
            readyRumbleArmed = false;
        }
    }

    /** D-pad up / down: with the table on it trims the table's RPM, otherwise it picks the RPM. */
    private void changeRPM(double delta) {
        if (SubsystemConstants.ShooterUseTable) {
            tableTrimRPM += delta;   // the final RPM is still clipped to 0..max by the shooter
            return;
        }
        // Range.clip needs min <= max; if min were bigger, the result would jump between the two
        double min = SubsystemConstants.MinRPM;
        double max = Math.max(min, shooter.getMaxRPM());
        selectedRPM = Range.clip(selectedRPM + delta, min, max);
    }

    // ----- Shooter RPM from distance (interpolation table) -----

    /**
     * Floor distance from the turret pivot to our upward CELL, in inches (the same distance the aim uses).
     * NaN when there's no pose yet (no Auto, X not pressed) or the CELL isn't picked (D-pad).
     */
    private double shotDistanceIn() {
        if (!drivetrain.isPoseKnown() || upCell == UpCell.UNKNOWN) return Double.NaN;
        Translation2d pivot = FieldMath.turretPivot(drivetrain.getPose(),
                FieldConstants.TurretPivotForwardIn * Pose2d.METERS_PER_INCH,
                FieldConstants.TurretPivotLeftIn * Pose2d.METERS_PER_INCH);
        Translation2d cell = FieldConstants.cell(redAlliance, upCell == UpCell.AUDIENCE);
        return pivot.getDistance(cell) / Pose2d.METERS_PER_INCH;
    }

    /** RPM from the table at the current distance, or NaN if the table is off, has a typo, or there's no distance. */
    private double tableRPM() {
        if (!SubsystemConstants.ShooterUseTable) return Double.NaN;
        return InterpolationTable.interpolate(
                SubsystemConstants.ShooterTableDistIn, SubsystemConstants.ShooterTableRPM, shotDistanceIn());
    }

    /** What the shooter spins at while L2 is held: table + trim, or the D-pad RPM when the table can't be used. */
    private double shooterRPM() {
        double table = tableRPM();
        return Double.isNaN(table) ? selectedRPM : table + tableTrimRPM;
    }

    /** For telemetry: where the RPM comes from right now. */
    private String rpmSource() {
        if (!SubsystemConstants.ShooterUseTable) return "D-PAD";
        if (!InterpolationTable.isValid(SubsystemConstants.ShooterTableDistIn, SubsystemConstants.ShooterTableRPM)) {
            return "D-PAD (table has a typo: distances must go up, same count as RPMs)";
        }
        if (Double.isNaN(shotDistanceIn())) return "D-PAD (table on, but no distance: need pose + CELL)";
        return String.format("TABLE %.0f  (trim %+.0f)", tableRPM(), tableTrimRPM);
    }
}
