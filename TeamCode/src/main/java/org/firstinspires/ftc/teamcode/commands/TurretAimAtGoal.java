package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.FieldMath;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Translation2d;
import org.firstinspires.ftc.teamcode.util.UpCell;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;

/** Points the turret at our alliance's upward-facing HIVE CELL every loop, using the OTOS pose. */
public class TurretAimAtGoal extends Command {
    private final TurretSubsystem turret;
    private final SwerveDrivetrain drivetrain;
    private final BooleanSupplier aimEnabled;
    private final BooleanSupplier redAlliance;
    private final Supplier<UpCell> upCell;

    private String status = "OFF";
    private String target = "-";
    private double distanceIn = Double.NaN;
    private double aimDeg = Double.NaN;
    private double approachDeg = Double.NaN;

    public TurretAimAtGoal(TurretSubsystem turret, SwerveDrivetrain drivetrain,
                           BooleanSupplier aimEnabled, BooleanSupplier redAlliance,
                           Supplier<UpCell> upCell) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        this.aimEnabled = aimEnabled;
        this.redAlliance = redAlliance;
        this.upCell = upCell;
        addRequirements(turret);   // not the drivetrain: this only reads its pose, so driving goes on
    }

    @Override
    public void update() {
        // 1. Aim mode off: face forward
        if (!aimEnabled.getAsBoolean()) {
            status = "OFF";
            turret.center();
            return;
        }

        // 2. Pose unknown: the CELL position means nothing in this frame, so face forward
        if (!drivetrain.isPoseKnown()) {
            status = "NO POSE: put the robot on the start spot and press X";
            turret.center();
            return;
        }

        // 3. Which of our CELLS faces up? Only the driver knows (D-pad), so wait until they say
        UpCell cell = upCell.get();
        if (cell == UpCell.UNKNOWN) {
            status = "PICK CELL: D-pad toward our upward CELL";
            target = "NOT SET";
            turret.center();
            return;
        }

        // 4. Target = our HIVE's upward-facing CELL (the downward one can't score)
        boolean red = redAlliance.getAsBoolean();
        boolean audienceCell = cell == UpCell.AUDIENCE;
        Translation2d goal = FieldConstants.cell(red, audienceCell);
        target = (red ? "RED " : "BLUE ") + (audienceCell ? "AUDIENCE CELL" : "REAR CELL");

        // 5. Where the turret pivot is, and how far the CELL is (field frame, meters)
        Pose2d robot = drivetrain.getPose();
        Translation2d pivot = FieldMath.turretPivot(robot,
                FieldConstants.TurretPivotForwardIn * Pose2d.METERS_PER_INCH,
                FieldConstants.TurretPivotLeftIn * Pose2d.METERS_PER_INCH);
        distanceIn = pivot.getDistance(goal) / Pose2d.METERS_PER_INCH;

        // 6. Too close: a small pose error swings the angle a lot, so keep the last target
        if (distanceIn < FieldConstants.MinAimDistanceIn) {
            status = "TOO CLOSE";
            return;
        }

        // 7. Field direction minus heading, plus trim; the turret picks the reachable angle
        aimDeg = FieldMath.turretAngleToGoalDeg(robot, pivot, goal) + FieldConstants.AimTrimDeg;
        turret.setTargetDirection(aimDeg);

        // 8. Is the robot in front of the opening? 0 = straight in, 180 = behind the CELL
        approachDeg = FieldMath.approachAngleDeg(goal, FieldConstants.cellFacingDeg(audienceCell), pivot);
        status = approachDeg <= FieldConstants.MaxApproachDeg ? "AIMING"
                : "WRONG SIDE: drive to the " + (audienceCell ? "AUDIENCE" : "REAR") + " side";
    }

    @Override
    public boolean isDone() {
        return false;
    }

    public String getStatus() {
        return status;
    }

    /** Which CELL is being aimed at, e.g. "RED AUDIENCE CELL". */
    public String getTarget() {
        return target;
    }

    /** True when the turret is aiming and the robot is in front of the opening (for a shooter gate later). */
    public boolean canScore() {
        return status.equals("AIMING");
    }

    /** Pivot-to-CELL distance on the floor, in inches (for picking shooter RPM later). */
    public double getDistanceIn() {
        return distanceIn;
    }

    /** The last aim angle sent, degrees from the robot's front. */
    public double getAimDeg() {
        return aimDeg;
    }

    /** Degrees between the shot and the way the opening faces. */
    public double getApproachDeg() {
        return approachDeg;
    }
}
