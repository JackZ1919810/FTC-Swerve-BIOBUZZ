package org.firstinspires.ftc.teamcode.subsystems;

/*
 * REFERENCE ONLY: the wrap-around test version of YOUR TurretSubsystem.
 * This file lives outside TeamCode, so it is not compiled into the robot app.
 * It is your current TurretSubsystem with the wrap-around ability added. Everything marked NEW
 * is what you would add to your own file; the rest is the same as yours.
 *
 * Angles: turret degrees, 0 = robot front, positive = counterclockwise seen from above.
 * There are two kinds of angle (guide, Part 7):
 *   position  = where the turret physically is, TurretMinDeg..TurretMaxDeg (-200..200).
 *               Clamp it, never normalize it.
 *   direction = which way the turret points, normalized to (-180, 180].
 *               160, -200 and 520 are the same direction.
 *
 * Servo: one Axon, configured as "turretLeft". Unlike your file, there is no right servo here:
 * everything for "turretRight" (the field, setup, trim and writes) is removed until it is configured.
 *
 * Constants used (SubsystemConstants): AxonPWMRange, TurretGearRatio, TurretServoRangeDeg,
 * TurretCenterPos, TurretMinDeg = -200, TurretMaxDeg = 200.
 */

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;

import java.util.Arrays;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;

public class TurretSubsystem implements Subsystem {

    private static final PwmControl.PwmRange AxonPWMRange = SubsystemConstants.AxonPWMRange;
    private final HardwareMap hardwareMap;
    private ServoImplEx turretLeft;
    private Command defaultCommand = new NullCommand();
    private boolean isInitialized = false;
    private double targetDeg = Double.NaN;      // turret POSITION to hold (NaN = nothing commanded yet, servo limp)
    private double lastLeftPos = Double.NaN;

    // NEW: what the last setTargetDirection() call saw, for test telemetry
    private double lastDirectionDeg = Double.NaN;
    private int lastCandidateCount = 0;
    private int unwindCount = 0;

    public TurretSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public Command getDefaultCommand() {
        return defaultCommand;
    }

    public void setDefaultCommand(Command defaultCommand) {
        this.defaultCommand = defaultCommand;
    }

    @Override
    public void initialize() {
        turretLeft = hardwareMap.get(ServoImplEx.class, "turretLeft");

        turretLeft.setPwmRange(AxonPWMRange);
        turretLeft.setDirection(ServoImplEx.Direction.FORWARD);

        // No setPosition() here: the servo stays limp until a command sets a target
        targetDeg = Double.NaN;
        lastLeftPos = Double.NaN;
        lastDirectionDeg = Double.NaN;      // NEW
        lastCandidateCount = 0;             // NEW
        unwindCount = 0;                    // NEW
        isInitialized = true;
    }

    @Override
    public void periodic() {
        if (!isInitialized) return;
        if (Double.isNaN(targetDeg)) return;

        // NEW: clamp again, so lowering TurretMinDeg / TurretMaxDeg on Dashboard takes effect
        // even while the stick is in the deadband and no command calls setTargetDeg()
        targetDeg = clipToLimits(targetDeg);

        double leftPos = degToPos(targetDeg);

        if (leftPos != lastLeftPos) {
            turretLeft.setPosition(leftPos);
            lastLeftPos = leftPos;
        }
    }

    // ----- Abilities that commands can use (all in turret degrees) -----

    /** Go to a turret POSITION. Clamped to the limits, never wrapped (guide, Part 4). */
    public void setTargetDeg(double deg) {
        if (Double.isNaN(deg)) return;      // NEW: a NaN target would freeze the turret
        targetDeg = clipToLimits(deg);
    }

    /**
     * NEW. Point the turret in a DIRECTION. Any angle works: 170, -190 and 530 are the same direction.
     * Of the positions inside the limits that face that way, picks the one closest to the current
     * target (guide, Part 7). With the 400 degree band:
     *   - in the middle (-160..160) one position fits, so the turret simply follows
     *   - near the back (160..200 and -200..-160) two fit, and the closer one keeps the turret on its side
     *   - past a limit only the far one fits, so the turret unwinds about 360 degrees
     */
    public void setTargetDirection(double directionDeg) {
        if (Double.isNaN(directionDeg)) return;

        double direction = normalizeDeg(directionDeg);
        double current = getTargetDeg();
        double[] candidates = reachablePositions(direction,
                SubsystemConstants.TurretMinDeg, SubsystemConstants.TurretMaxDeg);

        lastDirectionDeg = direction;
        lastCandidateCount = candidates.length;

        // Only possible if the band is narrower than 360 degrees: that direction can't be reached, so hold
        if (candidates.length == 0) return;

        // Plain subtraction on purpose: the cables care about real rotation, not direction
        double best = candidates[0];
        for (double candidate : candidates) {
            if (Math.abs(candidate - current) < Math.abs(best - current)) best = candidate;
        }

        // A move of more than half a turn only happens when unwinding at a limit
        if (Math.abs(best - current) > 180.0) unwindCount++;

        setTargetDeg(best);
    }

    public void center() {
        setTargetDeg(0.0);
    }

    /** Before the first command the turret is assumed to be centered. */
    public double getTargetDeg() {
        return Double.isNaN(targetDeg) ? 0.0 : targetDeg;
    }

    /** What was last sent to the left servo (0..1). NaN before the first command. */
    public double getCommandPos() {
        return lastLeftPos;
    }

    /** NEW. The direction the last setTargetDirection() asked for, normalized. NaN before the first one. */
    public double getLastDirectionDeg() {
        return lastDirectionDeg;
    }

    /** NEW. How many positions fit the last direction (1 or 2 with a 400 degree band). */
    public int getLastCandidateCount() {
        return lastCandidateCount;
    }

    /** NEW. How many times the turret has unwound at a limit since INIT. */
    public int getUnwindCount() {
        return unwindCount;
    }

    // ----- Angle math (NEW; public static so the test command and telemetry can use it) -----

    /**
     * A direction in (-180, 180]. Guide, Part 7: norm(a) = a - 360 * ceil((a - 180) / 360).
     * Use it on directions only, never on a position.
     */
    public static double normalizeDeg(double deg) {
        return deg - 360.0 * Math.ceil((deg - 180.0) / 360.0);
    }

    /**
     * The turret positions inside [minDeg, maxDeg] that face this direction, smallest first.
     * Every position facing direction a is a + 360k; with limits inside +-540 only k = -1, 0 and 1 can fit.
     */
    public static double[] reachablePositions(double directionDeg, double minDeg, double maxDeg) {
        double a = normalizeDeg(directionDeg);
        double[] found = new double[3];
        int count = 0;
        for (double candidate : new double[] {a - 360.0, a, a + 360.0}) {
            if (candidate >= minDeg && candidate <= maxDeg) found[count++] = candidate;
        }
        return Arrays.copyOf(found, count);
    }

    private static double clipToLimits(double deg) {
        return Range.clip(deg, SubsystemConstants.TurretMinDeg, SubsystemConstants.TurretMaxDeg);
    }

    // turret deg -> position; with the defaults this is 0.5 + turretDeg / 455 (guide, Part 3)
    private double degToPos(double turretDeg) {
        double pos = SubsystemConstants.TurretCenterPos
                + turretDeg / (SubsystemConstants.TurretServoRangeDeg * SubsystemConstants.TurretGearRatio);
        return Range.clip(pos, 0.0, 1.0);
    }
}
