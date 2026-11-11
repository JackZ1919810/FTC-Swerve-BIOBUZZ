package org.firstinspires.ftc.teamcode.subsystems;

/*
 * REFERENCE EXAMPLE ONLY (full version, for later: wrap-around logic, feedback wire, atTarget()).
 * This file lives outside TeamCode, so it is not compiled into the robot app.
 * Use it to compare against your own TurretSubsystem; don't copy it in as-is.
 *
 * Hardware: 1-2 Axon Mini MK2 servos in servo (not continuous) mode, driving the turret through a
 * 1:1.3 step-up (1 servo degree = 1.3 turret degrees; measured). Optional: the Axon's 4th wire (analog feedback)
 * plugged into an analog port and configured as "turretEncoder".
 *
 * Angles: degrees, 0 = robot front, positive = counterclockwise seen from above (same as Rotation2d / OTOS).
 *
 * Constants this class expects in SubsystemConstants (inside the @Config class, so Dashboard can tune them):
 *
 *     //Turret constants
 *     public static double TurretGearRatio = 1.3;          // turret degrees per servo degree (1:1.3 step-up, measured)
 *     public static double TurretServoRangeDeg = 350;      // Axon Mini MK2 travel for position 0 -> 1 at 500-2500 us
 *     public static double TurretCenterPos = 0.5;          // servo position where the turret faces forward (calibrate)
 *     public static double TurretMinDeg = -200;            // soft limits, inside the cable limits (hardware allows +-227.5)
 *     public static double TurretMaxDeg = 200;
 *     public static double TurretToleranceDeg = 2;
 *     public static double TurretFlipHysteresisDeg = 10;   // only used if the window is narrower than 360 deg
 *     public static double TurretRightTrim = 0.0;          // position offset so the two servos don't fight
 *     public static boolean TurretRightReversed = false;   // true if the second servo is mounted mirrored
 *     public static double TurretServoSecPer60Deg = 0.11;  // Axon Mini MK2 spec at 4.8 V, no load
 *     public static double TurretTravelSafety = 1.5;       // the loaded turret is slower than the no-load spec
 *     public static double TurretFeedbackCenterVolts = 1.65; // analog reading with the turret at 0 deg (calibrate)
 *     public static double TurretFeedbackDegPerVolt = 141.8; // turret deg per volt; assumes 3.3 V = 360 servo deg (calibrate, sign matters)
 */

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.util.Rotation2d;

public class TurretSubsystemWrapExample implements Subsystem {

    // Axon full range. The SDK default (600-2400 us) would only give 315 servo degrees,
    // making every turret angle ~10% short of what was asked for
    private static final PwmControl.PwmRange AXON_PWM_RANGE = new PwmControl.PwmRange(500, 2500);

    private final HardwareMap hardwareMap;
    private Command defaultCommand = new NullCommand();
    private ServoImplEx left;
    private ServoImplEx right;          // null until the second servo is configured
    private AnalogInput feedback;       // null until the feedback wire is configured
    private boolean isInitialized = false;

    private double targetDeg = Double.NaN;          // where the turret should be (NaN = nothing commanded yet)
    private double lastLeftPosition = Double.NaN;   // what was last sent to each servo
    private double lastRightPosition = Double.NaN;
    private double measuredDeg = Double.NaN;        // from the feedback wire, read once per loop

    // Used by atTarget() when there is no feedback wire
    private final ElapsedTime moveTimer = new ElapsedTime();
    private double expectedMoveSeconds = 0.0;

    public TurretSubsystemWrapExample(HardwareMap hardwareMap) {
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
        left = hardwareMap.get(ServoImplEx.class, "turretLeft");
        right = hardwareMap.tryGet(ServoImplEx.class, "turretRight");
        feedback = hardwareMap.tryGet(AnalogInput.class, "turretEncoder");

        // Servos are reset to FORWARD at the start of every OpMode, so set direction every time
        left.setPwmRange(AXON_PWM_RANGE);
        left.setDirection(Servo.Direction.FORWARD);
        if (right != null) {
            right.setPwmRange(AXON_PWM_RANGE);
            right.setDirection(SubsystemConstants.TurretRightReversed
                    ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        }

        // No setPosition() here: the servos stay limp until a command sets a target
        targetDeg = Double.NaN;
        lastLeftPosition = Double.NaN;
        lastRightPosition = Double.NaN;
        measuredDeg = Double.NaN;
        expectedMoveSeconds = 0.0;
        isInitialized = true;
    }

    @Override
    public void periodic() {
        if (!isInitialized) return;

        if (feedback != null) {
            measuredDeg = voltsToDegrees(feedback.getVoltage());
        }

        if (Double.isNaN(targetDeg)) return;

        // Compare positions, not degrees, so Dashboard changes to center/trim/ratio reach the servos live
        double leftPosition = degreesToPosition(targetDeg);
        if (leftPosition != lastLeftPosition) {
            left.setPosition(leftPosition);
            lastLeftPosition = leftPosition;
        }

        if (right != null) {
            double rightPosition = Range.clip(leftPosition + SubsystemConstants.TurretRightTrim, 0.0, 1.0);
            if (rightPosition != lastRightPosition) {
                right.setPosition(rightPosition);
                lastRightPosition = rightPosition;
            }
        }
    }

    // ----- Abilities that commands can use (all in turret degrees) -----

    /**
     * Point the turret in a direction. Any angle works (200, -160 and 560 are the same direction);
     * the turret picks the reachable angle that needs the least travel, unwinding only at a limit.
     */
    public void setTargetDegrees(double requestedDeg) {
        double previous = getTargetDegrees();
        double chosen = chooseReachableAngle(requestedDeg);
        if (chosen == targetDeg) return;

        // Unknown start (first command, no feedback): assume the worst-case swing.
        // Tiny tracking corrections don't restart the timer, or atTarget() would never be true while tracking
        double swingDeg = hasKnownPosition()
                ? Math.abs(chosen - previous)
                : SubsystemConstants.TurretMaxDeg - SubsystemConstants.TurretMinDeg;
        if (swingDeg > SubsystemConstants.TurretToleranceDeg) {
            expectedMoveSeconds = estimateTravelSeconds(swingDeg);
            moveTimer.reset();
        }

        targetDeg = chosen;
    }

    public void center() {
        setTargetDegrees(0.0);
    }

    /** The current target, or the best guess of where the turret is if nothing was commanded yet. */
    public double getTargetDegrees() {
        if (!Double.isNaN(targetDeg)) return targetDeg;
        if (hasFeedback()) return measuredDeg;
        return 0.0;
    }

    /** NaN when there is no feedback wire. */
    public double getMeasuredDegrees() {
        return measuredDeg;
    }

    public boolean hasFeedback() {
        return feedback != null && !Double.isNaN(measuredDeg);
    }

    public boolean atTarget() {
        if (Double.isNaN(targetDeg)) return false;
        if (hasFeedback()) {
            return Math.abs(measuredDeg - targetDeg) < SubsystemConstants.TurretToleranceDeg;
        }
        return moveTimer.seconds() > expectedMoveSeconds;
    }

    /** What was last sent to the left servo (0..1), for telemetry. NaN before the first command. */
    public double getCommandedPosition() {
        return lastLeftPosition;
    }

    // ----- Angle math: servo positions and wrapping are only used inside this class -----

    private boolean hasKnownPosition() {
        return !Double.isNaN(targetDeg) || hasFeedback();
    }

    /**
     * Every turret angle that points in direction a is a + 360k. With limits inside +-540,
     * only k = -1, 0, 1 can fit, so try those three.
     */
    private double chooseReachableAngle(double requestedDeg) {
        double min = SubsystemConstants.TurretMinDeg;
        double max = SubsystemConstants.TurretMaxDeg;
        double a = Rotation2d.fromDegrees(requestedDeg).getDegrees();   // normalized to (-180, 180]
        double current = getTargetDegrees();

        // Of the candidates inside the window, take the one closest to where the turret is now.
        // Plain subtraction on purpose: the cables care about real rotation, not direction
        double best = Double.NaN;
        for (double candidate : new double[] {a - 360.0, a, a + 360.0}) {
            if (candidate < min || candidate > max) continue;
            if (Double.isNaN(best) || Math.abs(candidate - current) < Math.abs(best - current)) {
                best = candidate;
            }
        }
        if (!Double.isNaN(best)) return best;

        // Dead zone (only possible when the window is narrower than 360): park at the closer limit,
        // and only switch sides when the other limit is clearly closer
        double pastMax = normalize360(a - max);
        double beforeMin = normalize360(min - a);
        double hysteresis = SubsystemConstants.TurretFlipHysteresisDeg;
        if (current >= max) return (beforeMin + hysteresis < pastMax) ? min : max;
        if (current <= min) return (pastMax + hysteresis < beforeMin) ? max : min;
        return (pastMax <= beforeMin) ? max : min;
    }

    // turret deg -> servo deg -> position; with the defaults this is 0.5 + turretDeg / 455
    private double degreesToPosition(double turretDeg) {
        double servoDeg = turretDeg / SubsystemConstants.TurretGearRatio;
        double position = SubsystemConstants.TurretCenterPos + servoDeg / SubsystemConstants.TurretServoRangeDeg;
        return Range.clip(position, 0.0, 1.0);
    }

    private double voltsToDegrees(double volts) {
        return (volts - SubsystemConstants.TurretFeedbackCenterVolts) * SubsystemConstants.TurretFeedbackDegPerVolt;
    }

    private double estimateTravelSeconds(double turretSwingDeg) {
        double servoSwingDeg = turretSwingDeg / SubsystemConstants.TurretGearRatio;
        return servoSwingDeg / 60.0 * SubsystemConstants.TurretServoSecPer60Deg * SubsystemConstants.TurretTravelSafety;
    }

    // Java's % keeps the sign of the left side (-190 % 360 == -190), so shift it back into [0, 360)
    private static double normalize360(double deg) {
        return ((deg % 360.0) + 360.0) % 360.0;
    }
}
