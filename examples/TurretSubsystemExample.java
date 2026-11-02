package org.firstinspires.ftc.teamcode.subsystems;

/*
 * REFERENCE EXAMPLE ONLY (simple version: turret degrees -> servo position, nothing else).
 * This file lives outside TeamCode, so it is not compiled into the robot app.
 * Use it to compare against your own TurretSubsystem; don't copy it in as-is.
 * The full version with wrap-around and feedback is in TurretSubsystemWrapExample.java, for later.
 *
 * Hardware: 1-2 Axon Mini MK2 servos in servo (not continuous) mode, driving the turret through a
 * 1:1.5 step-up (1 servo degree = 1.5 turret degrees).
 *
 * Angles: turret degrees, 0 = robot front, positive = counterclockwise seen from above.
 *
 * Constants this class expects in SubsystemConstants (inside the @Config class, so Dashboard can tune them):
 *
 *     //Turret constants
 *     public static double TurretGearRatio = 1.5;          // turret degrees per servo degree (1:1.5 step-up)
 *     public static double TurretServoRangeDeg = 350;      // Axon Mini MK2 travel for position 0 -> 1 at 500-2500 us
 *     public static double TurretCenterPos = 0.5;          // servo position where the turret faces forward (calibrate)
 *     public static double TurretMinDeg = -175;            // limits, inside the cable limits (hardware allows +-262.5)
 *     public static double TurretMaxDeg = 175;
 *     public static double TurretRightTrim = 0.0;          // position offset so the two servos don't fight
 *     public static boolean TurretRightReversed = false;   // true if the second servo is mounted mirrored
 *     public static double TurretStickDegPerSec = 180;     // turret speed at full stick (TurretFollowStickExample)
 *     public static double TurretStickDeadband = 0.05;     // ignore stick drift below this
 */

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;

public class TurretSubsystemExample implements Subsystem {

    // Axon full range. The SDK default (600-2400 us) would only give 315 servo degrees,
    // making every turret angle ~10% short of what was asked for
    private static final PwmControl.PwmRange AXON_PWM_RANGE = new PwmControl.PwmRange(500, 2500);

    private final HardwareMap hardwareMap;
    private Command defaultCommand = new NullCommand();
    private ServoImplEx left;
    private ServoImplEx right;          // null until the second servo is configured
    private boolean isInitialized = false;

    private double targetDeg = Double.NaN;          // what commands ask for (NaN = nothing yet, so no motion in INIT)
    private double lastLeftPosition = Double.NaN;   // what was last sent to each servo
    private double lastRightPosition = Double.NaN;

    public TurretSubsystemExample(HardwareMap hardwareMap) {
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
        isInitialized = true;
    }

    @Override
    public void periodic() {
        if (!isInitialized) return;
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

    public void setTargetDegrees(double degrees) {
        targetDeg = Range.clip(degrees, SubsystemConstants.TurretMinDeg, SubsystemConstants.TurretMaxDeg);
    }

    public void center() {
        setTargetDegrees(0.0);
    }

    /** Before the first command the turret is assumed to be centered. */
    public double getTargetDegrees() {
        return Double.isNaN(targetDeg) ? 0.0 : targetDeg;
    }

    /** What was last sent to the left servo (0..1), for telemetry. NaN before the first command. */
    public double getCommandedPosition() {
        return lastLeftPosition;
    }

    // ----- Unit conversion: servo positions are only used inside this class -----

    // turret deg -> servo deg -> position; with the defaults this is 0.5 + turretDeg / 525
    private double degreesToPosition(double turretDeg) {
        double servoDeg = turretDeg / SubsystemConstants.TurretGearRatio;
        double position = SubsystemConstants.TurretCenterPos + servoDeg / SubsystemConstants.TurretServoRangeDeg;
        return Range.clip(position, 0.0, 1.0);
    }
}
