package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;

/*
 * Shooter on the hubs' own speed control (RUN_USING_ENCODER + setVelocity): both motors get the same
 * target speed, and each motor's hub holds it with that motor's own encoder and its own velocity PIDF
 * (Left/RightShooterVelP, I, D, F in SubsystemConstants).
 *
 * Voltage compensation: the hub's PIDF output is motor power, and the same power gives less speed on a
 * lower battery. So the PIDF sent to the hubs is multiplied by TunedBatteryVolts / battery voltage,
 * and the motors get the same voltage as when the PIDF was tuned.
 */
public class ShooterSubsystem implements Subsystem {

    private final HardwareMap hardwareMap;
    private Command defaultCommand = new NullCommand();
    private DcMotorEx motorRight, motorLeft;
    private ServoImplEx blocker;
    private boolean isInitialized = false;

    private double targetRPM = 0.0;            // what commands ask for (both motors)
    private double lastTarget = Double.NaN;    // what was last sent to the motors

    // Hub velocity PIDF last sent to each motor, as {P, I, D, F} (NaN = not sent yet),
    // so new values go to a hub only when they change
    private final double[] sentLeft = new double[4];
    private final double[] sentRight = new double[4];

    // Measured speed: both encoders read once per loop in periodic() and smoothed (ShooterRPMFilter)
    private double leftRPM = Double.NaN;       // NaN = no reading yet
    private double rightRPM = Double.NaN;
    private boolean encoderMismatch = false;   // left and right differ by more than ShooterMismatchRPM

    // Blocker: nothing is sent until a command asks, so it doesn't move during INIT.
    // The open / closed positions are in SubsystemConstants (Dashboard)
    private boolean blockerCommanded = false;      // false until the first setServoPos
    private boolean blockerOpen = false;
    private double lastBlockerPos = Double.NaN;    // what was last sent to the servo
    private static final PwmControl.PwmRange AxonPWMRange = SubsystemConstants.AxonPWMRange;

    // Battery voltage: each read is its own hub command, so read it a few times a second, not every loop,
    // and smooth it (it jumps around under load)
    private static final long BATTERY_READ_PERIOD_NS = 100_000_000L;   // 0.1 s
    private static final double BATTERY_FILTER = 0.3;                  // 0..1, lower = smoother
    private static final double MIN_SCALE = 0.7, MAX_SCALE = 1.5;      // guards against a bad reading
    private static final double PIDF_SCALE_STEP = 0.01;                // resend the PIDF when the scale moves 1%
    private VoltageSensor battery;
    private double batteryVolts = Double.NaN;  // smoothed, NaN = no reading yet
    private long lastBatteryReadNs = 0;
    private double pidfScale = 1.0;            // the voltage scale the PIDF was last sent with

    public ShooterSubsystem(HardwareMap hardwareMap) {
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
        motorLeft = hardwareMap.get(DcMotorEx.class, "shooterL");
        motorRight = hardwareMap.get(DcMotorEx.class, "shooterR");
        blocker = hardwareMap.get(ServoImplEx.class, "blocker");

        blocker.setPwmRange(AxonPWMRange);

        // setVelocity uses the hub's own velocity PID, so EACH motor needs its own encoder cable
        // (a motor without one reads 0 speed and gets pushed to full power)
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Stopping disables the motors (see periodic), which always coasts
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set so both getVelocity() readings are positive when the flywheel spins the shooting way
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        blocker.setDirection(Servo.Direction.FORWARD);

        // Any hub's voltage sensor reads the same battery
        battery = null;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            battery = sensor;
            break;
        }

        targetRPM = 0.0;
        lastTarget = Double.NaN;
        leftRPM = Double.NaN;
        rightRPM = Double.NaN;
        encoderMismatch = false;
        blockerCommanded = false;
        blockerOpen = false;
        lastBlockerPos = Double.NaN;

        // First battery reading now, so the PIDF sent at INIT is already scaled for this battery
        batteryVolts = Double.NaN;
        lastBatteryReadNs = 0;
        updateBattery();
        pidfScale = getVoltageScale();

        // Always send the PIDF at init: don't rely on values left over from an earlier OpMode
        Arrays.fill(sentLeft, Double.NaN);
        Arrays.fill(sentRight, Double.NaN);
        applyVelocityPIDF();

        isInitialized = true;
    }

    @Override
    public void periodic() {
        if (!isInitialized) return;

        updateBattery();       // a few times a second
        applyVelocityPIDF();   // picks up Dashboard changes while tuning, and battery changes

        // Every loop, even while stopped, so the reading follows the flywheel as it coasts down
        updateMeasuredRPM();

        // Blocker before the flywheel: the flywheel code below returns early while the shooter is stopped
        updateBlocker();

        //stop setting for the motors
        if (targetRPM == 0){
            if (lastTarget != 0){
                // Not setPower(0): in RUN_USING_ENCODER that becomes a speed target of 0, so the hub
                // brakes the flywheel instead of letting it coast. The SDK also skips a setPower(0)
                // if the last setPower (up to 0.5 s ago) was also 0, even after a setVelocity in
                // between, so a quick tap of the button could leave the shooter running.
                // Disabling turns the motor's output off (coast); setVelocity turns it back on.
                motorLeft.setMotorDisable();
                motorRight.setMotorDisable();
                lastTarget = 0;
            }
            return;
        }

        //set both motors with their hub's velocity PID (setVelocity takes ticks per second, not RPM)
        if (targetRPM != lastTarget) {
            double tps = rpmToTps(targetRPM);
            motorLeft.setVelocity(tps);
            motorRight.setVelocity(tps);
            lastTarget = targetRPM;
        }
    }

    /**
     * Reads both encoders once and smooths each one with a low-pass filter:
     *   smoothed = smoothed + ShooterRPMFilter * (new reading - smoothed),
     * so one jittery sample doesn't flip atSpeed(). Then compares the two: on one flywheel they should match.
     */
    private void updateMeasuredRPM() {
        double rawLeft = tpsToRpm(motorLeft.getVelocity());
        double rawRight = tpsToRpm(motorRight.getVelocity());
        double a = Range.clip(SubsystemConstants.ShooterRPMFilter, 0.01, 1.0);   // 0 would freeze the reading
        leftRPM = Double.isNaN(leftRPM) ? rawLeft : leftRPM + a * (rawLeft - leftRPM);
        rightRPM = Double.isNaN(rightRPM) ? rawRight : rightRPM + a * (rawRight - rightRPM);
        encoderMismatch = Math.abs(leftRPM - rightRPM) > SubsystemConstants.ShooterMismatchRPM;
    }

    /**
     * Sends the blocker position, only when it changes. The positions are read every loop,
     * so a Dashboard change moves the servo right away.
     */
    private void updateBlocker() {
        if (!blockerCommanded) return;   // never asked: send nothing
        double pos = blockerOpen ? SubsystemConstants.BlockerOpenPos : SubsystemConstants.BlockerClosedPos;
        if (pos != lastBlockerPos) {
            blocker.setPosition(pos);
            lastBlockerPos = pos;
        }
    }

    /** Reads the battery at most every BATTERY_READ_PERIOD_NS and smooths it. Keeps the old value on a bad reading. */
    private void updateBattery() {
        if (battery == null) return;
        long now = System.nanoTime();
        if (!Double.isNaN(batteryVolts) && now - lastBatteryReadNs < BATTERY_READ_PERIOD_NS) return;
        lastBatteryReadNs = now;

        double volts = battery.getVoltage();
        if (!(volts > 1.0)) return;   // 0 or garbage: not a real battery reading
        batteryVolts = Double.isNaN(batteryVolts) ? volts : batteryVolts + BATTERY_FILTER * (volts - batteryVolts);
    }

    /**
     * Sends each motor its own velocity PIDF from SubsystemConstants, times the voltage scale, only if a
     * value changed since the last send (each send is a hub command, so not every loop). The scale follows
     * the battery in 1% steps, so a slowly draining battery doesn't resend the PIDF on every reading.
     * Old hub firmware throws "setting of pidf coefficients not supported": update the hub firmware if you see that.
     */
    private void applyVelocityPIDF() {
        double scale = getVoltageScale();
        if (Math.abs(scale - pidfScale) >= PIDF_SCALE_STEP || !SubsystemConstants.VoltageCompensation) {
            pidfScale = scale;
        }
        double s = pidfScale;
        sendPIDFIfChanged(motorLeft, sentLeft,
                SubsystemConstants.LeftShooterVelP * s, SubsystemConstants.LeftShooterVelI * s,
                SubsystemConstants.LeftShooterVelD * s, SubsystemConstants.LeftShooterVelF * s);
        sendPIDFIfChanged(motorRight, sentRight,
                SubsystemConstants.RightShooterVelP * s, SubsystemConstants.RightShooterVelI * s,
                SubsystemConstants.RightShooterVelD * s, SubsystemConstants.RightShooterVelF * s);
    }

    /** sent = what this motor got last time, {P, I, D, F}. Skips the hub command if nothing changed. */
    private static void sendPIDFIfChanged(DcMotorEx motor, double[] sent,
                                          double p, double i, double d, double f) {
        if (p == sent[0] && i == sent[1] && d == sent[2] && f == sent[3]) return;

        motor.setVelocityPIDFCoefficients(p, i, d, f);
        sent[0] = p;
        sent[1] = i;
        sent[2] = d;
        sent[3] = f;
    }

    // ----- Abilities that commands can use (all in RPM) -----

    /** Blocker: true = open (the ball can reach the flywheel), false = closed. */
    public void setServoPos(boolean open){
        blockerOpen = open;
        blockerCommanded = true;
    }

    /** True once the blocker has been told to open (closed or never commanded = false). */
    public boolean isBlockerOpen() {
        return blockerCommanded && blockerOpen;
    }

    public void setTargetRPM(double rpm) {
        targetRPM = Range.clip(rpm, 0.0, getMaxRPM());
    }

    public void stop() {
        setTargetRPM(0.0);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getMaxRPM() {
        return SubsystemConstants.ShooterMaxRPM;
    }

    /**
     * Flywheel speed: the average of both smoothed encoders. If they disagree (hasEncoderMismatch) while
     * running, the one closer to the target, since the other is probably unplugged or reversed.
     */
    public double getRPM() {
        double left = getLeftRPM();
        double right = getRightRPM();
        if (encoderMismatch && targetRPM > 0) {
            return Math.abs(left - targetRPM) < Math.abs(right - targetRPM) ? left : right;
        }
        return (left + right) / 2.0;
    }

    /** Left motor speed, smoothed. 0 before the first loop. */
    public double getLeftRPM() {
        return Double.isNaN(leftRPM) ? 0.0 : leftRPM;
    }

    /** Right motor speed, smoothed. 0 before the first loop. */
    public double getRightRPM() {
        return Double.isNaN(rightRPM) ? 0.0 : rightRPM;
    }

    /** True when the two encoders differ by more than ShooterMismatchRPM: check cables, directions, belt. */
    public boolean hasEncoderMismatch() {
        return encoderMismatch;
    }

    /**
     * Both motors within the tolerance and the encoders agree, so one slow (or unplugged) motor
     * can't hide behind the average. Uses the smoothed readings.
     */
    public boolean atSpeed() {
        return targetRPM > 0
                && !encoderMismatch
                && Math.abs(getLeftRPM() - targetRPM) < SubsystemConstants.RPMTolerance
                && Math.abs(getRightRPM() - targetRPM) < SubsystemConstants.RPMTolerance;
    }

    // ----- Battery -----

    /** Battery voltage, smoothed, read a few times a second by periodic(). Free to call: no hub read. 0 = no reading. */
    public double getBatteryVoltage() {
        return Double.isNaN(batteryVolts) ? 0.0 : batteryVolts;
    }

    /**
     * TunedBatteryVolts / battery voltage: multiply a motor power by this so the motor gets the same voltage
     * as when it was tuned (1.00 = as tuned, 1.08 = battery 8% lower, so 8% more power).
     * 1 when VoltageCompensation is off or there's no reading yet. Used for the PIDF here and the
     * shooter-feed intake power in TeleOp.
     */
    public double getVoltageScale() {
        if (!SubsystemConstants.VoltageCompensation || Double.isNaN(batteryVolts)) return 1.0;
        return Range.clip(SubsystemConstants.TunedBatteryVolts / batteryVolts, MIN_SCALE, MAX_SCALE);
    }

    // ----- Unit conversion: TPS is only used inside this class -----
    // Uses the real motor's ticks per revolution from SubsystemConstants, NOT the
    // configured motor type (motor.getMotorType()), which assumes a 20:1 gearbox

    private double rpmToTps(double rpm) {
        return rpm * SubsystemConstants.ShooterTicksPerRev / 60.0;
    }

    private double tpsToRpm(double tps) {
        return tps * 60.0 / SubsystemConstants.ShooterTicksPerRev;
    }
}
