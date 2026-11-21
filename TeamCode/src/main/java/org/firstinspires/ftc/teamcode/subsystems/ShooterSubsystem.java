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

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.util.PIDController;

/*
 * Shooter on our own speed control (RUN_WITHOUT_ENCODER): every loop,
 *   power = ShooterKV * targetRPM + PID(targetRPM - measured RPM), clipped to 0..1,
 * and the SAME power goes to both motors, so two motors on one flywheel never fight.
 * "Without encoder" only means the hub doesn't run its own speed loop: the encoders are still read.
 */
public class ShooterSubsystem implements Subsystem {

    private final HardwareMap hardwareMap;
    private Command defaultCommand = new NullCommand();
    private DcMotorEx motorRight, motorLeft;
    private ServoImplEx blocker;
    private boolean isInitialized = false;

    private double targetRPM = 0.0;            // what commands ask for (both motors)
    private double lastTarget = Double.NaN;    // the target the PID was last set up for
    private double power = 0.0;                // what was last sent to both motors
    private final PIDController pid = new PIDController(
            SubsystemConstants.ShooterKP, SubsystemConstants.ShooterKI, SubsystemConstants.ShooterKD);

    // Blocker: nothing is sent until a command asks, so it doesn't move during INIT.
    // The open / closed positions are in SubsystemConstants (Dashboard)
    private boolean blockerCommanded = false;      // false until the first setServoPos
    private boolean blockerOpen = false;
    private double lastBlockerPos = Double.NaN;    // what was last sent to the servo
    private static final PwmControl.PwmRange AxonPWMRange = SubsystemConstants.AxonPWMRange;

    // Tuning telemetry: the battery voltage
    private VoltageSensor battery;

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

        // setPower is raw power: our PID below does the speed control, not the hub
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Power 0 coasts the flywheel down instead of braking it
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set so both getVelocity() readings are positive when the flywheel spins the shooting way
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
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
        power = 0.0;
        pid.reset();
        blockerCommanded = false;
        blockerOpen = false;
        lastBlockerPos = Double.NaN;

        isInitialized = true;
    }

    @Override
    public void periodic() {
        if (!isInitialized) return;

        // Blocker before the flywheel: the flywheel code below returns early while the shooter is stopped
        updateBlocker();

        // Picks up Dashboard changes while tuning
        pid.setP(SubsystemConstants.ShooterKP);
        pid.setI(SubsystemConstants.ShooterKI);
        pid.setD(SubsystemConstants.ShooterKD);

        //stop setting for the motors
        if (targetRPM == 0){
            if (lastTarget != 0){
                setBothPower(0.0);   // FLOAT: coasts down
                pid.reset();
                lastTarget = 0;
            }
            return;
        }

        // New target: start the PID fresh (no leftover I sum or D jump from the old target)
        if (targetRPM != lastTarget) {
            pid.reset();
            pid.setSetpoint(targetRPM);
            lastTarget = targetRPM;
        }

        double feedforward = SubsystemConstants.ShooterKV * targetRPM;   // about the power this speed needs
        double feedback = pid.calculate(getRPM());                        // corrects what the feedforward misses
        setBothPower(Range.clip(feedforward + feedback, 0.0, 1.0));       // never backward
    }

    /** Same power to both motors, sent only when it changes. */
    private void setBothPower(double newPower) {
        if (newPower == power) return;
        motorLeft.setPower(newPower);
        motorRight.setPower(newPower);
        power = newPower;
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
     * Flywheel speed the PID uses: the average of the encoders that are turned on in SubsystemConstants
     * (ShooterLeftEncoder / ShooterRightEncoder). Turn off one that isn't plugged in, or it reads 0
     * and drags the average down, so the PID pushes full power.
     */
    public double getRPM() {
        boolean left = SubsystemConstants.ShooterLeftEncoder;
        boolean right = SubsystemConstants.ShooterRightEncoder;
        if (left && right) return (getLeftRPM() + getRightRPM()) / 2.0;
        if (left) return getLeftRPM();
        if (right) return getRightRPM();
        return 0.0;   // no encoder: atSpeed() can never be true
    }

    public double getLeftRPM() {
        if (!isInitialized) return 0.0;
        return tpsToRpm(motorLeft.getVelocity());
    }

    public double getRightRPM() {
        if (!isInitialized) return 0.0;
        return tpsToRpm(motorRight.getVelocity());
    }

    /** The power both motors get right now (0..1), for tuning ShooterKV. */
    public double getPower() {
        return power;
    }

    public boolean atSpeed() {
        return targetRPM > 0
                && Math.abs(getRPM() - targetRPM) < SubsystemConstants.RPMTolerance;
    }

    // ----- Tuning telemetry -----

    /** Battery voltage. Each call is a separate hub read, so call it once per loop at most. */
    public double getBatteryVoltage() {
        return battery == null ? 0.0 : battery.getVoltage();
    }

    // ----- Unit conversion: TPS is only used inside this class -----
    // Uses the real motor's ticks per revolution from SubsystemConstants, NOT the
    // configured motor type (motor.getMotorType()), which assumes a 20:1 gearbox

    private double tpsToRpm(double tps) {
        return tps * 60.0 / SubsystemConstants.ShooterTicksPerRev;
    }
}
