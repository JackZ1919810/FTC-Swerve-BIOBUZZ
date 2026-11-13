package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;

public class ShooterSubsystem implements Subsystem {

    private final HardwareMap hardwareMap;
    private Command defaultCommand = new NullCommand();
    private DcMotorEx motor;
    private boolean isInitialized = false;

    private double targetRPM = 0.0;            // what commands ask for
    private double lastTarget = Double.NaN;    // what was last sent to the motor

    // Hub velocity PIDF last sent (NaN = not sent yet), so new values go to the hub only when they change
    private double sentP = Double.NaN;
    private double sentI = Double.NaN;
    private double sentD = Double.NaN;
    private double sentF = Double.NaN;

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
        motor = hardwareMap.get(DcMotorEx.class, "shooter");
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);           // setVelocity uses the hub's own velocity PID
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // stopping disables the motor (see periodic), which always coasts
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Any hub's voltage sensor reads the same battery
        battery = null;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            battery = sensor;
            break;
        }

        targetRPM = 0.0;
        lastTarget = Double.NaN;

        // Always send the PIDF at init: don't rely on values left over from an earlier OpMode
        sentP = Double.NaN;
        sentI = Double.NaN;
        sentD = Double.NaN;
        sentF = Double.NaN;
        applyVelocityPIDF();

        isInitialized = true;
    }

    @Override
    public void periodic() {
        if (!isInitialized) return;

        applyVelocityPIDF();   // picks up Dashboard changes while tuning

        //stop setting for the motor
        if (targetRPM == 0){
            if (lastTarget != 0){
                // Not setPower(0): in RUN_USING_ENCODER that becomes a speed target of 0, so the hub
                // brakes the flywheel instead of letting it coast. The SDK also skips a setPower(0)
                // if the last setPower (up to 0.5 s ago) was also 0, even after a setVelocity in
                // between, so a quick tap of the button could leave the shooter running.
                // Disabling turns the motor's output off (coast); setVelocity turns it back on.
                motor.setMotorDisable();
                lastTarget = 0;
            }
            return;
        }

        //set shooter RPM with the hub's velocity PID (setVelocity takes ticks per second, not RPM)
        if (targetRPM != lastTarget) {
           motor.setVelocity(rpmToTps(targetRPM));
           lastTarget = targetRPM;
        }
    }

    /**
     * Sends the velocity PIDF from SubsystemConstants to the hub, only if a value changed since the last send
     * (each send is a hub command, so not every loop). Old hub firmware throws "setting of pidf coefficients
     * not supported": update the Control Hub / Expansion Hub firmware if you see that.
     */
    private void applyVelocityPIDF() {
        double p = SubsystemConstants.ShooterVelP;
        double i = SubsystemConstants.ShooterVelI;
        double d = SubsystemConstants.ShooterVelD;
        double f = SubsystemConstants.ShooterVelF;
        if (p == sentP && i == sentI && d == sentD && f == sentF) return;

        motor.setVelocityPIDFCoefficients(p, i, d, f);
        sentP = p;
        sentI = i;
        sentD = d;
        sentF = f;
    }

    // ----- Abilities that commands can use (all in RPM) -----

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

    public double getRPM() {
        if (!isInitialized) return 0.0;
        return tpsToRpm(motor.getVelocity());
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

    private double rpmToTps(double rpm) {
        return rpm * SubsystemConstants.ShooterTicksPerRev / 60.0;
    }

    private double tpsToRpm(double tps) {
        return tps * 60.0 / SubsystemConstants.ShooterTicksPerRev;
    }
}
