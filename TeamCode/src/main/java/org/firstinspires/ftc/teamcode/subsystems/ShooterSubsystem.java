package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);           // required for setVelocity
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast down instead of braking
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        targetRPM = 0.0;
        lastTarget = Double.NaN;
        isInitialized = true;
    }

    @Override
    public void periodic() {
        if (!isInitialized) return;

        if (targetRPM != lastTarget) {
            if (targetRPM == 0) {
                motor.setPower(0);                          // coast (setVelocity(0) would brake hard)
            } else {
                motor.setVelocity(rpmToTps(targetRPM));     // hub holds this speed on its own
            }
            lastTarget = targetRPM;
        }
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
