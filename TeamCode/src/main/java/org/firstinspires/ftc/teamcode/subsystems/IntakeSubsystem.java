package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.commands.utility.NullCommand;

/*
 * Two intake motors, "intakeL" and "intakeR".
 *   setPower(power, true)   left motor only, the right motor stopped: normal intake
 *   setPower(power)         both motors: shooter feed and outtake
 * The power and which motors run are always set together, so a mode left over from an earlier
 * command can't stick.
 */
public class IntakeSubsystem implements Subsystem{

    private DcMotorEx motorL, motorR;
    private final HardwareMap hardwareMap;
    private Command defaultCommand = new NullCommand();
    private boolean isInitialized = false;

    private double targetPower = 0.0;
    private boolean leftOnly = false;          // true = only the left motor runs, the right one gets 0
    private double lastPower = Double.NaN;     // what was last sent (NaN = nothing sent yet)
    private boolean lastLeftOnly = false;

    public IntakeSubsystem (HardwareMap hardwareMap) {
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
        motorL = hardwareMap.get(DcMotorEx.class, "intakeL");
        motorR = hardwareMap.get(DcMotorEx.class, "intakeR");
        motorL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetPower = 0;
        leftOnly = false;
        lastPower = Double.NaN;   // send on the first loop
        lastLeftOnly = false;
        isInitialized = true;
    }

    @Override
    public void periodic(){
        if (!isInitialized) return;

        // Resend when the power OR the mode changes: going from the feed (both motors) to the normal
        // intake (left only) must stop the right motor, even if the power stays the same
        if (targetPower != lastPower || leftOnly != lastLeftOnly){
            motorL.setPower(targetPower);
            motorR.setPower(leftOnly ? 0.0 : targetPower);
            lastPower = targetPower;
            lastLeftOnly = leftOnly;
        }
    }

    /** Both motors at this power (shooter feed, outtake). -1..1 */
    public void setPower(double power){
        setPower(power, false);
    }

    /** leftOnly = true: the left motor at this power and the right motor stopped (normal intake). false = both. */
    public void setPower(double power, boolean leftOnly){
        targetPower = Range.clip(power, -1.0,1.0);
        this.leftOnly = leftOnly;
    }

    public void stop(){
        setPower(0.0);
    }

    public double getPower(){
        return targetPower;
    }

    /** True while only the left motor is asked to run. */
    public boolean isLeftOnly(){
        return leftOnly;
    }

}
