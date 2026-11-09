package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.commands.utility.NullCommand;
public class IntakeSubsystem implements Subsystem{

    private DcMotorEx motorL, motorR;
    private final HardwareMap hardwareMap;
    private Command defaultCommand = new NullCommand();
    private boolean isInitialized = false;
    private double targetPower = 0.0;
    private double lastPower = Double.NaN;

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
        motorL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetPower = 0;
        isInitialized = true;
    }

    @Override
    public void periodic(){
        if (!isInitialized) return;

        if (targetPower != lastPower){
            motorL.setPower(targetPower);
            motorR.setPower(targetPower);
            lastPower = targetPower;
        }
    }

    public void setPower(double power){
        targetPower = Range.clip(power, -1.0,1.0);
    }

    public void stop(){
        setPower(0.0);
    }

    public double getPower(){
        return targetPower;
    }

}
