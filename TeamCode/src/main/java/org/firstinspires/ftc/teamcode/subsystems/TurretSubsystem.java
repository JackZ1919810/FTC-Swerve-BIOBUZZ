package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;

public class TurretSubsystem implements Subsystem {

    private static final PwmControl.PwmRange AxonPWMRange = SubsystemConstants.AxonPWMRange;
    private final HardwareMap hardwareMap;
    private ServoImplEx turretLeft, turretRight;
    private Command defaultCommand = new NullCommand();
    private boolean isInitialized = false;
    private double targetDeg = Double.NaN;
    private double lastLeftPos = Double.NaN;
    private double lastRightPos = Double.NaN;

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

    public void initialize(){
        turretLeft = hardwareMap.get(ServoImplEx.class, "turretLeft");
        turretRight = hardwareMap.get(ServoImplEx.class, "turretRight");

        turretLeft.setPwmRange(AxonPWMRange);
        turretRight.setPwmRange(AxonPWMRange);

        turretLeft.setDirection(ServoImplEx.Direction.FORWARD);
        turretRight.setDirection(ServoImplEx.Direction.REVERSE);

        targetDeg = Double.NaN;
        lastLeftPos = Double.NaN;
        lastRightPos = Double.NaN;
        isInitialized = true;
    }

    public void periodic(){
        if (!isInitialized) return;
        if (Double.isNaN(targetDeg)) return;

        double leftPos = degToPos(targetDeg);

        if (leftPos != lastLeftPos){
            turretLeft.setPosition(leftPos);
            lastLeftPos = leftPos;

        }

        double rightPos = Range.clip
                (leftPos + SubsystemConstants.TurretRightTrim, 0.0, 1.0);

        if (rightPos != lastRightPos){
            turretRight.setPosition(rightPos);
            lastRightPos = rightPos;
        }
    }

    // utility functions
    public void setTargetDeg (double deg){
        targetDeg = Range.clip
                (deg,SubsystemConstants.TurretMinDeg, SubsystemConstants.TurretMaxDeg);
    }

    public void center(){
        setTargetDeg(0.0);
    }

    public double getTargetDeg(){
        return Double.isNaN(targetDeg) ? 0.0 : targetDeg;
    }

    public double getCommandPos(){
        return lastLeftPos;
    }

    private double degToPos(double turretDeg){
        double servoDeg = turretDeg / SubsystemConstants.TurretGearRatio;
        double pos = SubsystemConstants.TurretCenterPos + servoDeg / SubsystemConstants.TurretServoRangeDeg;
        return Range.clip(pos, 0.0, 1.0);
    }


}
