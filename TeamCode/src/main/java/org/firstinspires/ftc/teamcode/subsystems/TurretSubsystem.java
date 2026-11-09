package org.firstinspires.ftc.teamcode.subsystems;

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
    private ServoImplEx turretLeft, turretRight;
    private Command defaultCommand = new NullCommand();
    private boolean isInitialized = false;
    private double targetDeg = Double.NaN;
    private double lastLeftPos = Double.NaN;
    private double lastRightPos = Double.NaN;

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
    public void initialize(){
        turretLeft = hardwareMap.get(ServoImplEx.class, "turretLeft");
        //turretRight = hardwareMap.get(ServoImplEx.class, "turretRight");

        turretLeft.setPwmRange(AxonPWMRange);
        //turretRight.setPwmRange(AxonPWMRange);

        turretLeft.setDirection(ServoImplEx.Direction.FORWARD);
        //turretRight.setDirection(ServoImplEx.Direction.REVERSE);

        targetDeg = Double.NaN;
        lastLeftPos = Double.NaN;
        //lastRightPos = Double.NaN;
        lastDirectionDeg = Double.NaN;
        lastCandidateCount = 0;
        unwindCount = 0;
        isInitialized = true;
    }

    @Override
    public void periodic(){
        if (!isInitialized) return;
        if (Double.isNaN(targetDeg)) return;

        double leftPos = degToPos(targetDeg);

        if (leftPos != lastLeftPos){
            turretLeft.setPosition(leftPos);
            lastLeftPos = leftPos;
        }

        /*
        double rightPos = Range.clip
                (leftPos + SubsystemConstants.TurretRightTrim, 0.0, 1.0);

        if (rightPos != lastRightPos){
            turretRight.setPosition(rightPos);
            lastRightPos = rightPos;
        }

         */
    }

    // utility functions: math functions

    public static double clipToLimits (double deg){
        return Range.clip
                (deg,SubsystemConstants.TurretMinDeg, SubsystemConstants.TurretMaxDeg);
    }

    public static double normalizeDeg (double deg){
        return deg - 360 * Math.ceil((deg-180)/360);
    }

    private double degToPos(double turretDeg){
        double pos = SubsystemConstants.TurretCenterPos
                + turretDeg / (SubsystemConstants.TurretServoRangeDeg * SubsystemConstants.TurretGearRatio);
        return Range.clip(pos, 0, 1);
    }

    public static double[] reachablePositions(double directionDeg, double minDeg, double maxDeg){
        //find the normalized a
        double a = normalizeDeg(directionDeg);

        //construct an empty list and its length counter
        double[] found = new double[3];
        int count = 0;

        //construct a list for three calculated candidates
        double[] candidates = {a - 360.0, a, a + 360.0};

        //for loop to figure out which candidates are in the reachable range
        for (int i = 0; i < candidates.length; i++){
            double candidate = candidates[i];
            if (candidate >= minDeg && candidate <= maxDeg) {
                //add the candidate in the range into the empty list
                found[count] = candidate;
                count = count + 1;
            }
        }

        //copy the "found" list, with "count" number of objects, and return
        return Arrays.copyOf(found,count);
    }

    //utility functions: setting positions

    /** this is a direct set position, no wrapping mechanism will be called if use this */
    public void setTargetDeg (double deg){
        if (Double.isNaN(deg)) return;
        targetDeg = clipToLimits(deg);
    }

    /** this is the setting position by imputing a direction, have the complete algorithm in part 7 of my math note*/
    public void setTargetDirection(double directionDeg){
        if (Double.isNaN(directionDeg)) return;

        //normalize the input deg
        double direction = normalizeDeg(directionDeg);

        //find the current position
        double current = getTargetDeg();

        //find the candidates
        double[] candidates = reachablePositions
                (directionDeg, SubsystemConstants.TurretMinDeg,SubsystemConstants.TurretMaxDeg);

        //for caching
        lastDirectionDeg = direction;
        lastCandidateCount = candidates.length;

        //this prevents the turret doing random things when there is no candidate
        if(candidates.length == 0) return;

        //picking the best candidate
        double best = candidates[0];
        for (int i = 0; i < candidates.length; i++){
            double candidate = candidates[i];

            //compare each candidate with the best one, if it's closer, then replace the last best one
            if(Math.abs(candidate - current) < Math.abs(best - current)){
                best = candidate;
            }
        }

        //counting times when we unwind the cable
        if(Math.abs(best - current) > 180.0) unwindCount++;

        setTargetDeg(best);
    }

    public void center(){
        setTargetDeg(0.0);
    }

    //utility functions: reading
    public double getTargetDeg(){
        return Double.isNaN(targetDeg) ? 0.0 : targetDeg;
    }

    public double getCommandPos(){
        return lastLeftPos;
    }

    public double getLastLDirectionDeg(){
        return lastDirectionDeg;
    }

    public int getLastCandidateCount(){
        return lastCandidateCount;
    }

    public int getUnwindCount(){
        return unwindCount;
    }
}
