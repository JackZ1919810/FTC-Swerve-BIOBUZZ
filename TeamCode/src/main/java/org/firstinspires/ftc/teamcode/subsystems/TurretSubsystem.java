package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;

import java.util.Arrays;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;

public class TurretSubsystem implements Subsystem {

    private static final PwmControl.PwmRange AxonPWMRange = SubsystemConstants.AxonPWMRange;
    private static final double ZERO_FILTER = 0.2;   // 0..1: how fast the zero follows new readings during INIT
    private final HardwareMap hardwareMap;
    private ServoImplEx turretLeft, turretRight;
    private AnalogInput feedbackLeft, feedbackRight;
    private double leftMaxVolts, rightMaxVolts;
    private Command defaultCommand = new NullCommand();
    private boolean isInitialized = false;
    private double targetDeg = Double.NaN;
    private double lastLeftPos = Double.NaN;
    private double lastRightPos = Double.NaN;
    private double measuredLeftPos = Double.NaN;
    private double measuredRightPos = Double.NaN;

    private double lastDirectionDeg = Double.NaN;
    private int lastCandidateCount = 0;
    private int unwindCount = 0;

    // Hand-set zero. Static, so it survives switching OpModes (Auto -> TeleOp) and is cleared when the
    // robot is power-cycled (the Robot Controller app restarts). Measured while the servos are limp,
    // frozen at the first servo command, because from then on the Axons hold until power is cut
    private static boolean zeroLocked = false;
    private static double zeroCenterPos = Double.NaN;   // left servo position with the turret facing forward
    private static double zeroRightTrim = Double.NaN;   // right servo position minus left, at the same moment

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
        turretRight = hardwareMap.get(ServoImplEx.class, "turretRight");

        // setPwmRange only configures the port; no signal goes out until the first setPosition
        turretLeft.setPwmRange(AxonPWMRange);
        turretRight.setPwmRange(AxonPWMRange);

        turretLeft.setDirection(ServoImplEx.Direction.REVERSE);
        turretRight.setDirection(ServoImplEx.Direction.REVERSE);

        feedbackLeft = hardwareMap.get(AnalogInput.class, "turretEncoderL");
        feedbackRight = hardwareMap.get(AnalogInput.class, "turretEncoderR");
        leftMaxVolts = feedbackLeft.getMaxVoltage();
        rightMaxVolts = feedbackRight.getMaxVoltage();

        targetDeg = Double.NaN;
        lastLeftPos = Double.NaN;
        lastRightPos = Double.NaN;
        lastDirectionDeg = Double.NaN;
        lastCandidateCount = 0;
        unwindCount = 0;
        isInitialized = true;
    }

    @Override
    public void periodic(){
        if (!isInitialized) return;

        // Feedback wires, every loop (part of the bulk read). Before the first command the servos are
        // limp, so this is wherever the drive team pointed the turret by hand: straight forward
        readFeedback();
        if (!zeroLocked) updateZero();

        if (Double.isNaN(targetDeg)) return;

        // First command since power-on: freeze the zero, because the servos hold from now on
        zeroLocked = true;

        double leftPos = degToPos(targetDeg);

        if (leftPos != lastLeftPos){
            turretLeft.setPosition(leftPos);
            lastLeftPos = leftPos;
        }

        // Both servos drive the same gear, so the right one follows the left one
        double rightPos = Range.clip(leftPos + rightTrim(), 0.0, 1.0);

        if (rightPos != lastRightPos){
            turretRight.setPosition(rightPos);
            lastRightPos = rightPos;
        }
    }

    // ----- hand-set zero -----

    private void readFeedback() {
        measuredLeftPos = feedbackToPos(feedbackLeft.getVoltage(), leftMaxVolts,
                SubsystemConstants.TurretLeftFeedbackCenterDeg, SubsystemConstants.TurretLeftFeedbackReversed);
        measuredRightPos = feedbackToPos(feedbackRight.getVoltage(), rightMaxVolts,
                SubsystemConstants.TurretRightFeedbackCenterDeg, SubsystemConstants.TurretRightFeedbackReversed);
    }

    /** Low-pass filter the readings, so one noisy sample doesn't become the zero. */
    private void updateZero() {
        double trim = measuredRightPos - measuredLeftPos;
        if (Double.isNaN(zeroCenterPos)) {
            zeroCenterPos = measuredLeftPos;
            zeroRightTrim = trim;
        } else {
            zeroCenterPos += ZERO_FILTER * (measuredLeftPos - zeroCenterPos);
            zeroRightTrim += ZERO_FILTER * (trim - zeroRightTrim);
        }
    }

    /**
     * Servo position (this subsystem's setPosition scale) from a feedback voltage.
     * Feedback angle = volts / max volts * 360; position = 0.5 + (angle - center) / range,
     * with the difference wrapped to -180..180. Center = the feedback angle at position 0.5.
     */
    public static double feedbackToPos(double volts, double maxVolts, double centerDeg, boolean reversed) {
        double angle = volts / maxVolts * 360.0;
        double fromCenter = normalizeDeg(angle - centerDeg);
        if (reversed) fromCenter = -fromCenter;
        return 0.5 + fromCenter / SubsystemConstants.TurretServoRangeDeg;
    }

    /** Left servo position with the turret facing forward: the hand-set zero, or the constant. */
    public static double centerPos() {
        return useZero() ? zeroCenterPos : SubsystemConstants.TurretCenterPos;
    }

    /** Right servo position minus left: the hand-set zero, or the constant. */
    public static double rightTrim() {
        return useZero() ? zeroRightTrim : SubsystemConstants.TurretRightTrim;
    }

    private static boolean useZero() {
        return SubsystemConstants.TurretZeroFromFeedback && !Double.isNaN(zeroCenterPos);
    }

    /** Turret degrees for the full servo travel (position 0 -> 1): 350 * 1.3 = 455. */
    private static double degPerPos() {
        return SubsystemConstants.TurretServoRangeDeg * SubsystemConstants.TurretGearRatio;
    }

    /** Lowest turret angle: the limit, or less if a servo would run out of travel first. */
    public static double minDeg() {
        double lowestLeftPos = Math.max(0.0, -rightTrim());      // right = left + trim must stay >= 0
        return Math.max(SubsystemConstants.TurretMinDeg, (lowestLeftPos - centerPos()) * degPerPos());
    }

    /** Highest turret angle: the limit, or less if a servo would run out of travel first. */
    public static double maxDeg() {
        double highestLeftPos = Math.min(1.0, 1.0 - rightTrim()); // right = left + trim must stay <= 1
        return Math.min(SubsystemConstants.TurretMaxDeg, (highestLeftPos - centerPos()) * degPerPos());
    }

    /** True when the reach covers a full circle, so every direction has an angle to aim with. */
    public static boolean reachesEveryDirection() {
        return maxDeg() - minDeg() >= 360.0;
    }

    /** Where the zero comes from right now. */
    public static String getZeroStatus() {
        if (!useZero()) return "CONSTANTS (SubsystemConstants)";
        return zeroLocked ? "LOCKED (hand-set this power-on)" : "MEASURING: point the turret straight forward";
    }

    // utility functions: math functions

    public static double clipToLimits (double deg){
        double lo = minDeg();
        double hi = Math.max(lo, maxDeg());   // never let the range turn inside out
        return Range.clip(deg, lo, hi);
    }

    public static double normalizeDeg (double deg){
        return deg - 360 * Math.ceil((deg-180)/360);
    }

    private double degToPos(double turretDeg){
        double pos = centerPos() + turretDeg / degPerPos();
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

        //find the candidates (inside the limits AND inside both servos' travel)
        double[] candidates = reachablePositions(directionDeg, minDeg(), maxDeg());

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

    /** The last position sent to the right servo. */
    public double getRightCommandPos(){
        return lastRightPos;
    }

    /** Left servo position read from its feedback wire this loop (same scale as getCommandPos). */
    public double getMeasuredLeftPos(){
        return measuredLeftPos;
    }

    /** Right servo position read from its feedback wire this loop (same scale as getRightCommandPos). */
    public double getMeasuredRightPos(){
        return measuredRightPos;
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
