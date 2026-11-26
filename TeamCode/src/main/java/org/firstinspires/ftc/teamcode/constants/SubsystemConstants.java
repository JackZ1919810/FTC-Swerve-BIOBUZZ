package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;

@Config
public class SubsystemConstants {
    //Intake constants
    public static double IntakePower = 1.0;
    public static double OuttakePower = -0.6;
    public static double SlowIntakePower = 0.3;

    //Turret constants
    public static final PwmControl.PwmRange AxonPWMRange =
            new PwmControl.PwmRange(500, 2500);
    public static double TurretGearRatio = 1.3;         // turret degrees per servo degree (1:1.3 step-up, measured)
    public static double TurretServoRangeDeg = 350;     // Axon Mini MK2 travel for position 0 -> 1 at 500-2500 us
    public static double TurretCenterPos = 0.500;         // servo position where the turret faces forward (calibrate)
    public static double TurretMinDeg = -200;           // limits: 400 degree band, wrap-around unwinds here (hardware allows +-227.5)
    public static double TurretMaxDeg = 200;
    // Right servo position = left servo position + trim, so the two servos don't fight.
    // 0 = both servos meshed at 0.5. To measure it: TurretServoReadOpMode, trim = Right - Left
    // (the old mesh needed -0.3956)
    public static double TurretRightTrim = 0.0;

    // Turret servo analog feedback (TurretServoReadOpMode): feedback angle = voltage / max voltage * 360
    // servo position = 0.5 + (feedback angle - center) / TurretServoRangeDeg, the difference wrapped to -180..180
    // Center = the feedback angle when that servo sits at position 0.5. 180 is a guess until calibrated
    // (X / B in TurretServoReadOpMode)
    public static double TurretLeftFeedbackCenterDeg = 178;    // left servo feedback angle at position 0.5
    public static double TurretRightFeedbackCenterDeg = 178;   // right servo feedback angle at position 0.5
    public static boolean TurretLeftFeedbackReversed = true;  // flip if the reading goes down when the servo turns toward position 1
    public static boolean TurretRightFeedbackReversed = true;
    // Hand-set zero: before INIT, point the turret straight forward by hand; TurretSubsystem reads both
    // feedback wires until its first command and uses that as center and trim (instead of the two above).
    // false = use TurretCenterPos and TurretRightTrim as before
    public static boolean TurretZeroFromFeedback = true;
    public static double TurretStickDegPerSec = 180;    // turret speed at full stick in TeleOp
    public static double TurretStickDeadband = 0.05;    // ignore stick drift below this
    public static double TurretWrapTestDegPerSec = 450;  // aim speed at full stick in the wrap-around test (start slow)

    //Shooter constants
    // The real motor, not the configured motor type: the SDK's "UltraPlanetary" type assumes a
    // 20:1 gearbox (560 ticks/rev, 255 RPM max), but the shooter motor is 1:1
    public static double ShooterTicksPerRev = 28;   // encoder ticks per motor revolution (1:1)
    public static double ShooterMaxRPM = 6000;      // fastest speed the shooter may be asked for
    public static double defaultRPM = 3000;
    public static double ShooterIdleRPM = 200;
    public static double RPMTolerance = 80;
    public static double ShooterRPMFilter = 0.3;     // 0..1 smoothing of the measured RPM: lower = smoother but slower, 1 = off
    public static double ShooterMismatchRPM = 150;   // left and right encoders differ by more than this = one is wrong
    public static double RPMStep = 100;     // change per d-pad press in TeleOp
    public static double MinRPM = 1000;     // lowest speed the d-pad can select
    public static boolean FeedOnlyAtSpeed = false;  // true = indexer waits until the shooter is at speed

    // Shooter interpolation table: shooter RPM from the turret-pivot-to-CELL floor distance
    // (TeleOp telemetry "Shot distance"). Distances smallest first, one RPM for each distance.
    // PLACEHOLDERS: every RPM is defaultRPM until measured, so turning the table on early changes nothing
    public static boolean ShooterUseTable = false;  // false = D-pad picks the RPM (use this while measuring)
    public static double[] ShooterTableDistIn = {24, 36, 48, 60, 72, 84};
    public static double[] ShooterTableRPM    = {3000, 3000, 3000, 3000, 3000, 3000};

    // Blocker servo ("blocker", Axon, 500-2500 us): holds the ball back until it's time to shoot.
    // 0.5 -> 1.0 is half the servo's travel (about 175 deg): tune both in Dashboard
    public static double BlockerOpenPos = 0.95;      // the ball can pass
    public static double BlockerClosedPos = 0.6;    // the ball is held back

    // Flower intake servo ("flower", Axon, 500-2500 us): opens on a command and stays open until the next one.
    // Placeholders: 0.5 -> 1.0 is about 175 deg of travel, tune both in Dashboard
    public static double FlowerOpenPos = 0.8;
    public static double FlowerClosedPos = 0.4;

    // Starting values from the REV/FIRST PIDF guide: F = 32767 / max ticks per second, P = 0.1 * F, I = 0.1 * P, D = 0
    // Max speed from ShooterKV: 1 / 0.0001925 = 5195 RPM = 2424 ticks/s, so F = 32767 / 2424 = 13.5
    // Better: measure max ticks/s on a full battery (RUN_WITHOUT_ENCODER, power 1, highest getVelocity) and redo F
    //
    // Battery voltage compensation: the PIDF below (and SlowIntakePower, the shooter feed) are multiplied by
    // TunedBatteryVolts / battery voltage, so the motors get the same voltage on a drained battery as when tuned.
    // The values below are "at TunedBatteryVolts": keep this number fixed once set, and retune with it on
    public static boolean VoltageCompensation = true;
    public static double TunedBatteryVolts = 13.0;   // telemetry "Battery (V)" while the PIDF / SlowIntakePower were tuned
    public static double RightShooterVelP = 2.0;
    public static double RightShooterVelI = 0.10;
    public static double RightShooterVelD = 0.0;
    public static double RightShooterVelF = 15.2;
    public static double LeftShooterVelP = 2.0;
    public static double LeftShooterVelI = 0.10;
    public static double LeftShooterVelD = 0.0;
    public static double LeftShooterVelF = 15.2;

    // Shooter PID: what ShooterSubsystem runs on (RUN_WITHOUT_ENCODER). Live-tunable in Dashboard.
    // power = ShooterKV * targetRPM + PID(targetRPM - measuredRPM), clipped to [0, 1], same power to both motors
    public static double ShooterKV = 0.0001925;  // feedforward power per RPM = 1 / (RPM at full power). Placeholder (1/5800): measure it
    public static double ShooterKP = 0.0005;    // extra power per RPM of error (200 RPM low -> +0.1 power)
    public static double ShooterKI = 0.0;       // start at 0: no limit on the sum in util.PIDController
    public static double ShooterKD = 0.0;       // start at 0: velocity readings are noisy
    // Which encoders the PID reads (it averages the ones that are on). Turn off one that isn't plugged in:
    // it reads 0, drags the average down, and the PID pushes full power
    public static boolean ShooterLeftEncoder = true;
    public static boolean ShooterRightEncoder = true;
}
