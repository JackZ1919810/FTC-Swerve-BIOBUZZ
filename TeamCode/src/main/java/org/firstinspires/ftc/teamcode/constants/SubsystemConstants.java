package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;

@Config
public class SubsystemConstants {
    //Intake constants
    public static double IntakePower = 1.0;
    public static double OuttakePower = -0.6;

    //Indexer constants
    public static double IndexPower = 1.0;
    public static double ReIndexPower = -0.6;

    //Turret constants
    public static final PwmControl.PwmRange AxonPWMRange =
            new PwmControl.PwmRange(500, 2500);
    public static double TurretGearRatio = 1.3;         // turret degrees per servo degree (1:1.3 step-up, measured)
    public static double TurretServoRangeDeg = 350;     // Axon Mini MK2 travel for position 0 -> 1 at 500-2500 us
    public static double TurretCenterPos = 0.530;         // servo position where the turret faces forward (calibrate)
    public static double TurretMinDeg = -200;           // limits: 400 degree band, wrap-around unwinds here (hardware allows +-227.5)
    public static double TurretMaxDeg = 200;
    public static double TurretRightTrim = -0.3956;         // position offset so the two servos don't fight
    public static double TurretStickDegPerSec = 180;    // turret speed at full stick in TeleOp
    public static double TurretStickDeadband = 0.05;    // ignore stick drift below this
    public static double TurretWrapTestDegPerSec = 450;  // aim speed at full stick in the wrap-around test (start slow)

    //Shooter constants
    // The real motor, not the configured motor type: the SDK's "UltraPlanetary" type assumes a
    // 20:1 gearbox (560 ticks/rev, 255 RPM max), but the shooter motor is 1:1
    public static double ShooterTicksPerRev = 28;   // encoder ticks per motor revolution (1:1)
    public static double ShooterMaxRPM = 5700;      // fastest speed the shooter may be asked for
    public static double defaultRPM = 3000;
    public static double ShooterIdleRPM = 200;
    public static double RPMTolerance = 50;
    public static double RPMStep = 100;     // change per d-pad press in TeleOp
    public static double MinRPM = 1000;     // lowest speed the d-pad can select
    public static boolean FeedOnlyAtSpeed = false;  // true = indexer waits until the shooter is at speed

    // Shooter velocity PIDF: the hub's own velocity loop that setVelocity uses (units are the hub's, not ours).
    // Live-tunable in Dashboard: ShooterSubsystem sends new values to the hub when they change.
    // Starting values from the REV/FIRST PIDF guide: F = 32767 / max ticks per second, P = 0.1 * F, I = 0.1 * P, D = 0
    // Max speed from ShooterKV: 1 / 0.0001925 = 5195 RPM = 2424 ticks/s, so F = 32767 / 2424 = 13.5
    // Better: measure max ticks/s on a full battery (RUN_WITHOUT_ENCODER, power 1, highest getVelocity) and redo F
    public static double ShooterVelP = 1.35;
    public static double ShooterVelI = 0.135;
    public static double ShooterVelD = 0.0;
    public static double ShooterVelF = 13.5;

    // Shooter PID: NOT used right now. ShooterSubsystem runs on setVelocity (the hub's own velocity PID).
    // Kept for switching back to our own controller (see git commit db26c96):
    // power = ShooterKV * targetRPM + PID(targetRPM - measuredRPM), clipped to [0, 1]
    public static double ShooterKV = 0.0001925;  // feedforward power per RPM = 1 / (RPM at full power). Placeholder (1/5800): measure it
    public static double ShooterKP = 0.0005;    // extra power per RPM of error (200 RPM low -> +0.1 power)
    public static double ShooterKI = 0.0;       // start at 0: no limit on the sum in util.PIDController
    public static double ShooterKD = 0.0;       // start at 0: velocity readings are noisy
}
