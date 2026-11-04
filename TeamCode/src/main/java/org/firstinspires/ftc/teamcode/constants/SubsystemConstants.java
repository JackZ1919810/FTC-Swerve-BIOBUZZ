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
    public static double TurretGearRatio = 1.5;         // turret degrees per servo degree (1:1.5 step-up)
    public static double TurretServoRangeDeg = 350;     // Axon Mini MK2 travel for position 0 -> 1 at 500-2500 us
    public static double TurretCenterPos = 0.5;         // servo position where the turret faces forward (calibrate)
    public static double TurretMinDeg = -180;           // limits: 360 degrees of travel around the robot front (hardware allows +-262.5)
    public static double TurretMaxDeg = 180;
    public static double TurretRightTrim = 0.0;         // position offset so the two servos don't fight
    public static double TurretStickDegPerSec = 180;    // turret speed at full stick in TeleOp
    public static double TurretStickDeadband = 0.05;    // ignore stick drift below this

    //Shooter constants
    // The real motor, not the configured motor type: the SDK's "UltraPlanetary" type assumes a
    // 20:1 gearbox (560 ticks/rev, 255 RPM max), but the shooter motor is 1:1
    public static double ShooterTicksPerRev = 28;   // encoder ticks per motor revolution (1:1)
    public static double ShooterMaxRPM = 5700;      // fastest speed the shooter may be asked for
    public static double defaultRPM = 4900;
    public static double RPMTolerance = 50;
    public static double RPMStep = 100;     // change per d-pad press in TeleOp
    public static double MinRPM = 1000;     // lowest speed the d-pad can select
    public static boolean FeedOnlyAtSpeed = false;  // true = indexer waits until the shooter is at speed
}
