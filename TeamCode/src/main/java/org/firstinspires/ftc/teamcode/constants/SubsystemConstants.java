package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
@Config
public class SubsystemConstants {
    //Intake constants
    public static double IntakePower = 1.0;
    public static double OuttakePower = -0.6;

    //Indexer constants
    public static double IndexPower = 1.0;
    public static double ReIndexPower = -0.6;

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
