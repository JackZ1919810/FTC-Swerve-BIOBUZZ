package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.FieldMath;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.Rotation2d;
import org.firstinspires.ftc.teamcode.util.Translation2d;

/*
 * BIOBUZZ (2026-2027), FTC field frame, inches:
 *   origin = field center, +Y = away from the Red Wall (toward Blue),
 *   +X = toward the AUDIENCE (red is on the audience's left, so X points at the audience),
 *   heading = degrees counterclockwise from +X (90 = facing away from the Red Wall).
 * Pose2d stores meters, so the helpers below convert.
 */
@Config
public class FieldConstants {
    // ----- HIVE (Competition Manual 9.6, Figures 9-9, 9-10, 9-11, 9-17) -----
    // The HIVE Structure is at the field center. Each alliance has one HIVE: a seesaw along X with a CELL
    // at each end. Only the upward-facing CELL scores. Its opening is the CELL's outer end, so it faces
    // away from the center: the audience CELL opens toward +X, the rear CELL toward -X (tilted 30 deg up).
    public static double CellXIn = 17;          // opening center, distance from the field center along X (estimate, +/-1.5: MEASURE)
    public static double HiveYIn = 13;         // HIVE center line, distance from the field center along Y (25.5 / 2)
    public static double CellOpeningZIn = 58.3;   // opening center height above the TILES (for shooter RPM / hood later)
    public static double MaxApproachDeg = 60;     // farther than this off the opening's facing direction = wrong side

    // ----- Auto start poses: the robot center (the point the OTOS tracks). MEASURE these on the field -----
    // G304: start touching the perimeter wall, fully on your own side (red: Y < 0, blue: Y > 0),
    // not in the LOADING ZONE and not touching a FLOWER. The walls are about 70.5 in from the center.
    // Red: back against the audience wall, in line with the red HIVE, front facing the rear wall.
    // The heading must match how the robot really sits: TeleOp's field-centric drive and the turret aim use it
    public static double RedStartXIn = 65.75;
    public static double RedStartYIn = -13;
    public static double RedStartHeadingDeg = 180;     // front facing the rear wall (-X)
    public static double BlueStartXIn = -65.75;        // blue = red turned 180 deg about the field center
    public static double BlueStartYIn = 13;
    public static double BlueStartHeadingDeg = 0;      // front facing the audience (+X)

    // OTOS sensor measured from the robot center: forward and left are positive. MEASURE
    public static double OtosForwardIn = 0;
    public static double OtosLeftIn = 0;

    // Turret pivot measured from the robot center: forward and left are positive. MEASURE
    public static double TurretPivotForwardIn = 0;
    public static double TurretPivotLeftIn = 0;

    public static double AimTrimDeg = 8.0;          // added to every aim angle: + aims further left (CCW)
    public static double MinAimDistanceIn = 12;   // closer than this, keep the last target

    // TeleOp: the field heading each driver calls "forward" (they stand behind their own wall)
    public static double RedDriverForwardDeg = 90;
    public static double BlueDriverForwardDeg = -90;

    /** Center of a CELL opening. Red HIVE at -Y, blue at +Y; audience CELL at +X, rear CELL at -X. */
    public static Translation2d cell(boolean red, boolean audienceCell) {
        return FieldMath.inches(audienceCell ? CellXIn : -CellXIn, red ? -HiveYIn : HiveYIn);
    }

    /** Field direction a CELL opening faces: 0 deg (+X, audience) or 180 deg (-X, rear). */
    public static double cellFacingDeg(boolean audienceCell) {
        return audienceCell ? 0 : 180;
    }

    public static Pose2d startPose(boolean red) {
        return red ? Pose2d.fromInches(RedStartXIn, RedStartYIn, Rotation2d.fromDegrees(RedStartHeadingDeg))
                   : Pose2d.fromInches(BlueStartXIn, BlueStartYIn, Rotation2d.fromDegrees(BlueStartHeadingDeg));
    }

    public static Rotation2d driverForward(boolean red) {
        return Rotation2d.fromDegrees(red ? RedDriverForwardDeg : BlueDriverForwardDeg);
    }
}
