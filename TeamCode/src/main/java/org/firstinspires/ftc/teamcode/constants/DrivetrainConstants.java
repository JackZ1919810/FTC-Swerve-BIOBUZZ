package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.util.Translation2d;

@Config
public class DrivetrainConstants {
    // Module locations (m)
    public static final double WHEEL_BASE_METERS = 0.2286; // 9 inches
    public static final double TRACK_WIDTH_METERS = 0.2286; // 9 inches

    public static final Translation2d LEFT_FRONT_LOCATION = new Translation2d(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d RIGHT_FRONT_LOCATION = new Translation2d(WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d LEFT_REAR_LOCATION = new Translation2d(-WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d RIGHT_REAR_LOCATION = new Translation2d(-WHEEL_BASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);

    // Physical limits
    public static final double MAX_SPEED_MPS = 2.033; // 6.67 ft/s
    // Real max angular velocity is constrained by max module speed and distance from center
    public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = MAX_SPEED_MPS / Math.hypot(WHEEL_BASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0);

    // Encoder Offsets (radians)
    public static double LEFT_FRONT_OFFSET = 2.983;
    public static double RIGHT_FRONT_OFFSET = 4.893;
    public static double LEFT_REAR_OFFSET = 2.414;
    public static double RIGHT_REAR_OFFSET = 2.562;

    // Servo Inversions
    public static boolean LEFT_FRONT_SERVO_REVERSED = false;
    public static boolean RIGHT_FRONT_SERVO_REVERSED = false;
    public static boolean LEFT_REAR_SERVO_REVERSED = false;
    public static boolean RIGHT_REAR_SERVO_REVERSED = false;

    // Module constants
    public static final double WHEEL_DIAMETER_METERS = 0.072;
    public static final double TICKS_PER_REV = 537.7;
    public static final double TICKS_PER_METER = TICKS_PER_REV / (WHEEL_DIAMETER_METERS * Math.PI);

    // Steering PID
    public static double STEER_P = 0.4;
    public static double STEER_I = 0.0;
    public static double STEER_D = 0.01;

    public static final DrivetrainPIDConstants autoPIDConstants = new DrivetrainPIDConstants(
            new PIDCoefficients(6.5, 0.0, 0.3), //this P value make the wheel adjusting a bit, a higher P value would decrease that.
            new PIDCoefficients(7.0, 0.0, 0.2),
            new Tolerances(0.02, 0.01), //lower the tolerance once the robot is tuned to accurate
            new Tolerances(Math.toRadians(3.0), 0.01)
    );

    public final static class PIDCoefficients {
        public final double kP;
        public final double kI;
        public final double kD;

        public PIDCoefficients(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }

    public final static class Tolerances {
        public final double position;
        public final double velocity;

        public Tolerances(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }
    }

    public final static class DrivetrainPIDConstants {
        public final PIDCoefficients translationCoefficients;
        public final PIDCoefficients rotationCoefficients;
        public final Tolerances translationTolerances;
        public final Tolerances rotationTolerances;

        public DrivetrainPIDConstants(
                PIDCoefficients translationCoefficients,
                PIDCoefficients rotationCoefficients,
                Tolerances translationTolerances,
                Tolerances rotationTolerances) {
            this.translationCoefficients = translationCoefficients;
            this.rotationCoefficients = rotationCoefficients;
            this.translationTolerances = translationTolerances;
            this.rotationTolerances = rotationTolerances;
        }
    }

}
