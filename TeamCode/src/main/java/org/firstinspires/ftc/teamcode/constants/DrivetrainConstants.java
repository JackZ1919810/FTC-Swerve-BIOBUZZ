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
    public static double LEFT_FRONT_OFFSET = 4.225;
    public static double RIGHT_FRONT_OFFSET = 1.323;
    public static double LEFT_REAR_OFFSET = 4.541;
    public static double RIGHT_REAR_OFFSET = 0.476;

    // Servo Inversions
    public static boolean LEFT_FRONT_SERVO_REVERSED = false;
    public static boolean RIGHT_FRONT_SERVO_REVERSED = false;
    public static boolean LEFT_REAR_SERVO_REVERSED = false;
    public static boolean RIGHT_REAR_SERVO_REVERSED = false;

    // Module constants
    public static final double WHEEL_DIAMETER_METERS = 0.072;
    public static final double TICKS_PER_REV = 537.7;
    public static final double TICKS_PER_METER = TICKS_PER_REV / (WHEEL_DIAMETER_METERS * Math.PI);

    // Default Steering PID
    public static double STEER_P = 0.4;
    public static double STEER_I = 0.0;
    public static double STEER_D = 0.01;

    // Individual Steering PID
    // LF Steering
    public static double LF_STEER_P = 0.4;
    public static double LF_STEER_I = 0.0;
    public static double LF_STEER_D = 0.01;
    // RF Steering
    public static double RF_STEER_P = 0.4;
    public static double RF_STEER_I = 0.0;
    public static double RF_STEER_D = 0.01;
    // LR Steering
    public static double LR_STEER_P = 0.4;
    public static double LR_STEER_I = 0.0;
    public static double LR_STEER_D = 0.01;
    // RR Steering
    public static double RR_STEER_P = 0.4;
    public static double RR_STEER_I = 0.0;
    public static double RR_STEER_D = 0.01;

    // "Settled" means slower than this, measured by the OTOS
    public static double SETTLE_SPEED = 0.05;      // m/s (about 2 in/s)
    public static double SETTLE_TURN_RATE = 0.1;   // rad/s (about 6 deg/s)

    // Path following (FollowPath). Meters, seconds, radians.
    public static double PATH_LOOKAHEAD = 0.25;  // m (10 in): how far ahead the robot aims
    public static double PATH_MAX_SPEED = 0.5;   // m/s: start slow, raise once it works
    public static double PATH_MAX_ACCEL = 1.0;   // m/s^2: speeding up and slowing down (start, final stop)
    // m/s^2: changing direction (sideways accel), so corners. Start at about 2x PATH_MAX_ACCEL.
    // Turn accel / speed = how fast the modules must rotate (2.0 at 0.5 m/s = 230 deg/s)
    public static double PATH_TURN_ACCEL = 2.0;
    public static double PATH_HANDOFF   = 0.10;  // m (4 in): the final PID takes over inside this

    // Per-path limits: new FollowPath(drive, DrivetrainConstants.PATH_FAST, waypoints...).
    // Without one, FollowPath uses the PATH_ values above. Each preset is tunable in Dashboard.
    // Raise speed, lookahead and turn accel together (tuning stage 4)
    // (max speed m/s, lookahead m, max accel m/s^2, turn accel m/s^2)
    public static PathConstraints PATH_FAST = new PathConstraints(1.5, 0.40, 1.5, 5.0);  // from the simulation: tune on the robot
    public static PathConstraints PATH_SLOW = new PathConstraints(0.5, 0.25, 1.0, 2.0);
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

    /** Speed and accel limits for one FollowPath. Not final, so Dashboard can change them live. */
    public static class PathConstraints {
        public double maxSpeed;    // m/s: cruise speed
        public double lookahead;   // m: how far ahead the robot aims
        public double maxAccel;    // m/s^2: speeding up and slowing down
        public double turnAccel;   // m/s^2: changing direction (corners)

        public PathConstraints(double maxSpeed, double lookahead, double maxAccel, double turnAccel) {
            this.maxSpeed = maxSpeed;
            this.lookahead = lookahead;
            this.maxAccel = maxAccel;
            this.turnAccel = turnAccel;
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
