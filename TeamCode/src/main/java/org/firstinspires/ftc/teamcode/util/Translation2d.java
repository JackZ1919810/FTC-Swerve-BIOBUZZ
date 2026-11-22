package org.firstinspires.ftc.teamcode.util;

public class Translation2d {
    public static final Translation2d kZero = new Translation2d(0, 0);
    private final double x;
    private final double y;

    public Translation2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Translation2d() {
        this(0, 0);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Translation2d rotateBy(Rotation2d other) {
        return new Translation2d(
            x * other.cos() - y * other.sin(),
            x * other.sin() + y * other.cos()
        );
    }

    public Translation2d plus(Translation2d other) {
        return new Translation2d(x + other.x, y + other.y);
    }

    public Translation2d minus(Translation2d other) {
        return new Translation2d(x - other.x, y - other.y);
    }

    public Translation2d unaryMinus() {
        return new Translation2d(-x, -y);
    }

    public Translation2d times(double scalar) {
        return new Translation2d(x * scalar, y * scalar);
    }

    public Translation2d div(double scalar) {
        return new Translation2d(x / scalar, y / scalar);
    }

    public double getNorm() {
        return Math.hypot(x, y);
    }

    public double getDistance(Translation2d other) {
        return Math.hypot(x - other.x, y - other.y);
    }

    public double dot(Translation2d other) {
        return x * other.x + y * other.y;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Translation2d) {
            Translation2d other = (Translation2d) obj;
            return Math.abs(other.x - x) < 1e-9 && Math.abs(other.y - y) < 1e-9;
        }
        return false;
    }

    @Override
    public String toString() {
        return String.format(java.util.Locale.US, "Translation2d(X: %.2f, Y: %.2f)", x, y);
    }
}
