package org.firstinspires.ftc.teamcode.util;

public class PIDController {
    private double kP;
    private double kI;
    private double kD;

    private double setpoint;
    private double errorSum;
    private double lastError;
    private double lastTime;
    private double lastRate;

    private boolean continuous;
    private double minInput;
    private double maxInput;

    private double positionTolerance = 0.05;
    private double velocityTolerance = Double.POSITIVE_INFINITY;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setTolerance(double positionTolerance) {
        this.positionTolerance = positionTolerance;
    }

    public void setTolerance(double positionTolerance, double velocityTolerance) {
        this.positionTolerance = positionTolerance;
        this.velocityTolerance = velocityTolerance;
    }

    public void enableContinuousInput(double min, double max) {
        this.continuous = true;
        this.minInput = min;
        this.maxInput = max;
    }

    public double calculate(double measurement) {
        double currentTime = System.nanoTime() / 1e9;
        if (lastTime == 0) lastTime = currentTime;
        double dt = currentTime - lastTime;

        double error = setpoint - measurement;

        if (continuous) {
            double range = maxInput - minInput;
            error %= range;
            if (Math.abs(error) > range / 2.0) {
                error -= Math.copySign(range, error);
            }
        }

        errorSum += error * dt;
        double errorRate = dt > 0 ? (error - lastError) / dt : 0;

        lastError = error;
        lastTime = currentTime;
        lastRate = errorRate;

        return kP * error + kI * errorSum + kD * errorRate;
    }

    public boolean atSetpoint() {
        return Math.abs(lastError) < positionTolerance && Math.abs(lastRate) < velocityTolerance;
    }

    public boolean atPosition(){
        return Math.abs(lastError) < positionTolerance;
    }

    public void reset() {
        errorSum = 0;
        lastError = 0;
        lastTime = 0;
        lastRate = 0;
    }

    public void setP(double kP) { this.kP = kP; }
    public void setI(double kI) { this.kI = kI; }
    public void setD(double kD) { this.kD = kD; }
}
