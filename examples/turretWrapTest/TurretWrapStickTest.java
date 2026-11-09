package org.firstinspires.ftc.teamcode.commands;

/*
 * REFERENCE ONLY: the joystick command for testing the wrap-around turret.
 * This file lives outside TeamCode, so it is not compiled into the robot app.
 * It needs the NEW methods from the reference TurretSubsystem (setTargetDirection).
 *
 * Each loop the stick moves the turret like your normal stick command (guide, Part 6), but the
 * new angle goes to the turret in one of two ways:
 *   wrap on : turret.setTargetDirection(theta + dTheta)   treated as a DIRECTION, unwinds at +-200 (Part 7)
 *   wrap off: turret.setTargetDeg(theta + dTheta)         treated as a POSITION, stops at +-200 (Part 4)
 * The test OpMode switches between them with a button, so you can compare both on the robot.
 *
 * The aim is read back from the turret every loop: the turret at position theta faces direction
 * norm(theta), so center() or any other command can move it without the aim falling out of sync.
 */

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;

/** Never ends on its own. Meant to be the turret's default command in the test OpMode. */
public class TurretWrapStickTest extends Command {
    // A slow loop shouldn't turn into a big jump: at most 60 x 0.1 = 6 degrees per loop
    private static final double MAX_LOOP_SECONDS = 0.1;

    private final TurretSubsystem turret;
    private final DoubleSupplier stick;
    private final BooleanSupplier wrapEnabled;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastStick = 0.0;     // for telemetry

    public TurretWrapStickTest(TurretSubsystem turret, DoubleSupplier stick, BooleanSupplier wrapEnabled) {
        this.turret = turret;
        this.stick = stick;
        this.wrapEnabled = wrapEnabled;
        addRequirements(turret);
    }

    @Override
    public void start() {
        loopTimer.reset();
    }

    @Override
    public void update() {
        // Seconds since the last loop, so the speed doesn't depend on loop time
        double dt = Math.min(loopTimer.seconds(), MAX_LOOP_SECONDS);
        loopTimer.reset();

        double u = stick.getAsDouble();
        lastStick = u;
        if (Math.abs(u) < SubsystemConstants.TurretStickDeadband) return;

        // Forward Euler: (no units) x (deg/s) x (s) = deg
        double nextDeg = turret.getTargetDeg() + u * SubsystemConstants.TurretWrapTestDegPerSec * dt;

        if (wrapEnabled.getAsBoolean()) {
            turret.setTargetDirection(nextDeg);     // normalized, then the closest reachable position
        } else {
            turret.setTargetDeg(nextDeg);           // clamped at the limits
        }
    }

    @Override
    public boolean isDone() {
        return false;
    }

    /*
    no stop() here: the turret holds wherever the stick left it
     */

    /** The last stick value read, for telemetry. */
    public double getStick() {
        return lastStick;
    }
}
