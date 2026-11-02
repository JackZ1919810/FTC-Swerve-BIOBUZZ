package org.firstinspires.ftc.teamcode.commands;

/*
 * REFERENCE EXAMPLE ONLY. This file lives outside TeamCode, so it is not compiled into the robot app.
 *
 * TeleOp wiring (same places as the shooter):
 *   onInit():
 *       turret = new TurretSubsystem(hardwareMap);
 *       turret.initialize();
 *       ...add turret to the SubsystemComponent
 *   onStartButtonPressed():
 *       turret.center();   // first move happens at START, not INIT
 *       turret.setDefaultCommand(new TurretFollowStick(turret, () -> -gamepad2.right_stick_x));
 *       Gamepads.gamepad2().a().whenBecomesTrue(() -> turret.center());
 *   onUpdate():
 *       telemetry: turret.getTargetDegrees(), turret.getCommandedPosition()
 *
 * The stick is negated like the drive's turn input: pushing right turns the turret clockwise (negative degrees).
 */

import java.util.function.DoubleSupplier;

import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystemExample;

/**
 * Every loop, moves the turret's target by stick * speed, so the turret stays where you leave it
 * (positive = counterclockwise). Never ends on its own. Meant to be a default command.
 */
public class TurretFollowStickExample extends Command {
    // A slow loop shouldn't turn into a big jump
    private static final double MAX_LOOP_SECONDS = 0.1;

    private final TurretSubsystemExample turret;
    private final DoubleSupplier stick;
    private final ElapsedTime loopTimer = new ElapsedTime();

    public TurretFollowStickExample(TurretSubsystemExample turret, DoubleSupplier stick) {
        this.turret = turret;
        this.stick = stick;
        addRequirements(turret);
    }

    @Override
    public void start() {
        loopTimer.reset();
    }

    @Override
    public void update() {
        // Seconds since the last loop, so the turret speed doesn't depend on loop time
        double dt = Math.min(loopTimer.seconds(), MAX_LOOP_SECONDS);
        loopTimer.reset();

        double input = stick.getAsDouble();
        if (Math.abs(input) < SubsystemConstants.TurretStickDeadband) return;

        // The subsystem clips this to the turret's limits
        turret.setTargetDegrees(turret.getTargetDegrees() + input * SubsystemConstants.TurretStickDegPerSec * dt);
    }

    @Override
    public boolean isDone() {
        return false;
    }

    /*
    no stop() here: the turret holds wherever the stick left it
     */
}
