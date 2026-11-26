package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import dev.nextftc.core.commands.Command;

/**
 * Points the turret at a direction (degrees from the robot's front, the same angles TurretAimAtGoal uses)
 * and finishes immediately. Meant for Autonomous.
 *
 * It goes through setTargetDirection, so the angle is normalized and the turret takes the closest
 * reachable way there (inside the limits and both servos' travel). If no way is reachable, the turret
 * stays where it is.
 *
 * It doesn't wait for the turret to arrive: the servos keep moving there on their own.
 * Put a delay after it if a shot depends on the turret being there.
 */
public class TurretSetPosition extends Command {
    private final TurretSubsystem turret;
    private final double directionDeg;

    public TurretSetPosition(TurretSubsystem turret, double directionDeg) {
        this.turret = turret;
        this.directionDeg = directionDeg;
        addRequirements(turret);
    }

    @Override
    public void start() {
        turret.setTargetDirection(directionDeg);
    }

    @Override
    public boolean isDone() {
        return true;
    }

    /*
    no stop() here: the turret holds that direction until another command changes it
     */
}
