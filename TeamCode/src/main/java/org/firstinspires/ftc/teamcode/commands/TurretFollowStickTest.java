package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;
public class TurretFollowStickTest extends Command{

    //limit the speed of the turret by limiting the loop time
    private static final double Max_Loop_Seconds = 0.1;

    private final TurretSubsystem turret;
    private final DoubleSupplier stick;
    private final BooleanSupplier wrapEnabled;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastStick = 0.0;

    public TurretFollowStickTest(TurretSubsystem turret, DoubleSupplier stick, BooleanSupplier wrapEnabled){
        this.turret = turret;
        this.stick = stick;
        this.wrapEnabled = wrapEnabled;
        addRequirements(turret);
    }

    @Override
    public void start(){
        loopTimer.reset();
    }

    @Override
    public void update(){
        double dt = Math.min(loopTimer.seconds(), Max_Loop_Seconds);
        loopTimer.reset();

        double u = stick.getAsDouble();
        lastStick = u;
        if (Math.abs(u) < SubsystemConstants.TurretStickDeadband) return;

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

