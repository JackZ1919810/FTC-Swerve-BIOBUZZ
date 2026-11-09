package org.firstinspires.ftc.teamcode.teleOp;

/*
 * REFERENCE ONLY: a test OpMode for the wrap-around turret.
 * This file lives outside TeamCode, so it is not compiled into the robot app.
 * It needs the reference TurretSubsystem and TurretWrapStickTest from this folder.
 *
 * Turret only: the drivetrain, intake and shooter are not created, so the robot stays still.
 * One servo only: "turretLeft" in the robot configuration.
 *
 * Controls (gamepad1):
 *   Right stick X   turn the turret (push right = clockwise)
 *   B               switch between WRAP (unwinds at the limits) and CLAMP (stops at the limits); starts in WRAP
 *   A               center: go to position 0 (robot front)
 *   Y               face straight backward (direction 180): shows which side the turret picks
 *
 * Before the first run:
 *   - check that the cables can take the full 400 degrees (-200 to 200)
 *   - keep hands clear: unwinding is a fast swing of about 360 degrees (about 0.44 s)
 *   - start with TurretWrapTestDegPerSec = 60 (Dashboard), and raise it once it behaves
 */

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.commands.TurretWrapStickTest;
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "Turret Wrap Test", group = "Test")
public class TurretWrapTest extends NextFTCOpMode {
    private TurretSubsystem turret;
    private TurretWrapStickTest stickCommand;   // null until START
    private boolean wrapEnabled = true;

    @Override
    public void onInit() {
        // Redirect telemetry to FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new TurretSubsystem(hardwareMap);

        /*
        Same as TeleOp: addComponents() is called inside onInit, which is too late
        for NextFTC to call initialize() for us, so call it here
        */
        turret.initialize();

        addComponents(
                new SubsystemComponent(turret),
                BindingsComponent.INSTANCE
        );

        telemetry.addLine("Turret wrap test ready. The turret stays limp until START.");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // The first move happens at START, not INIT
        turret.center();

        // Negated like the drive's turn input: pushing right turns the turret clockwise (negative degrees)
        stickCommand = new TurretWrapStickTest(turret, () -> -gamepad1.right_stick_x, () -> wrapEnabled);
        turret.setDefaultCommand(stickCommand);

        // One action per press
        Gamepads.gamepad1().b().whenBecomesTrue(() -> wrapEnabled = !wrapEnabled);
        Gamepads.gamepad1().a().whenBecomesTrue(() -> turret.center());
        Gamepads.gamepad1().y().whenBecomesTrue(() -> turret.setTargetDirection(180.0));
    }

    @Override
    public void onUpdate() {
        double targetDeg = turret.getTargetDeg();
        double servoPos = turret.getCommandPos();

        telemetry.addData("Mode", wrapEnabled ? "WRAP (unwinds at the limits)" : "CLAMP (stops at the limits)");
        telemetry.addData("Stick u", "%.2f", stickCommand == null ? 0.0 : stickCommand.getStick());
        telemetry.addData("Limits", "%.0f to %.0f deg",
                SubsystemConstants.TurretMinDeg, SubsystemConstants.TurretMaxDeg);

        telemetry.addLine();
        telemetry.addLine("--- Turret ---");
        telemetry.addData("Position (target)", "%.1f deg", targetDeg);
        telemetry.addData("Facing direction", "%.1f deg", TurretSubsystem.normalizeDeg(targetDeg));
        telemetry.addData("Candidates (wrap)", turret.getLastCandidateCount());
        telemetry.addData("Unwinds", turret.getUnwindCount());

        telemetry.addLine();
        telemetry.addLine("--- Servo ---");
        telemetry.addData("Left", "p = %.4f   %.0f us", servoPos, pulseUs(servoPos));

        telemetry.update();
    }

    // Guide, Part 2: w = w_min + p (w_max - w_min). NaN before the first command
    private static double pulseUs(double position) {
        PwmControl.PwmRange range = SubsystemConstants.AxonPWMRange;
        return range.usPulseLower + position * (range.usPulseUpper - range.usPulseLower);
    }
}
