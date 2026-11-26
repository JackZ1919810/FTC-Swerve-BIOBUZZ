package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.commands.IntakeFollowPower;
import org.firstinspires.ftc.teamcode.commands.ShooterFollowRPM;
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "shooter tuning")
public class ShooterTuning extends NextFTCOpMode {
    private static final double TRIGGER_THRESHOLD = 0.5;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private boolean ShooterIdle = false;
    private double selectedRPM = SubsystemConstants.defaultRPM;

    @Override
    public void onInit(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        shooter.initialize();
        intake.initialize();

        addComponents(
                new SubsystemComponent(shooter, intake),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE);

        telemetry.addLine("initialized");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed(){
        shooter.setDefaultCommand(new ShooterFollowRPM(shooter,
                () -> shootHeld() ? selectedRPM : 0.0));
        intake.setDefaultCommand(new IntakeFollowPower(intake,
                () -> intakePower() ));

        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> changeRPM(+SubsystemConstants.RPMStep));
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> changeRPM(-SubsystemConstants.RPMStep));
    }

    @Override
    public void onUpdate(){
        telemetry.addLine("--- Shooter ---");
        telemetry.addData("State", shootHeld() ? "SHOOTING" : "IDLE");
        telemetry.addData("Selected RPM", "%.0f  (max %.0f)", selectedRPM, shooter.getMaxRPM());
        telemetry.addData("Target RPM", "%.0f", shooter.getTargetRPM());
        telemetry.addData(" RPM", "%.0f", shooter.getLeftRPM());

        telemetry.addData("Encoders", shooter.hasEncoderMismatch() ? "MISMATCH: check cables / directions" : "OK");
        telemetry.addData("Battery (V)", "%.2f", shooter.getBatteryVoltage());
        telemetry.addData("Voltage scale", "x%.3f  (tuned at %.1f V%s)", shooter.getVoltageScale(),
                SubsystemConstants.TunedBatteryVolts, SubsystemConstants.VoltageCompensation ? "" : ", OFF");
        telemetry.addData("At speed", shooter.atSpeed());

        telemetry.update();
    }
    private boolean shootHeld() {
        return gamepad1.right_bumper;
    }

    private double intakePower() {
        if (intakeHeld()) return 0.4;
        return 0.0;
    }
    private boolean intakeHeld(){
        return gamepad1.right_trigger > TRIGGER_THRESHOLD;
    }

    private void changeRPM(double delta) {
        // Range.clip needs min <= max; if min were bigger, the result would jump between the two
        double min = SubsystemConstants.MinRPM;
        double max = Math.max(min, shooter.getMaxRPM());
        selectedRPM = Range.clip(selectedRPM + delta, min, max);
    }
}
