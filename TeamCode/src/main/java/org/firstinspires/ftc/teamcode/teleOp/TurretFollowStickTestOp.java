package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.commands.TurretFollowStickTest;
import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "Turret Test With Joystick")
public class TurretFollowStickTestOp extends NextFTCOpMode {

    //Subsystem & command setup
    private TurretSubsystem turret;
    private TurretFollowStickTest stickCommand;

    //Enable the wrapping mode
    private boolean wrapEnabled = true;

    @Override
    public void onInit(){
        //set up telemetry for FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //constructor
        turret = new TurretSubsystem(hardwareMap);

        turret.initialize();

        //Register subsystem component
        addComponents(
                new SubsystemComponent(turret), BindingsComponent.INSTANCE
        );

        telemetry.addLine("initialized");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed(){
        turret.center();

        //set up default command for stick controlling the turn
        stickCommand = new TurretFollowStickTest(turret, () -> -gamepad1.right_stick_x, () -> wrapEnabled);
        turret.setDefaultCommand(stickCommand);

        //One action per press commands
        Gamepads.gamepad1().b().whenBecomesTrue(() -> wrapEnabled = !wrapEnabled);
        Gamepads.gamepad1().a().whenBecomesTrue(() -> turret.center());
        Gamepads.gamepad1().y().whenBecomesTrue(() -> turret.setTargetDirection(180.0));
    }

    @Override
    public void onUpdate(){
        //everything under onUpdate is for telemetry
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

    private static double pulseUs(double position){
        PwmControl.PwmRange range = SubsystemConstants.AxonPWMRange;
        return  range.usPulseLower + position * (range.usPulseUpper - range.usPulseLower);
    }

}

