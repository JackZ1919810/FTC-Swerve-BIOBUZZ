package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.constants.SubsystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/*
 * Reads each turret servo's position (0 to 1, the same scale as setPosition) from its analog
 * feedback wire, WITHOUT giving the servos a signal.
 *
 * An Axon servo that has received a signal keeps resisting until its power is cut, and that makes
 * the other turret servo hard to turn. So this OpMode never touches the servos themselves (no
 * setPosition, no setPower). The SDK turns every servo signal off when an OpMode starts, and nothing
 * here turns one back on. If a servo was driven since the robot was powered on, power-cycle first.
 *
 * The feedback is an angle, not a position, so each servo needs one calibration point:
 *   position = 0.5 + (feedback angle - center) / TurretServoRangeDeg, the difference wrapped to -180..180
 * where center is the feedback angle when the servo sits at position 0.5. To find it, leave the servo
 * at 0.5 (servo programmer center, or setPosition(0.5)), power-cycle, run this, and press X (left) or
 * B (right). Copy the shown center into SubsystemConstants to keep it.
 *
 * Configuration: add each turret servo's feedback wire as an Analog Input with the names below.
 */
@TeleOp(name = "Turret Servo Read (No Power)")
public class TurretServoReadOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AnalogInput encoderTurretLeft = hardwareMap.get(AnalogInput.class, "turretEncoderL");
        AnalogInput encoderTurretRight = hardwareMap.get(AnalogInput.class, "turretEncoderR");

        // Bulk caching: AUTO reads all of a hub's sensors in one transfer, and reads
        // again by itself when the same value is read a second time
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addLine("Reading only: no signal is sent to the turret servos");
        telemetry.addLine("Turn the servos by hand and watch the positions");
        telemetry.addLine("X: left servo is at 0.5 now   B: right servo is at 0.5 now");
        telemetry.update();

        waitForStart();

        boolean lastX = false;
        boolean lastB = false;

        while (opModeIsActive()) {
            // Read each voltage once per loop, so every number below comes from the same reading
            double leftVolts = encoderTurretLeft.getVoltage();
            double rightVolts = encoderTurretRight.getVoltage();
            double leftAngle = feedbackDeg(leftVolts, encoderTurretLeft.getMaxVoltage());
            double rightAngle = feedbackDeg(rightVolts, encoderTurretRight.getMaxVoltage());

            // X / B (once per press): that servo is sitting at position 0.5 right now
            if (gamepad1.x && !lastX) SubsystemConstants.TurretLeftFeedbackCenterDeg = leftAngle;
            if (gamepad1.b && !lastB) SubsystemConstants.TurretRightFeedbackCenterDeg = rightAngle;
            lastX = gamepad1.x;
            lastB = gamepad1.b;

            double leftPos = servoPos(leftAngle,
                    SubsystemConstants.TurretLeftFeedbackCenterDeg, SubsystemConstants.TurretLeftFeedbackReversed);
            double rightPos = servoPos(rightAngle,
                    SubsystemConstants.TurretRightFeedbackCenterDeg, SubsystemConstants.TurretRightFeedbackReversed);

            telemetry.addLine("--- Servo position (0 to 1) ---");
            telemetry.addData("Left", "%.4f", leftPos);
            telemetry.addData("Right", "%.4f", rightPos);

            telemetry.addLine();
            telemetry.addLine("--- Servo feedback ---");
            telemetry.addData("Left feedback", "%.1f deg (%.2fV)   center %.1f", leftAngle, leftVolts,
                    SubsystemConstants.TurretLeftFeedbackCenterDeg);
            telemetry.addData("Right feedback", "%.1f deg (%.2fV)   center %.1f", rightAngle, rightVolts,
                    SubsystemConstants.TurretRightFeedbackCenterDeg);
            telemetry.addLine("X: left servo is at 0.5 now   B: right servo is at 0.5 now");
            telemetry.update();
        }
    }

    /** Servo shaft angle from the feedback voltage, 0 to 360 degrees. */
    private static double feedbackDeg(double volts, double maxVolts) {
        return volts / maxVolts * 360.0;
    }

    /**
     * Servo position (setPosition scale) from the feedback angle. Below 0 or above 1 means the
     * shaft is past the end of the servo's travel.
     */
    private static double servoPos(double angle, double centerDeg, boolean reversed) {
        double fromCenter = TurretSubsystem.normalizeDeg(angle - centerDeg);
        if (reversed) fromCenter = -fromCenter;
        return 0.5 + fromCenter / SubsystemConstants.TurretServoRangeDeg;
    }
}
