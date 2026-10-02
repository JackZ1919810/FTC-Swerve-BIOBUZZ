package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Servo Test OpMode")
public class ServoTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CRServo servoLF = hardwareMap.get(CRServo.class, "servoLF");
        CRServo servoRF = hardwareMap.get(CRServo.class, "servoRF");
        CRServo servoLR = hardwareMap.get(CRServo.class, "servoLR");
        CRServo servoRR = hardwareMap.get(CRServo.class, "servoRR");

        AnalogInput encoderLF = hardwareMap.get(AnalogInput.class, "encoderLF");
        AnalogInput encoderRF = hardwareMap.get(AnalogInput.class, "encoderRF");
        AnalogInput encoderLR = hardwareMap.get(AnalogInput.class, "encoderLR");
        AnalogInput encoderRR = hardwareMap.get(AnalogInput.class, "encoderRR");

        telemetry.addLine("Hold button to test servo:");
        telemetry.addLine("A: Left Front, B: Right Front, X: Left Rear, Y: Right Rear");
        telemetry.addLine("Left Stick Y: Power");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;

            if (gamepad1.a) servoLF.setPower(power);
            else servoLF.setPower(0);

            if (gamepad1.b) servoRF.setPower(power);
            else servoRF.setPower(0);

            if (gamepad1.x) servoLR.setPower(power);
            else servoLR.setPower(0);

            if (gamepad1.y) servoRR.setPower(power);
            else servoRR.setPower(0);

            telemetry.addData("Target Power", "%.2f", power);
            telemetry.addLine("--- Encoders (Radians) ---");
            telemetry.addData("LF", "%.3f rad (%.2fV / %.2fV)", getRadians(encoderLF), encoderLF.getVoltage(), encoderLF.getMaxVoltage());
            telemetry.addData("RF", "%.3f rad (%.2fV / %.2fV)", getRadians(encoderRF), encoderRF.getVoltage(), encoderRF.getMaxVoltage());
            telemetry.addData("LR", "%.3f rad (%.2fV / %.2fV)", getRadians(encoderLR), encoderLR.getVoltage(), encoderLR.getMaxVoltage());
            telemetry.addData("RR", "%.3f rad (%.2fV / %.2fV)", getRadians(encoderRR), encoderRR.getVoltage(), encoderRR.getMaxVoltage());
            telemetry.update();
        }
    }

    private double getRadians(AnalogInput encoder) {
        return (encoder.getVoltage() / encoder.getMaxVoltage()) * 2.0 * Math.PI;
    }
}
