package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.SubsystemComponent;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.commands.SwerveDriveCommand;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends NextFTCOpMode {
    private SwerveDrivetrain drivetrain;
    private boolean fieldCentric = true;
    private boolean lastBack = false;

    @Override
    public void onInit() {
        // Redirect telemetry to FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drivetrain = new SwerveDrivetrain(hardwareMap);
        drivetrain.initialize(); // Ensure hardware is ready before periodic() starts
        
        // Set up default drive command using the NextFTC scheduler
        drivetrain.setDefaultCommand(new SwerveDriveCommand(
                drivetrain,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                () -> fieldCentric
        ));

        // Register the drivetrain subsystem component
        addComponents(new SubsystemComponent(drivetrain));
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        Pose2d pose = drivetrain.getPose();
        telemetry.addData("X", "%.2f in", pose.getX() * 39.3701);
        telemetry.addData("Y", "%.2f in", pose.getY() * 39.3701);
        telemetry.addData("Heading", "%.2f deg", pose.getHeading().getDegrees());
        telemetry.addData("Drive Mode", fieldCentric ? "Field Centric" : "Robot Centric");

        // Heading reset
        if (gamepad1.y) {
            drivetrain.resetPose();
        }

        // Toggle field centric mode
        if (gamepad1.back && !lastBack) {
            fieldCentric = !fieldCentric;
        }
        lastBack = gamepad1.back;
        
        telemetry.update();
    }
}
