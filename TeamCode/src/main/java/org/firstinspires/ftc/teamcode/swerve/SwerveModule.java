package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.util.AnalogEncoder;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.Rotation2d;
import org.firstinspires.ftc.teamcode.util.SwerveModuleState;

public class SwerveModule {
    private DcMotorEx motor;
    private CRServoImplEx servo;
    private AnalogEncoder encoder;
    private SwerveModuleState desiredState = new SwerveModuleState();

    private final PIDController steeringController = new PIDController(
            DrivetrainConstants.STEER_P,
            DrivetrainConstants.STEER_I,
            DrivetrainConstants.STEER_D
    );

    public void init(HardwareMap hMap, String motorName, String servoName, String encoderName, double offset) {
        motor = hMap.get(DcMotorEx.class, motorName);
        servo = hMap.get(CRServoImplEx.class, servoName);
        encoder = new AnalogEncoder(hMap, encoderName, offset);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        servo.setPwmRange(new PwmControl.PwmRange(500.0, 2500.0));
        servo.setDirection(DcMotorSimple.Direction.FORWARD);

        steeringController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * @return current rotation of the module
     */
    public Rotation2d getRotation() {
        return encoder.getRotation();
    }

    /**
     * @return current state of the module
     */
    public SwerveModuleState getState() {
        // approximate speed using power * max speed
        return new SwerveModuleState(motor.getPower() * DrivetrainConstants.MAX_SPEED_MPS, getRotation());
    }

    public void setDesiredState(SwerveModuleState state) {
        // Optimize the state to avoid spinning more than 90 degrees
        this.desiredState = SwerveModuleState.optimize(state, getRotation());
    }

    public void update() {
        // Live update PID constants from Dashboard
        steeringController.setP(DrivetrainConstants.STEER_P);
        steeringController.setI(DrivetrainConstants.STEER_I);
        steeringController.setD(DrivetrainConstants.STEER_D);

        // Convert m/s to motor power [-1.0, 1.0]
        double drivePower = desiredState.speedMetersPerSecond / DrivetrainConstants.MAX_SPEED_MPS;
        motor.setPower(drivePower);

        steeringController.setSetpoint(desiredState.angle.getRadians());
        double steerPower = steeringController.calculate(getRotation().getRadians());
        
        // Clamp output to [-1, 1]
        steerPower = Math.max(-1.0, Math.min(1.0, steerPower));
        servo.setPower(steerPower);
    }

    public void setPID(double p, double i, double d) {
        steeringController.setP(p);
        steeringController.setI(i);
        steeringController.setD(d);
    }

    public void setOffset(double offset) {
        encoder.setOffset(offset);
    }

    public void setServoReversed(boolean reversed) {
        servo.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }
}
