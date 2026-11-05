package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

/** POLLEN (bottom) shooter: one flywheel motor with speed control, plus the ball stopper servo. */
public class SUB_ShooterPollen extends SubsystemBase {

    // ---- Speed control gains ----
    private static final double kP = 40.0;//1.067;
    private static final double kI = 0;//0.11;
    private static final double kD = 0.0;
    private static final double kF = 13.7;//11.7;

    private static final double AT_SPEED_TOLERANCE_RPM = 20;
    private static final double TICKS_PER_REV = 84;   // 28 ticks x 3:1 reduction = flywheel revs

    private final DcMotorEx motor;
    private final ServoImplEx stopper;
    private boolean enabled = false;
    private double targetRpm = 0;
    private double lastPower = 0;

    private final OpMode m_opMode;

    public SUB_ShooterPollen(OpMode p_opMode) {
        m_opMode = p_opMode;

        motor = m_opMode.hardwareMap.get(DcMotorEx.class, "shooterPollenMotor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        stopper = m_opMode.hardwareMap.get(ServoImplEx.class, "stopperPollenServo");
        stopper.setPwmRange(new PwmControl.PwmRange(500, 2500));
        setStopperClosed();

    }

    @Override
    public void periodic() {
        if (motor == null) return;

        motor.setVelocity(targetRpm);
        motor.setPower(lastPower);

        m_opMode.telemetry.addData("Pollen Target RPM", "%.0f", targetRpm);
        m_opMode.telemetry.addData("Pollen Actual RPM", "%.0f", getRpm());
        m_opMode.telemetry.addData("Pollen At Speed", atSpeed());
    }

    // ---- Shooter ----

    /** Spin up to the speed in Constants.ShooterConstants.POLLEN_TARGET_VELOCITY. */
    public void spinUp() {
        start(Constants.ShooterConstants.POLLEN_TARGET_VELOCITY);
    }

    public void start(double rpm) {
        targetRpm = rpm;
        enabled = true;
    }

    public void stop() {
        enabled = false;
        targetRpm = 0;
    }

    public double getRpm() {
        return motor.getVelocity();
    }

    public boolean atSpeed() {
        return enabled && Math.abs(targetRpm - getRpm()) <= AT_SPEED_TOLERANCE_RPM;
    }

    // ---- Stopper ----

    public void setStopperClosed() {
        stopper.setPosition(Constants.ShooterConstants.POLLEN_SERVO_INITIAL_POS);
    }

    public void setStopperOpen() {
        stopper.setPosition(Constants.ShooterConstants.POLLEN_SERVO_SECOND_POS);
    }
}