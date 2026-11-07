package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants;

/** NECTER (top) shooter: one flywheel motor with speed control, plus the ball stopper servo. */
public class SUB_ShooterNectar extends SubsystemBase {

    // ---- Speed control gains ----
    private static final double kP = 50.0;
    private static final double kI = 0;
    private static final double kD = 0.0;
    private static final double kF = 13.7;

    private static final double AT_SPEED_TOLERANCE_RPM = 20;
    private static final double TICKS_PER_REV = 84;   // 28 ticks x 3:1 reduction = flywheel revs

    private final DcMotorEx motor;
    private final ServoImplEx stopper;
    private boolean enabled = false;
    private double targetRpm = 0;
    private double lastPower = 0;

    private final OpMode m_opMode;

    public SUB_ShooterNectar(OpMode p_opMode) {
        m_opMode = p_opMode;

        motor = m_opMode.hardwareMap.get(DcMotorEx.class, "shooterNectarMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        stopper = m_opMode.hardwareMap.get(ServoImplEx.class, "stopperNectarServo");
        stopper.setPwmRange(new PwmControl.PwmRange(500, 2500));
        stopper.setDirection(Servo.Direction.REVERSE);   // use FORWARD if it turns the wrong way
        setStopperClosed();
    }

    @Override
    public void periodic() {
        if (motor == null) return;

        motor.setVelocity(targetRpm);
        motor.setPower(lastPower);

        m_opMode.telemetry.addData("Necter Target RPM", "%.0f", targetRpm);
        m_opMode.telemetry.addData("Necter Actual RPM", "%.0f", getRpm());
        m_opMode.telemetry.addData("Necter At Speed", atSpeed());
    }

    // ---- Shooter ----

    /** Spin up to the speed in Constants.ShooterConstants.NECTAR_TARGET_VELOCITY. */
    public void spinUp() {
        start(Constants.ShooterConstants.NECTAR_TARGET_VELOCITY);
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
        stopper.setPosition(Constants.ShooterConstants.NECTAR_SERVO_INITIAL_POS);
    }

    public void setStopperOpen() {
        stopper.setPosition(Constants.ShooterConstants.NECTAR_SERVO_SECOND_POS);
    }

    /** Position the stopper holds for the whole auto (Constants NECTAR_SERVO_AUTO_POS). */
    public void setStopperAuto() {
        stopper.setPosition(Constants.ShooterConstants.NECTAR_SERVO_AUTO_POS);
    }

    /** Slightly lower than closed, used while the intake is running. */
    public void setStopperIntake() {
        stopper.setPosition(Constants.ShooterConstants.NECTAR_SERVO_INITIAL_POS
                - Constants.ShooterConstants.NECTAR_SERVO_INTAKE_OFFSET);
    }
}